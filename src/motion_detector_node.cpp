#include <cafer_core/cafer_core.hpp>
#include <rgbd_utils/rgbd_subscriber.hpp>
#include <image_processing/MotionDetection.h>
#include <image_processing/default_parameters.hpp>
#include <object_babbling/rgbd_motion_data.h>
#include <object_babbling/is_moving.h>
#include "globals.h"
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Bool.h>

using namespace cafer_core;

class MotionSensor : public Component {
    using Component::Component;

public:

    ~MotionSensor()
    {
        client_disconnect_from_ros();
    }

    void init() override
    {
        _image_callback = [this](const sensor_msgs::ImageConstPtr& depth_msg,
                                 const sensor_msgs::ImageConstPtr& rgb_msg,
                                 const sensor_msgs::CameraInfoConstPtr& rgb_info_msg,
                                 const sensor_msgs::CameraInfoConstPtr& depth_info_msg)
        {
            object_babbling::rgbd_motion_data msg;
            object_babbling::motion_rect serialized_rect;
            std::vector<cv::Rect> motion_ROIs;
            cv_bridge::CvImagePtr cv_ptr;
            try {
                //Detect motion
                cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::RGB8);
                _image_worker.detect_MOG(cv_ptr->image);
                motion_ROIs = _image_worker.getResultsRects();

                if (_current_past_frame[0].empty()) {
                    cv_ptr->image.copyTo(_current_past_frame[0]);
                }
                else {
                    cv_ptr->image.copyTo(_current_past_frame[1]);
                }

                //Fill and publish message
                if (!motion_ROIs.empty()) {
                    for (auto rect:motion_ROIs) {
                        serialized_rect.x = rect.x;
                        serialized_rect.y = rect.y;
                        serialized_rect.width = rect.width;
                        serialized_rect.height = rect.height;

                        msg.motion_rects.push_back(serialized_rect);
                    }
                }

                msg.header.stamp = ros::Time::now();
                msg.rgb_info = *rgb_info_msg;
                msg.rgb = *rgb_msg;
                msg.depth = *depth_msg;

                _motion_data_publisher->publish(msg);
            }
            catch (cv_bridge::Exception& e) { ROS_ERROR("MOTION_DETECTOR : cv_bridge exception: %s", e.what()); }
        };

        client_connect_to_ros();
        _is_init = true;
    }


    void client_connect_to_ros() override
    {

        XmlRpc::XmlRpcValue params;

        cafer_core::ros_nh->getParam("/object_babbling/params", params);

        ros::NodeHandle mt_callback_nh;

        mt_callback_nh.setCallbackQueue(&_image_processing_cb_q);
        _motion_data_publisher.reset(new ros::Publisher(
                mt_callback_nh.advertise<object_babbling::rgbd_motion_data>(ros::this_node::getName(), 5)));

        _rgbd_subscriber.reset(new rgbd_utils::RGBD_Subscriber(
                params["rgb_info_topic"], params["rgb_topic"], params["depth_info_topic"],
                params["depth_topic"], mt_callback_nh, _image_callback));

        _is_motion_srv.reset(new ros::ServiceServer(cafer_core::ros_nh->advertiseService
                ("is_moving", &MotionSensor::is_moving_cb, this)));

        _iter_end.reset(new Subscriber(ros_nh->subscribe<std_msgs::Bool>(params["is_finish"],5,&MotionSensor::end_iteration_cb,this)));

        _spinner.reset(new ros::AsyncSpinner(4, &_image_processing_cb_q));
        _spinner->start();
    }

    void client_disconnect_from_ros() override
    {
        _spinner->stop();
        _motion_data_publisher.reset();
        _spinner.reset();
        _rgbd_subscriber.reset();
        _is_motion_srv.reset();
        _iter_end.reset();
    }

    void update() override
    {

    };

    void end_iteration_cb(const std_msgs::BoolConstPtr& is_finish)
    {
        if(is_finish->data){
            ROS_INFO_STREAM("MOTION_DETECTOR : iteration is finish");
        }
    }

    bool is_moving_cb(object_babbling::is_moving::Request& req, object_babbling::is_moving::Response& rep)
    {
        ROS_INFO_STREAM("MOTION_DETECTOR : is_moving_cb");
        try {

            std::vector<cv::Mat> frames(2);
            cv::Mat mask = cv_bridge::toCvCopy(req.supervoxel, sensor_msgs::image_encodings::MONO8)->image;
            _current_past_frame[0].copyTo(frames[0], mask);
            _current_past_frame[1].copyTo(frames[1], mask);

            _image_worker.setInputFrames(frames);
            cv::Mat diff;

            bool is_moving = _image_worker.detect(diff,20);
            if (is_moving) {
                rep.has_moved.data = 1;
                ROS_INFO_STREAM("MOTION_DETECTOR : positive sample");
            }
            else {
                rep.has_moved.data = 0;

                ROS_INFO_STREAM("MOTION_DETECTOR : negative sample");
            }
            _current_past_frame[1].copyTo(_current_past_frame[0]);


            ROS_INFO_STREAM("MOTION_DETECTOR : done ");
        }
        catch (cv::Exception& e) {
            ROS_ERROR_STREAM(e.what());
            exit(1);
        }

        return true;
    }

private:

    void _3d_coord_to_pixel_coord(std::array<double, 2>& pix, const std::array<double, 3>& coord)
    {
        pix[0] = image_processing::parameters::camera::focal_length_x * coord[0] / coord[2]
                 + image_processing::parameters::camera::rgb_princ_pt_x;
        pix[1] = image_processing::parameters::camera::focal_length_y * coord[1] / coord[2]
                 + image_processing::parameters::camera::rgb_princ_pt_y;
    }

    MotionDetection _image_worker;
    std::unique_ptr<rgbd_utils::RGBD_Subscriber> _rgbd_subscriber;
    std::unique_ptr<ros::Publisher> _motion_data_publisher;
    std::unique_ptr<ros::ServiceServer> _is_motion_srv;
    std::unique_ptr<Subscriber> _iter_end;

    std::function<void(const sensor_msgs::ImageConstPtr& depth_msg,
                       const sensor_msgs::ImageConstPtr& rgb_msg,
                       const sensor_msgs::CameraInfoConstPtr& rgb_info_msg,
                       const sensor_msgs::CameraInfoConstPtr& depth_info_msg)> _image_callback;

    ros::CallbackQueue _image_processing_cb_q;
    std::unique_ptr<ros::AsyncSpinner> _spinner;

    std::array<cv::Mat, 2> _current_past_frame;
};


int main(int argc, char** argv)
{
    std::string node_name;
    XmlRpc::XmlRpcValue cafer;

    cafer_core::init(argc, argv, node_name);
    cafer_core::ros_nh->getParam("cafer", cafer);

    node_name = global::parse_arg(argc, argv, "motion_detector_node");

    cafer_core::init(argc, argv, node_name);

    MotionSensor motion_detect(cafer["mgmt"], cafer["type"], cafer["freq"], cafer["uuid"]);

    motion_detect.wait_for_init();
    motion_detect.spin();

    while (ros::ok() && (!motion_detect.get_terminate())) {
        motion_detect.spin();
        motion_detect.update();
        motion_detect.sleep();
    }

    return 0;
}
