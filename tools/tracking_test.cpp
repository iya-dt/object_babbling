#include <iostream>
#include <math.h>

#include <Eigen/Core>
#include <thread>
#include <chrono>
#include <string>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>

#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>

#include <image_processing/Objects.h>

#include <cafer_core/cafer_core.hpp>

#include "object_babbling/is_moving.h"
#include "object_babbling/get_targets.h"
#include "object_babbling/set_target.h"
#include "object_babbling/track.h"

#include "globals.h"

using namespace Eigen;
using namespace rgbd_utils;
namespace ip = image_processing;

using namespace cafer_core;
using namespace object_babbling;


class TrackingTest : public Component {
    using Component::Component;

public:

    ~TrackingTest()
    {
        ROS_ERROR("TRACKING_TEST : destroy?");
        client_disconnect_from_ros();
    }

    void client_connect_to_ros() override
    {
        std::stringstream display_params;
        XmlRpc::XmlRpcValue glob_params;
        XmlRpc::XmlRpcValue exp_params;

        cafer_core::ros_nh->getParam("/object_babbling/params", glob_params);

        for (auto& param:glob_params) {
            display_params << "\t" << param.first << ":\t" << param.second << std::endl;
        }

        ROS_INFO_STREAM("BABBLING_NODE : global parameters retrieved:" << std::endl << display_params.str());

        // Images
        _images_sub.reset(
            new RGBD_Subscriber(
                glob_params["rgb_info_topic"],
                glob_params["rgb_topic"],
                glob_params["depth_info_topic"],
                glob_params["depth_topic"],
                *cafer_core::ros_nh
            )
        );

        // Target
        _get_targets_cli.reset(
            new ros::ServiceClient(ros_nh->serviceClient<get_targets>(glob_params["get_targets_service"]))
        );
        _set_target_cli.reset(
            new ros::ServiceClient(ros_nh->serviceClient<set_target>(glob_params["set_target_service"]))
        );

        _track_cli.reset(
            new ros::ServiceClient(ros_nh->serviceClient<track>(glob_params["track_service"]))
        );

        // Supervoxel target
        _goal_point_pub.reset(
            new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>(glob_params["goal_point_topic"], 5))
        );

        _is_finish_pub.reset(
            new Publisher(ros_nh->advertise<std_msgs::Bool>(glob_params["is_finish_topic"],5))
        );
    }

    void init() override
    {
        client_connect_to_ros();

        _target_center << 0.0, 0.0, 0.0, 0.0;

        _tracking = false;

        _counter_iter = 0;
    }

    void client_disconnect_from_ros() override
    {
        cafer_core::DBManager db_request;
        std::string supervisor_name =
            ros::names::parentNamespace(cafer_core::ros_nh->getNamespace()) + "/babbling";

        // stop tracking
        ROS_INFO_STREAM("TRACKING_TEST : stop tracking");
        track track_request;
        track_request.request.tracking = false;
        _track_cli->call(track_request);

        _images_sub.reset();
        _get_targets_cli.reset();
        _set_target_cli.reset();
        _track_cli.reset();
        _goal_point_pub.reset();
        _is_finish_pub.reset();
    }

    void update() override
    {
        std::string supervisor_name =
            ros::names::parentNamespace(cafer_core::ros_nh->getNamespace()) + "/babbling";

        if (!_tracking) {
            // get targets
            ROS_INFO_STREAM("TRACKING_TEST : getting targets");
            get_targets get_request;
            if (!_get_targets_cli->call(get_request)) {
                ROS_ERROR_STREAM("TRACKING_TEST : error getting targets");
                return;
            }

            std::vector<ip::Object> objs;
            std::istringstream sstream(get_request.response.targets);
            boost::archive::text_iarchive iTextArchive(sstream);
            iTextArchive >> objs;

            ROS_INFO_STREAM("TRACKING_TEST : " << objs.size() << " targets received");

            int target_id;
            _choose_target(objs, target_id, _target_center);

            // set target
            ROS_INFO_STREAM("TRACKING_TEST : setting targets");
            set_target set_request;
            set_request.request.target_id = target_id;
            if (!_set_target_cli->call(set_request)) {
                ROS_ERROR_STREAM("TRACKING_TEST : error setting target");
                return;
            }

            // start tracking
            ROS_INFO_STREAM("TRACKING_TEST : start tracking");
            track track_request;
            track_request.request.tracking = true;
            _track_cli->call(track_request);
            _tracking = true;
        }

        _counter_iter += 1;
        ROS_INFO_STREAM("TRACKING_TEST : iteration " << _counter_iter);

        publish_feedback();
    }

    bool is_finish(){
        return _counter_iter>=_nb_iter;
    }

    void publish_feedback() {
        if (!(_target_center[0] == 0 || _target_center[1] == 0 || _target_center[2] == 0)) {
            ip::PointCloudXYZ point_goal;
            point_goal.push_back(pcl::PointXYZ(_target_center[0], _target_center[1], _target_center[2]));
            sensor_msgs::PointCloud2 point_goal_msg;

            pcl::toROSMsg(point_goal, point_goal_msg);
            point_goal_msg.header = _images_sub->get_depth().header;
            _goal_point_pub->publish(point_goal_msg);
        }
    }

private:

    RGBD_Subscriber::Ptr _images_sub;

    std::unique_ptr<ros::ServiceClient> _get_targets_cli;
    std::unique_ptr<ros::ServiceClient> _set_target_cli;
    std::unique_ptr<ros::ServiceClient> _track_cli;

    std::unique_ptr<Publisher> _goal_point_pub;
    std::unique_ptr<Publisher> _is_finish_pub;

    Vector4d _target_center;

    bool _tracking;

    int _counter_iter;
    int _nb_iter = 500;

    void _choose_target(std::vector<ip::Object> objs, int& target_id, Vector4d& target_center)
    {
        ROS_INFO_STREAM("TRACKING_TEST : choosing target");
        target_id = 0;
        for (unsigned int i = 0; i < objs.size(); i++) {
            if (objs[i].object_cloud.size() > objs[target_id].object_cloud.size()) {
              target_id = i;
            }
        }
        pcl::compute3DCentroid(objs[target_id].object_cloud, target_center);
    }
};

int main(int argc, char** argv)
{
    std::string node_name;
    XmlRpc::XmlRpcValue cafer;

    node_name = global::parse_arg(argc, argv, "tracking_test_node");

    cafer_core::init(argc, argv, node_name);
    cafer_core::ros_nh->getParam("cafer", cafer);

    TrackingTest tracking_test(cafer["mgmt"], cafer["type"], cafer["freq"], cafer["uuid"]);

    tracking_test.wait_for_init();

    while (ros::ok() && (!tracking_test.get_terminate()) && !tracking_test.is_finish()) {
        tracking_test.spin();
        tracking_test.update();
        tracking_test.sleep();
    }

    ROS_INFO_STREAM("TRACKING_TEST : ros::ok = " << ros::ok()
                    << " terminate = " << tracking_test.get_terminate()
                    << " is finish = " << tracking_test.is_finish());

    ROS_INFO_STREAM("TRACKING_TEST : is finish");

    return 0;
}
