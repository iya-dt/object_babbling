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
#include <actionlib/client/simple_action_client.h>
#include <yaml-cpp/yaml.h>

#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>

#include <image_processing/Objects.h>

#include <cafer_core/cafer_core.hpp>

#include "object_babbling/pose_goalAction.h"
#include "object_babbling/is_moving.h"
#include "object_babbling/get_targets.h"
#include "object_babbling/set_target.h"
#include "object_babbling/track.h"
#include "object_babbling/dataset.h"
#include "object_babbling/gmm_archive.h"

#include "globals.h"

using namespace Eigen;
using namespace rgbd_utils;
namespace ip = image_processing;

using namespace cafer_core;
using namespace object_babbling;


class Babbling : public Component {
    using Component::Component;

public:

    ~Babbling()
    {
        ROS_ERROR("BABBLING_NODE : destroy?");
        client_disconnect_from_ros();
    }

    void client_connect_to_ros() override
    {
        std::stringstream display_params;
        XmlRpc::XmlRpcValue glob_params;
        XmlRpc::XmlRpcValue exp_params;

        cafer_core::ros_nh->getParam("/object_babbling/params", glob_params);
        cafer_core::ros_nh->getParam("/object_babbling/babbling/experiment", exp_params);

        for (auto& param:glob_params) {
            display_params << "\t" << param.first << ":\t" << param.second << std::endl;
        }

        ROS_INFO_STREAM("BABBLING_NODE : global parameters retrieved:" << std::endl << display_params.str());

        _nb_iter = static_cast<int>(exp_params["number_of_iteration"]);

        _client_controller.reset(
                new actionlib::SimpleActionClient<pose_goalAction>("controller_node/"+static_cast<std::string>(glob_params["controller_server"]), false));

        _client_motion.reset(
                new ros::ServiceClient(ros_nh->serviceClient<is_moving>(glob_params["motion_detector_service"])));

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


        //DB Manager client
        _db_request_publisher.reset(
            new Publisher(ros_nh->advertise<cafer_core::DBManager>(
                static_cast<std::string>(glob_params["database_server"]) + "/request", 10))
        );
        _db_status_subscriber.reset(
            new Subscriber(ros_nh->subscribe<cafer_core::DBManager>(
                static_cast<std::string>(glob_params["database_server"]) + "/status", 1,
                boost::bind(&Babbling::_done_callback, this, _1)))
        );
    }

    void init() override
    {
        client_connect_to_ros();

        _target_center << 0.0, 0.0, 0.0, 0.0;
        _target_normal << 0.0, 0.0, 0.0, 0.0;

        _db_init = false;
        _db_ready = true;
        _recording = false;

        _counter_iter = 0;
    }

    void client_disconnect_from_ros() override
    {
        cafer_core::DBManager db_request;
        std::string supervisor_name =
            ros::names::parentNamespace(cafer_core::ros_nh->getNamespace()) + "/babbling";

        //PLACEHOLDER: This is necessary as the supervisor node is supposed to do this...
        ClientDescriptor supervisor;
        if (find_by_name(supervisor_name, supervisor)) {
            db_request.type = static_cast<uint8_t>(DatabaseManager::Request::STOP_RECORDING);
            db_request.id = supervisor.id;

            _db_request_publisher->publish(db_request);
        }
        else {
            ROS_ERROR_STREAM("BABBLING_NODE : unable to find supervisor at: " << supervisor_name);
        }

        _images_sub.reset();
        _get_targets_cli.reset();
        _set_target_cli.reset();
        _track_cli.reset();
        _goal_point_pub.reset();
        _is_finish_pub.reset();

        _db_request_publisher.reset();
        _db_status_subscriber.reset();
    }

    void update() override
    {
        std::string supervisor_name =
            ros::names::parentNamespace(cafer_core::ros_nh->getNamespace()) + "/babbling";

        pose_goalGoal poseGoal;

        if (_db_init) {
            if (_db_ready && !_recording) {
                ROS_WARN_STREAM("BABBLING_NODE : STARTING RECORD");
                //Enable recording to DB
                //PLACEHOLDER: This is necessary as the supervisor node is supposed to do this...
                cafer_core::DBManager db_request;
                ClientDescriptor supervisor;
                if (find_by_name(supervisor_name, supervisor)) {
                    db_request.type = static_cast<uint8_t>(DatabaseManager::Request::RECORD_DATA);
                    db_request.id = supervisor.id;
                    _db_request_publisher->publish(db_request);
                }
                else {
                    ROS_ERROR_STREAM("BABBLING_NODE : unable to find supervisor at: " << supervisor_name);
                }

                _db_ready = false;
                _recording = true;
            }

            if (_robot_controller_ready && _recording) {
                // get targets
                ROS_INFO_STREAM("BABBLING_NODE : getting targets");
                get_targets get_request;
                if (!_get_targets_cli->call(get_request)) {
                    ROS_ERROR_STREAM("BABBLING_NODE : error getting targets");
                    return;
                }

                std::vector<ip::Object> objs;
                std::istringstream sstream(get_request.response.targets);
                boost::archive::text_iarchive iTextArchive(sstream);
                iTextArchive >> objs;

                ROS_INFO_STREAM("BABBLING_NODE : " << objs.size() << " targets received");

                int target_id;
                _choose_target(objs, target_id, _target_center);

                // set target
                ROS_INFO_STREAM("BABBLING_NODE : setting targets");
                set_target set_request;
                set_request.request.target_id = target_id;
                if (!_set_target_cli->call(set_request)) {
                    ROS_ERROR_STREAM("BABBLING_NODE : error setting target");
                    return;
                }

                _choose_action(objs, target_id, _target_normal);

                Vector4d target_center_robot;
                babbling::base_conversion(target_center_robot, _target_center);

                poseGoal.target_pose.resize(3);
                poseGoal.target_pose[0] = target_center_robot(0);
                poseGoal.target_pose[1] = target_center_robot(1);
                poseGoal.target_pose[2] = target_center_robot(2);

                ROS_INFO_STREAM("BABBLING_NODE : going to pose "
                    << poseGoal.target_pose[0] << " "
                    << poseGoal.target_pose[1] << " "
                    << poseGoal.target_pose[2]);

                // start tracking
                ROS_INFO_STREAM("BABBLING_NODE : start tracking");
                track track_request;
                track_request.request.tracking = true;
                _track_cli->call(track_request);

                _client_controller->sendGoal(poseGoal);
                _robot_controller_ready = false;
            }

            if (!_robot_controller_ready) {
                _client_controller->waitForResult(ros::Duration(1.0));
                auto client_status = _client_controller->getState();
                if (client_status == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO_STREAM("BABBLING_NODE : position reached.");
                    _robot_controller_ready = true;

                    // stop tracking
                    ROS_INFO_STREAM("BABBLING_NODE : stop tracking");
                    track stop_track_request;
                    stop_track_request.request.tracking = false;
                    _track_cli->call(stop_track_request);

                    std::this_thread::sleep_for(std::chrono::milliseconds(150));

                    ROS_WARN_STREAM("BABBLING_NODE : STOP RECORD");
                    //Disable recording to DB
                    //PLACEHOLDER: This is necessary as the supervisor node is supposed to do this...
                    cafer_core::DBManager db_request;
                    ClientDescriptor supervisor;
                    if (find_by_name(supervisor_name, supervisor)) {
                        db_request.type = static_cast<uint8_t>(DatabaseManager::Request::STOP_RECORDING);
                        db_request.id = supervisor.id;
                        _db_request_publisher->publish(db_request);
                    }
                    else {
                        ROS_ERROR_STREAM("BABBLING_NODE : unable to find supervisor at: " << supervisor_name);
                    }
                    _recording = false;


                    _counter_iter += 1;
                    ROS_INFO_STREAM("BABBLING_NODE : iteration " << _counter_iter << " done (nb_iter = " << _nb_iter << ")");
                }
                else if (client_status == actionlib::SimpleClientGoalState::ABORTED) {
                    ROS_INFO_STREAM("BABBLING_NODE : position wasn't reachable");
                    _robot_controller_ready = true;

                    // stop tracking
                    ROS_INFO_STREAM("BABBLING_NODE : stop tracking");
                    track stop_track_request;
                    stop_track_request.request.tracking = false;
                    _track_cli->call(stop_track_request);
                }
                else {
                    ROS_INFO_STREAM("BABBLING_NODE : waiting for controller...");
                }
            }
        }

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

    std::unique_ptr<ros::ServiceClient> _client_motion;
    std::unique_ptr<actionlib::SimpleActionClient<pose_goalAction>> _client_controller;

    RGBD_Subscriber::Ptr _images_sub;

    std::unique_ptr<ros::ServiceClient> _get_targets_cli;
    std::unique_ptr<ros::ServiceClient> _set_target_cli;
    std::unique_ptr<ros::ServiceClient> _track_cli;

    std::unique_ptr<Publisher> _goal_point_pub;
    std::unique_ptr<Publisher> _is_finish_pub;

    std::unique_ptr<Publisher> _db_request_publisher;
    std::unique_ptr<Subscriber> _db_status_subscriber;

    Vector4d _target_center;
    Vector4d _target_normal;

    bool _robot_controller_ready = true;
    bool _db_ready;
    bool _db_init;
    bool _recording;

    int _counter_iter;
    int _nb_iter;

    void _choose_target(std::vector<ip::Object> objs, int& target_id, Vector4d& target_center)
    {
        ROS_INFO_STREAM("BABBLING_NODE : choosing target");
        target_id = rand() % objs.size();
        pcl::compute3DCentroid(objs[target_id].object_cloud, target_center);
    }

    void _choose_action(std::vector<ip::Object> objs, int target_id, Vector4d& target_normal)
    {
        ROS_INFO_STREAM("BABBLING_NODE : choosing action");
        double theta = 2 * M_PI * ( (double) rand() / (double) RAND_MAX );
        target_normal << cos(theta), sin(theta), 0, 0;
    }

    void _done_callback(const cafer_core::DBManagerConstPtr& status_msg)
    {
        std::string supervisor_name =
            ros::names::parentNamespace(cafer_core::ros_nh->getNamespace()) + "/babbling";

        //PLACEHOLDER: This is necessary as the supervisor node is supposed to do this...
        ClientDescriptor supervisor;

        if (find_by_name(supervisor_name, supervisor)) {
            if (status_msg->id == supervisor.id) {
                if (static_cast<DatabaseManager::Response>(status_msg->type) ==
                        DatabaseManager::Response::STATUS_READY) {
                    _db_init = true;
                    _db_ready = true;
                }
                else {
                    _db_ready = false;
                }
            }
        }
        else {
            ROS_ERROR_STREAM("BABBLING_NODE : unable to find supervisor at: " << supervisor_name);
        }
    }

};

int main(int argc, char** argv)
{
    std::string node_name;
    XmlRpc::XmlRpcValue cafer;

    node_name = global::parse_arg(argc, argv, "babbling_node");

    cafer_core::init(argc, argv, node_name);
    cafer_core::ros_nh->getParam("cafer", cafer);

    Babbling babbling(cafer["mgmt"], cafer["type"], cafer["freq"], cafer["uuid"]);

    babbling.wait_for_init();

    while (ros::ok() && (!babbling.get_terminate()) && !babbling.is_finish()) {
        babbling.spin();
        babbling.update();
        babbling.sleep();
    }

    ROS_INFO_STREAM("BABBLING : ros::ok = " << ros::ok()
                    << " terminate = " << babbling.get_terminate()
                    << " is finish = " << babbling.is_finish());

    ROS_INFO_STREAM("BABBLING : is finish");

    return 0;
}
