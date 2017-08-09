#include <iostream>
#include <string>
#include <random>
#include <fstream>
#include <cmath>

#include <boost/timer.hpp>
#include <boost/timer.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <actionlib/server/simple_action_server.h>
#include <tf/transform_datatypes.h>
#include <ros/callback_queue.h>
#include <moveit/robot_state/conversions.h>
#include "baxter_mover_utils/baxter_mover_utils/baxter_mover.hpp"
#include <baxter_mover_utils/move_baxter_arm.h>

#include <cafer_core/cafer_core.hpp>

#include "object_babbling/pose_goalAction.h"


using namespace object_babbling;
using namespace cafer_core;

class Controller: public Component{
    using Component::Component; // C++11 requirement to inherit the constructor

public:

    void client_connect_to_ros(){
        XmlRpc::XmlRpcValue glob_params;
        std::stringstream display_params;

        cafer_core::ros_nh->getParam("/object_babbling/params", glob_params);
        for (auto& param:glob_params) {
            display_params << "\t" << param.first << ":\t" << param.second << std::endl;
        }

        ROS_INFO_STREAM("CONTROLLER : Global parameters retrieved:" << std::endl << display_params.str());
        ROS_INFO_STREAM("CONTROLLER : node name space is: " << cafer_core::ros_nh->getNamespace());

        XmlRpc::XmlRpcValue wks;
        cafer_core::ros_nh->getParamCached("/object_babbling/controller_node/experiment/workspace/csg_intersect_cuboid", wks);

        _wks_center_x = static_cast<double>(wks["x_max"]) + static_cast<double>(wks["x_min"]);
        _wks_center_x /= 2;
        _wks_center_y = static_cast<double>(wks["y_max"]) + static_cast<double>(wks["y_min"]);
        _wks_center_y /= 2;

        ROS_INFO_STREAM("CONTROLLER : workspace center is (" << _wks_center_x << ", " << _wks_center_y << ")");

        _serv.reset(new actionlib::SimpleActionServer<pose_goalAction>(*cafer_core::ros_nh, glob_params["controller_server"],
                    boost::bind(&Controller::execute, this, _1),false));
        _baxter_mover.reset(new baxter_mover::BAXTER_Mover(*cafer_core::ros_nh));
        _move_baxter_arm.reset(new ros::ServiceClient(cafer_core::ros_nh->serviceClient<baxter_mover_utils::move_baxter_arm>("/move_baxter_arm")));
    }

    void init(){
        connect_to_ros();

        usleep(2e6);

        _serv->start();
        _is_init = true;

        _home_variable_values.insert ( std::pair<std::string, double>("left_s0",  1.5) );
        _home_variable_values.insert ( std::pair<std::string, double>("left_s1", -1.2) );
        _home_variable_values.insert ( std::pair<std::string, double>("left_e0", -1.0) );
        _home_variable_values.insert ( std::pair<std::string, double>("left_e1",  1.5) );
        _home_variable_values.insert ( std::pair<std::string, double>("left_w0",  0.4) );
        _home_variable_values.insert ( std::pair<std::string, double>("left_w1",  1.5) );
        _home_variable_values.insert ( std::pair<std::string, double>("left_w2",  1.02) );

        home_values_ = {1.5, -1.2 -1.0, 1.5, 0.4, 1.5, 1.02};

        _uniform = std::uniform_real_distribution<double>(0, M_PI/4);

        ros::AsyncSpinner my_spinner(4);
        my_spinner.start();

        _baxter_mover->group->setPlannerId(static_cast<std::string>(_baxter_mover->global_parameters.get_planner_parameters()["planner_id"]));
        ROS_INFO_STREAM("CONTROLLER: planner id is: " << static_cast<std::string>(_baxter_mover->global_parameters.get_planner_parameters()["planner_id"]));
        //_baxter_mover->group->setPlanningTime(std::stod(_baxter_mover->global_parameters.get_planner_parameters()["planning_time"]));

        // geometry_msgs::PoseStamped pose = _baxter_mover->group->getPoseTarget();
        // ROS_WARN_STREAM("CONTROLLER_NODE : " << pose.pose.position.x << " "
        //                                      << pose.pose.position.y << " "
        //                                      << pose.pose.position.z << " "
        //                                    );

        while (!home_position()) {
          ROS_INFO("CONTROLLER: failed to go home");
        }
    }

    //get largest difference between elements of two vectors
    double largest_difference(std::vector<double> &first, std::vector<double> &second){
        Eigen::VectorXd difference(first.size());
        double my_max = 0;
        for(size_t j = 0; j < first.size(); ++j)
            difference(j) = fabs(first[j] - second[j]);
        for(size_t j = 0; j < first.size(); ++j){
            if(difference(j) > my_max)
                my_max = difference(j);
        }
        return my_max;
    }

// Don't work with the baxter :(
//    void extract_arm_joints_values(){
//        std::vector<std::string> joint_names;
//        std::vector<double> joint_values;
//        joint_names = _baxter_mover->global_parameters.get_baxter_arm_joints_names();

//        for(unsigned i = 0; i < joint_names.size(); ++i){
//            joint_values.push_back(_baxter_mover->global_parameters.get_joint_state().position[distance
//                                   (_baxter_mover->global_parameters.get_joint_state().name.begin(),
//                                    find(_baxter_mover->global_parameters.get_joint_state().name.begin(),
//                                         _baxter_mover->global_parameters.get_joint_state().name.end(),
//                                         joint_names[i]))]);
//        }
//        arm_joint_values_ = joint_values;
//    }

    bool home_position()
    {
      if (_baxter_mover->group->setJointValueTarget(_home_variable_values)) {
          if (_baxter_mover->group->plan(_group_plan)) {
              if (_baxter_mover->group->execute(_group_plan)) {
                  usleep(2e6);
                  return true;
              }
          }
      }

      usleep(1e6);
      return false;
    }

    void execute(const pose_goalGoalConstPtr& poseGoal)
    {
        /* make sure you are at home position */
        while (!home_position()) {
          ROS_INFO("CONTROLLER: failed to go home, retrying ...");
        }

        ROS_INFO("CONTROLLER: goal received (" << poseGoal->target_pose[0] << ", "
                                               << poseGoal->target_pose[1] << ", "
                                               << poseGoal->target_pose[2] << "), trying to execute");

        // gripper orientation
        double alpha = 3*M_PI/4;

        double left_right = 0.0;
        if (poseGoal->target_pose[0] > _wks_center_x) {
            left_right = +1.0;
        }
        else {
            left_right = -1.0;
        }

        double top_bottom = 0.0;
        if (poseGoal->target_pose[1] > _wks_center_y) {
            top_bottom = +1.0;
        }
        else {
            top_bottom = -1.0;
        }

        // push orientation
        double theta = _uniform(_re);

        _touch_object_success = true;

        /* first trajectory */
        geometry_msgs::Pose first_pose;
        bool first_pose_success = true;
        first_pose.position.x = poseGoal->target_pose[0] - left_right*cos(top_bottom*theta)*0.05;
        first_pose.position.y = poseGoal->target_pose[1] - left_right*sin(top_bottom*theta)*0.05;
        first_pose.position.z = poseGoal->target_pose[2];
        first_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, alpha, 0.0);
        ROS_INFO_STREAM("CONTROLLER_NODE : first trajectory" << first_pose.position.x << ", "
                                                             << first_pose.position.y << ", "
                                                             << first_pose.position.z << ")");

        // drive closer to pose
        first_pose_success &= _baxter_mover->group->setPositionTarget(
          first_pose.position.x,
          first_pose.position.y,
          first_pose.position.z + 0.5
        );
        first_pose_success &= _baxter_mover->group->plan(_group_plan);
        first_pose_success &= _baxter_mover->group->execute(_group_plan);

        // going to pose
        first_pose_success &= _baxter_mover->group->setPoseTarget(first_pose);
        first_pose_success &= _baxter_mover->group->plan(_group_plan);
        first_pose_success &= _baxter_mover->group->execute(_group_plan);

        usleep(1e6);


        /* second trajectory */
        geometry_msgs::Pose final_pose;
        bool final_pose_success = true;
        final_pose.position.x = poseGoal->target_pose[0] + left_right*cos(top_bottom*theta)*0.05;
        final_pose.position.y = poseGoal->target_pose[1] + left_right*sin(top_bottom*theta)*0.05;
        final_pose.position.z = poseGoal->target_pose[2];
        final_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, alpha, 0.0);
        ROS_INFO_STREAM("CONTROLLER_NODE : second trajectory" << final_pose.position.x << ", "
                                                              << final_pose.position.y << ", "
                                                              << final_pose.position.z << ")");

        moveit_msgs::RobotTrajectory pushing_trajectory;

        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(first_pose);
        waypoints.push_back(final_pose);

        // going to pose
        _baxter_mover->group->computeCartesianPath(waypoints, 0.025, 0.0, pushing_trajectory, false);
        moveit::planning_interface::MoveGroup::Plan touch_plan;
        touch_plan.trajectory_ = pushing_trajectory;
        final_pose_success &= _baxter_mover->group->execute(touch_plan);

        // drive away from pose
        final_pose_success &= _baxter_mover->group->setPositionTarget(
          final_pose.position.x,
          final_pose.position.y,
          final_pose.position.z + 0.5
        );
        final_pose_success &= _baxter_mover->group->plan(_group_plan);
        final_pose_success &= _baxter_mover->group->execute(_group_plan);


        /* return to home position */
        while (!home_position()) {
          ROS_INFO("CONTROLLER: failed to go home, retrying ...");
        }

        _touch_object_success = first_pose_success && final_pose_success;
        if(_touch_object_success){
            ROS_INFO_STREAM("CONTROLLER_NODE : push succeeded");
            _serv->setSucceeded();
        }
        else{
            if (!first_pose_success) {
                ROS_WARN_STREAM("CONTROLLER_NODE : push failed (first pose)");
            }
            else if (!final_pose_success) {
                ROS_WARN_STREAM("CONTROLLER_NODE : push failed (final pose)");
            }
            _serv->setAborted();
        }

    }

    void client_disconnect_from_ros(){}
    void update(){}

private:
    std::shared_ptr<actionlib::SimpleActionServer<pose_goalAction>> _serv;
    std::shared_ptr<ros::Publisher> _scene_publisher;
    moveit_msgs::PlanningScene _new_scene;
    std::unique_ptr<ros::ServiceClient> _move_baxter_arm;
    Eigen::VectorXd _pose_home;
    baxter_mover_utils::move_baxter_arm::Request _motion_request;
    baxter_mover_utils::move_baxter_arm::Response _motion_response;
    baxter_mover::BAXTER_Mover::Ptr _baxter_mover;
    object_babbling::pose_goalFeedback _joints_pose_feedback;
    std::map<std::string, double> _home_variable_values;
    moveit::planning_interface::MoveGroup::Plan _group_plan;
    std::vector<geometry_msgs::Pose> waypoints_;
    double fraction_;
    moveit_msgs::RobotTrajectory robot_trajectory_;
    std::shared_ptr<robot_state::RobotState> _robot_state;
    bool _touch_object_success = false;
    std::vector<double> arm_joint_values_, home_values_;

    std::uniform_real_distribution<double> _uniform;
    std::default_random_engine _re;

    double _wks_center_x = 0;
    double _wks_center_y = 0;
};

std::string parse_arg(int& argc, char **& argv, const std::string& default_val)
{
    std::string key;
    std::string value;
    std::string temp_str;
    std::string::size_type res;

    key = "__name:=";

    for (unsigned short i = 0; i < argc; ++i) {
        temp_str = argv[i];
        res = temp_str.find(key);

        if (res != std::string::npos) {
            value = temp_str.erase(res, key.length());
            break;
        }
        else if (i == argc - 1) {
            value = default_val;
        }
    }
    return value;
}

int main(int argc, char** argv){
    std::string node_name;
    XmlRpc::XmlRpcValue cafer;

    node_name = parse_arg(argc, argv, "controller_node");

    cafer_core::init(argc, argv, node_name);
    cafer_core::ros_nh->getParam("cafer", cafer);
    ROS_INFO_STREAM(cafer_core::ros_nh->getNamespace());

    Controller controller(cafer["mgmt"], cafer["type"], cafer["freq"],cafer["uuid"]);

    controller.wait_for_init();
    controller.spin();

    ROS_INFO_STREAM("CONTROLLER_NODE : Robot controller ready !");

    while (ros::ok() && (!controller.get_terminate())) {
        controller.spin();
    }
    return 0;
}
