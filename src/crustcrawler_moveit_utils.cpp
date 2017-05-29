//
// Created by phlf on 24/03/16.
//
#include <iostream>
#include <string>
#include <boost/timer.hpp>
#include <cafer_core/cafer_core.hpp>

#include <dream_babbling/pose_goalAction.h>
#include <actionlib/server/simple_action_server.h>

#include <ros/callback_queue.h>
#include <fstream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <boost/timer.hpp>

#include <moveit/robot_state/conversions.h>

#include <crustcrawler_mover_utils/crustcrawler_mover.hpp>
#include <crustcrawler_mover_utils/move_crustcrawler_arm.h>
#include <crustcrawler_core_msgs/EndEffectorCommand.h>

using namespace dream_babbling;
using namespace cafer_core;

class Controller: public Component{
    using Component::Component; // C++11 requirement to inherit the constructor

public :

    void client_connect_to_ros(){
        XmlRpc::XmlRpcValue glob_params;
        std::stringstream display_params;

        cafer_core::ros_nh->getParam("/dream_babbling/params", glob_params);
        for (auto& param:glob_params) {
            display_params << "\t" << param.first << ":\t" << param.second << std::endl;
        }

        ROS_INFO_STREAM("CONTROLLER : Global parameters retrieved:" << std::endl << display_params.str());

        _serv.reset(new actionlib::SimpleActionServer<pose_goalAction>(*cafer_core::ros_nh, glob_params["controller_server"],
                    boost::bind(&Controller::execute, this, _1),false));

        _crustcrawler_mover.reset(new crustcrawler_mover::CRUSTCRAWLER_Mover(*cafer_core::ros_nh));
        _move_crustcrawler_arm.reset(new ros::ServiceClient(cafer_core::ros_nh->serviceClient<crustcrawler_mover_utils::move_crustcrawler_arm>("/move_crustcrawler_arm")));
        _gripper_command_publisher.reset(new ros::Publisher(cafer_core::ros_nh->advertise<crustcrawler_core_msgs::EndEffectorCommand>
                                                            ("/crustcrawler/end_effector/gripper/command", 1, this)));
    }

    void init(){
        connect_to_ros();

        usleep(2e6);

        _serv->start();
        _is_init = true;

        _home_variable_values.insert ( std::pair<std::string, double>("joint_1", -1.3) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_2",  0.3) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_3", -1.1) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_4",  0.0) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_5", -0.5) );
        _home_variable_values.insert ( std::pair<std::string, double>("joint_6",  0.0) );

        home_values_ = {-1.3, 0.3, -1.1, 0.0, -0.5, 0.0};

        ros::AsyncSpinner my_spinner(4);
        my_spinner.start();

        _crustcrawler_mover->group->setPlannerId(static_cast<std::string>(_crustcrawler_mover->global_parameters.get_planner_parameters()["planner_id"]));
        _crustcrawler_mover->group->setPlanningTime(std::stod(_crustcrawler_mover->global_parameters.get_planner_parameters()["planning_time"]));
        /*
    _motion_request.type = "go_home";
    _motion_request.goal = {0.05, -0.2, 0.54};
    _move_crustcrawler_arm->call(_motion_request, _motion_response);
    */
        _crustcrawler_mover->group->setJointValueTarget(_home_variable_values);
        if(_crustcrawler_mover->group->plan(_group_plan))
            _crustcrawler_mover->group->execute(_group_plan);


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

    void extract_arm_joints_values(){
        std::vector<std::string> joint_names;
        std::vector<double> joint_values;
        //std::vector<int> joint_index;
        joint_names = _crustcrawler_mover->global_parameters.get_crustcrawler_arm_joints_names();

        for(unsigned i = 0; i < joint_names.size(); ++i){
            joint_values.push_back(_crustcrawler_mover->global_parameters.get_joint_state().position[distance
                                   (_crustcrawler_mover->global_parameters.get_joint_state().name.begin(),
                                    find(_crustcrawler_mover->global_parameters.get_joint_state().name.begin(),
                                         _crustcrawler_mover->global_parameters.get_joint_state().name.end(),
                                         joint_names[i]))]);
        }
        arm_joint_values_ = joint_values;
    }

    void execute(const pose_goalGoalConstPtr& poseGoal)
    {
        /*make sure you are at home position*/
        extract_arm_joints_values();
        if(largest_difference(arm_joint_values_,
                              home_values_) > 0.2){
            _crustcrawler_mover->group->setJointValueTarget(_home_variable_values);
            if(_crustcrawler_mover->group->plan(_group_plan))
                _crustcrawler_mover->group->execute(_group_plan);
        }

        /* close the gripper and move to point with 15cm height above the goal*/
        _gripper_command.args = "{position: 0.0}";
        _gripper_command.command = "go";
        _gripper_command_publisher->publish(_gripper_command);

        _crustcrawler_mover->group->setPositionTarget(poseGoal->target_pose[0], poseGoal->target_pose[1], poseGoal->target_pose[2] + 0.15);
        if(_crustcrawler_mover->group->plan(_group_plan))
            _crustcrawler_mover->group->execute(_group_plan);

        /*Then touch the object*/
        _crustcrawler_mover->group->setPositionTarget(poseGoal->target_pose[0], poseGoal->target_pose[1], poseGoal->target_pose[2]);
        if(_crustcrawler_mover->group->plan(_group_plan)){
            ROS_INFO_STREAM("CONTROLLER : There is a valid plan lets move");
            if(_crustcrawler_mover->group->execute(_group_plan)){
                if((fabs(_crustcrawler_mover->global_parameters.get_eef_pose().position.x - poseGoal->target_pose[0]) < 0.013) &&
                        (fabs(_crustcrawler_mover->global_parameters.get_eef_pose().position.y - poseGoal->target_pose[1]) < 0.013) &&
                        (fabs(_crustcrawler_mover->global_parameters.get_eef_pose().position.z - poseGoal->target_pose[2]) < 0.013))
                    _touch_object_success = true;
                else{
                    ROS_WARN_STREAM("CONTROLLER : Difference in x coordinate is: "
                                    << fabs(_crustcrawler_mover->global_parameters.get_eef_pose().position.x - poseGoal->target_pose[0])
                            << " in y is: "
                            << fabs(_crustcrawler_mover->global_parameters.get_eef_pose().position.y - poseGoal->target_pose[1])
                            << " and in z is: " << fabs(_crustcrawler_mover->global_parameters.get_eef_pose().position.z - poseGoal->target_pose[2]));
                    _touch_object_success = false;
                }
            }

        }
        else{
            ROS_WARN_STREAM("CONTROLLER : Failed to find any plans :( :( :( ");
            _touch_object_success = false;
        }

        _crustcrawler_mover->group->setJointValueTarget(_home_variable_values);
        if(_crustcrawler_mover->group->plan(_group_plan))
            _crustcrawler_mover->group->execute(_group_plan);

        if(_touch_object_success){
            ROS_INFO_STREAM("CONTROLLER : The motion was successful !!");
            _serv->setSucceeded();
        }
        else{
            ROS_WARN_STREAM("CONTROLLER : Something went wrong");
            _serv->setAborted();
        }

    }



    void client_disconnect_from_ros(){}
    void update(){}

private:
    std::shared_ptr<actionlib::SimpleActionServer<pose_goalAction>> _serv;
    std::shared_ptr<ros::Publisher> _scene_publisher, _gripper_command_publisher;
    moveit_msgs::PlanningScene _new_scene;
    std::unique_ptr<ros::ServiceClient> _move_crustcrawler_arm;
    Eigen::VectorXd _pose_home;
    crustcrawler_mover_utils::move_crustcrawler_arm::Request _motion_request;
    crustcrawler_mover_utils::move_crustcrawler_arm::Response _motion_response;
    crustcrawler_mover::CRUSTCRAWLER_Mover::Ptr _crustcrawler_mover;
    dream_babbling::pose_goalFeedback _joints_pose_feedback;
    std::map<std::string, double> _home_variable_values;
    moveit::planning_interface::MoveGroup::Plan _group_plan;
    std::vector<geometry_msgs::Pose> waypoints_;
    double fraction_;
    moveit_msgs::RobotTrajectory robot_trajectory_;
    std::shared_ptr<robot_state::RobotState> _robot_state;
    crustcrawler_core_msgs::EndEffectorCommand _gripper_command;
    bool _touch_object_success = false;
    std::vector<double> arm_joint_values_, home_values_;
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

    ROS_INFO_STREAM("CONTROLLER : Robot controller ready !");

    while (ros::ok() && (!controller.get_terminate())) {
        controller.spin();
    }
    return 0;
}
