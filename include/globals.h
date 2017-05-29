//
// Created by phlf on 06/07/16.
//

#ifndef DREAM_BABBLING_BABBLING_CONSTANTS_HPP
#define DREAM_BABBLING_BABBLING_CONSTANTS_HPP

#include <Eigen/Core>
#include <math.h>
#include <string>

#include <tf/transform_listener.h>

namespace global {
    std::string parse_arg(int& argc, char**& argv, const std::string& default_val)
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
}//global namespace

namespace babbling {
    /**
    * @brief Conversion from the camera frame to the robot frame
    * @param out_pose
    * @param in_pose
    * @param translation along x
    * @param translation along y
    * @param translation along z
    * @param rotation around x (yaw)
    * @param rotation around y (pitch)
    * @param rotation around z (roll)
    */
    void base_conversion(Eigen::Vector4d& out_pose, const Eigen::Vector4d& in_pose)
    {
        Eigen::Matrix4d Trans_M;

        // front table camera
        Trans_M <<  0.107771,  0.816759,  -0.77785,  0.899714,
             1.05818,  0.029358,  0.166672, -0.203503,
           0.0456001, -0.769741,  -0.67249,  0.814146,
            0,                0,        0,        1;

        out_pose = Trans_M * in_pose;
    }


    //Convert object position from camera frame to robot frame
    void tf_base_conversion(Eigen::Vector3d& object_pose_in_robot_frame, Eigen::Vector3d& object_pose_in_camera_frame){
        tf::TransformListener listener;
        tf::StampedTransform stamped_transform;
        //std::string child_frame = "/camera_depth_optical_frame";
        std::string child_frame = "/camera_rgb_optical_frame";
        std::string parent_frame = "base";
        try{
            listener.lookupTransform(child_frame, parent_frame,
                                     ros::Time::now(), stamped_transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        geometry_msgs::PointStamped camera_point;
        geometry_msgs::PointStamped base_point;
        camera_point.header.frame_id = child_frame;

        //we'll just use the most recent transform available for our simple example
        camera_point.header.stamp = ros::Time();

        //just an arbitrary point in space
        camera_point.point.x = object_pose_in_camera_frame(0);
        camera_point.point.y = object_pose_in_camera_frame(1);
        camera_point.point.z = object_pose_in_camera_frame(2);

        try{

            listener.transformPoint(parent_frame, camera_point, base_point);

            ROS_INFO("camera_depth_optical_frame: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                     camera_point.point.x, camera_point.point.y, camera_point.point.z,
                     base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
        }
        catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
        }
        object_pose_in_robot_frame << base_point.point.x,
                base_point.point.y,
                base_point.point.z;
    }

}//babbling namespace

#endif // DREAM_BABBLING_BABBLING_CONSTANTS_HPP
