#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <string>
#include <stdexcept>

#include <Eigen/Core>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>

#include <image_processing/pcl_types.h>
#include <image_processing/features.hpp>
#include <image_processing/BabblingDataset.h>
#include <image_processing/SupervoxelSet.h>
#include <image_processing/SurfaceOfInterest.h>
#include <image_processing/DescriptorExtraction.h>

#include <iagmm/data.hpp>

#include <cafer_core/cafer_core.hpp>

#include "globals.h"
#include "object_babbling/dataset.h"

using namespace Eigen;
using namespace rgbd_utils;
namespace ip = image_processing;

using namespace cafer_core;
using namespace object_babbling;


class ManualDataset : public Component {
    using Component::Component;

public:

    ~ManualDataset()
    {
        ROS_ERROR("MANUAL_DATASET_NODE : destroy?");
        client_disconnect_from_ros();
    }

    /**
     *@brief Connect to ros.
     */
    void client_connect_to_ros() override
    {
        std::stringstream display_params;
        XmlRpc::XmlRpcValue glob_params;
        XmlRpc::XmlRpcValue exp_params;

        cafer_core::ros_nh->getParam("/manual_dataset/params", glob_params);
        cafer_core::ros_nh->getParam("/manual_dataset/babbling/experiment", exp_params);

        for (auto& param:glob_params) {
            display_params << "\t" << param.first << ":\t" << param.second << std::endl;
        }

        ROS_INFO_STREAM("MANUAL_DATASET_NODE : global parameters retrieved:" << std::endl << display_params.str());

        _modality = static_cast<std::string>(exp_params["_modality"]);

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

        // Feedback
        _ptcl_pub.reset(
            new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>(
                glob_params["workspace_ptcl"], 10)
            )
        );

        _object_ptcl_pub.reset(
            new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>(
                glob_params["object_ptcl"], 10)
            )
        );
    }

    /**
     *@brief Init.
     */
    void init() override
    {
        client_connect_to_ros();

        _update_workspace();

        _clouds_ready = false;
        _background_saved = true;
        _finished = false;

        _save_path = "/home/jonathan/manual_dataset";

        usleep(1e6);
    }

    /**
     *@brief Disconnect from ros.
     */
    void client_disconnect_from_ros() override
    {
        _images_sub.reset();

        _ptcl_pub.reset();
        _object_ptcl_pub.reset();
    }

    /**
     *@brief Update.
     */
    void update() override
    {
        // Image
        _depth_msg = sensor_msgs::ImageConstPtr(new sensor_msgs::Image(_images_sub->get_depth()));
        _rgb_msg = sensor_msgs::ImageConstPtr(new sensor_msgs::Image(_images_sub->get_rgb()));
        _info_msg = sensor_msgs::CameraInfoConstPtr(new sensor_msgs::CameraInfo(_images_sub->get_rgb_info()));

        if (_rgb_msg->data.empty() || _depth_msg->data.empty()) {
            return;
        }

        if (!_background_saved) {
            ROS_ERROR("MANUAL_DATASET_NODE : saving background");
            rgbd_utils::RGBD_to_Pointcloud converter(_depth_msg, _rgb_msg, _info_msg);
            sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();

            _background_ptcl.reset(new ip::PointCloudT);
            pcl::fromROSMsg(ptcl_msg, *_background_ptcl);

            _background_saved = true;

            _dataset.clear();

            save_background();

            return;
        }

        if (_background_saved && !_finished) {
            ROS_ERROR("MANUAL_DATASET_NODE : press enter to take a picture");
            std::cin.ignore();

            ROS_ERROR("MANUAL_DATASET_NODE : saving background");
            rgbd_utils::RGBD_to_Pointcloud converter(_depth_msg, _rgb_msg, _info_msg);
            sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();

            _workspace_ptcl = ip::PointCloudT::Ptr(new ip::PointCloudT);
            pcl::fromROSMsg(ptcl_msg, *_workspace_ptcl);

            _workspace->filter(_workspace_ptcl);

            ip::SurfaceOfInterest surface;
            surface.clear();
            surface.setInputCloud(_workspace_ptcl);
            surface.delete_background(_background_ptcl);

            _object_ptcl = surface.getInputCloud();

            if(!surface.computeSupervoxel()) {
                ROS_ERROR_STREAM("MANUAL_DATASET_NODE : error computing supervoxels");
            }

            surface.compute_feature(_modality);

            // Extract features
            ip::SupervoxelArray svs = surface.getSupervoxels();
            for (const auto& sv : svs)
            {
                Eigen::VectorXd features = surface.get_feature(sv.first, _modality);
                _dataset.add(1, features);
            }

            _clouds_ready = true;
            save_iteration();
        }

        publish_feedback();
    }

    void save_background()
    {
        std::string file_name = _save_path + "/background.pcd";
        ROS_ERROR_STREAM("MANUAL_DATASET_NODE : saving background to " << file_name);
        pcl::io::savePCDFile(file_name, *_background_ptcl);
    }

    void save_iteration()
    {
        std::string wks_file_name = _save_path + "/iteration_" + std::to_string(_nb_iter) + "/workspace.pcd";
        ROS_ERROR_STREAM("MANUAL_DATASET_NODE : saving workspace to " << wks_file_name);
        pcl::io::savePCDFile(wks_file_name, *_workspace_ptcl);

        std::string obj_file_name = _save_path + "/iteration_" + std::to_string(_nb_iter) + "/object.pcd";
        ROS_ERROR_STREAM("MANUAL_DATASET_NODE : saving object to " << obj_file_name);
        pcl::io::savePCDFile(obj_file_name, *_object_ptcl);

        _nb_iter += 1;
    }

    void save_dataset(iagmm::TrainingData tr_data) {
        // dataset to ros message
        dataset dataset_msg;
        sv_feature temp_feature;

        dataset_msg.type.data = _modality;

        iagmm::TrainingData::data_t dataset = tr_data.get();
        std::vector<double> data_vct(dataset[0].second.rows());
        for (const auto& data : dataset) {

            for(int i = 0; i < data.second.rows(); i++)
                data_vct[i] = data.second(i);
            temp_feature.feature = data_vct;
            temp_feature.label = data.first;

            dataset_msg.features.push_back(temp_feature);
        }

        // ros message to file
        std::string frame_id, feat_id;
        YAML::Emitter soi_yml;

        frame_id = "frame_" + std::to_string(dataset_msg.header.stamp.sec + dataset_msg.header.stamp.nsec);

        soi_yml << YAML::BeginMap //BEGIN MAP_0
                    << YAML::Key << frame_id << YAML::Value
                    << YAML::BeginMap //BEGIN MAP_1
                        << YAML::Key << "timestamp" << YAML::Value
                        << YAML::BeginMap //BEGIN MAP_2
                            << YAML::Key << "sec" << YAML::Value << dataset_msg.header.stamp.sec
                            << YAML::Key << "nsec" << YAML::Value << dataset_msg.header.stamp.nsec
                        << YAML::EndMap //END MAP_2
                        << YAML::Key << "features" << YAML::Value
                        << YAML::BeginMap; //BEGIN MAP_3

        for (unsigned int i = 0; i < dataset_msg.features.size(); ++i) {
            feat_id = "feature_" + std::to_string(i);

            soi_yml << YAML::Key << feat_id << YAML::Value
                    << YAML::BeginMap //BEGIN MAP_4
                        << YAML::Key << "label" << YAML::Value << (dataset_msg.features[i].label /*!= '1'*/)
                        << YAML::Key << "value" << YAML::Value
                        << YAML::BeginSeq;
            for (unsigned int j = 0; j < dataset_msg.features[i].feature.size(); ++j) {
                soi_yml  << dataset_msg.features[i].feature[j];
            }
            soi_yml << YAML::EndSeq
                    << YAML::EndMap; //END MAP_4
        }
        soi_yml << YAML::EndMap //END MAP_3
                << YAML::EndMap //END MAP_1
                << YAML::EndMap //END MAP_0
                << YAML::Newline;

        //write
        soi_yml.c_str();
    }

    /**
     *@brief Publish pointclouds feedback at each update() call.
     */
    void publish_feedback()
    {
        // feedback clouds
        if (_clouds_ready) {
            sensor_msgs::PointCloud2 workspace_ptcl_msg;
            pcl::toROSMsg(*_workspace_ptcl, workspace_ptcl_msg);
            workspace_ptcl_msg.header = _images_sub->get_depth().header;
            _ptcl_pub->publish(workspace_ptcl_msg);

            sensor_msgs::PointCloud2 object_ptcl_msg;
            pcl::toROSMsg(*_object_ptcl, object_ptcl_msg);
            object_ptcl_msg.header = _images_sub->get_depth().header;
            _object_ptcl_pub->publish(object_ptcl_msg);
        }
    }

private:

    RGBD_Subscriber::Ptr _images_sub;
    sensor_msgs::ImageConstPtr _depth_msg;
    sensor_msgs::ImageConstPtr _rgb_msg;
    sensor_msgs::CameraInfoConstPtr _info_msg;

    std::unique_ptr<Publisher> _ptcl_pub;
    std::unique_ptr<Publisher> _object_ptcl_pub;

    std::unique_ptr<ip::workspace_t> _workspace;

    // Background
    ip::PointCloudT::Ptr _background_ptcl;

    // Dataset
    iagmm::TrainingData _dataset;
    std::string _modality;

    // Feedback clouds
    ip::PointCloudT::Ptr _workspace_ptcl;
    ip::PointCloudT::Ptr _object_ptcl;

    bool _clouds_ready;
    bool _background_saved;
    bool _finished;

    int _counter_iter;
    int _nb_iter;

    std::string _save_path;

    /**
     *@brief Update the workspace.
     */
    void _update_workspace()
    {
        XmlRpc::XmlRpcValue wks;

        cafer_core::ros_nh->getParamCached("/manual_dataset/babbling/experiment/workspace", wks);

        _workspace.reset(
                new ip::workspace_t(
                  true,
                  static_cast<double>(wks["sphere"]["x"]),
                  static_cast<double>(wks["sphere"]["y"]),
                  static_cast<double>(wks["sphere"]["z"]),
                  static_cast<double>(wks["sphere"]["radius"]),
                  static_cast<double>(wks["sphere"]["threshold"]),
                 {static_cast<double>(wks["csg_intersect_cuboid"]["x_min"]),
                  static_cast<double>(wks["csg_intersect_cuboid"]["x_max"]),
                  static_cast<double>(wks["csg_intersect_cuboid"]["y_min"]),
                  static_cast<double>(wks["csg_intersect_cuboid"]["y_max"]),
                  static_cast<double>(wks["csg_intersect_cuboid"]["z_min"]),
                  static_cast<double>(wks["csg_intersect_cuboid"]["z_max"])}
                )
        );
    }

};

int main(int argc, char** argv)
{
    std::string node_name;
    XmlRpc::XmlRpcValue cafer;

    node_name = global::parse_arg(argc, argv, "MANUAL_DATASET_NODE");

    cafer_core::init(argc, argv, node_name);
    cafer_core::ros_nh->getParam("cafer", cafer);

    ManualDataset manual_dataset(cafer["mgmt"], cafer["type"], cafer["freq"], cafer["uuid"]);

    manual_dataset.wait_for_init();
    manual_dataset.spin();

    ROS_INFO_STREAM("MANUAL_DATASET_NODE : Manual Dataset Builder ready !");

    while (ros::ok() && !manual_dataset.get_terminate()) {
        manual_dataset.spin();
        manual_dataset.update();
        manual_dataset.sleep();
    }

    ROS_INFO_STREAM("MANUAL_DATASET_NODE : ros::ok = " << ros::ok()
                    << " terminate = " << manual_dataset.get_terminate());

    ROS_INFO_STREAM("MANUAL_DATASET_NODE : is finish");

    return 0;
}
