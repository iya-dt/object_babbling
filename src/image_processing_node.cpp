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
#include <yaml-cpp/yaml.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>

#include <iagmm/gmm.hpp>
#include <iagmm/nnmap.hpp>

#include <image_processing/pcl_types.h>
#include <image_processing/BabblingDataset.h>
#include <image_processing/SurfaceOfInterest.h>
#include <image_processing/DescriptorExtraction.h>
#include <image_processing/Objects.h>

#include <cafer_core/cafer_core.hpp>

#include "object_babbling/get_targets.h"
#include "object_babbling/set_target.h"
#include "object_babbling/track.h"
#include "object_babbling/rgbd_motion_data.h"

#include "globals.h"


using namespace Eigen;
namespace ip = image_processing;
using namespace pcl::tracking;

using namespace cafer_core;
using namespace object_babbling;


class ImageProcessing : public Component {
    using Component::Component;

public:

    ~ImageProcessing()
    {
        ROS_ERROR("IMAGE_PROCESSING_NODE : Destroy?");
        client_disconnect_from_ros();
    }

    bool get_targets(get_targets::Request &req, get_targets::Response &res)
    {
        ROS_INFO_STREAM("IMAGE_PROCESSING_NODE : get_targets");

        // Image
        sensor_msgs::ImageConstPtr depth_msg(new sensor_msgs::Image(_images_sub->get_depth()));
        sensor_msgs::ImageConstPtr rgb_msg(new sensor_msgs::Image(_images_sub->get_rgb()));
        sensor_msgs::CameraInfoConstPtr info_msg(new sensor_msgs::CameraInfo(_images_sub->get_rgb_info()));

        if (rgb_msg->data.empty() || depth_msg->data.empty()) {
            ROS_ERROR_STREAM("IMAGE_PROCESSING_NODE : empty cloud");
            return false;
        }

        rgbd_utils::RGBD_to_Pointcloud converter(depth_msg, rgb_msg, info_msg);

        sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();

        _workspace_ptcl = ip::PointCloudT::Ptr(new ip::PointCloudT);
        pcl::fromROSMsg(ptcl_msg, *_workspace_ptcl);
        if (_workspace_ptcl->empty()) {
            ROS_ERROR_STREAM("IMAGE_PROCESSING_NODE : empty cloud message");
            return false;
        }

        _workspace->filter(_workspace_ptcl);
        if (_workspace_ptcl->empty()) {
            ROS_ERROR_STREAM("IMAGE_PROCESSING_NODE : empty filtered cloud");
            return false;
        }


        // Saliency map
        std::string modality = "normal";
        ip::SurfaceOfInterest soi;
        soi.clear();
        soi.setInputCloud(_workspace_ptcl);
        ROS_INFO_STREAM("IMAGE_PROCESSING_NODE : compute supervoxels");
        if(!soi.computeSupervoxel()) {
            ROS_ERROR_STREAM("IMAGE_PROCESSING_NODE : error computing supervoxels");
            return false;
        }
        ROS_INFO_STREAM("IMAGE_PROCESSING_NODE : compute saliency");
        soi.init_weights(modality, .5);
        soi.compute_weights<iagmm::NNMap>(modality, _classifier);
        _saliency_ptcl = soi.getColoredWeightedCloud(modality).makeShared();

        if (_saliency_ptcl->empty()) {
            ROS_ERROR_STREAM("IMAGE_PROCESSING_NODE : empty saliency cloud");
            return false;
        }


        // Object extraction
        ROS_INFO_STREAM("IMAGE_PROCESSING_NODE : extract objects");
        double saliency_threshold = 0.2;
        int points_threshold = 30;
        _targets = soi.get_objects(modality, saliency_threshold, points_threshold);

        _targets_ptcl = ip::PointCloudT::Ptr(new ip::PointCloudT);
        for (auto it = _targets.begin(); it != _targets.end(); it++) {
            int rd = rand();
            for (auto point = it->object_cloud.begin(); point != it->object_cloud.end(); point++) {
                ip::PointT new_point = *point;
                new_point.r = (rd*2 + 23) % 255;
                new_point.g = (rd*7 + 19) % 255;
                new_point.b = (rd*37 + 5) % 255;
                _targets_ptcl->push_back(new_point);
            }
        }
        if (_targets_ptcl->empty()) {
            ROS_ERROR_STREAM("IMAGE_PROCESSING_NODE : empty targets cloud");
            return false;
        }

        // DEBUG
        std::string filename = "targets_cloud.pcd";
        pcl::io::savePCDFileASCII(filename, *_targets_ptcl);

        _clouds_ready = true;

        ROS_INFO_STREAM("IMAGE_PROCESSING_NODE : sending " << _targets.size() << " objects");
        std::stringstream sstream;
        boost::archive::text_oarchive oTextArchive(sstream);
        oTextArchive << _targets;
        res.targets = sstream.str();

        return true;
    }

    bool set_target(set_target::Request &req, set_target::Response &res)
    {
        ROS_INFO_STREAM("IMAGE_PROCESSING_NODE : set_target = " << req.target_id);

        _target_id = req.target_id;

        return true;
    }

    bool track(track::Request &req, track::Response &res)
    {
        if (req.tracking == _tracking) {
            std::string boolean = _tracking ? "true" : "false";
            ROS_ERROR_STREAM("IMAGE_PROCESSING_NODE : already received \"" << boolean << "\"");
            return false;
        }
        else {
            _tracking = req.tracking;
            if (_tracking) {
                ROS_INFO_STREAM("IMAGE_PROCESSING_NODE : starting tracking");
                XmlRpc::XmlRpcValue glob_params;
                cafer_core::ros_nh->getParam("/object_babbling/params", glob_params);

                _tracking_sub.reset(
                    new Subscriber(
                        ros_nh->subscribe<rgbd_motion_data>(
                            glob_params["motion_detector_topic"], 10,
                            boost::bind(&ImageProcessing::image_processing_callback, this, _1))
                    )
                );

                // Tracker creation
                std::vector<double> default_step_covariance = std::vector<double>(6, 0.015 * 0.015);
                default_step_covariance[3] *= 40.0;
                default_step_covariance[4] *= 40.0;
                default_step_covariance[5] *= 40.0;

                std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.00001);
                std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);

                _tracker.reset(new KLDAdaptiveParticleFilterOMPTracker<ip::PointT, ParticleXYZRPY>(8));

                ParticleXYZRPY bin_size;
                bin_size.x = 0.1f;
                bin_size.y = 0.1f;
                bin_size.z = 0.1f;
                bin_size.roll = 0.1f;
                bin_size.pitch = 0.1f;
                bin_size.yaw = 0.1f;

                _tracker->setMaximumParticleNum(1000);
                _tracker->setDelta(0.99);
                _tracker->setEpsilon(0.2);
                _tracker->setBinSize(bin_size);
                _tracker->setTrans(Eigen::Affine3f::Identity());
                _tracker->setStepNoiseCovariance(default_step_covariance);
                _tracker->setInitialNoiseCovariance(initial_noise_covariance);
                _tracker->setInitialNoiseMean(default_initial_mean);
                _tracker->setIterationNum(1);
                _tracker->setParticleNum(600);
                _tracker->setResampleLikelihoodThr(0.00);
                _tracker->setUseNormal(false);


                // Coherence for tracking
                ApproxNearestPairPointCloudCoherence<ip::PointT>::Ptr coherence = ApproxNearestPairPointCloudCoherence<ip::PointT>::Ptr
                  (new ApproxNearestPairPointCloudCoherence<ip::PointT>());

                boost::shared_ptr<DistanceCoherence<ip::PointT> > distance_coherence
                  = boost::shared_ptr<DistanceCoherence<ip::PointT> >(new DistanceCoherence<ip::PointT>());
                coherence->addPointCoherence(distance_coherence);

                boost::shared_ptr<pcl::search::Octree<ip::PointT> > search (new pcl::search::Octree<ip::PointT>(0.01));
                coherence->setSearchMethod(search);
                coherence->setMaximumDistance(0.01);

                _tracker->setCloudCoherence(coherence);

                // Preparing target cloud
                Eigen::Vector4f c;
                Eigen::Affine3f trans = Eigen::Affine3f::Identity();
                ip::PointCloudT::Ptr transed_ref(new ip::PointCloudT);
                ip::PointCloudT::Ptr transed_ref_downsampled(new ip::PointCloudT);

                pcl::compute3DCentroid<ip::PointT>(_targets[_target_id].object_cloud, c);
                trans.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);
                pcl::transformPointCloud<ip::PointT>(_targets[_target_id].object_cloud, *transed_ref, trans.inverse());

                _gridSampleApprox(transed_ref, *transed_ref_downsampled, _downsampling_grid_size);

                // Set reference model and trans
                _tracker->setReferenceCloud(transed_ref_downsampled);
                _tracker->setTrans(trans);

            }
            else {
                ROS_INFO_STREAM("IMAGE_PROCESSING_NODE : stoping tracking");
                _tracking_sub.reset();
                _tracker.reset();
            }
        }

        return true;
    }

    void image_processing_callback(const rgbd_motion_data::ConstPtr&  msg)
    {
        ROS_INFO_STREAM("IMAGE_PROCESSING_NODE : image processing callback");
        sensor_msgs::ImageConstPtr depth_msg(new sensor_msgs::Image(msg->depth));
        sensor_msgs::ImageConstPtr rgb_msg(new sensor_msgs::Image(msg->rgb));
        sensor_msgs::CameraInfoConstPtr info_msg(new sensor_msgs::CameraInfo(msg->rgb_info));

        if (rgb_msg->data.empty() || depth_msg->data.empty()) {
            ROS_ERROR_STREAM("IMAGE_PROCESSING_NODE : tracking empty cloud");
            return;
        }

        rgbd_utils::RGBD_to_Pointcloud converter(depth_msg, rgb_msg, info_msg);

        sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();
        ip::PointCloudT::Ptr cloud = ip::PointCloudT::Ptr(new ip::PointCloudT);
        ip::PointCloudT::Ptr cloud_downsampled = ip::PointCloudT::Ptr(new ip::PointCloudT);
        pcl::fromROSMsg(ptcl_msg, *cloud);
        _gridSampleApprox(cloud, *cloud_downsampled, _downsampling_grid_size);

        _tracker->setInputCloud(cloud_downsampled);
      	_tracker->compute();

        ParticleXYZRPY result = _tracker->getResult();
        Eigen::Affine3f transformation = _tracker->toEigenMatrix(result);

        _tracked_object_ptcl = ip::PointCloudT::Ptr(new ip::PointCloudT);
        pcl::transformPointCloud<ip::PointT>(*(_tracker->getReferenceCloud()), *_tracked_object_ptcl, transformation);

        pcl::compute3DCentroid(*_tracked_object_ptcl, _tracked_point);
    }

    void client_connect_to_ros() override
    {
        std::stringstream display_params;
        XmlRpc::XmlRpcValue glob_params;
        XmlRpc::XmlRpcValue exp_params;

        cafer_core::ros_nh->getParam("/object_babbling/params", glob_params);
        cafer_core::ros_nh->getParam("/object_babbling/babbling/experiment", exp_params);

        _classifier_path = static_cast<std::string>(exp_params["classifier_path"]);

        // Images
        _images_sub.reset(new rgbd_utils::RGBD_Subscriber(glob_params["rgb_info_topic"],
                                                          glob_params["rgb_topic"],
                                                          glob_params["depth_info_topic"],
                                                          glob_params["depth_topic"],
                                                          *cafer_core::ros_nh));

        // Tracked object
        _tracked_point_pub.reset(
            new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>(glob_params["tracked_point_topic"], 5))
        );
        _tracked_object_pub.reset(
            new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>(glob_params["tracked_object_topic"], 5))
        );

        // Services
        _get_targets_srv.reset(
            new ros::ServiceServer(
                ros_nh->advertiseService<get_targets::Request, get_targets::Response>(
                    glob_params["get_targets_service"],
                    boost::bind(&ImageProcessing::get_targets, this, _1, _2))
            )
        );
        _set_target_srv.reset(
            new ros::ServiceServer(
                ros_nh->advertiseService<set_target::Request, set_target::Response>(
                    glob_params["set_target_service"],
                    boost::bind(&ImageProcessing::set_target, this, _1, _2))
            )
        );
        _track_srv.reset(
            new ros::ServiceServer(
                ros_nh->advertiseService<track::Request, track::Response>(
                    glob_params["track_service"],
                    boost::bind(&ImageProcessing::track, this, _1, _2))
            )
        );

        _ptcl_pub.reset(
            new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>(
                glob_params["workspace_ptcl"], 10)
            )
        );
        _saliency_ptcl_pub.reset(
            new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>(
                glob_params["saliency_ptcl"], 10)
            )
        );
        _targets_ptcl_pub.reset(
            new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>(
                glob_params["objects_ptcl"], 10)
            )
        );

    }

    void init() override
    {
        _update_workspace();

        client_connect_to_ros();

        _dataset = load_dataset(_classifier_path);
        _classifier = iagmm::NNMap(3, 2, 0.3, 0.05);
        _classifier.set_samples(_dataset);

        _clouds_ready = false;
        _tracking = false;
        _ending_tracking = false;
        _tracked_point << 0.0, 0.0, 0.0, 0.0;
    }

    void client_disconnect_from_ros() override
    {
        cafer_core::DBManager db_request;
        std::string supervisor_name =
            ros::names::parentNamespace(cafer_core::ros_nh->getNamespace()) + "/babbling";

        _images_sub.reset();
        _tracked_point_pub.reset();
        _tracked_object_pub.reset();

        _get_targets_srv.reset();
        _set_target_srv.reset();
        _track_srv.reset();

        _ptcl_pub.reset();
        _saliency_ptcl_pub.reset();
        _targets_ptcl_pub.reset();
    }

    void update() override
    {
        publish_feedback();
    }

    iagmm::TrainingData load_dataset(const std::string& filename)
    {
        iagmm::TrainingData dataset;

        YAML::Node fileNode = YAML::LoadFile(filename);
        if (fileNode.IsNull()) {
            ROS_ERROR_STREAM("IMAGE_PROCESSING_NODE : could not load dataset at " << filename);
            return dataset;
        }

        YAML::Node features = fileNode["frame_0"]["features"];

        for (unsigned int i = 0; i < features.size(); ++i) {
            std::stringstream stream;
            stream << "feature_" << i;
            YAML::Node tmp_node = features[stream.str()];

            Eigen::VectorXd feature(tmp_node["value"].size());
            for(size_t i = 0; i < tmp_node["value"].size(); ++i)
                feature(i) = tmp_node["value"][i].as<double>();

            dataset.add(tmp_node["label"].as<int>(),feature);
        }
        return dataset;
    }

    void publish_feedback() {
        if (_clouds_ready) {
            sensor_msgs::PointCloud2 workspace_ptcl_msg;
            pcl::toROSMsg(*_workspace_ptcl, workspace_ptcl_msg);
            workspace_ptcl_msg.header = _images_sub->get_depth().header;
            _ptcl_pub->publish(workspace_ptcl_msg);

            sensor_msgs::PointCloud2 saliency_ptcl_msg;
            pcl::toROSMsg(*_saliency_ptcl, saliency_ptcl_msg);
            saliency_ptcl_msg.header = _images_sub->get_depth().header;
            _saliency_ptcl_pub->publish(saliency_ptcl_msg);

            sensor_msgs::PointCloud2 targets_ptcl_msg;
            pcl::toROSMsg(*_targets_ptcl, targets_ptcl_msg);
            targets_ptcl_msg.header = _images_sub->get_depth().header;
            _targets_ptcl_pub->publish(targets_ptcl_msg);
        }

        if (!(_tracked_point[0] == 0 || _tracked_point[1] == 0 || _tracked_point[2] == 0)) {
            ip::PointCloudXYZ point_tracked;
            point_tracked.push_back(pcl::PointXYZ(_tracked_point[0], _tracked_point[1], _tracked_point[2]));

            sensor_msgs::PointCloud2 point_tracked_msg;
            pcl::toROSMsg(point_tracked, point_tracked_msg);
            point_tracked_msg.header = _images_sub->get_depth().header;
            _tracked_point_pub->publish(point_tracked_msg);

            sensor_msgs::PointCloud2 tracked_object_msg;
            pcl::toROSMsg(*_tracked_object_ptcl, tracked_object_msg);
            tracked_object_msg.header = _images_sub->get_depth().header;
            _tracked_object_pub->publish(tracked_object_msg);
        }
    }

private:

    rgbd_utils::RGBD_Subscriber::Ptr _images_sub;
    std::unique_ptr<Publisher> _tracked_point_pub;
    std::unique_ptr<Publisher> _tracked_object_pub;

    std::unique_ptr<ros::ServiceServer> _get_targets_srv;
    std::unique_ptr<ros::ServiceServer> _set_target_srv;
    std::unique_ptr<ros::ServiceServer> _track_srv;

    std::unique_ptr<Subscriber> _tracking_sub;

    std::unique_ptr<Publisher> _ptcl_pub;
    std::unique_ptr<Publisher> _saliency_ptcl_pub;
    std::unique_ptr<Publisher> _targets_ptcl_pub;

    std::unique_ptr<ip::workspace_t> _workspace;

    iagmm::TrainingData _dataset;
    iagmm::NNMap _classifier;

    ip::PointCloudT::Ptr _workspace_ptcl;
    ip::PointCloudT::Ptr _saliency_ptcl;
    ip::PointCloudT::Ptr _targets_ptcl;

    std::vector<ip::Object> _targets;
    int _target_id;
    // ip::PointCloudT::Ptr _target_cloud;

    Vector4d _tracked_point;
    ip::PointCloudT::Ptr _tracked_object_ptcl;
    int _track_count = 0;
    double _downsampling_grid_size = 0.001;
    std::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<ip::PointT, ParticleXYZRPY> > _tracker;

    bool _clouds_ready;
    bool _tracking;
    bool _ending_tracking;
    std::string _classifier_path;

    void _update_workspace()
    {
        XmlRpc::XmlRpcValue wks;

        cafer_core::ros_nh->getParamCached("/object_babbling/babbling/experiment/workspace", wks);

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

    void _gridSampleApprox (const ip::PointCloudT::ConstPtr &cloud, ip::PointCloudT &result, double leaf_size)
    {
      pcl::ApproximateVoxelGrid<ip::PointT> grid;
      grid.setLeafSize(static_cast<float>(leaf_size), static_cast<float>(leaf_size), static_cast<float>(leaf_size));
      grid.setInputCloud(cloud);
      grid.filter(result);
    }

};

int main(int argc, char** argv)
{
    std::string node_name;
    XmlRpc::XmlRpcValue cafer;

    tbb::task_scheduler_init init;

    node_name = global::parse_arg(argc, argv, "image_processing_node");

    cafer_core::init(argc, argv, node_name);
    cafer_core::ros_nh->getParam("cafer", cafer);

    ImageProcessing imgProc(cafer["mgmt"], cafer["type"], cafer["freq"], cafer["uuid"]);

    imgProc.wait_for_init();

    while (ros::ok() && (!imgProc.get_terminate())) {
        imgProc.spin();
        imgProc.update();
        imgProc.sleep();
    }

    ROS_INFO_STREAM("IMAGE PROCESSING : ros::ok = " << ros::ok()
                    << " terminate = " << imgProc.get_terminate());

    ROS_INFO_STREAM("IMAGE PROCESSING : is finish");

    return 0;
}
