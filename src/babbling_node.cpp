#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <string>
#include <stdexcept>

#include <Eigen/Core>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <actionlib/client/simple_action_client.h>
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
// #include <pcl/tracking/normal_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
#include <pcl/filters/extract_indices.h>

#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>

#include <iagmm/gmm.hpp>
#include <iagmm/nnmap.hpp>

#include <image_processing/pcl_types.h>
#include <image_processing/features.hpp>
#include <image_processing/BabblingDataset.h>
#include <image_processing/SupervoxelSet.h>
#include <image_processing/SurfaceOfInterest.h>
#include <image_processing/DescriptorExtraction.h>
#include <image_processing/Object.h>

#include <cafer_core/cafer_core.hpp>

#include "object_babbling/pose_goalAction.h"
#include "object_babbling/is_moving.h"
#include "object_babbling/dataset.h"
#include "object_babbling/gmm_archive.h"
#include "object_babbling/rgbd_motion_data.h"

#include "globals.h"

using namespace Eigen;
using namespace rgbd_utils;
namespace ip = image_processing;
using namespace pcl::tracking;

using namespace cafer_core;
using namespace object_babbling;


class Babbling : public Component {
    using Component::Component;

public:

    typedef iagmm::GMM Classifier;
    typedef ip::Object<Classifier> ObjectHyp;

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

        _dataset_path = static_cast<std::string>(exp_params["dataset_path"]);
        _classifier_path = static_cast<std::string>(exp_params["classifier_path"]);
        _saliency_modality = static_cast<std::string>(exp_params["classifier_modality"]);
        _saliency_sample_size = static_cast<int>(exp_params["classifier_sample_size"]);

        _modality = static_cast<std::string>(exp_params["modality"]);
        _sample_size = static_cast<int>(exp_params["sample_size"]);

        _nb_iter = static_cast<int>(exp_params["number_of_iterations"]);
        _nb_samples = static_cast<int>(exp_params["number_of_samples"]);

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

        // Feedback
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
        _object_saliency_ptcl_pub.reset(
            new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>(
                glob_params["object_saliency_ptcl"], 10)
            )
        );

        _target_ptcl_pub.reset(
            new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>(
                glob_params["target_ptcl"], 10)
            )
        );
        _tracked_ptcl_pub.reset(
            new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>(
                glob_params["tracked_ptcl"], 10)
            )
        );
        _result_ptcl_pub.reset(
            new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>(
                glob_params["result_ptcl"], 10)
            )
        );

        _target_point_pub.reset(
            new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>(
                glob_params["target_point_topic"], 10)
            )
        );
        _tracked_point_pub.reset(
            new Publisher(ros_nh->advertise<sensor_msgs::PointCloud2>(
                glob_params["tracked_point_topic"], 10)
            )
        );

        //Publishers to send the training dataset to the data manager node
        _dataset_pub.reset(
            new Publisher(ros_nh->advertise<dataset>(
                glob_params["dataset_topic"], 10)
            )
        );

        //Publishers to send the GMM Classifier archive to the data manager node
        _classifier_pub.reset(
            new Publisher(ros_nh->advertise<gmm_archive>(
                glob_params["classifier_topic"], 10)
            )
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

        _update_workspace();

        std::ifstream ifs(_classifier_path);
        boost::archive::text_iarchive ia(ifs);
        ia >> _saliency_classifier;

        _dataset = load_dataset(_dataset_path);
        _saliency_classifier.set_samples(_dataset);

        _target_center << 0.0, 0.0, 0.0, 0.0;
        _tracked_center << 0.0, 0.0, 0.0, 0.0;
        _tracked_ptcl = ip::PointCloudT::Ptr(new ip::PointCloudT);

        _result_ptcl = ip::PointCloudT::Ptr(new ip::PointCloudT);

        _db_init = false;
        _db_ready = true;
        _recording = false;
        _create_new_hypothesis = true;
        _clouds_ready = false;

        _counter_iter = 0;

        usleep(1e6);
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

        _ptcl_pub.reset();
        _saliency_ptcl_pub.reset();
        _object_saliency_ptcl_pub.reset();

        _target_ptcl_pub.reset();
        _tracked_ptcl_pub.reset();
        _result_ptcl_pub.reset();

        _target_point_pub.reset();
        _tracked_point_pub.reset();

        _dataset_pub.reset();
        _classifier_pub.reset();

        _db_request_publisher.reset();
        _db_status_subscriber.reset();
    }

    void update() override
    {
        std::string supervisor_name =
            ros::names::parentNamespace(cafer_core::ros_nh->getNamespace()) + "/babbling";

        // Image
        _depth_msg = sensor_msgs::ImageConstPtr(new sensor_msgs::Image(_images_sub->get_depth()));
        _rgb_msg = sensor_msgs::ImageConstPtr(new sensor_msgs::Image(_images_sub->get_rgb()));
        _info_msg = sensor_msgs::CameraInfoConstPtr(new sensor_msgs::CameraInfo(_images_sub->get_rgb_info()));

        if (_rgb_msg->data.empty() || _depth_msg->data.empty()) {
            return;
        }

        if (!_db_init) {
            ROS_INFO_STREAM("BABBLING_NODE : waiting for db manager");
            return;
        }

        if (_db_ready && !_recording) {
            ip::PointCloudXYZ point_tracked;
            sensor_msgs::PointCloud2 point_tracked_msg;
            pcl::toROSMsg(point_tracked, point_tracked_msg);
            point_tracked_msg.header = _images_sub->get_depth().header;
            _tracked_point_pub->publish(point_tracked_msg);

            sensor_msgs::PointCloud2 tracked_ptcl_msg;
            pcl::toROSMsg(*_tracked_ptcl, tracked_ptcl_msg);
            tracked_ptcl_msg.header = _images_sub->get_depth().header;
            _tracked_ptcl_pub->publish(tracked_ptcl_msg);

            ROS_WARN_STREAM("BABBLING_NODE : BEGIN ITERATION " << _counter_iter
                            << " (" << _objects_hypotheses.size() << " hypotheses availables)");
            // Initialize the iteration
            _surface = _extract_surface(_saliency_modality, _modality);

            _surface.init_weights(_saliency_modality, 0.5);
            _surface.compute_weights<Classifier>(_saliency_modality, _saliency_classifier);

            if (_create_new_hypothesis) {
                ROS_WARN_STREAM("BABBLING_NODE : creating new hypothesis");

                ObjectHyp new_hyp = _new_hypothesis(
                    _surface,
                    _saliency_modality,
                    _modality
                );
                _objects_hypotheses.push_back(new_hyp);

                _hypothesis_id = _objects_hypotheses.size() - 1;

                _create_new_hypothesis = false;
            }
            else {
                int nb_s = _objects_hypotheses[_hypothesis_id].get_classifier().dataset_size();
                ROS_WARN_STREAM("BABBLING_NODE : improving hypothesis " << _hypothesis_id
                                << " (" << nb_s <<" samples availables)");
            }

            // Feedback information
            _saliency_weights = _surface.get_weights()[_saliency_modality];
            _weights = _compute_object_weights(
                _surface,
                _objects_hypotheses
            );

            _saliency_ptcl = _surface.getColoredWeightedCloud(_saliency_modality).makeShared();
            _object_saliency_ptcl = _surface.getColoredWeightedCloud(_weights[_hypothesis_id]).makeShared();
            _clouds_ready = true;

            publish_feedback();

            _start_db_recording();
        }

        if (_robot_controller_ready && _recording) {
            if(!_objects_hypotheses[_hypothesis_id].set_initial(_surface)) {
                ROS_ERROR_STREAM("BABBLING_NODE : failed to set object's initial surface");

                _objects_hypotheses[_hypothesis_id].recover_center(_surface);
                return;
            }

            _target_ptcl = _objects_hypotheses[_hypothesis_id].get_initial_cloud();
            _target_center = _objects_hypotheses[_hypothesis_id].get_center();

            ROS_INFO_STREAM("BABBLING_NODE : target cloud size " << _target_ptcl->size());

            Vector3d center_camera_frame;
            center_camera_frame << _target_center(0), _target_center(1), _target_center(2);
            Vector3d target_center_robot;
            babbling::tf_base_conversion(target_center_robot, center_camera_frame);

            pose_goalGoal poseGoal;
            poseGoal.target_pose.resize(3);
            poseGoal.target_pose[0] = target_center_robot(0);
            poseGoal.target_pose[1] = target_center_robot(1);
            poseGoal.target_pose[2] = target_center_robot(2);

            _start_tracking();

            _client_controller->sendGoal(poseGoal);
            _robot_controller_ready = false;
        }

        if (!_robot_controller_ready) {
            auto client_status = _client_controller->getState();
            if (client_status == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO_STREAM("BABBLING_NODE : position reached");
                _robot_controller_ready = true;

                _stop_tracking();

                // Update object hypothesis
                ip::SurfaceOfInterest current_surface = _extract_surface(_saliency_modality, _modality);
                current_surface.init_weights(_saliency_modality, 0.5);
                current_surface.compute_weights<Classifier>(_saliency_modality, _saliency_classifier);

                _saliency_ptcl = current_surface.getColoredWeightedCloud(_saliency_modality).makeShared();

                if(!_objects_hypotheses[_hypothesis_id].set_current(current_surface, _tracked_transformation)) {
                    ROS_ERROR_STREAM("BABBLING_NODE : model was not updated");

                    _objects_hypotheses[_hypothesis_id].recover_center(current_surface);
                }

                _result_ptcl = _objects_hypotheses[_hypothesis_id].get_result_cloud();
                ROS_INFO_STREAM("BABBLING_NODE : result cloud size " << _result_ptcl->size());

                _stop_db_recording();

                if (_bored(_objects_hypotheses[_hypothesis_id])) {
                    _create_new_hypothesis = true;
                }

                _counter_iter += 1;
                ROS_INFO_STREAM("BABBLING_NODE : iteration " << _counter_iter << " done (nb_iter = " << _nb_iter << ")");
            }
            else if (client_status == actionlib::SimpleClientGoalState::ABORTED) {
                ROS_INFO_STREAM("BABBLING_NODE : position wasn't reachable");
                _robot_controller_ready = true;

                _stop_tracking();
            }
            else {
                // ROS_INFO_STREAM("BABBLING_NODE : waiting for controller...");
            }
        }

        publish_feedback();
    }

    bool is_finish()
    {
        return _counter_iter>=_nb_iter;
    }

    void publish_feedback()
    {
        // target
        ip::PointCloudXYZ point_target;
        sensor_msgs::PointCloud2 point_target_msg;
        if (!(_target_center[0] == 0 || _target_center[1] == 0 || _target_center[2] == 0)) {
            point_target.push_back(pcl::PointXYZ(_target_center[0], _target_center[1], _target_center[2]));
        }
        pcl::toROSMsg(point_target, point_target_msg);
        point_target_msg.header = _images_sub->get_depth().header;
        _target_point_pub->publish(point_target_msg);

        // tracking results
        sensor_msgs::PointCloud2 result_ptcl_msg;
        pcl::toROSMsg(*_result_ptcl, result_ptcl_msg);
        result_ptcl_msg.header = _images_sub->get_depth().header;
        _result_ptcl_pub->publish(result_ptcl_msg);

        // feedback clouds
        if (_clouds_ready) {
            sensor_msgs::PointCloud2 workspace_ptcl_msg;
            pcl::toROSMsg(*_workspace_ptcl, workspace_ptcl_msg);
            workspace_ptcl_msg.header = _images_sub->get_depth().header;
            _ptcl_pub->publish(workspace_ptcl_msg);

            sensor_msgs::PointCloud2 saliency_ptcl_msg;
            pcl::toROSMsg(*_saliency_ptcl, saliency_ptcl_msg);
            saliency_ptcl_msg.header = _images_sub->get_depth().header;
            _saliency_ptcl_pub->publish(saliency_ptcl_msg);

            sensor_msgs::PointCloud2 object_saliency_ptcl_msg;
            pcl::toROSMsg(*_object_saliency_ptcl, object_saliency_ptcl_msg);
            object_saliency_ptcl_msg.header = _images_sub->get_depth().header;
            _object_saliency_ptcl_pub->publish(object_saliency_ptcl_msg);

            // for all object hyp ?
        }
    }

private:

    // Publisher and Subscriber
    std::unique_ptr<ros::ServiceClient> _client_motion;
    std::unique_ptr<actionlib::SimpleActionClient<pose_goalAction>> _client_controller;

    RGBD_Subscriber::Ptr _images_sub;
    sensor_msgs::ImageConstPtr _depth_msg;
    sensor_msgs::ImageConstPtr _rgb_msg;
    sensor_msgs::CameraInfoConstPtr _info_msg;

    std::unique_ptr<Publisher> _ptcl_pub;
    std::unique_ptr<Publisher> _saliency_ptcl_pub;
    std::unique_ptr<Publisher> _object_saliency_ptcl_pub;

    std::unique_ptr<Publisher> _target_ptcl_pub;
    std::unique_ptr<Publisher> _tracked_ptcl_pub;
    std::unique_ptr<Publisher> _result_ptcl_pub;

    std::unique_ptr<Publisher> _target_point_pub;
    std::unique_ptr<Publisher> _tracked_point_pub;

    std::unique_ptr<Publisher> _dataset_pub;
    std::unique_ptr<Publisher> _classifier_pub;

    std::unique_ptr<Publisher> _db_request_publisher;
    std::unique_ptr<Subscriber> _db_status_subscriber;

    std::unique_ptr<ip::workspace_t> _workspace;

    // Initial saliency map
    std::string _dataset_path;
    iagmm::TrainingData _dataset;
    std::string _classifier_path;
    Classifier _saliency_classifier;
    std::string _saliency_modality;
    int _saliency_sample_size;
    ip::saliency_map_t _saliency_weights;

    // Objects
    ip::SurfaceOfInterest _surface;
    std::vector<ObjectHyp> _objects_hypotheses;
    std::string _modality;
    int _sample_size;
    std::map<size_t, ip::saliency_map_t> _weights;
    size_t _hypothesis_id;

    // Feedback clouds
    ip::PointCloudT::Ptr _workspace_ptcl;
    ip::PointCloudT::Ptr _saliency_ptcl;
    ip::PointCloudT::Ptr _object_saliency_ptcl;

    // Tracking
    std::unique_ptr<Subscriber> _tracking_sub;
    Vector4d _target_center;
    ip::PointCloudT::Ptr _target_ptcl;
    Vector4d _tracked_center;
    ip::PointCloudT::Ptr _tracked_ptcl;
    ip::PointCloudT::Ptr _result_ptcl;
    Eigen::Affine3f _tracked_transformation;
    int _track_count = 0;
    double _downsampling_grid_size = 0.001;
    std::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<ip::PointT, ParticleXYZRPY> > _tracker;


    bool _robot_controller_ready = true;
    bool _db_ready;
    bool _db_init;
    bool _recording;
    bool _create_new_hypothesis;
    bool _clouds_ready;

    int _counter_iter;
    int _nb_iter;
    int _nb_samples;

    ip::SurfaceOfInterest _extract_surface(const std::string& saliency_modality, const std::string& modality)
    {
        ROS_INFO_STREAM("BABBLING_NODE : extracting supervoxels");

        rgbd_utils::RGBD_to_Pointcloud converter(_depth_msg, _rgb_msg, _info_msg);

        sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();

        _workspace_ptcl = ip::PointCloudT::Ptr(new ip::PointCloudT);
        pcl::fromROSMsg(ptcl_msg, *_workspace_ptcl);
        if (_workspace_ptcl->empty()) {
            ROS_ERROR_STREAM("BABBLING_NODE : empty cloud message");
            throw std::runtime_error("BABBLING_NODE : empty cloud message");
        }

        _workspace->filter(_workspace_ptcl);
        if (_workspace_ptcl->empty()) {
            ROS_ERROR_STREAM("BABBLING_NODE : empty filtered cloud");
            throw std::runtime_error("BABBLING_NODE : empty filtered cloud");
        }

        ip::SurfaceOfInterest surface;
        surface.clear();
        surface.setInputCloud(_workspace_ptcl);
        if(!surface.computeSupervoxel()) {
            ROS_ERROR_STREAM("BABBLING_NODE : error computing supervoxels");
            throw std::runtime_error("BABBLING_NODE : error computing supervoxels");
        }

        surface.compute_feature(saliency_modality);
        if (modality != saliency_modality) {
            surface.compute_feature(modality);
        }

        return surface;
    }

    std::map<size_t, ip::saliency_map_t> _compute_object_weights(ip::SurfaceOfInterest& surface,
                                                            std::vector<ObjectHyp>& objects_hypotheses)
    {
        ROS_INFO_STREAM("BABBLING_NODE : computing objects' saliency maps");

        std::map<size_t, ip::saliency_map_t> weights;

        for (size_t i = 0; i < objects_hypotheses.size(); i++)
        {
            Classifier classifier = objects_hypotheses[i].get_classifier();
            std::string modality = objects_hypotheses[i].get_modality();
            weights[i] = surface.compute_saliency_map<Classifier>(modality, classifier);
        }

        return weights;
    }

    ObjectHyp _new_hypothesis(ip::SurfaceOfInterest& surface,
                              const std::string& saliency_modality,
                              const std::string& modality)
    {
        ROS_INFO_STREAM("BABBLING_NODE : creating new object hypothesis");

        std::vector<std::vector<uint32_t>> regions = surface.extract_regions(saliency_modality, 0.5);
        if (regions.size() == 0 ) {
            ROS_ERROR_STREAM("BABBLING_NODE : no region detected");
            throw std::runtime_error("BABBLING_NODE : no region detected");
        }

        // Select region (@TODO : select according previous object hyp)
        std::vector<uint32_t> max_region = regions[0];
        for (const auto& region : regions)
        {
            if (region.size() > max_region.size()) {
                max_region = region;
            }
        }

        // Init classifier
        Classifier classifier(_sample_size, 2);

        // Compute center
        Eigen::Vector4d center;
        pcl::compute3DCentroid(surface.get_cloud(max_region), center);
        ROS_INFO_STREAM("BABBLING_NODE : object center " << center);

        ObjectHyp hyp(classifier, saliency_modality, modality, center);
        return hyp;
    }

    void _start_tracking()
    {
        ROS_INFO_STREAM("BABBLING_NODE : starting tracking");
        XmlRpc::XmlRpcValue glob_params;
        cafer_core::ros_nh->getParam("/object_babbling/params", glob_params);

        _tracking_sub.reset(
            new Subscriber(
                ros_nh->subscribe<rgbd_motion_data>(
                    glob_params["motion_detector_topic"], 10,
                    boost::bind(&Babbling::_image_processing_callback, this, _1))
            )
        );

        // Tracker creation
        std::vector<double> default_step_covariance = std::vector<double>(6, 0.015 * 0.015);
        default_step_covariance[3] *= 40.0;
        default_step_covariance[4] *= 40.0;
        default_step_covariance[5] *= 40.0;

        std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.00001);
        std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);

        _tracker.reset(new KLDAdaptiveParticleFilterOMPTracker<ip::PointT, ParticleXYZRPY>(30));

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
        _tracker->setResampleLikelihoodThr(0.05);
        _tracker->setUseNormal(false);


        // Coherence for tracking
        ApproxNearestPairPointCloudCoherence<ip::PointT>::Ptr coherence = ApproxNearestPairPointCloudCoherence<ip::PointT>::Ptr
          (new ApproxNearestPairPointCloudCoherence<ip::PointT>());

        boost::shared_ptr<DistanceCoherence<ip::PointT> > distance_coherence
          = boost::shared_ptr<DistanceCoherence<ip::PointT> >(new DistanceCoherence<ip::PointT>());
        coherence->addPointCoherence(distance_coherence);

        boost::shared_ptr<HSVColorCoherence<ip::PointT> > color_coherence
          = boost::shared_ptr<HSVColorCoherence<ip::PointT> >(new HSVColorCoherence<ip::PointT>());
        coherence->addPointCoherence(color_coherence);

        // boost::shared_ptr<NormalCoherence<ip::PointT> > normal_coherence
        //   = boost::shared_ptr<NormalCoherence<ip::PointT> >(new NormalCoherence<ip::PointT>());
        // coherence->addPointCoherence(normal_coherence);

        boost::shared_ptr<pcl::search::Octree<ip::PointT> > search (new pcl::search::Octree<ip::PointT>(0.01));
        coherence->setSearchMethod(search);
        coherence->setMaximumDistance(0.005);

        _tracker->setCloudCoherence(coherence);

        // Preparing target cloud
        Eigen::Vector4f c;
        Eigen::Affine3f trans = Eigen::Affine3f::Identity();
        ip::PointCloudT::Ptr transed_ref(new ip::PointCloudT);
        ip::PointCloudT::Ptr transed_ref_downsampled(new ip::PointCloudT);

        pcl::compute3DCentroid<ip::PointT>(*_target_ptcl, c);
        trans.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);
        pcl::transformPointCloud<ip::PointT>(*_target_ptcl, *transed_ref, trans.inverse());

        _grid_sample_approx(transed_ref, *transed_ref_downsampled, _downsampling_grid_size);

        // Set reference model and trans
        _tracker->setReferenceCloud(transed_ref_downsampled);
        _tracker->setTrans(trans);
    }

    void _stop_tracking()
    {
        ROS_INFO_STREAM("BABBLING_NODE : stoping tracking");
        _tracking_sub.reset();
        _tracker.reset();

        _tracked_center << 0.0, 0.0, 0.0, 0.0;
        _tracked_ptcl = ip::PointCloudT::Ptr(new ip::PointCloudT);
    }

    void _image_processing_callback(const rgbd_motion_data::ConstPtr&  msg)
    {
        sensor_msgs::ImageConstPtr depth_msg(new sensor_msgs::Image(msg->depth));
        sensor_msgs::ImageConstPtr rgb_msg(new sensor_msgs::Image(msg->rgb));
        sensor_msgs::CameraInfoConstPtr info_msg(new sensor_msgs::CameraInfo(msg->rgb_info));

        if (rgb_msg->data.empty() && depth_msg->data.empty()) {
            ROS_ERROR_STREAM("BABBLING_NODE : tracking empty cloud");
            return;
        }

        rgbd_utils::RGBD_to_Pointcloud converter(depth_msg, rgb_msg, info_msg);

        sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();
        ip::PointCloudT::Ptr cloud = ip::PointCloudT::Ptr(new ip::PointCloudT);
        ip::PointCloudT::Ptr cloud_downsampled = ip::PointCloudT::Ptr(new ip::PointCloudT);
        pcl::fromROSMsg(ptcl_msg, *cloud);
        _grid_sample_approx(cloud, *cloud_downsampled, _downsampling_grid_size);

        _tracker->setInputCloud(cloud_downsampled);
      	_tracker->compute();

        ParticleXYZRPY result = _tracker->getResult();
        _tracked_transformation = _tracker->toEigenMatrix(result);

        _tracked_ptcl = ip::PointCloudT::Ptr(new ip::PointCloudT);
        pcl::transformPointCloud<ip::PointT>(*(_tracker->getReferenceCloud()), *_tracked_ptcl, _tracked_transformation);

        pcl::compute3DCentroid<ip::PointT>(*_tracked_ptcl, _tracked_center);

        // publish feedback
        ip::PointCloudXYZ point_tracked;
        point_tracked.push_back(pcl::PointXYZ(_tracked_center[0], _tracked_center[1], _tracked_center[2]));

        sensor_msgs::PointCloud2 point_tracked_msg;
        pcl::toROSMsg(point_tracked, point_tracked_msg);
        point_tracked_msg.header = _images_sub->get_depth().header;
        _tracked_point_pub->publish(point_tracked_msg);

        sensor_msgs::PointCloud2 tracked_ptcl_msg;
        pcl::toROSMsg(*_tracked_ptcl, tracked_ptcl_msg);
        tracked_ptcl_msg.header = _images_sub->get_depth().header;
        _tracked_ptcl_pub->publish(tracked_ptcl_msg);
    }

    void _start_db_recording()
    {
        std::string supervisor_name =
            ros::names::parentNamespace(cafer_core::ros_nh->getNamespace()) + "/babbling";

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

    void _stop_db_recording()
    {
        std::string supervisor_name =
            ros::names::parentNamespace(cafer_core::ros_nh->getNamespace()) + "/babbling";

        // send additional data like classifier

        Classifier classifier = _objects_hypotheses[_hypothesis_id].get_classifier();
        if (classifier.dataset_size() != 0){
            _dataset_pub->publish(
                _training_data_to_ros_msg(
                    _modality,
                    classifier.get_samples()
                )
            );

            std::stringstream sstream;
            boost::archive::text_oarchive oarch(sstream);
            oarch << classifier;
            object_babbling::gmm_archive msg;
            msg.archive.data = sstream.str();
            msg.type.data = _modality;
            _classifier_pub->publish(msg);
        }

        // wait them to arrive
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

    bool _bored(ObjectHyp& object)
    {
        int nb_s = object.get_classifier().dataset_size();
        return nb_s > _nb_samples;
    }

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

    void _grid_sample_approx (const ip::PointCloudT::ConstPtr &cloud, ip::PointCloudT &result, double leaf_size)
    {
        pcl::ApproximateVoxelGrid<ip::PointT> grid;
        grid.setLeafSize(static_cast<float>(leaf_size), static_cast<float>(leaf_size), static_cast<float>(leaf_size));
        grid.setInputCloud(cloud);
        grid.filter(result);
    }

    dataset _training_data_to_ros_msg(const std::string& type, const iagmm::TrainingData& tr_data)
    {
        dataset dataset_msg;
        sv_feature temp_feature;

        dataset_msg.type.data = type;

        iagmm::TrainingData::data_t dataset = tr_data.get();
        std::vector<double> data_vct(dataset[0].second.rows());
        for (const auto& data : dataset) {

            for(int i = 0; i < data.second.rows(); i++)
                data_vct[i] = data.second(i);
            temp_feature.feature = data_vct;
            temp_feature.label = data.first;

            dataset_msg.features.push_back(temp_feature);
        }
        return dataset_msg;
    }

    iagmm::TrainingData load_dataset(const std::string& filename)
    {
        iagmm::TrainingData dataset;

        YAML::Node fileNode = YAML::LoadFile(filename);
        if (fileNode.IsNull()) {
            ROS_ERROR_STREAM("BABBLING_NODE : could not load dataset at " << filename);
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
    babbling.spin();

    ROS_INFO_STREAM("BABBLING_NODE : Babbling ready !");

    while (ros::ok() && (!babbling.get_terminate()) && !babbling.is_finish()) {
        babbling.spin();
        babbling.update();
        babbling.sleep();
    }

    ROS_INFO_STREAM("BABBLING_NODE : ros::ok = " << ros::ok()
                    << " terminate = " << babbling.get_terminate()
                    << " is finish = " << babbling.is_finish());

    ROS_INFO_STREAM("BABBLING_NODE : is finish");

    return 0;
}
