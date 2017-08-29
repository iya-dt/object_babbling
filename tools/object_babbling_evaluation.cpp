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



int main(int argc, char** argv)
{

  return 0;
}
