#include <boost/algorithm/string.hpp>

#include <cafer_core/cafer_core.hpp>

#include "data/data_controller_feedback.hpp"
#include "data/data_motion_rgbd.hpp"

#include "globals.h"

using namespace cafer_core;

int main(int argc, char** argv)
{
    shared_ptr<DatabaseManager::_Wave> wave_3;

    std::string node_name;
    XmlRpc::XmlRpcValue cafer;

    node_name = global::parse_arg(argc, argv, "db_manager_node");
    cafer_core::init(argc, argv, node_name);
    cafer_core::ros_nh->getParam("cafer", cafer);

    DatabaseManager db_manager(cafer["mgmt"], cafer["type"], cafer["freq"]);

    // Retrieve WATCHDOG messages from the management topic.
    db_manager.wait_for_init();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    db_manager.spin();

    while(!db_manager.add_wave("/object_babbling/babbling")){
        db_manager.spin();
        db_manager.update();
        db_manager.sleep();
    }

    db_manager.find_wave_by_name("/object_babbling/babbling", wave_3);

    std::unique_ptr<ManagerQueue<DataController>> controller_manager{new ManagerQueue<DataController>()};
    std::unique_ptr<ManagerQueue<DataMotionRGBD>> motion_rgbd_manager{new ManagerQueue<DataMotionRGBD>()};

    for(const auto& topic : wave_3->data_topics)
        ROS_INFO_STREAM(topic.first << ": " << topic.second);

    if (wave_3 != nullptr) {
        wave_3->add_manager(motion_rgbd_manager.release(), wave_3->data_topics["motion"]);
        wave_3->add_manager(controller_manager.release(), wave_3->data_topics["joints_values"]);
    }

    while (ros::ok() && (!db_manager.get_terminate())) {
        db_manager.spin();
        db_manager.update();
        db_manager.sleep();
    }

    return EXIT_SUCCESS;
}
