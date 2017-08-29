#include <ros/ros.h>
#include <ros/package.h>
#include <globals.h>

#include "cafer_core/cafer_core.hpp"


namespace cc = cafer_core;

class Supervisor_node : public cc::Supervisor {
    using cc::Supervisor::Supervisor;
public:

    void database_manager_cb(const cc::DBManagerConstPtr& status_msg){

        auto type = static_cast<cc::DatabaseManager::Response>(status_msg->type);
        switch(type){
        case cc::DatabaseManager::Response::STATUS_READY :
            if (!db_done_recording) {
                ROS_INFO_STREAM("SUPERVISOR : DBManager is waiting");
            }
            db_done_recording = true;
            break;
        case cc::DatabaseManager::Response::STATUS_ACTIVE :
            if (db_done_recording) {
                ROS_INFO_STREAM("SUPERVISOR : DBManager is recording");
            }
            db_done_recording = false;
            break;
        case cc::DatabaseManager::Response::ERROR :
            break;
        case cc::DatabaseManager::Response::DATA :
            break;
        default :
            ROS_WARN_STREAM("SUPERVISOR : DBManager unknown response message receive");
        }
    }

    /**
     * Check state of managed components.
     * Try to relaunch them if they cannot be found on the management topic.
     */
    void update() override
    {
        uint32_t tries;
        cc::ClientDescriptor descriptor;
        std::string uuid;
        std::string namespace_;

        for (auto& comp:uuid_launched_components) {
            uuid = comp.first;
            tries = 10;
            bool found = false;
            do {
                found = find_by_uuid(uuid, descriptor);
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
                --tries;
            }
            while (!found && tries > 0);
            if (!found) {
                ROS_FATAL_STREAM("SUPERVISOR : Unable to communicate with component launched with: roslaunch " << comp.second);
            }

        }

        // ROS_INFO_STREAM("Supervisor id : " << _descriptor.id);
        cc::DBManager db_request;
        db_request.id = _descriptor.id;
        db_request.type = static_cast<uint8_t>(cc::DatabaseManager::Request::ASK_STATUS);
        _pub_db_manager->publish(db_request);
    }
};

int main(int argc, char** argv)
{
    std::string node_name;
    XmlRpc::XmlRpcValue cafer;

    node_name = global::parse_arg(argc, argv, "babbling");

    cafer_core::init(argc, argv, node_name);
    cafer_core::ros_nh->getParam("cafer", cafer);

    Supervisor_node supervisor(cafer["mgmt"], cafer["type"], cafer["freq"],cafer["uuid"]);

    supervisor.wait_for_init();
    supervisor.spin();

    while (ros::ok() && !supervisor.get_terminate()) {
        supervisor.spin();
        supervisor.update();
        supervisor.sleep();
    }

    return 0;
}
