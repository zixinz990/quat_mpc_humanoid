#include <mutex>
#include <thread>

#include "GazeboInterface.h"
#include "RobotState.h"

using namespace std;
using namespace robot;

std::mutex mtx;

int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    // Read ROS parameters
    string robot_name;
    nh.param<string>("robot_name", robot_name, "mit_humanoid");
    cout << "Robot name: " << robot_name << endl;

    // Initialize interface
    auto intf_ptr = std::make_unique<GazeboInterface>(nh, robot_name);
    
    // Thread 1: feedback
    std::thread feedback_thread([&]() {
        ros::Rate loop_rate(2000);
        while (ros::ok()) {
            mtx.lock();
            intf_ptr->fbk_update();
            intf_ptr->ctrl_update();
            loop_rate.sleep();
            mtx.unlock();
        }
    });

    ros::spin();

    return 0;
}
