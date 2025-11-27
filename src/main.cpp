#include "rclcpp/rclcpp.hpp"
#include "ap1/pnc_sim/sim_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ap1::pnc_sim::SimpleVehicleSimNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
