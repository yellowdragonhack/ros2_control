#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "my_robot_hardware/xl330_driver.hpp"
#include <vector> 
#include <memory>
#include <string>

namespace mobile_base_hardware {

class MobileBaseHardwareInterface : public hardware_interface::SystemInterface
{
public:
    // Lifecycle node override
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // SystemInterface override
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::return_type
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        // 关键：声明接口导出方法（必须与源文件实现一致）
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;


private:
    std::shared_ptr<XL330Driver> driver_;
    int left_motor_id_;
    int right_motor_id_;
    std::string port_;
    hardware_interface::HardwareInfo info_;
      // 用哈希表存储关节状态（关节名称 -> 状态值）
    // 状态变量：使用vector而非unordered_map
    std::vector<double> hw_positions_;   // 位置（支持索引访问）
    std::vector<double> hw_velocities_;  // 速度（支持索引访问）
    std::vector<double> hw_commands_;    // 命令（支持索引访问）

}; // class MobileBaseHardwareInterface

} // namespace mobile_base_hardware


#endif