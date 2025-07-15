#include "my_robot_hardware/mobile_base_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mobile_base_hardware {

// 初始化硬件接口
hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    // 读取硬件参数
    left_motor_id_ = std::stoi(info_.hardware_parameters["left_motor_id"]);
    right_motor_id_ = std::stoi(info_.hardware_parameters["right_motor_id"]);
    port_ = info_.hardware_parameters["dynamixel_port"];

    driver_ = std::make_shared<XL330Driver>(port_);

    // 初始化状态向量
    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
}

// 导出状态接口（关键：替换直接操作state_interfaces_的方式）
std::vector<hardware_interface::StateInterface> MobileBaseHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        const auto & joint = info_.joints[i];
        // 位置状态接口
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.name,
                hardware_interface::HW_IF_POSITION,
                &hw_positions_[i]
            )
        );
        // 速度状态接口
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.name,
                hardware_interface::HW_IF_VELOCITY,
                &hw_velocities_[i]
            )
        );
    }

    return state_interfaces;
}

// 导出命令接口（关键：替换直接操作command_interfaces_的方式）
std::vector<hardware_interface::CommandInterface> MobileBaseHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        const auto & joint = info_.joints[i];
        // 速度命令接口
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                joint.name,
                hardware_interface::HW_IF_VELOCITY,
                &hw_commands_[i]
            )
        );
    }

    return command_interfaces;
}

// 配置硬件
hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    if (driver_->init() != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("MobileBaseHardwareInterface"), "Failed to initialize driver");
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), "Driver initialized successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// 激活硬件
hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    // 初始化关节状态
    for (size_t i = 0; i < info_.joints.size(); i++) {
        hw_positions_[i] = 0.0;
        hw_velocities_[i] = 0.0;
        hw_commands_[i] = 0.0;
    }
    // 激活电机
    driver_->activateWithVelocityMode(left_motor_id_);
    driver_->activateWithVelocityMode(right_motor_id_);
    RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), "Hardware activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// 停止硬件
hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    driver_->deactivate(left_motor_id_);
    driver_->deactivate(right_motor_id_);
    RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), "Hardware deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// 读取硬件状态
hardware_interface::return_type MobileBaseHardwareInterface::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    // 从硬件读取速度
    double left_vel = driver_->getVelocityRadianPerSec(left_motor_id_);
    double right_vel = -1.0 * driver_->getVelocityRadianPerSec(right_motor_id_);
    
    // 过滤噪声
    if (std::abs(left_vel) < 0.03) left_vel = 0.0;
    if (std::abs(right_vel) < 0.03) right_vel = 0.0;
    
    // 更新状态向量
    for (size_t i = 0; i < info_.joints.size(); i++) {
        const auto & joint = info_.joints[i];
        if (joint.name == "left_wheel_joint") {
            hw_velocities_[i] = left_vel;
            hw_positions_[i] += left_vel * period.seconds();
        } else if (joint.name == "right_wheel_joint") {
            hw_velocities_[i] = right_vel;
            hw_positions_[i] += right_vel * period.seconds();
        }
    }
    return hardware_interface::return_type::OK;
}

// 写入控制命令
hardware_interface::return_type MobileBaseHardwareInterface::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    // 发送命令到硬件
    for (size_t i = 0; i < info_.joints.size(); i++) {
        const auto & joint = info_.joints[i];
        if (joint.name == "left_wheel_joint") {
            driver_->setTargetVelocityRadianPerSec(left_motor_id_, hw_commands_[i]);
        } else if (joint.name == "right_wheel_joint") {
            driver_->setTargetVelocityRadianPerSec(right_motor_id_, -1.0 * hw_commands_[i]);
        }
    }
    return hardware_interface::return_type::OK;
}

} // namespace mobile_base_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardwareInterface, hardware_interface::SystemInterface)
