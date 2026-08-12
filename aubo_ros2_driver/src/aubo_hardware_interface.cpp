#include "aubo_hardware_interface.h"
#include <pluginlib/class_list_macros.hpp>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <ctime>
namespace aubo_driver {

AuboHardwareInterface::~AuboHardwareInterface()
{
    stopping_.store(true);
    reconnect_cv_.notify_all();
    if (reconnect_thread_.joinable()) {
        reconnect_thread_.join();
    }
    publishConnectionState(false);
    disconnectClients();
}

void AuboHardwareInterface::disconnectClients()
{
    std::lock_guard<std::mutex> lock(client_mtx_);
    if (rtde_client_) {
        rtde_client_->disconnect();
    }
    if (rpc_client_) {
        if (rpc_client_->hasConnected()) {
            try {
                rpc_client_->getRobotInterface(robot_name_)
                    ->getMotionControl()->setServoMode(false);
            } catch (const std::exception &) {
            }
        }
        rpc_client_->disconnect();
    }
    servo_mode_start_ = false;
    channels_initialized_.store(false);
}

bool AuboHardwareInterface::connectOnce()
{
    const auto generation = connection_generation_.fetch_add(1) + 1;
    auto rpc_client = std::make_shared<RpcClient>();
    auto rtde_client = std::make_shared<RtdeClient>();

    rpc_client->setEventHandler([this, generation](int event) {
        if (event == RpcClient::Disconnected) {
            markDisconnected(generation);
        }
    });
    rtde_client->setEventHandler([this, generation](int event) {
        if (event == RtdeClient::Disconnected) {
            markDisconnected(generation);
        }
    });

    rpc_client->setRequestTimeout(1000);
    if (rpc_client->connect(robot_ip_, 30004) < 0 ||
        rpc_client->login("aubo", "123456") != 0 ||
        rtde_client->connect(robot_ip_, 30010) < 0 ||
        rtde_client->login("aubo", "123456") != 0) {
        return false;
    }
    int topic = rtde_client->setTopic(false, { "R1_message" }, 200, 0);
    if (topic < 0) {
        return false;
    }
    rtde_client->subscribe(topic, [](InputParser &parser) {
        arcs::common_interface::RobotMsgVector msgs;
        msgs = parser.popRobotMsgVector();
    });
    const auto robot_names = rpc_client->getRobotNames();
    if (robot_names.empty()) {
        return false;
    }
    robot_name_ = robot_names.front();

    rpc_client->getRobotInterface(robot_name_)
    ->getRobotConfig()
    ->setHardwareCustomParameters("[joint_func] \n vff_enable = false\n");

    setInput(rtde_client);
    configSubscribe(rtde_client, generation);
    rpc_client->getRobotInterface(robot_name_)->getMotionControl()->setServoMode(true);
    for (int attempt = 0; attempt < 6; ++attempt) {
        if (rpc_client->getRobotInterface(robot_name_)
                ->getMotionControl()->isServoModeEnabled()) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!rpc_client->getRobotInterface(robot_name_)
            ->getMotionControl()->isServoModeEnabled()) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(client_mtx_);
        rpc_client_ = std::move(rpc_client);
        rtde_client_ = std::move(rtde_client);
        servo_mode_start_ = true;
        channels_initialized_.store(true);
    }
    return true;
}

void AuboHardwareInterface::publishConnectionState(bool connected)
{
    if (connection_state_publisher_) {
        connection_state_publisher_->publish(connected);
    }
}

void AuboHardwareInterface::markDisconnected(std::uint64_t generation)
{
    if (generation != connection_generation_.load()) {
        return;
    }
    channels_initialized_.store(false);
    initialized_ = false;
    if (connection_ready_.exchange(false)) {
        publishConnectionState(false);
        RCLCPP_ERROR(node_->get_logger(), "AUBO connection lost; reconnecting");
    }
    requestReconnect();
}

void AuboHardwareInterface::requestReconnect()
{
    reconnect_requested_.store(true);
    reconnect_cv_.notify_one();
}

void AuboHardwareInterface::reconnectLoop()
{
    while (!stopping_.load()) {
        std::unique_lock<std::mutex> lock(reconnect_mtx_);
        reconnect_cv_.wait(lock, [this] {
            return stopping_.load() || reconnect_requested_.load();
        });
        lock.unlock();
        if (stopping_.load()) {
            return;
        }
        channels_initialized_.store(false);
        if (connectOnce()) {
            reconnect_requested_.store(false);
            RCLCPP_INFO(node_->get_logger(), "AUBO transports initialized; waiting for fresh RTDE state");
        } else {
            std::unique_lock<std::mutex> retry_lock(reconnect_mtx_);
            reconnect_cv_.wait_for(retry_lock, std::chrono::seconds(1));
        }
    }
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &system_info)
{
    if (hardware_interface::SystemInterface::on_init(system_info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = system_info;
    robot_ip_ = info_.hardware_parameters["robot_ip"];
    initialized_ = false;
    node_ = rclcpp::Node::make_shared("aubo_connection_state");
    connection_state_publisher_ =
        std::make_unique<robot_connection_recovery::ConnectionStatePublisher>(*node_);
    publishConnectionState(false);

    for (const hardware_interface::ComponentInfo &joint : info_.joints) {
        // RRBotSystemPositionOnly has exactly one state and command interface
        // on each joint
        if (joint.command_interfaces.size() != 2) {
            RCLCPP_FATAL(
                rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                "Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                "Joint '%s' have %s command interfaces found. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                         "Joint '%s' has %zu state interface. 1 expected.",
                         joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                         "Joint '%s' have %s state interface. '%s' expected.",
                         joint.name.c_str(),
                         joint.state_interfaces[0].name.c_str(),
                         hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn AuboHardwareInterface::on_activate(
    const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                "Starting ...please wait...");
    stopping_.store(false);
    if (!reconnect_thread_.joinable()) {
        reconnect_thread_ = std::thread(&AuboHardwareInterface::reconnectLoop, this);
    }
    requestReconnect();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State &previous_state)
{
    stopping_.store(true);
    reconnect_cv_.notify_all();
    if (reconnect_thread_.joinable()) {
        reconnect_thread_.join();
    }
    connection_ready_.store(false);
    initialized_ = false;
    publishConnectionState(false);
    disconnectClients();
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AuboHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &actual_q_copy_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &joint_velocity_copy_[i]));
    }

    return state_interfaces;
}
std::vector<hardware_interface::CommandInterface>
AuboHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &aubo_position_commands_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &aubo_velocity_commands_[i]));
    }

    return command_interfaces;
}

hardware_interface::return_type AuboHardwareInterface::read(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    constexpr std::int64_t state_timeout_ns = 100'000'000;
    if (!channels_initialized_.load() ||
        now_ns - last_rtde_sample_ns_.load() > state_timeout_ns) {
        markDisconnected(connection_generation_.load());
        return hardware_interface::return_type::OK;
    }
    readActualQ();
    RobotModeType robot_mode;
    SafetyModeType safety_mode;
    {
        std::lock_guard<std::mutex> lock(rtde_mtx_);
        robot_mode = robot_mode_;
        safety_mode = safety_mode_;
    }
    if (robot_mode != RobotModeType::Running ||
        (safety_mode != SafetyModeType::Normal &&
        safety_mode != SafetyModeType::ReducedMode)) {
        markDisconnected(connection_generation_.load());
        return hardware_interface::return_type::OK;
    }
    if (!initialized_) {
        aubo_position_commands_ = actual_q_copy_;
        aubo_velocity_commands_.fill(0.0);
        initialized_ = true;
    }
    if (!connection_ready_.exchange(true)) {
        publishConnectionState(true);
        RCLCPP_INFO(node_->get_logger(), "AUBO connection recovered with fresh RTDE state");
    }

    return hardware_interface::return_type::OK;
}
hardware_interface::return_type AuboHardwareInterface::write(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    if (!connection_ready_.load()) {
        return hardware_interface::return_type::OK;
    }
    RobotModeType robot_mode;
    SafetyModeType safety_mode;
    {
        std::lock_guard<std::mutex> lock(rtde_mtx_);
        robot_mode = robot_mode_;
        safety_mode = safety_mode_;
    }
    if (robot_mode == RobotModeType::Running && (safety_mode ==
        SafetyModeType::Normal || safety_mode == SafetyModeType::ReducedMode)) {
        try {
            std::lock_guard<std::mutex> lock(client_mtx_);
            if (rpc_client_) {
                Servoj(aubo_position_commands_);
            }
        } catch (const std::exception &e) {
            markDisconnected(connection_generation_.load());
        }
    }else{
        // 机器人状态异常
        RCLCPP_WARN_STREAM(
            rclcpp::get_logger("AuboHardwareInterface"),
            "Robot not in valid state for motion command. Plz check&fix robot status firstly then restart driver"
            << "robot_mode_: " << static_cast<int>(robot_mode)
            << ", safety_mode_: " << static_cast<int>(safety_mode));

        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

void AuboHardwareInterface::readActualQ()
{
    // 使用 actual_q_copy_
    // 固定该时间戳下read到的位姿，否则读取到的关节状态不稳定
    // actual_q_copy_必须用 array 否则会 bad_alloc
    {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        actual_q_copy_[0] = actual_q_[0];
        actual_q_copy_[1] = actual_q_[1];
        actual_q_copy_[2] = actual_q_[2];
        actual_q_copy_[3] = actual_q_[3];
        actual_q_copy_[4] = actual_q_[4];
        actual_q_copy_[5] = actual_q_[5];

    //获取机械臂关节速度

        joint_velocity_copy_[0] = joint_velocity_[0];
        joint_velocity_copy_[1] = joint_velocity_[1];
        joint_velocity_copy_[2] = joint_velocity_[2];
        joint_velocity_copy_[3] = joint_velocity_[3];
        joint_velocity_copy_[4] = joint_velocity_[4];
        joint_velocity_copy_[5] = joint_velocity_[5];
    }
}
// 设置rtde输入

bool AuboHardwareInterface::isServoModeStart()
{
    return servo_mode_start_;
}
int AuboHardwareInterface::startServoMode()
{
    if (servo_mode_start_) {
        return 0;
    }
    // 接口调用 : 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();

    //开启servo模式
    rpc_client_->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoMode(true);
    int i = 0;
    while (!rpc_client_->getRobotInterface(robot_name)
                ->getMotionControl()
                ->isServoModeEnabled()) {
        if (i++ > 5) {
            std::cout << "Servo Mode enable fail! Servo Mode is "
                      << rpc_client_->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->isServoModeEnabled()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    servo_mode_start_ = true;
    return 0;
}

int AuboHardwareInterface::stopServoMode()
{
    if (!servo_mode_start_) {
        return 0;
    }
    // 接口调用 : 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();

    while (!rpc_client_->getRobotInterface(robot_name)
                ->getRobotState()
                ->isSteady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    // 关闭servo模式
    int i = 0;
    rpc_client_->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoMode(false);
    while (rpc_client_->getRobotInterface(robot_name)
               ->getMotionControl()
               ->isServoModeEnabled()) {
        if (i++ > 5) {
            std::cout << "Servo Mode disable fail! Servo Mode is "
                      << rpc_client_->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->isServoModeEnabled()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::cout << "Servoj end" << std::endl;
    servo_mode_start_ = false;
    return 0;
}

int AuboHardwareInterface::Servoj(
    const std::array<double, 6> joint_position_command)
{
    // 接口调用 : 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();

    std::vector<double> traj(6, 0);
    for (size_t i = 0; i < traj.size(); i++) {
        traj[i] = joint_position_command[i];
    }
    
    if(!rpc_client_->getRobotInterface(robot_name)
                ->getMotionControl()
                ->isServoModeEnabled()){
                
        rpc_client_->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoMode(true);           
    }
    // 接口调用: servoJoint
    while (true) {
        int servoJoint_num = rpc_client_->getRobotInterface(robot_name)
                                ->getMotionControl()
                                ->servoJoint(traj, 0.2, 0.2, 0.01, 0.1, 200);
        if(servoJoint_num != 2){
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}
// 设置rtde输入
void AuboHardwareInterface::setInput(RtdeClientPtr cli)
{
    // 接口调用: 发布
    // 组合设置输入
    int topic5 = cli->setTopic(
        true,
        { "input_bit_registers0_to_31", "input_bit_registers32_to_63",
          "input_bit_registers64_to_127", "input_int_registers_0" },
        1, 5);

    std::vector<int> value = { 0x00ff, 0x00, 0x00, 44 };
    cli->publish(
        5, [value](arcs::aubo_sdk::OutputBuilder &ro) { ro.push(value); });

    int topic6 = cli->setTopic(
        true, { "input_float_registers_0", "input_double_registers_1" }, 1, 6);

    std::vector<double> value2 = { 3.1, 4.1 };
    cli->publish(
        6, [value2](arcs::aubo_sdk::OutputBuilder &ro) { ro.push(value2); });
}
void AuboHardwareInterface::configSubscribe(
    RtdeClientPtr cli, std::uint64_t generation)
{
    // 接口调用: 设置 topic1
    int topic1 = cli->setTopic(
        false,
        { "R1_actual_q", "R1_actual_qd", "R1_robot_mode", "R1_safety_mode",
          "runtime_state", "line_number", "R1_actual_TCP_pose" },
        500, 0);
    // 接口调用: 订阅
    cli->subscribe(topic1, [this, generation](InputParser &parser) {
        if (generation != connection_generation_.load()) {
            return;
        }
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        auto actual_q = parser.popVectorDouble();
        auto joint_velocity = parser.popVectorDouble();
        if (actual_q.size() != 6 || joint_velocity.size() != 6) {
            return;
        }
        actual_q_ = std::move(actual_q);
        joint_velocity_ = std::move(joint_velocity);
        robot_mode_ = parser.popRobotModeType();
        safety_mode_ = parser.popSafetyModeType();
        runtime_state_ = parser.popRuntimeState();
        line_ = parser.popInt32();
        actual_TCP_pose_ = parser.popVectorDouble();
        last_rtde_sample_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
    });
}
} // namespace aubo_driver

PLUGINLIB_EXPORT_CLASS(aubo_driver::AuboHardwareInterface,
                       hardware_interface::SystemInterface)
