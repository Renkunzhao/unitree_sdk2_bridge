#include <array>
#include <iostream>
#include <mutex>
#include <stdint.h>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>

#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include <unitree_go/msg/wireless_controller.hpp>

using namespace unitree::common;
using namespace unitree::robot;

unitree_go::msg::IMUState IMUStateToRos(const unitree_go::msg::dds_::IMUState_ &imustate_dds) {
    unitree_go::msg::IMUState imustate_ros;

    imustate_ros.set__quaternion(imustate_dds.quaternion());
    imustate_ros.set__gyroscope(imustate_dds.gyroscope());
    imustate_ros.set__accelerometer(imustate_dds.accelerometer());
    imustate_ros.set__rpy(imustate_dds.rpy());
    imustate_ros.set__temperature(imustate_dds.temperature());

    return imustate_ros;
}

template <size_t N>
std::array<unitree_go::msg::MotorState, N> MotorStateToRos(
    const std::array<unitree_go::msg::dds_::MotorState_, N> &motorstate_dds)
{
    std::array<unitree_go::msg::MotorState, N> motorstate_ros;

    for (size_t i = 0; i < N; i++)
    {
        motorstate_ros[i].set__mode(motorstate_dds[i].mode());
        motorstate_ros[i].set__q(motorstate_dds[i].q());
        motorstate_ros[i].set__dq(motorstate_dds[i].dq());
        motorstate_ros[i].set__ddq(motorstate_dds[i].ddq());
        motorstate_ros[i].set__tau_est(motorstate_dds[i].tau_est());

        motorstate_ros[i].set__q_raw(motorstate_dds[i].q_raw());
        motorstate_ros[i].set__dq_raw(motorstate_dds[i].dq_raw());
        motorstate_ros[i].set__ddq_raw(motorstate_dds[i].ddq_raw());
        motorstate_ros[i].set__temperature(motorstate_dds[i].temperature());
        motorstate_ros[i].set__lost(motorstate_dds[i].lost());
        motorstate_ros[i].set__reserve(motorstate_dds[i].reserve());
    }

    return motorstate_ros;
}

unitree_go::msg::BmsState BmsStateToRos(const unitree_go::msg::dds_::BmsState_ &bmsstate_dds) {
    unitree_go::msg::BmsState bmsstate_ros;

    bmsstate_ros.set__version_high(bmsstate_dds.version_high());
    bmsstate_ros.set__version_low(bmsstate_dds.version_low());
    bmsstate_ros.set__soc(bmsstate_dds.soc());
    bmsstate_ros.set__current(bmsstate_dds.current());
    bmsstate_ros.set__cycle(bmsstate_dds.cycle());

    std::array<int8_t, 2> bq_ntc_arr;
    bq_ntc_arr[0] = bmsstate_dds.bq_ntc()[0];
    bq_ntc_arr[1] = bmsstate_dds.bq_ntc()[1];
    bmsstate_ros.set__bq_ntc(bq_ntc_arr);

    std::array<int8_t, 2> mcu_ntc_arr;
    mcu_ntc_arr[0] = bmsstate_dds.mcu_ntc()[0];
    mcu_ntc_arr[1] = bmsstate_dds.mcu_ntc()[1];
    bmsstate_ros.set__mcu_ntc(mcu_ntc_arr);

    bmsstate_ros.set__cell_vol(bmsstate_dds.cell_vol());

    return bmsstate_ros;
}

unitree_go::msg::LowState LowStateToRos(unitree_go::msg::dds_::LowState_ lowstate_dds){
    unitree_go::msg::LowState lowstate_ros;

    lowstate_ros.set__head(lowstate_dds.head());
    lowstate_ros.set__level_flag(lowstate_dds.level_flag());
    lowstate_ros.set__frame_reserve(lowstate_dds.frame_reserve());
    lowstate_ros.set__sn(lowstate_dds.sn());
    lowstate_ros.set__version(lowstate_dds.version());
    lowstate_ros.set__bandwidth(lowstate_dds.bandwidth());

    lowstate_ros.set__imu_state(IMUStateToRos(lowstate_dds.imu_state()));
    lowstate_ros.set__motor_state(MotorStateToRos(lowstate_dds.motor_state()));
    lowstate_ros.set__bms_state(BmsStateToRos(lowstate_dds.bms_state()));
    
    lowstate_ros.set__foot_force(lowstate_dds.foot_force());
    lowstate_ros.set__foot_force_est(lowstate_dds.foot_force_est());
    lowstate_ros.set__tick(lowstate_dds.tick());
    lowstate_ros.set__wireless_remote(lowstate_dds.wireless_remote());
    lowstate_ros.set__bit_flag(lowstate_dds.bit_flag());
    lowstate_ros.set__adc_reel(lowstate_dds.adc_reel());
    lowstate_ros.set__temperature_ntc1(lowstate_dds.temperature_ntc1());
    lowstate_ros.set__temperature_ntc2(lowstate_dds.temperature_ntc2());
    lowstate_ros.set__fan_frequency(lowstate_dds.fan_frequency());
    lowstate_ros.set__reserve(lowstate_dds.reserve());
    lowstate_ros.set__crc(lowstate_dds.crc());

    return lowstate_ros;
}

unitree_go::msg::LowCmd LowCmdToRos(unitree_go::msg::dds_::LowCmd_ lowcmd_dds){
    unitree_go::msg::LowCmd lowcmd_ros;
    return lowcmd_ros;
}

unitree_go::msg::SportModeState SportModeStateToRos(unitree_go::msg::dds_::SportModeState_ sportmodestate_dds){
    unitree_go::msg::SportModeState sportmodestate_ros;
    sportmodestate_ros.set__position(sportmodestate_dds.position());
    sportmodestate_ros.set__velocity(sportmodestate_dds.velocity());
    return sportmodestate_ros;
}

unitree_go::msg::WirelessController WirelessControllerToRos(unitree_go::msg::dds_::WirelessController_ wirelesscontroller_dds){
    unitree_go::msg::WirelessController wirelesscontroller_ros;
    return wirelesscontroller_ros;
}

class UnitreeSdk2BridgeNode : public rclcpp::Node{
public:
    UnitreeSdk2BridgeNode() : Node("unitree_sdk2_bridge_node")  {};
    ~UnitreeSdk2BridgeNode(){};
    void Init();

private:
    // LowState
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_dds_sub;
    unitree_go::msg::dds_::LowState_ lowstate_dds{}; // default init
    std::mutex lowstate_mtx;
    rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr lowstate_ros_pub;
    rclcpp::TimerBase::SharedPtr lowstate_timer;
    
    // LowCmd
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_dds_sub;
    void LowCmdMsgHandler(const void *msg) {
        std::lock_guard<std::mutex> lock(lowcmd_mtx);
        lowcmd_dds = *(unitree_go::msg::dds_::LowCmd_*)msg;
    }
    unitree_go::msg::dds_::LowCmd_ lowcmd_dds{}; // default init
    std::mutex lowcmd_mtx;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowcmd_ros_pub;
    rclcpp::TimerBase::SharedPtr lowcmd_timer;

    // SportModeState
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sportmodestate_dds_sub;
    void SportModeStateMsgHandler(const void *msg) {
        std::lock_guard<std::mutex> lock(sportmodestate_mtx);
        sportmodestate_dds = *(unitree_go::msg::dds_::SportModeState_*)msg;
    }
    unitree_go::msg::dds_::SportModeState_ sportmodestate_dds{}; // default init
    std::mutex sportmodestate_mtx;
    rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr sportmodestate_ros_pub;
    rclcpp::TimerBase::SharedPtr sportmodestate_timer;

    // WirelessController
    ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> wirelesscontroller_dds_sub;
    void WirelessControllerMsgHandler(const void *msg) {
        std::lock_guard<std::mutex> lock(wirelesscontroller_mtx);
        wirelesscontroller_dds = *(unitree_go::msg::dds_::WirelessController_*)msg;
    }
    unitree_go::msg::dds_::WirelessController_ wirelesscontroller_dds{}; // default init
    std::mutex wirelesscontroller_mtx;
    rclcpp::Publisher<unitree_go::msg::WirelessController>::SharedPtr wirelesscontroller_ros_pub;
    rclcpp::TimerBase::SharedPtr wirelesscontroller_timer;
};

void UnitreeSdk2BridgeNode::Init()
{
    // LowState
    lowstate_dds_sub.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lowstate"));
    lowstate_dds_sub->InitChannel(
        [this](const void *msg){
            std::lock_guard<std::mutex> lock(lowstate_mtx);
            lowstate_dds = *(unitree_go::msg::dds_::LowState_*)msg;
        }, 1);
    lowstate_ros_pub = this->create_publisher<unitree_go::msg::LowState>("/lowstate", 10);
    lowstate_timer = this->create_wall_timer(
        std::chrono::milliseconds(2),
        [this]() {
            std::lock_guard<std::mutex> lock(this->lowstate_mtx);
            lowstate_ros_pub->publish(LowStateToRos(this->lowstate_dds));
        });

    // LowCmd
    lowcmd_dds_sub.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>("rt/lowcmd"));
    lowcmd_dds_sub->InitChannel(
        [this](const void *msg){
            std::lock_guard<std::mutex> lock(lowcmd_mtx);
            lowcmd_dds = *(unitree_go::msg::dds_::LowCmd_*)msg;
        }, 1);
    lowcmd_ros_pub = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
    lowcmd_timer = this->create_wall_timer(
        std::chrono::milliseconds(2),
        [this]() {
            std::lock_guard<std::mutex> lock(this->lowcmd_mtx);
            lowcmd_ros_pub->publish(LowCmdToRos(this->lowcmd_dds));
        });

    // SportModeState
    sportmodestate_dds_sub.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>("rt/sportmodestate"));
    sportmodestate_dds_sub->InitChannel(
        [this](const void *msg){
            std::lock_guard<std::mutex> lock(sportmodestate_mtx);
            sportmodestate_dds = *(unitree_go::msg::dds_::SportModeState_*)msg;
        }, 1);
    sportmodestate_ros_pub = this->create_publisher<unitree_go::msg::SportModeState>("/sportmodestate", 10);
    sportmodestate_timer = this->create_wall_timer(
        std::chrono::milliseconds(2),
        [this]() {
            std::lock_guard<std::mutex> lock(this->sportmodestate_mtx);
            sportmodestate_ros_pub->publish(SportModeStateToRos(this->sportmodestate_dds));
        });

    // WirelessController
    wirelesscontroller_dds_sub.reset(new ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>("rt/wirelesscontroller"));
    wirelesscontroller_dds_sub->InitChannel(
        [this](const void *msg){
            std::lock_guard<std::mutex> lock(wirelesscontroller_mtx);
            wirelesscontroller_dds = *(unitree_go::msg::dds_::WirelessController_*)msg;
        }, 1);
    wirelesscontroller_ros_pub = this->create_publisher<unitree_go::msg::WirelessController>("/wirelesscontroller", 10);
    wirelesscontroller_timer = this->create_wall_timer(
        std::chrono::milliseconds(2),
        [this]() {
            std::lock_guard<std::mutex> lock(this->wirelesscontroller_mtx);
            wirelesscontroller_ros_pub->publish(WirelessControllerToRos(this->wirelesscontroller_dds));
        });
}

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        ChannelFactory::Instance()->Init(0, "lo");
        std::cout << "[UnitreeSdk2BridgeNode] Initialized on interface: lo" << std::endl;
    }
    else
    {
        ChannelFactory::Instance()->Init(0, argv[1]);
        std::cout << "[UnitreeSdk2BridgeNode] Initialized on interface: " << argv[1] << std::endl;
    }
    std::cout << "[UnitreeSdk2BridgeNode] Topic Forwarding.";

    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnitreeSdk2BridgeNode>();
    node->Init();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
