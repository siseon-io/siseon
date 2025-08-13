#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "arm_control_node/msg/cmd_pose.hpp"
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <map>
#include <string>
#include <vector>
#include <memory>
#include <cmath>

// --- 다이나믹셀 설정 ---
#define PORT_NAME           "/dev/ttyUSB0"
#define BAUDRATE            1000000
#define PROTOCOL_VERSION    2.0

// XM430 기준 Control Table 주소
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132
#define ADDR_MOVING_SPEED     112 // Profile Velocity (속도 제어)

// 0~4095 (4096 단위)
#define DXL_MINIMUM_POSITION_VALUE  0
#define DXL_MAXIMUM_POSITION_VALUE  4095

// 이동 속도 값 (단위: 0.229 rpm)
// 0 = 최대속도, 50 = 약 11.45rpm
#define DXL_MOVING_SPEED_VALUE      25 // 값을 50에서 25로 낮춰 속도를 줄입니다.


class ArmControlNode : public rclcpp::Node
{
public:
    ArmControlNode() : Node("arm_control_node")
    {
        // 조인트 이름과 다이나믹셀 ID 매핑
        joint_names_ = {"joint1", "joint2", "joint3", "joint4"};
        joint_to_id_ = {
            {"joint1", 11},
            {"joint2", 12},
            {"joint3", 13},
            {"joint4", 14}
        };

        // 다이나믹셀 SDK 핸들러 초기화
        portHandler_ = dynamixel::PortHandler::getPortHandler(PORT_NAME);
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (!setupDynamixel()) {
            RCLCPP_ERROR(this->get_logger(), "다이나믹셀 설정에 실패했습니다. 노드를 종료합니다.");
            rclcpp::shutdown();
            return;
        }

        // 퍼블리셔, 구독자, 타이머 생성
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        cmd_pose_subscriber_ = this->create_subscription<arm_control_node::msg::CmdPose>(
            "/cmd_pose", 10,
            std::bind(&ArmControlNode::cmd_pose_callback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ArmControlNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "🦾 다이나믹셀 직접 제어 노드가 시작되었습니다.");
    }

    ~ArmControlNode()
    {
        RCLCPP_INFO(this->get_logger(), "노드 종료... 모터 토크를 해제합니다.");
        for (const auto& pair : joint_to_id_) {
            packetHandler_->write1ByteTxRx(portHandler_, pair.second, ADDR_TORQUE_ENABLE, 0);
        }
        portHandler_->closePort();
    }

private:
    // ROS 관련 멤버
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<arm_control_node::msg::CmdPose>::SharedPtr cmd_pose_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 다이나믹셀 SDK 관련 멤버
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    std::map<std::string, int> joint_to_id_;
    std::vector<std::string> joint_names_;

    // 라디안 값을 다이나믹셀 위치 값(0-4095)으로 변환
    int convertRadianToDxl(double radian)
    {
        // [-PI, PI] 범위를 [0, 4095]로 매핑
        int dxl_position = static_cast<int>((radian + M_PI) / (2.0 * M_PI) * DXL_MAXIMUM_POSITION_VALUE);
        if (dxl_position < DXL_MINIMUM_POSITION_VALUE) return DXL_MINIMUM_POSITION_VALUE;
        if (dxl_position > DXL_MAXIMUM_POSITION_VALUE) return DXL_MAXIMUM_POSITION_VALUE;
        return dxl_position;
    }

    // 다이나믹셀 위치 값을 라디안 값으로 변환
    double convertDxlToRadian(int dxl_position)
    {
        return (static_cast<double>(dxl_position - DXL_MINIMUM_POSITION_VALUE) / DXL_MAXIMUM_POSITION_VALUE) * 2.0 * M_PI - M_PI;
    }

    // '/cmd_pose' 토픽 수신 시 호출되는 콜백 함수
    void cmd_pose_callback(const arm_control_node::msg::CmdPose::SharedPtr msg)
    {
        // --- Latency Calculation ---
        auto now = this->now();
        auto source_time = rclcpp::Time(msg->header.stamp);
        auto latency = now - source_time;
        RCLCPP_INFO(this->get_logger(), "Total Latency: %.2f ms", latency.seconds() * 1000.0);
        
        // RCLCPP_INFO(this->get_logger(), "Received /cmd_pose command");
        
        std::map<std::string, double> commands = {
            {"joint1", msg->m11}, {"joint2", msg->m12},
            {"joint3", msg->m13} //, {"joint4", msg->m14} // ID 14번 모터는 움직이지 않도록 주석 처리
        };

        for (const auto& cmd : commands) {
            int dxl_id = joint_to_id_[cmd.first];
            int dxl_goal_position = convertRadianToDxl(cmd.second);

            uint8_t dxl_error = 0;
            int dxl_comm_result = packetHandler_->write4ByteTxRx(
                portHandler_, dxl_id, ADDR_GOAL_POSITION, dxl_goal_position, &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "ID %d 통신 실패: %s", dxl_id, packetHandler_->getTxRxResult(dxl_comm_result));
            } else if (dxl_error != 0) {
                RCLCPP_ERROR(this->get_logger(), "ID %d 에러 발생: %s", dxl_id, packetHandler_->getRxPacketError(dxl_error));
            } else {
                // RCLCPP_INFO(this->get_logger(), "ID %d → %d (%.2f rad)", dxl_id, dxl_goal_position, cmd.second);
            }
        }
    }

    // 주기적으로 현재 로봇 상태를 읽어 'joint_states' 토픽으로 발행
    void timer_callback()
    {
        auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
        joint_state_msg->header.stamp = this->now();
        
        for(const auto& name : joint_names_) {
            int dxl_id = joint_to_id_[name];
            uint32_t present_position = 0;
            uint8_t dxl_error = 0;
            
            int dxl_comm_result = packetHandler_->read4ByteTxRx(
                portHandler_, dxl_id, ADDR_PRESENT_POSITION, &present_position, &dxl_error);

            if (dxl_comm_result == COMM_SUCCESS && dxl_error == 0) {
                joint_state_msg->name.push_back(name);
                joint_state_msg->position.push_back(convertDxlToRadian(present_position));
            }
        }
        
        if (!joint_state_msg->name.empty()) {
            joint_state_publisher_->publish(std::move(joint_state_msg));
        }
    }

    // 다이나믹셀 초기 설정 (포트 열기, 토크 켜기)
    bool setupDynamixel()
    {
        if (!portHandler_->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "포트 열기 실패: %s", PORT_NAME);
            return false;
        }
        if (!portHandler_->setBaudRate(BAUDRATE)) {
            RCLCPP_ERROR(this->get_logger(), "Baudrate 설정 실패: %d", BAUDRATE);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "포트 연결 성공: %s, Baudrate: %d", PORT_NAME, BAUDRATE);

        for (const auto& pair : joint_to_id_) {
            uint8_t dxl_error = 0;
            
            // 1. 토크 ON
            int dxl_comm_result = packetHandler_->write1ByteTxRx(
                portHandler_, pair.second, ADDR_TORQUE_ENABLE, 1, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "ID %d 토크 ON 통신 실패: %s", pair.second, packetHandler_->getTxRxResult(dxl_comm_result));
                return false;
            } else if (dxl_error != 0) {
                RCLCPP_ERROR(this->get_logger(), "ID %d 토크 ON 에러 발생: %s", pair.second, packetHandler_->getRxPacketError(dxl_error));
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "ID %d 토크 ON 성공", pair.second);

            // 2. 이동 속도 설정
            dxl_comm_result = packetHandler_->write4ByteTxRx( 
                portHandler_, pair.second, ADDR_MOVING_SPEED, DXL_MOVING_SPEED_VALUE, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "ID %d 속도 설정 통신 실패: %s", pair.second, packetHandler_->getTxRxResult(dxl_comm_result));
                return false;
            } else if (dxl_error != 0) {
                RCLCPP_ERROR(this->get_logger(), "ID %d 속도 설정 에러 발생: %s", pair.second, packetHandler_->getRxPacketError(dxl_error));
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "ID %d 속도 설정 성공: %d", pair.second, DXL_MOVING_SPEED_VALUE);
        }
        return true;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}