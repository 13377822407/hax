/**
 * @file velocity_subscriber.cpp
 * @brief 速度订阅节点 - 订阅/cmd_vel话题并打印速度信息
 * 
 * 功能说明：
 * 1. 订阅/cmd_vel话题的Twist消息
 * 2. 每收到一条消息就打印详细信息
 * 3. 计算并显示速度的大小（标量）
 * 
 * 学习要点：
 * - 如何创建Subscriber
 * - 如何处理回调函数
 * - 如何访问消息的字段
 * - 理解发布-订阅的异步通信模式
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>  // 用于sqrt函数

/**
 * @class VelocitySubscriber
 * @brief 速度订阅节点类
 * 
 * 订阅/cmd_vel话题，打印并分析接收到的速度命令
 */
class VelocitySubscriber : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * 
     * 初始化节点并创建订阅者
     */
    VelocitySubscriber() : Node("velocity_subscriber"), msg_count_(0)
    {
        // 创建订阅者
        // 参数1: 话题名称 "/cmd_vel"
        // 参数2: QoS队列大小为10
        // 参数3: 回调函数，使用std::bind绑定成员函数
        //        std::placeholders::_1 表示消息参数的占位符
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&VelocitySubscriber::topic_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "速度订阅节点已启动");
        RCLCPP_INFO(this->get_logger(), "订阅话题: /cmd_vel");
        RCLCPP_INFO(this->get_logger(), "等待消息...");
    }

private:
    /**
     * @brief 话题回调函数
     * 
     * 每当有新消息到达/cmd_vel话题时被调用
     * 
     * @param msg 接收到的Twist消息（智能指针）
     * 
     * 注意事项：
     * - 回调函数应该尽快执行完毕，避免阻塞消息处理
     * - 如果需要耗时操作，考虑使用多线程或异步处理
     * - 消息使用智能指针传递，无需手动释放内存
     */
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        msg_count_++;
        
        // 计算线速度的大小（向量的模）
        // |v| = sqrt(vx² + vy² + vz²)
        double linear_speed = std::sqrt(
            msg->linear.x * msg->linear.x +
            msg->linear.y * msg->linear.y +
            msg->linear.z * msg->linear.z
        );
        
        // 计算角速度的大小
        double angular_speed = std::sqrt(
            msg->angular.x * msg->angular.x +
            msg->angular.y * msg->angular.y +
            msg->angular.z * msg->angular.z
        );
        
        // 打印分隔线，使输出更清晰
        RCLCPP_INFO(this->get_logger(), "========== 消息 #%ld ==========", msg_count_);
        
        // 打印线速度分量
        RCLCPP_INFO(this->get_logger(), "线速度 (m/s):");
        RCLCPP_INFO(this->get_logger(), "  x (前后): %7.3f", msg->linear.x);
        RCLCPP_INFO(this->get_logger(), "  y (左右): %7.3f", msg->linear.y);
        RCLCPP_INFO(this->get_logger(), "  z (上下): %7.3f", msg->linear.z);
        RCLCPP_INFO(this->get_logger(), "  速度大小: %7.3f", linear_speed);
        
        // 打印角速度分量
        RCLCPP_INFO(this->get_logger(), "角速度 (rad/s):");
        RCLCPP_INFO(this->get_logger(), "  x (翻滚): %7.3f", msg->angular.x);
        RCLCPP_INFO(this->get_logger(), "  y (俯仰): %7.3f", msg->angular.y);
        RCLCPP_INFO(this->get_logger(), "  z (转向): %7.3f", msg->angular.z);
        RCLCPP_INFO(this->get_logger(), "  角速度大小: %7.3f", angular_speed);
        
        // 根据速度给出运动描述
        describe_motion(msg);
        
        RCLCPP_INFO(this->get_logger(), "=====================================\n");
    }
    
    /**
     * @brief 描述机器人运动状态
     * 
     * 根据速度值判断机器人的运动方向
     * 
     * @param msg Twist消息
     */
    void describe_motion(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::string motion_desc = "运动状态: ";
        
        // 判断线速度方向
        if (std::abs(msg->linear.x) > 0.01) {
            if (msg->linear.x > 0) {
                motion_desc += "前进 ";
            } else {
                motion_desc += "后退 ";
            }
        }
        
        if (std::abs(msg->linear.y) > 0.01) {
            if (msg->linear.y > 0) {
                motion_desc += "左移 ";
            } else {
                motion_desc += "右移 ";
            }
        }
        
        // 判断角速度方向
        if (std::abs(msg->angular.z) > 0.01) {
            if (msg->angular.z > 0) {
                motion_desc += "+ 左转";
            } else {
                motion_desc += "+ 右转";
            }
        }
        
        // 判断是否静止
        if (std::abs(msg->linear.x) < 0.01 && 
            std::abs(msg->linear.y) < 0.01 && 
            std::abs(msg->angular.z) < 0.01) {
            motion_desc += "静止";
        }
        
        RCLCPP_INFO(this->get_logger(), "%s", motion_desc.c_str());
    }
    
    // 成员变量
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;  ///< 订阅者智能指针
    size_t msg_count_;  ///< 接收消息计数器
};

/**
 * @brief 主函数
 * 
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 退出状态码
 */
int main(int argc, char * argv[])
{
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建订阅节点
    auto node = std::make_shared<VelocitySubscriber>();
    
    // 进入事件循环
    // spin()会持续等待并处理话题消息
    RCLCPP_INFO(node->get_logger(), "节点运行中，按Ctrl+C停止...");
    rclcpp::spin(node);
    
    // 清理资源
    rclcpp::shutdown();
    
    return 0;
}
