/**
 * @file velocity_publisher.cpp
 * @brief 速度发布节点 - 定期发布固定速度命令到/cmd_vel话题
 * 
 * 功能说明：
 * 1. 每500ms发布一次Twist消息
 * 2. 发送固定的线速度(0.5 m/s)和角速度(0.2 rad/s)
 * 3. 用于测试订阅者节点和理解发布-订阅模式
 * 
 * 学习要点：
 * - 如何创建Publisher
 * - 如何使用Timer定时触发回调
 * - 如何构造和发布geometry_msgs::msg::Twist消息
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

using namespace std::chrono_literals;  // 允许使用 500ms 这样的字面量

/**
 * @class VelocityPublisher
 * @brief 速度发布节点类
 * 
 * 继承自rclcpp::Node，实现了定时发布速度命令的功能
 */
class VelocityPublisher : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * 
     * 初始化节点、创建发布者和定时器
     */
    VelocityPublisher() : Node("velocity_publisher"), count_(0)
    {
        // 创建发布者
        // 参数1: 话题名称 "/cmd_vel" (velocity command的标准话题)
        // 参数2: QoS队列大小为10（如果发布速度过快，最多缓存10条消息）
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // 创建定时器，每500毫秒触发一次timer_callback函数
        // std::bind将成员函数绑定到this对象上
        timer_ = this->create_wall_timer(
            500ms,  // 等价于 std::chrono::milliseconds(500)
            std::bind(&VelocityPublisher::timer_callback, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "速度发布节点已启动，每500ms发布一次速度命令");
        RCLCPP_INFO(this->get_logger(), "发布话题: /cmd_vel");
    }

private:
    /**
     * @brief 定时器回调函数
     * 
     * 每500ms被调用一次，构造Twist消息并发布
     * 
     * Twist消息结构：
     * - linear.x: 前进/后退速度 (m/s)，正值前进，负值后退
     * - linear.y: 左右平移速度 (差分驱动机器人通常为0)
     * - linear.z: 上下速度 (地面机器人通常为0)
     * - angular.x: 绕X轴旋转 (翻滚角速度，地面机器人通常为0)
     * - angular.y: 绕Y轴旋转 (俯仰角速度，地面机器人通常为0)
     * - angular.z: 绕Z轴旋转 (转向角速度 rad/s)，正值左转，负值右转
     */
    void timer_callback()
    {
        // 创建Twist消息实例
        auto msg = geometry_msgs::msg::Twist();
        
        // 设置线速度 - 让机器人以0.5 m/s的速度前进
        msg.linear.x = 0.5;
        msg.linear.y = 0.0;  // 差分驱动机器人不能横向移动
        msg.linear.z = 0.0;  // 地面机器人不能垂直移动
        
        // 设置角速度 - 让机器人以0.2 rad/s的速度左转
        msg.angular.x = 0.0;  // 不翻滚
        msg.angular.y = 0.0;  // 不俯仰
        msg.angular.z = 0.2;  // 左转 (逆时针)
        
        // 发布消息到/cmd_vel话题
        publisher_->publish(msg);
        
        // 记录日志，显示发布的速度值
        count_++;
        RCLCPP_INFO(this->get_logger(), 
                    "[#%ld] 发布速度 -> linear.x: %.2f m/s, angular.z: %.2f rad/s", 
                    count_, msg.linear.x, msg.angular.z);
    }
    
    // 成员变量
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;  ///< 发布者智能指针
    rclcpp::TimerBase::SharedPtr timer_;  ///< 定时器智能指针
    size_t count_;  ///< 发布计数器
};

/**
 * @brief 主函数
 * 
 * ROS2节点的标准入口点
 * 
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 退出状态码
 */
int main(int argc, char * argv[])
{
    // 1. 初始化ROS2
    // 必须在使用任何ROS2功能之前调用
    rclcpp::init(argc, argv);
    
    // 2. 创建节点实例
    // 使用智能指针管理节点生命周期
    auto node = std::make_shared<VelocityPublisher>();
    
    // 3. 进入事件循环
    // spin()会阻塞，持续处理回调函数（定时器、订阅等）
    // 直到收到Ctrl+C信号或调用rclcpp::shutdown()
    RCLCPP_INFO(node->get_logger(), "开始运行，按Ctrl+C停止...");
    rclcpp::spin(node);
    
    // 4. 清理资源
    // 释放ROS2相关资源
    rclcpp::shutdown();
    
    return 0;
}
