#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"  //rclcpp/rclcpp.hpp 中包含了 大部分ROS2 模块
#include "tutorial_interfaces/msg/num.hpp"  // 用于发布数据的 类型（这里是 自定义的 interfaces）

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher():Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("topic", 10);
        // 初始化 发布器（publisher_）,其发布的 话题名称为"topic",并对 缓冲区消息队列限制为10个
        
        timer_ = this->create_wall_timer(5000ms, std::bind(&MinimalPublisher::timer_callback, this));
        // 初始化 timer_ ，500ms 调用一次回调函数 &MinimalPublisher::timer_callback
    }

  private:
    void timer_callback()
    {
      auto message = tutorial_interfaces::msg::Num();
      
      message.num = this->count_++;
      
      RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.num);
      //控制台显示打印消息
      
      publisher_->publish(message);
      //发布消息
    }

    rclcpp::TimerBase::SharedPtr timer_;
    //计时器，指定 间隔多长时间 调用 回调函数 
    
    rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_; 
    //发布tutorial_interfaces::msg::Num类型消息 的 发布器 
    
    size_t count_;
    //用于统计 消息 发布次数的变量
};

int main(int argc, char* argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::spin( std::make_shared<MinimalPublisher>() );
   rclcpp::shutdown();
   
   return 0;
}




