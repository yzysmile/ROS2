#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
   MinimalSubscriber():Node("minimal_subscriber")
   {
      subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>
      (
       "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1)
      );
      // 没有timer_ ，构造函数中自然不用初始化 timer_.
   }

  private:
  
    void topic_callback(const tutorial_interfaces::msg::Num::SharedPtr msg) const
    {
       RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->num);
    }
  
   rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;
   // 用于接受 tutorial_interfaces::msg::Num 消息类型 的 订阅器
   // 定阅器在任意时刻都响应 发布器 发布的消息
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  
  return 0;

}
