#include "rclcpp/rclcpp.hpp"c
#include "tutorial_interfaces/srv/add_three_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_three_ints_client X Y Z");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_client");

  //the client is created in the node
  rclcpp::Client<tutorial_interfaces::srv::AddThreeInts>::SharedPtr client =
    node->create_client<tutorial_interfaces::srv::AddThreeInts>("add_three_ints");


  auto request = std::make_shared<tutorial_interfaces::srv::AddThreeInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);
  request->c = atoll(argv[3]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // the client node sends the request, corresponding server will response.So we run service node firstly before a client node sends request
  auto result = client->async_send_request(request);

  // Wait for the result
   // spin_until_future_complete(node, result) 首先spin这个node（确保这个node没有添加过执行器），
   // 且等待这个node发送request后得到的response
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } 

  else
   {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");
   }

  rclcpp::shutdown();
  return 0;
}

