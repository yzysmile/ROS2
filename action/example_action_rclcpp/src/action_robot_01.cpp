# include "example_action_rclcpp/robot.h"
# include "rclcpp/rclcpp.hpp"
# include "rclcpp_action/rclcpp_action.hpp"
# include "robot_control_interfaces/action/move_robot.hpp"

// create a ActionSever Class
class ActionRobot01 : public rclcpp::Node {
 public:
  // action 接口
  using MoveRobot = robot_control_interfaces::action::MoveRobot;

  // GoalHandleMoveRobot is a server of a specific action (is a template class)
  // 服务端, MoveRobot为action
  using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;

  explicit ActionRobot01(std::string name) : Node(name){
    RCLCPP_INFO(this->get_logger(), "node has been started: %s.", name.c_str());

    using namespace std::placeholders;

// template<typename ActionT, typename NodeT>
// Server<ActionT>::SharedPtr rclcpp_action::create_server(NodeT                                              node,
//                                                         const std::string &                                name,
//                                                         typename Server< ActionT >::GoalCallback           handle_goal,
//                                                         typename Server< ActionT >::CancelCallback         handle_cancel,
//                                                         typename Server< ActionT >::AcceptedCallback       handle_accepted,
//                                                         const rcl_action_server_options_t &                options = rcl_action_server_get_default_options(),
//                                                         rclcpp::CallbackGroup::SharedPtr                   group = nullptr
//                                                        )
// three callback func, aimed to deal with 'receive goal', 'receive cancel' and 'make sure accept and execute' 
    this->action_server_ = rclcpp_action::create_server<MoveRobot>(
        this, "move_robot",
        std::bind(&ActionRobot01::handle_goal, this, _1, _2),
        std::bind(&ActionRobot01::handle_cancel, this, _1),
        std::bind(&ActionRobot01::handle_accepted, this, _1));
  }

  private:
   Robot robot;

   rclcpp_action::Server<MoveRobot>::SharedPtr action_server_;

   // pass a goal share_ptr to the func,then return 'execute goal'(ACCEPT_AND_EXECUTE) or 'reject goal(REJECT)'
   rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const MoveRobot::Goal> goal){

      RCLCPP_INFO(this->get_logger(), "Received goal request with distance %f",
                  goal->distance);

      (void)uuid;

      if (fabs(goal->distance > 100)){
        RCLCPP_WARN(this->get_logger(), "The target is too far, the robot reject the order!");
        return rclcpp_action::GoalResponse::REJECT;
      }

      RCLCPP_INFO(this->get_logger(),"The target is %f,the robot can move there.", goal->distance);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // pass rclcpp_action::ServerGoalHandle<...> share_ptr to the func, 
    // then return ACCEPT or REJECT
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveRobot> goal_handle){
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      robot.stop_move(); 
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    
    void execute_move(const std::shared_ptr<GoalHandleMoveRobot> goal_handle) {
    // get_goal() 是 模板类 rclcpp_action::ServerGoalHandle<MoveRobot> 的 成员函数 
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(this->get_logger(), "The robot start move %f ...", goal->distance);

    auto result = std::make_shared<MoveRobot::Result>();
    // Rate函数 来精准控制循环的周期，让其保持为2HZ
    rclcpp::Rate rate = rclcpp::Rate(2);
    robot.set_goal(goal->distance);
    
    // 这里采用了while循环的形式，不断 调用机器人移动并获取机器人的位置，client自動調用feedback进行反馈。
    // 同时检测任务是否被取消，如顺利执行完成则反馈最终结果。
    while (rclcpp::ok() && !robot.close_goal()) {
      robot.move_step();

      auto feedback = std::make_shared<MoveRobot::Feedback>();
      feedback->pose = robot.get_current_pose();
      feedback->status = robot.get_status();

      // topic
      // publish_feedback() 是 模板类 rclcpp_action::ServerGoalHandle<MoveRobot> 的 成员函数 
      goal_handle->publish_feedback(feedback);

      /* detect the mission cancel or not */
       // is_canceling() 是 模板类 rclcpp_action::ServerGoalHandle<MoveRobot> 的 成员函数 
      if (goal_handle->is_canceling()) {
        result->pose = robot.get_current_pose();

        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Publish Feedback"); /*Publish feedback*/
      rate.sleep();
    }

    result->pose = robot.get_current_pose();
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
  }

  // 当handle_goal中对 移动请求 ACCEPT后 则进入到这里进行执行，这里是单独开了个线程进行执行execute_move函数，目的是避免阻塞主线程。
  void handle_accepted(const std::shared_ptr<GoalHandleMoveRobot> goal_handle) {
    using std::placeholders::_1;
    std::thread{std::bind(&ActionRobot01::execute_move, this, _1), goal_handle}
        .detach();
  }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<ActionRobot01>("action_robot_01");
    rclcpp::spin(action_server);
    rclcpp::shutdown();
    return 0;
}
