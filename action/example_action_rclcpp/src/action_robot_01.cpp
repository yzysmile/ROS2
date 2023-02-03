# include "example_action_rclcpp/robot.h"
# include "rclcpp/rclcpp.hpp"
# include "rclcpp_action/rclcpp_action.hpp"
# include "robot_control_interfaces/action/move_robot.hpp"

// create a ActionSever Class
class ActionRobot01 : public rclcpp::Node {
 public:
  // action 接口（协议）
  using MoveRobot = robot_control_interfaces::action::MoveRobot;

  // Class to interact with goals on a server. 
  // Use this class to check the status of a goal as well as set the result.
  // This class is not meant to be created by a user, instead it is created when a goal has been accepted. 
  // A Server will create an instance and give it to the user in their handle_accepted callback.
  // 该类在接受 客户端的goal后 自动实例化 
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
    // 在终端 执行 ctrl+c 或 动作客户端 调用async_cancel_goal(...)
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveRobot> goal_handle){
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      robot.stop_move(); 
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 是客户端 实际执行goal的函数，被封装在 handle_accepted(...) 回调函数中
    void execute_move(const std::shared_ptr<GoalHandleMoveRobot> goal_handle) {
    // const std::shared_ptr< const typename ActionT::Goal > 	get_goal () const
    // Get the user provided message describing the goal.
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(this->get_logger(), "The robot start move %f ...", goal->distance);

    // result在 客户端 发送取消goal 或 在 goal执行完成时使用到，
    // 客户端调用 result_callback? 
    auto result = std::make_shared<MoveRobot::Result>();

    // Rate函数 来精准控制循环的周期，让其保持为2HZ
    rclcpp::Rate rate = rclcpp::Rate(2);

    robot.set_goal(goal->distance);
    
    // 采用了while循环的形式，获取机器人位置 并 发布反馈（publish_feedback）机器人的位置
    // rate 用于控制 获取机器人的位置 和 发布反馈的 频率
    // 同时检测任务是否被取消，如顺利执行完成则反馈最终结果。
    while (rclcpp::ok() && !robot.close_goal()) {
      robot.move_step();

      auto feedback = std::make_shared<MoveRobot::Feedback>();
      feedback->pose = robot.get_current_pose();
      feedback->status = robot.get_status();
      
      // 服务端发布 反馈， 客户端 执行 相应回调
      goal_handle->publish_feedback(feedback);

      // bool rclcpp_action::ServerGoalHandleBase::is_canceling	(		)	const
       // Indicate if client has requested this goal be cancelled.
       // true if a cancelation request has been accepted for this goal.
      if (goal_handle->is_canceling()) {
        result->pose = robot.get_current_pose();
        
      // void rclcpp_action::ServerGoalHandle< ActionT >::canceled(	typename ActionT::Result::SharedPtr result_msg	)	
        // Indicate that a goal has been canceled.
        // 传入实参：the final result to send to clients.
         // 服务端发布 结果， 客户端 执行 相应回调
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Publish Feedback"); /*Publish feedback*/
      rate.sleep();
    }

    result->pose = robot.get_current_pose();

    // void rclcpp_action::ServerGoalHandle< ActionT >::succeed	(	typename ActionT::Result::SharedPtr 	result_msg	)	
    // Indicate that a goal has succeeded.
     // 服务端发布 结果， 客户端 执行 相应回调
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
  }

  // 客户端发送 目标后，action_server_ 调用handle_goal()回调 若接受客户端目标并ACCEPT后，
  // 此时 rclcpp_action::ServerGoalHandle< ActionT >模板类对象 将会被自动创建， 
  // 并且传入 handle_accepted(...)作为该回调函数的实参  
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
