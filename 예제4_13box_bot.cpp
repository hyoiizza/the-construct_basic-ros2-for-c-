#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/Odometry.hpp"
#include "std_msgs/msg/String.hpp"
#include "geometry_msgs/msg/Point.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include <cmath>
#include <sstream>

class box_bot_node : public rclcpp::Node
{
public:
	box_bot_node()
        : Node("box_bot_node"), goal1_(goal1_), goal2_(goal2_), goal3_(goal3_),
          current_pose1_(), current_pose2_(), current_pose3_()
	:Node("box_bot_node")
	{   
        //3개의 콜백 그룹 생성
		odom_callback_group_1 = this->create_callback_group(
			rclcpp::CallbackGroupType::MutuallyExclusive);
        odom_callback_group_2 = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        odom_callback_group_3 = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
                       
        //각 그룹에 대해 odometry subscriber 생성
        rclcpp::SubscriptionOptions options1;
        options1.callback_group = odom_callback_group_1;
        rclcpp::SubscriptionOptions options2;
        options2.callback_group = odom_callback_group_2;
        rclcpp::SubscriptionOptions options3;
        options3.callback_group = odom_callback_group_3;

		subscription1_= this->create_subscription<nav_msgs::msg::Odometry>(
			                "/box_bot_1/odom", //topic_name
			                10, //queue_size 
			                std::bind(&box_bot_node::odom1_callback, this, std::placeholders::_1),options1);
        subscription2_= this->create_subscription<nav_msgs::msg::Odometry>(
			                "/box_bot_2/odom", //topic_name
			                10, //queue_size 
			                std::bind(&box_bot_node::odom2_callback, this, std::placeholders::_1),options2);
        subscription3_= this->create_subscription<nav_msgs::msg::Odometry>(
			                "/box_bot_3/odom", //topic_name
			                10, //queue_size 
			                std::bind(&box_bot_node::odom3_callback, this, std::placeholders::_1),options3);
        
        //3개의 타이머 생성
        timer1_ = this->create_wall_timer(1000ms, std::bind(&box_bot_node::timer1_callback, this),odom_callback_group_1);
        timer2_ = this->create_wall_timer(1000ms, std::bind(&box_bot_node::timer2_callback, this),odom_callback_group_2);
        timer3_ = this->create_wall_timer(1000ms, std::bind(&box_bot_node::timer3_callback, this),odom_callback_group_3);

        //3개의 publisher 생성
		publisher1_ = this->create_publisher<std_msgs::msg::String>("reached_goal/box_bot_1",10);
        publisher2_ = this->create_publisher<std_msgs::msg::String>("reached_goal/box_bot_2",10);
        publisher3_ = this->create_publisher<std_msgs::msg::String>("reached_goal/box_bot_3",10);
	}
	
private:
	void odom1_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        current_pose1_ = msg->pose.pose;
	}
	void timer1_callback(){
        std_msgs::msg::String msg;
        std::ostringstream ss;
	    if (distance(goal1_, current_pose1_)<0.1){
	        ss <<"BOX BOT 1 REACHED GOAL =" << goal1_.x << ", " << goal1_.y;
            msg.data = ss.str();
        } else {
            ss << "BOX BOT 1 NOT REACHED GOAL =" << goal1_.x << ", " << goal1_.y;
            msg.data = ss.str();
        }
        RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
		publisher1_->publish(msg);
	}
    
    void odom2_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        current_pose2_ = msg->pose.pose;
	}
	void timer2_callback(){
        std_msgs::msg::String msg;
        std::ostringstream ss;
	    if (distance(goal2_ , current_pose2_)<0.1){
	        ss << "BOX BOT 2 REACHED GOAL =" << goal2_.x << ", " << goal2_.y;
            msg.data = ss.str();
        } else {
            ss << "BOX BOT 2 NOT REACHED GOAL =" << goal2_.x << ", " << goal2_.y;
            msg.data = ss.str();
        }   
        RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
		publisher2_->publish(msg);
	}

    void odom3_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose3_ = msg->pose.pose;
	}
	void timer3_callback(){
        std_msgs::msg::String msg;
        std::ostringstream ss;
	    if (distance(goal3_,current_pose3_)<0.1){
	        ss << "BOX BOT 3 REACHED GOAL =" << goal3_.x << ", " << goal3_.y;
            msg.data = ss.str();
        } else {
            ss << "BOX BOT 3 NOT REACHED GOAL =" << goal3_.x << ", " << goal3_.y;
            msg.data = ss.str();
        }   
        RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
		publisher3_->publish(msg);
	}

    double distance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b){
        return std::sqrt(std::pow(a.x-b.x,2)+std::pow(a.y-b.y,2));
    }

    //멤버변수 선언_(1) 각 bos_bot의 goal 좌표 
    geometry_msgs::msg::Point goal1_;
    geometry_msgs::msg::Point goal2_;
    geometry_msgs::msg::Point goal3_;
    //멤버변수 선언_(2) current pose
    geometry_msgs::msg::Point current_pose1_;
    geometry_msgs::msg::Point current_pose2_;
    geometry_msgs::msg::Point current_pose3_;
    //멤버 선언_(3) 콜백 그룹
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_1;
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_2;
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_3; 
    //멤버 선언_(4) subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription1_;    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription2_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription3_;
    //멤버 선언_(5) publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher1_;    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher2_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher3_;
    //멤버 선언_(6) 타이머
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::TimerBase::SharedPtr timer3_;  
};

	

//main함수 
int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    // Instantiate the Node
    geometry_msgs::msg::Point goal1_;
    geometry_msgs::msg::Point goal2_;
    geometry_msgs::msg::Point goal3_;

    goal1_.x = 2.498409;
    goal1_.y = -1.132045;

    goal2_.x = 0.974281;
    goal2_.y = -1.132045;

    goal3_.x = -0.507990;
    goal3_.y = -1.132045;


    std::shared_ptr<box_bot_node> box_bot_node = std::make_shared<box_bot_node>(const geometry_msgs::msg::Point goal1_,
                                                                                const geometry_msgs::msg::Point goal2_,
                                                                                const geometry_msgs::msg::Point goal3_);

    // Initialize one MultiThreadedExecutor object
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(box_bot_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
