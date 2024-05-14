/*
  Autoware goal pose publisher node in carla simulatior
  map name : Town04
  node name : goal_pose_router_node 
  e-mail : yunsh3594@gmail.com 
  made by. Seunghoe Yun
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <queue>

class GoalPoseRouter : public rclcpp::Node
{
public:
  bool valid_flag;
  int location, pub_location;
  double position_x, position_y;
  double orientation_z, orientation_w;    
  double goal_pose_x[14] = {341.9141540527344, 383.7712097167969, 384.7696228027344, 328.87677001953125, 161.14590454101562,
                            -387.454345703125, -510.21832275390625, -312.4837951660156, -196.6580810546875, -14.273059844970703,
                            12.917856216430664, 13.349388122558594, 167.94473266601562, 310.6523742675781};
  double goal_pose_y[14] = {332.83349609375, 257.15118408203125, 107.71929931640625, -13.698135375976562, -10.666810035705566,
                            -9.019315719604492, -124.61986541748047, -432.527587890625, -432.0052795410156, -324.5279541015625,
                            -152.6903533935547, 246.3049774169922, 368.4743347167969, 353.96270751953125};
  double goal_orie_z[14] = {-0.3853504631140807, -0.6349668299787756, -0.7070555894847445, 0.9998992491529888, 0.9999885223521947,
                            -0.9999580361473548, -0.698101464202829, 0.008332526464908311, 0.003570403921171802, 0.5083907474984307,
                            0.6984328975497384, 0.6122161629496351, 0.02668937319478313, -0.24857647986934328};
  double goal_orie_w[14] = {0.9227702967574127, 0.7725394001775603, 0.7071579691825445, 0.014194771688521828, 0.0047911547537685464,
                            0.009161110430803768, 0.7159988447461813, 0.9999652838987519, 0.9999936260876064, 0.861126499335601,
                            0.7156755463338653, 0.7906904386820583, 0.9996437752311919, 0.968612272096408};
  std::queue<int> sub_queue;

  GoalPoseRouter() : Node("goal_pose_router")
  {
    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", 1);
        
    odm_sub_ = create_subscription<nav_msgs::msg::Odometry>( "/carla/ego_vehicle/odometry", rclcpp::QoS{1},
                std::bind(&GoalPoseRouter::callbackCarlaOdom, this, std::placeholders::_1));

    route_sub_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>( "/planning/mission_planning/route", rclcpp::QoS{1},
                std::bind(&GoalPoseRouter::callbackRoute, this, std::placeholders::_1));
    valid_flag = false;
    location = -1;
    pub_location = -1;
    position_x = 0.0;
    position_y = 0.0;
    orientation_z = 0.0;
    orientation_w = 0.0;
  }

private:
    void find_location(double x, double y){
      if (-165.0 < x && x <= 165.0 && 255.0 < y)
        location = 0;
      else if (165.0 < x && x <= 325.0 && 255.0 < y)
        location = 1;
      else if (325.0 < x && 255.0 < y)
        location = 2;
      else if (325.0 < x && 115.0 < y && y <= 255.0)
        location = 3;
      else if (325.0 < x && -165.0 < y && y <= 115.0)
        location = 4;
      else if (165.0 < x && x <= 325.0 && -165.0 < y && y <= 115.0)
        location = 5;
      else if (-385.0 < x && x <= -165.0 && -165.0 < y && y <= 115.0)
        location = 6;
      else if (x <= -385.0 && -165.0 < y && y <= 115.0)
        location = 7;
      else if (x <= -385.0 && -320.0 < y && y <= -165.0)
        location = 8;
      else if (x <= -385.0 && y <= -320.0)
        location = 9;
      else if (-385.0 < x && x <= -165.0 && y <= -320.0)
        location = 10;
      else if (-165.0 < x && x <= 165.0 && y <= -320.0)
        location = 11;
      else if (-165.0 < x && x <= 165.0 && -320.0 < y && y <= -165.0)
        location = 12;
      else if (-165.0 < x && x <= 165.0 && 115.0 < y && y <= 255.0)
        location = 13;
    }

    void callbackCarlaOdom(const nav_msgs::msg::Odometry::SharedPtr msg){
      find_location(msg->pose.pose.position.x, msg->pose.pose.position.y);
      
      if(location != -1)
        sub_queue.push(location);
      
      if (sub_queue.size() == 11){
        sub_queue.pop();

        std::queue<int> temp_queue = sub_queue;
        int queue_arr[14];

        memset(queue_arr, 0, sizeof(queue_arr));
        
        while(temp_queue.size() != 0){
          queue_arr[temp_queue.front()] += 1;
          temp_queue.pop();
        }

        for (int i=0; i<int(sizeof(queue_arr)/sizeof(int)); i++){
          if (queue_arr[i] >= sizeof(queue_arr)*0.7/sizeof(int) && pub_location != i){
            RCLCPP_WARN(get_logger(), "goal_pose switching  %d -> %d\n", pub_location, i);
            pub_location = i;
            pub_goal(pub_location);
            break;
          }
        }
      }
    }

    void callbackRoute(const autoware_planning_msgs::msg::LaneletRoute msg){
      if (location != -1){
        if (goal_pose_x[pub_location] - 0.001 > msg.goal_pose.position.x || msg.goal_pose.position.x > goal_pose_x[pub_location] + 0.001 ||
        goal_pose_y[pub_location] - 0.001 > msg.goal_pose.position.y || msg.goal_pose.position.y > goal_pose_y[pub_location] + 0.001 ||
        goal_orie_z[pub_location] - 0.001 > msg.goal_pose.orientation.z || msg.goal_pose.orientation.z > goal_orie_z[pub_location] + 0.001 ||
        goal_orie_w[pub_location] - 0.001 > msg.goal_pose.orientation.w || msg.goal_pose.orientation.w > goal_orie_w[pub_location] + 0.001){
          // RCLCPP_WARN(get_logger(),"pose false !!!\n");
          pub_goal(pub_location);
        }
        // else{
        //   RCLCPP_WARN(get_logger(),"pose true !!!\n");
        // }
      }
    }

    void pub_goal(int location_idx){
      auto goal_pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();

      goal_pose_msg->header.stamp = this->now();

      goal_pose_msg->header.frame_id = "map";

      goal_pose_msg->pose.position.x = goal_pose_x[location_idx];
      goal_pose_msg->pose.position.y = goal_pose_y[location_idx];
      goal_pose_msg->pose.position.z = 0.0;

      goal_pose_msg->pose.orientation.x = 0.0;
      goal_pose_msg->pose.orientation.y = 0.0;
      goal_pose_msg->pose.orientation.z = goal_orie_z[location_idx];
      goal_pose_msg->pose.orientation.w = goal_orie_w[location_idx];

      goal_pose_pub_->publish(*goal_pose_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odm_sub_;
    rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalPoseRouter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
