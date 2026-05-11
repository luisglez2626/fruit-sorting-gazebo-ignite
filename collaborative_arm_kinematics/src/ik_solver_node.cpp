// ==============================================================================
// File: ik_solver_node.cpp
// Purpose: C++ node that uses the Orocos Kinematics and Dynamics Library (KDL) 
//          to solve inverse kinematics. Updated with a multi-seed loop to 
//          prevent local minima failures when moving from awkward joint poses.
// ==============================================================================

#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/frames.hpp"

using std::placeholders::_1;

class IkSolverNode : public rclcpp::Node {
public:
  IkSolverNode() : Node("ik_solver_node") {
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_description", qos, std::bind(&IkSolverNode::urdf_callback, this, _1));

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&IkSolverNode::joint_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Waiting for robot blueprint to build KDL library chain...");
  }

private:
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (current_joints_.rows() < 6) return;
    
    std::vector<std::string> target_names = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };

    for (size_t i = 0; i < target_names.size(); ++i) {
        auto it = std::find(msg->name.begin(), msg->name.end(), target_names[i]);
        if (it != msg->name.end()) {
            size_t index = std::distance(msg->name.begin(), it);
            current_joints_(i) = msg->position[index];
        }
    }
  }

  void urdf_callback(const std_msgs::msg::String::SharedPtr msg) {
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromString(msg->data, my_tree)) {
      RCLCPP_ERROR(this->get_logger(), "Error: Failed to read robot blueprint. Check if Gazebo is running.");
      return;
    }

    if (!my_tree.getChain("world", "gripper_base", chain_)) {
      RCLCPP_ERROR(this->get_logger(), "Error: Failed to find the path from world to gripper_base.");
      return;
    }

    KDL::Segment tcp_segment("tcp_offset", KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0.0, 0.0, 0.045)));
    chain_.addSegment(tcp_segment);

    ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_);
    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
    
    current_joints_.resize(chain_.getNrOfJoints());
    KDL::SetToZero(current_joints_);

    RCLCPP_INFO(this->get_logger(), "Setup Complete! KDL solver ready. Listening to Python jogger...");

    target_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/target_position", 10, std::bind(&IkSolverNode::calculate_ik, this, _1));

    urdf_sub_.reset();
  }

  void calculate_ik(const geometry_msgs::msg::Pose::SharedPtr msg) {
    KDL::Rotation rot = KDL::Rotation::Quaternion(
        msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    KDL::Vector pos(msg->position.x, msg->position.y, msg->position.z);
    KDL::Frame target_frame(rot, pos);

    KDL::JntArray result_joints(chain_.getNrOfJoints());
    
    // Attempt 1: Try solving using the real current position as the seed
    int ret = ik_solver_->CartToJnt(current_joints_, target_frame, result_joints);

    // If Attempt 1 fails due to a mathematical local minimum, try multiple backup seeds
    if (ret < 0) {
      std::vector<KDL::JntArray> backup_seeds;
      
      KDL::JntArray home_seed(chain_.getNrOfJoints());
      KDL::SetToZero(home_seed);
      home_seed(1) = -1.5708; home_seed(2) = -1.5708; home_seed(3) = -1.5708; home_seed(4) = 1.5708;
      backup_seeds.push_back(home_seed);
      
      KDL::JntArray extended_seed = home_seed;
      extended_seed(2) = 0.0; // Elbow straight
      backup_seeds.push_back(extended_seed);

      KDL::JntArray left_seed = home_seed;
      left_seed(0) = 1.5708; // Base rotated left
      backup_seeds.push_back(left_seed);

      KDL::JntArray right_seed = home_seed;
      right_seed(0) = -1.5708; // Base rotated right
      backup_seeds.push_back(right_seed);

      // Loop through seeds until one successfully mathematically converges
      for(const auto& seed : backup_seeds) {
          ret = ik_solver_->CartToJnt(seed, target_frame, result_joints);
          if (ret >= 0) break;
      }
    }

    if (ret >= 0) {
      trajectory_msgs::msg::JointTrajectory traj_msg;
      traj_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

      trajectory_msgs::msg::JointTrajectoryPoint point;
      
      for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
        double current = current_joints_(i);
        double target = result_joints(i);
        
        // --- CRITICAL FIX: Joint Wrapping ---
        // Forces the raw mathematical solution to the closest physical equivalent angle.
        while (target - current > M_PI) target -= 2.0 * M_PI;
        while (target - current < -M_PI) target += 2.0 * M_PI;
        
        point.positions.push_back(target);
      }
      
      point.time_from_start.sec = 1;
      traj_msg.points.push_back(point);
      publisher_->publish(traj_msg);
      
    } else {
      RCLCPP_WARN(this->get_logger(), "Failure: The robot cannot physically stretch to this position.");
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;

  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  KDL::JntArray current_joints_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IkSolverNode>());
  rclcpp::shutdown();
  return 0;
}