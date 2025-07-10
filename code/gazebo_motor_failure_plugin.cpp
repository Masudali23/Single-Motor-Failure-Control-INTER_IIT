// #include <gazebo_motor_failure_plugin.h>

// namespace gazebo {

// GazeboMotorFailure::GazeboMotorFailure() :
//     ModelPlugin(),
//     ROS_motor_num_sub_topic_(kDefaultROSMotorNumSubTopic),
//     motor_failure_num_pub_topic_(kDefaultMotorFailureNumPubTopic)
// {
//     std::cout << "[gazebo_motor_failure_plugin]: GazeboMotorFailure constructor called." << std::endl;
// }

// GazeboMotorFailure::~GazeboMotorFailure() {
//     std::cout << "[gazebo_motor_failure_plugin]: Destructor called, cleaning up." << std::endl;
//     this->updateConnection_.reset();
// }

// void GazeboMotorFailure::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
//     std::cout << "[gazebo_motor_failure_plugin]: Loading plugin." << std::endl;

//     // Initialize ROS if it hasn't been initialized
//     if (!ros::isInitialized()) {
//         int argc = 0;
//         char** argv = nullptr;
//         ros::init(argc, argv, "gazebo_motor_failure_plugin", ros::init_options::NoSigintHandler);
//         std::cout << "[gazebo_motor_failure_plugin]: ROS initialized." << std::endl;
//     }

//     ros::NodeHandle nh;

//     this->namespace_.clear();
//     if (_sdf->HasElement("robotNamespace")) {
//         this->namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
//         std::cout << "[gazebo_motor_failure_plugin]: Namespace set to: " << this->namespace_ << std::endl;
//     }

//     node_handle_ = transport::NodePtr(new transport::Node());
//     node_handle_->Init(namespace_);
//     std::cout << "[gazebo_motor_failure_plugin]: Node initialized with namespace: " << namespace_ << std::endl;

//     motor_failure_pub_ = node_handle_->Advertise<msgs::Int>(motor_failure_num_pub_topic_, 1);
//     std::cout << "[gazebo_motor_failure_plugin]: Advertising to Gazebo topic: " << motor_failure_num_pub_topic_ << std::endl;

//     if (_sdf->HasElement("ROSMotorNumSubTopic")) {
//         this->ROS_motor_num_sub_topic_ = _sdf->GetElement("ROSMotorNumSubTopic")->Get<std::string>();
//         std::cout << "[gazebo_motor_failure_plugin]: ROS Motor Number Subscription Topic: " << ROS_motor_num_sub_topic_ << std::endl;
//     }

//     if (_sdf->HasElement("MotorFailureNumPubTopic")) {
//         this->motor_failure_num_pub_topic_ = _sdf->GetElement("MotorFailureNumPubTopic")->Get<std::string>();
//         std::cout << "[gazebo_motor_failure_plugin]: Motor Failure Number Publish Topic: " << motor_failure_num_pub_topic_ << std::endl;
//     }

//     // ROS Topic subscriber
//     motor_failure_sub_ = nh.subscribe(this->ROS_motor_num_sub_topic_, 10, &GazeboMotorFailure::motorFailNumCallBack, this);
//     std::cout << "[gazebo_motor_failure_plugin]: Subscribed to ROS topic: " << ROS_motor_num_sub_topic_ << std::endl;

//     // Listen to the update event.
//     this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
//         boost::bind(&GazeboMotorFailure::OnUpdate, this, _1));
//     std::cout << "[gazebo_motor_failure_plugin]: Connected to world update event." << std::endl;
// }

// void GazeboMotorFailure::OnUpdate(const common::UpdateInfo &info) {
//     std::cout << "[gazebo_motor_failure_plugin]: OnUpdate called, publishing motor failure number: " << motor_Failure_Number_ << std::endl;
//     this->motor_failure_msg_.set_data(motor_Failure_Number_);
//     this->motor_failure_pub_->Publish(motor_failure_msg_);
// }

// void GazeboMotorFailure::motorFailNumCallBack(const std_msgs::Int32::ConstPtr& msg) {
//     std::cout << "[gazebo_motor_failure_plugin]: motorFailNumCallBack received message: " << msg->data << std::endl;
//     this->motor_Failure_Number_ = msg->data;
// }

// GZ_REGISTER_MODEL_PLUGIN(GazeboMotorFailure);
// }






#include <iostream>
#include <stdio.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include <ros/ros.h>
#include <std_msgs/Int32.h>

namespace gazebo {

// Default values
static const std::string kDefaultROSMotorNumSubTopic = "/motor_failure/motor_number";
static const std::string kDefaultMotorFailureNumPubTopic = "/gazebo/motor_failure_num";

class GazeboMotorFailure : public ModelPlugin {
 public:
  GazeboMotorFailure() :
      ROS_motor_num_sub_topic_(kDefaultROSMotorNumSubTopic),
      motor_failure_num_pub_topic_(kDefaultMotorFailureNumPubTopic),
      motor_Failure_Number_(0)
  {
      std::cout << "[gazebo_motor_failure_plugin]: Constructor called." << std::endl;
  }

  virtual ~GazeboMotorFailure() {
      std::cout << "[gazebo_motor_failure_plugin]: Destructor called, cleaning up." << std::endl;
      this->updateConnection_.reset();
  }

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
      std::cout << "[gazebo_motor_failure_plugin]: Loading plugin." << std::endl;

      // Initialize ROS if it hasn't been initialized
      if (!ros::isInitialized()) {
          int argc = 0;
          char** argv = nullptr;
          ros::init(argc, argv, "gazebo_motor_failure_plugin", ros::init_options::NoSigintHandler);
          std::cout << "[gazebo_motor_failure_plugin]: ROS initialized." << std::endl;
      }

      ros::NodeHandle nh;

      // Read SDF parameters
      if (_sdf->HasElement("robotNamespace")) {
          this->namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
          std::cout << "[gazebo_motor_failure_plugin]: Namespace set to: " << this->namespace_ << std::endl;
      }

      if (_sdf->HasElement("ROSMotorNumSubTopic")) {
          this->ROS_motor_num_sub_topic_ = _sdf->GetElement("ROSMotorNumSubTopic")->Get<std::string>();
          std::cout << "[gazebo_motor_failure_plugin]: ROS Motor Number Subscription Topic: " << ROS_motor_num_sub_topic_ << std::endl;
      }

      if (_sdf->HasElement("MotorFailureNumPubTopic")) {
          this->motor_failure_num_pub_topic_ = _sdf->GetElement("MotorFailureNumPubTopic")->Get<std::string>();
          std::cout << "[gazebo_motor_failure_plugin]: Motor Failure Number Publish Topic: " << motor_failure_num_pub_topic_ << std::endl;
      }

      // Set up Gazebo transport node and publisher
      node_handle_ = transport::NodePtr(new transport::Node());
      node_handle_->Init(namespace_);
      std::cout << "[gazebo_motor_failure_plugin]: Node initialized with namespace: " << namespace_ << std::endl;

      motor_failure_pub_ = node_handle_->Advertise<msgs::Int>(motor_failure_num_pub_topic_, 1);
      std::cout << "[gazebo_motor_failure_plugin]: Advertising to Gazebo topic: " << motor_failure_num_pub_topic_ << std::endl;

      // ROS Topic subscriber
      motor_failure_sub_ = nh.subscribe(this->ROS_motor_num_sub_topic_, 10, &GazeboMotorFailure::motorFailNumCallBack, this);
      std::cout << "[gazebo_motor_failure_plugin]: Subscribed to ROS topic: " << ROS_motor_num_sub_topic_ << std::endl;

      // Listen to the update event
      this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboMotorFailure::OnUpdate, this, _1));
      std::cout << "[gazebo_motor_failure_plugin]: Connected to world update event." << std::endl;
  }

  void OnUpdate(const common::UpdateInfo &info) {
      // std::cout << "[gazebo_motor_failure_plugin]: OnUpdate called, publishing motor failure number: " << motor_Failure_Number_ << std::endl;
      this->motor_failure_msg_.set_data(motor_Failure_Number_);
      this->motor_failure_pub_->Publish(motor_failure_msg_);
  }

  void motorFailNumCallBack(const std_msgs::Int32::ConstPtr& msg) {
      std::cout << "[gazebo_motor_failure_plugin]: motorFailNumCallBack received message: " << msg->data << std::endl;
      this->motor_Failure_Number_ = msg->data;
  }

 private:
  event::ConnectionPtr updateConnection_;
  std::string ROS_motor_num_sub_topic_;
  std::string motor_failure_num_pub_topic_;
  std::string namespace_;
  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_failure_pub_;
  msgs::Int motor_failure_msg_;
  int32_t motor_Failure_Number_;
  ros::Subscriber motor_failure_sub_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboMotorFailure)

}  // namespace gazebo
