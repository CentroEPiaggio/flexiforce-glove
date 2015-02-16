/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Research Center "E. Piaggio"
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// STD headers
#include <vector>
#include <string>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16MultiArray.h>
 #include <std_msgs/Float64MultiArray.h>

// IMPORTANT TODO: avoid global variables, convert this in a class instead

// create the objects where to store the parameters
unsigned int n_sensors_;
unsigned int n_joints_;
unsigned int n_joints_total_;
XmlRpc::XmlRpcValue flexiforce_setup_;
std_msgs::Int16MultiArray msg_ports_;
std::vector<double> gains_;
std::vector<double> bias_;
std::vector<int> n_joints_at_sensor;
std::vector<double> joint_ratios_;
std::string hand_name_;

// publishers and subscribers
ros::Subscriber sub_flexiforce_raw_;
ros::Publisher pub_flexiforce_calibrated_;
ros::Publisher pub_joint_states_;
ros::Publisher pub_flexiforce_analog_ports_;

// and the data to be published
sensor_msgs::JointState joint_states_;
std_msgs::Float64MultiArray calibrated_data_;

// a flag to set the home position
bool isInit_ = false;

void parseParameters(const XmlRpc::XmlRpcValue &sensors)
{
  ROS_ASSERT(sensors.getType() == XmlRpc::XmlRpcValue::TypeArray);

  // resize all vectors containing the parameter information
  n_sensors_ = sensors.size();
  msg_ports_.data.resize(n_sensors_);
  msg_ports_.layout.dim.resize(n_sensors_);
  gains_.resize( n_sensors_ );
  bias_.resize( n_sensors_ );
  n_joints_at_sensor.resize( n_sensors_ );

  // resize also the data to be published
  calibrated_data_.data.resize( n_sensors_ );
  calibrated_data_.layout.dim.resize( n_sensors_ );

  n_joints_total_ = 0;

  for( int i = 0; i < n_sensors_; i++ )
  {
    XmlRpc::XmlRpcValue current_sensor = sensors[i];

    ROS_ASSERT(current_sensor.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    if( current_sensor.hasMember("name") )
    {
        ROS_ASSERT(current_sensor["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
        msg_ports_.layout.dim[i].label = std::string(current_sensor["name"]);
        calibrated_data_.layout.dim[i].label = std::string(current_sensor["name"]);
    }
    else
    {
        ROS_ERROR("No name value for the current sensor. Check the yaml configuration for this sensor");
        return;
    }

    if( current_sensor.hasMember("analog_port") )
    {
        ROS_ASSERT(current_sensor["analog_port"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        msg_ports_.data[i] = (int) current_sensor["analog_port"];     
    }
    else
    {
        ROS_ERROR("No analog port value for the current sensor. Check the yaml configuration for this sensor");
        return;
    }

    if( current_sensor.hasMember("gain") )
    {
        ROS_ASSERT(current_sensor["gain"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        gains_[i] = current_sensor["gain"];
        //std::cout << current_sensor["name"] << " gain " << gains_[i] << std::endl;
    }
    else
    {
        ROS_ERROR("No gain value for the current sensor. Check the yaml configuration for this sensor");
        return;
    }

    if( current_sensor.hasMember("bias") )
    {
        ROS_ASSERT(current_sensor["bias"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        bias_[i] = current_sensor["bias"];

        //std::cout << current_sensor["name"] << " bias " << bias_[i] << std::endl;
    }
    else
    {
        ROS_ERROR("No bias value for the current sensor. Check the yaml configuration for this sensor");
        return;
    }

    if( current_sensor.hasMember("joints") )
    {
        ROS_ASSERT(current_sensor["joints"].getType() == XmlRpc::XmlRpcValue::TypeArray);

        int n_joints = current_sensor["joints"].size();
        n_joints_at_sensor[i] = n_joints;

        for( int j = 0; j < n_joints; j++ )
        {
            XmlRpc::XmlRpcValue current_joint = current_sensor["joints"][j];

            if( current_joint.hasMember("name") )
            {
                ROS_ASSERT(current_joint["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
                joint_states_.name.push_back( hand_name_ + std::string("_") + std::string(current_joint["name"]) );                
            }
            else
            {
                ROS_ERROR("No name value for the current joint of the current sensor. Check the yaml configuration for this sensor");
                return;
            }

            if( current_joint.hasMember("ratio") )
            {
                ROS_ASSERT(current_joint["ratio"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                joint_ratios_.push_back( current_joint["ratio"] );
                //std::cout << joint_states_.name[j] << " ratio " << joint_ratios_[j] << std::endl;
            }
            else
            {
                ROS_ERROR("No ration value for the current joint of the current sensor. Check the yaml configuration for this sensor");
                return;
            }

            n_joints_total_++;
        }
    }
    else
    {
        ROS_ERROR("No joints value for the current sensor. Check the yaml configuration for this sensor");
        return;
    }
  }

  // and resize the joint states message with the amount of names
  std::cout << "n_joints_total_: " << n_joints_total_ << std::endl;
  joint_states_.position.resize( n_joints_total_ );
  joint_states_.velocity.resize( n_joints_total_ );
  joint_states_.effort.resize( n_joints_total_ );

  pub_flexiforce_analog_ports_.publish(msg_ports_);

  ROS_INFO("Succesfully parsed the flexiforce setup parameters and published the port configuration");
}

// calback function that publishes a calibrated values and a joint state message when a new raw value arrives
void publishJointStates(const std_msgs::Int16MultiArray &data)
{
    if( !(isInit_) )
    {
        for(int i = 0; i < n_sensors_; i++)
        {
            // set the current pose as the reference pose, tyipically, the hand extended   
            bias_[i] = bias_[i] - data.data[i]*gains_[i];
        }
        isInit_ = true;
    }
    else
    {
        int joint_counter = 0;
        for(int i = 0; i < n_sensors_; i++)
        {
            // first calibrate data     
            calibrated_data_.data[i] = data.data[i]*gains_[i] + bias_[i];

            // second compute joint angles
            for(int j = 0; j < n_joints_at_sensor[i]; j++)
            {            
                joint_states_.position[joint_counter] = calibrated_data_.data[i]*joint_ratios_[j];
                joint_counter++;
            }
        }
    }

    // and publish both, calibrated data and joint states 
    pub_flexiforce_calibrated_.publish(calibrated_data_);
    pub_joint_states_.publish(joint_states_);
}

int main(int argc, char** argv)
{
    // ros init
    ros::init(argc, argv, "flexiforce_joint_state_publisher");
    ros::NodeHandle nh_;

    // get the hand name to attach to the joint states names
    nh_.param<std::string>("hand_name", hand_name_, "soft_hand");

    // subscribe to the flexiforce raw values
    sub_flexiforce_raw_ = nh_.subscribe("/flexiforce/raw_values", 1000, publishJointStates);

    // advertise the calibrated and the joint states values
    pub_joint_states_ = nh_.advertise<sensor_msgs::JointState>("/flexiforce/joint_states", 1000);
    pub_flexiforce_calibrated_ = nh_.advertise<std_msgs::Float64MultiArray>("/flexiforce/calibrated_values", 1000);
    // latch the port configuration
    pub_flexiforce_analog_ports_ = nh_.advertise<std_msgs::Int16MultiArray>("/flexiforce/connected_ports", 1000); 

    // get the parameters form the rosparam server defined in the yaml file and store them
    nh_.getParam("flexiforce_sensors", flexiforce_setup_);
    parseParameters( flexiforce_setup_ );

    while(ros::ok())
    {
        // keep publishing the port configuration to guarantee the configuration
        pub_flexiforce_analog_ports_.publish(msg_ports_);
        ros::spinOnce();
    }

    return 0;
}
