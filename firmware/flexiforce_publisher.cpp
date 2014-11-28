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

// in case it does not connect with the board, uncomment this
// taken from http://answers.ros.org/question/164191/rosserial-arduino-cant-connect-arduino-micro/
// #define USE_USBCON

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

// necessary to compile code outside arduino IDE
#include <Arduino.h>

// use underscore for global variables

ros::NodeHandle nh_;

std_msgs::Int16MultiArray sensor_values_;
std_msgs::Int16MultiArray ports_;
unsigned int n_sensors_;

bool isConfigured = false;
bool isInformed = false;

void configurePorts(const std_msgs::Int16MultiArray & ports)
{
  n_sensors_ = ports.data_length;
  sensor_values_.data_length = n_sensors_;
  ports_.data_length = n_sensors_;
  
  ports_.data = ports.data;

  isConfigured = true;
  isInformed = true;
}

ros::Publisher pub_sensor_values_("/flexiforce/flexiforce_raw_values", &sensor_values_);
ros::Subscriber<std_msgs::Int16MultiArray> sub_ports_("/flexiforce/connected_ports", configurePorts);

void setup() 
{
  nh_.initNode();
  nh_.advertise(pub_sensor_values_);
}

void loop() 
{
  if(isConfigured)
  {
    // fill message once it is configured
    for(int i = 0; i < n_sensors_; i++)
    {
      sensor_values_.data[i] = analogRead( ports_.data[i] );
    }

    // publish the message
    pub_sensor_values_.publish(&sensor_values_);
  }
  else
  {
    // inform the user what to do to configure the ports only once
    if(!isInformed)
    {
      //ROS_INFO("Arduino ports are not configured. Set the yaml file properly and launch the flexiforce_joint_state_publisher node");  
      isInformed = true;
    }

  }

  // spin and delay
  nh_.spinOnce();
  delay(100);
}