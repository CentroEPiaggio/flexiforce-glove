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
#define USE_USBCON

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

// necessary to compile code outside arduino IDE
#include <Arduino.h>

// use underscore for global variables

ros::NodeHandle nh_;

std_msgs::Int16MultiArray sensor_values_;
std_msgs::Int16MultiArray ports_;

bool isConfigured = false;
bool isInformed = false;

void configurePorts(const std_msgs::Int16MultiArray & ports)
{
  if(!isConfigured)
  {
    /*sensor_values_.data_length = (int)ports.data_length;
    ports_.data_length = (int)ports.data_length;
 
    // copy the port numbers
    for(int i = 0; i < ports.data_length; i++)
    {
      ports_.data[i] = (int)ports.data[i];
    }*/

    isConfigured = true;
    isInformed = true;
  }
}

ros::Publisher pub_sensor_values_("/flexiforce/raw_values", &sensor_values_);
ros::Subscriber<std_msgs::Int16MultiArray> sub_ports_("/flexiforce/connected_ports", &configurePorts);

void setup() 
{
  nh_.initNode();
  nh_.advertise(pub_sensor_values_);
  nh_.subscribe(sub_ports_);
  sensor_values_.data_length = 6;
}

void loop() 
{
  if(isConfigured)
  {
    // fill message once it is configured
    /*for(int i = 0; i < sensor_values_.data_length; i++)
    {
      sensor_values_.data[i] = analogRead( (unsigned int)ports_.data[i] );
    }
    */
    
    
    sensor_values_.data[0] = analogRead( 1 ); // thumb 
    sensor_values_.data[1] = analogRead( 3 ); // index
    sensor_values_.data[2] = analogRead( 5 ); // middle
    sensor_values_.data[3] = analogRead( 7 ); // ring
    sensor_values_.data[4] = analogRead( 9 ); // little
    sensor_values_.data[5] = analogRead( 0 ); // thumb abd
    // sensor_values_.data[6] = analogRead(  ); // little abd

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