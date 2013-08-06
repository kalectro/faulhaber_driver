/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Robert Bosch LLC.
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
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
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
 *
 *********************************************************************/

//\Author Kai Franke

#include "motion_control_systems_driver/motion_control_systems_driver.h"

MotionControlSystemsDriver::MotionControlSystemsDriver( bosch_hardware_interface* hw, std::string mount_point, int frequency ): sensor_driver( hw )
{
  mount_point_ = mount_point;  
  frequency_ = frequency;
}

MotionControlSystemsDriver::~MotionControlSystemsDriver()
{
}

uint8_t MotionControlSystemsDriver::getDeviceAddress()
{
  // the answer to all questions and two less
  return 40;
}

bool MotionControlSystemsDriver::initialize()
{  
  // Initialize the hardware interface
  if( hardware_->initialize() == false )
  {
    ROS_ERROR("MotionControlSystemsDriver::initialize(): Could not initialize a hardware interface!");
    return false;
  }
  return true;
}

bool MotionControlSystemsDriver::enable()
{
  int flags[] = {0};
  uint8_t reg_address = 0;
  std::string command = mount_point_;
  
  command.append(":EN");
  uint8_t* data = reinterpret_cast<uint8_t*>(&command[0]);
  if( hardware_->write( this->getDeviceAddress(), RS232, frequency_, flags, reg_address, data, command.length() ) < 0 )
  {
    ROS_ERROR("MotionControlSystemsDriver::enable(): could not access serial interface");
    return false;
  } 

  return true;
}


