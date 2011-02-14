/* -*- mode: C++ -*- */
/* $Id: camera_info_manager.h 30292 2010-06-16 14:24:36Z joq $ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010 Jack O'Quin
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
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
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

#ifndef _CAMERA_INFO_MANAGER_H_
#define _CAMERA_INFO_MANAGER_H_

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

/** @file

    @brief CameraInfo Manager interface

    @author Jack O'Quin
 */

/** @brief CameraInfo Manager class

    Provides CameraInfo, handles the SetCameraInfo service requests,
    saves and restores sensor_msgs/CameraInfo data.  The calling node
    must invoke ros::spin() or ros::spinOnce() in some thread, so
    CameraInfoManager can handle arriving service requests.

    The location for getting and saving calibration data is expressed
    by Uniform Resource Locator. Some cameras are capable of saving
    these data in flash memory, others use a local file to keep track
    of calibration information.

    Example URL syntax:

    - file:///full/path/to/local/file.yaml

    - flash:///1 (not yet implemented)

    If the URL is empty, no calibration data are loaded, and any data
    provided via set_camera_info will be stored in:

    - file:///tmp/calibration_<camera_name>.yaml.

@par Services

 - @b set_camera_info (sensor_msgs/SetCameraInfo) to set calibration
   information

*/
class CameraInfoManager
{
 public:

  CameraInfoManager(ros::NodeHandle nh,
                    const std::string &cname="camera",
                    const std::string &url="");

  /** Returns the current CameraInfo data.
   *
   * The matrices are all zeros if no calibration was available. The
   * image pipeline handles that as uncalibrated data.
   *
   * @warning The caller must fill in the CameraInfo message Header.
   *          The time stamp and frame_id should normally be the same
   *          as the corresponding Image message Header fields.
   */
  sensor_msgs::CameraInfo getCameraInfo(void)
  {
    boost::mutex::scoped_lock lock_(mutex_);
    return cam_info_;
  }

  /** Returns true if the current CameraInfo is calibrated. */
  bool isCalibrated(void)
  {
    boost::mutex::scoped_lock lock_(mutex_);
    return (cam_info_.K[0] != 0.0);
  }

  bool loadCameraInfo(const std::string &url);
  bool setCameraName(const std::string &cname);
  bool validateURL(const std::string &url);

 private:

  // private methods
  bool loadCalibration(const std::string &url,
                       const std::string &cname);
  bool loadCalibrationFile(const std::string &filename,
                           const std::string &cname);
  bool saveCalibration(const sensor_msgs::CameraInfo &new_info,
                       const std::string &url,
                       const std::string &cname);
  bool saveCalibrationFile(const sensor_msgs::CameraInfo &new_info,
                           const std::string &filename,
                           const std::string &cname);
  bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req,
                     sensor_msgs::SetCameraInfo::Response &rsp);

  /** This non-recursive mutex is only held for a short time while
   *  accessing or changing private class variables.  To avoid
   *  deadlocks, it is never held during I/O or while invoking a
   *  callback.
   */
  boost::mutex mutex_;

  // private data
  ros::NodeHandle nh_;                  ///< node handle for service
  ros::ServiceServer info_service_;     ///< set_camera_info service
  std::string camera_name_;             ///< camera name
  std::string url_;                     ///< URL for calibration data
  sensor_msgs::CameraInfo cam_info_;    ///< current CameraInfo
};

#endif // _CAMERA_INFO_MANAGER_H_
