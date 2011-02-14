/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, 2010 Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

// TODO: doxygen mainpage

// TODO: Timeout receiving packet.
// TODO: Check that partial EOF missing frames get caught.
// @todo Do the triggering based on a stream of incoming timestamps.

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <driver_base/SensorLevels.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/publisher.h>
#include <self_test/self_test.h>
//#include <robot_mechanism_controllers/SetWaveform.h>
//#include <robot_mechanism_controllers/trigger_controller.h>
#include <stdlib.h>
#include <limits>
#include <math.h>
#include <linux/sysctl.h>
#include <iostream>
#include <fstream>
#include <wge100_camera/BoardConfig.h>
#include <boost/tokenizer.hpp>
#include <boost/format.hpp>
#include <driver_base/driver.h>
#include <driver_base/driver_node.h>
#include <camera_calibration_parsers/parse.h>
#include <wge100_camera/WGE100Camera_oldConfig.h>
#include <sstream>

#include "wge100_camera/wge100lib.h"
#include "wge100_camera/host_netutil.h"
#include "wge100_camera/mt9v.h"

#define RMEM_MAX_RECOMMENDED 20000000

// @todo this should come straight from camera_calibration_parsers as soon as a release happens.
struct SimpleMatrix
{
  int rows;
  int cols;
  const double* data;

  SimpleMatrix(int rows, int cols, const double* data)
    : rows(rows), cols(cols), data(data)
  {}
};

std::ostream& operator << (std::ostream& out, const SimpleMatrix& m)
{
  for (int i = 0; i < m.rows; ++i) {
    for (int j = 0; j < m.cols; ++j) {
      out << m.data[m.cols*i+j] << " ";
    }
    out << std::endl;
  }
  return out;
}

bool writeCalibrationIni(std::ostream& out, const std::string& camera_name,
                         const sensor_msgs::CameraInfo& cam_info)
{
  out.precision(5);
  out << std::fixed;
  
  out << "# Camera intrinsics\n\n";
  /// @todo time?
  out << "[image]\n\n";
  out << "width\n" << cam_info.width << "\n\n";
  out << "height\n" << cam_info.height << "\n\n";
  out << "[" << camera_name << "]\n\n";

  out << "camera matrix\n"     << SimpleMatrix(3, 3, &cam_info.K[0]);
  out << "\ndistortion\n"      << SimpleMatrix(1, 5, &cam_info.D[0]);
  out << "\n\nrectification\n" << SimpleMatrix(3, 3, &cam_info.R[0]);
  out << "\nprojection\n"      << SimpleMatrix(3, 4, &cam_info.P[0]);

  return true;
}

// The FrameTimeFilter class takes a stream of image arrival times that
// include time due to system load and network asynchrony, and generates a
// (hopefully) more steady stream of arrival times. The filtering is based
// on the assumption that the frame rate is known, and that often the time
// stamp is correct. The general idea of the algorithm is:
//
// anticipated_time_ = previous time stamp + frame_period_
// is a good estimate of the current frame time stamp.
//
// Take the incoming time stamp, or the anticipated_time_, whichever one is
// lower. The rationale here is that when the latency is low or zero, the
// incoming time stamp is correct and will dominate. If latency occurs, the
// anticipated_time_ will be used.
//
// To avoid problems with clock skew, max_skew indicates the maximum
// expected clock skew. frame_period_ is set to correspond to the 
// slowest expected rate when clock skew is taken into account. If
// frame_period_ was less than the actual frame rate, then
// anticipated_time_ would always dominate, and the output time stamps
// would slowly diverge from the actual time stamps.
//
// Because the frame rate may sometimes skip around in an unanticipated way
// the filter detects if anticipated_time_ has dominated by more than
// locked_threshold_ more than max_recovery_frames_ in a row. In that case,
// it picks the lowest latency input value that occurs during the
// max_recovery_frames_ to reset anticipated_time_.
//
// Finally, if the filter misses too many frames in a row, it assumes that
// its anticipated_time is invalid and immediately resets the filter.

class FrameTimeFilter
{
public:  
  FrameTimeFilter(double frame_rate = 1., double late_locked_threshold = 1e-3, double early_locked_threshold = 3e-3, int early_recovery_frames = 5, int max_recovery_frames = 10, double max_skew = 1.001, double max_skipped_frames = 100)
  {
    frame_period_ = max_skew / frame_rate;
    max_recovery_frames_ = max_recovery_frames;
    early_recovery_frames_ = early_recovery_frames;
    late_locked_threshold_ = late_locked_threshold;
    early_locked_threshold_ = early_locked_threshold;
    max_skipped_frames_ = max_skipped_frames;
    last_frame_number_ = 0; // Don't really care about this value.
    reset_filter();
  }
  
  void reset_filter()
  {
    relock_time_ = anticipated_time_ = std::numeric_limits<double>::infinity();
    unlocked_count_ = late_locked_threshold_;
  }

  double run(double in_time, int frame_number)
  {
    double out_time = in_time;
    int delta = (frame_number - last_frame_number_) & 0xFFFF; // Hack because the frame rate currently wraps around too early.

    if (delta > max_skipped_frames_)
    {
      ROS_WARN("FrameTimeFilter missed too many frames from #%u to #%u. Resetting.", last_frame_number_, frame_number);
      reset_filter();
    }

    anticipated_time_ += frame_period_ * delta; 
    relock_time_ += frame_period_ * delta;

    if (out_time < relock_time_)
      relock_time_ = out_time;
      
    //ROS_DEBUG("in: %f ant: %f", in_time, anticipated_time_);

    bool early_trigger = false;
    if (out_time < anticipated_time_ - early_locked_threshold_ && finite(anticipated_time_))
    {
      ROS_WARN("FrameTimeFilter saw frame #%u was early by %f.", frame_number, anticipated_time_ - out_time);
      early_trigger = true;
    }
    if (out_time < anticipated_time_)
    {
      anticipated_time_ = out_time;
      relock_time_ = std::numeric_limits<double>::infinity();
      unlocked_count_ = 0;
    }
    else if (out_time > anticipated_time_ + late_locked_threshold_)
    {
      unlocked_count_++;
      if (unlocked_count_ > max_recovery_frames_)
      {
        ROS_WARN("FrameTimeFilter lost lock at frame #%u, shifting by %f.", frame_number, relock_time_ - anticipated_time_);
        anticipated_time_ = relock_time_;
        relock_time_ = std::numeric_limits<double>::infinity();
      }
      //else 
      //  ROS_DEBUG("FrameTimeFilter losing lock at frame #%u, lock %i/%i, off by %f.", frame_number, unlocked_count_, max_recovery_frames_, anticipated_time_ - out_time);
      out_time = anticipated_time_;
    }
    
    last_frame_number_ = frame_number;

    return early_trigger ? -out_time : out_time;
  }
  
private:  
  int max_recovery_frames_;
  int unlocked_count_;
  int last_frame_number_;
  int max_skipped_frames_;
  int early_recovery_frames_;
  double early_locked_threshold_;
  double late_locked_threshold_;
  double anticipated_time_;
  double relock_time_;
  double frame_period_;
};

class WGE100CameraDriver : public driver_base::Driver
{
  friend class WGE100CameraNode;

public:
  wge100_camera::WGE100Camera_oldConfig config_;

private:
  // Driver classes
  ros::NodeHandle node_handle_; /// @todo this should end up being eliminated.
  
  // Services
  ros::ServiceClient trig_service_;
  ros::ServiceServer config_bord_service_;
  ros::ServiceServer set_camera_info_service_;

  // Video mode
  int video_mode_;
  unsigned int width_;
  unsigned int height_;
  double imager_freq_;
  double desired_freq_;
  
  // Statistics
  int rmem_max_;
  int missed_eof_count_;
  int dropped_packets_;
  int massive_frame_losses_;
  bool dropped_packet_event_;
  int missed_line_count_;
  double last_image_time_;
  unsigned int last_frame_number_;
  int lost_image_thread_count_;
 
  // Timing
  int misfire_blank_;
  FrameTimeFilter frameTimeFilter_;

  // Link information
  sockaddr localMac_;
  in_addr localIp_;
  
  // Camera information
  std::string hwinfo_;
  IpCamList camera_;
  double last_camera_ok_time_;

  // Trigger information
  std::string trig_controller_cmd_;
//  controller::trigger_configuration trig_req_;
//  robot_mechanism_controllers::SetWaveform::Response trig_rsp_;
  
  // Threads
  boost::shared_ptr<boost::thread> image_thread_;

  // Frame function (gets called when a frame arrives).
  typedef boost::function<int(size_t, size_t, uint8_t*, ros::Time)> UseFrameFunction;
  UseFrameFunction useFrame_;
  
public:
  typedef wge100_camera::WGE100Camera_oldConfig Config;
  
  WGE100CameraDriver()
  {
    misfire_blank_ = 0;
    
    // Create a new IpCamList to hold the camera list
//    wge100CamListInit(&camList);
    
    // Clear statistics
    last_camera_ok_time_ = 0;
    last_image_time_ = 0;
    missed_line_count_ = 0;
    missed_eof_count_ = 0;
    dropped_packets_ = 0;
    lost_image_thread_count_ = 0;
    massive_frame_losses_ = 0;
    dropped_packet_event_ = false;
  }
  
  void config_update(const Config &new_config, int32_t level = 0)
  {
    config_ = new_config;

    for (video_mode_ = 0; video_mode_ < MT9V_NUM_MODES; video_mode_++)
      if (config_.video_mode.compare(MT9VModes[video_mode_].name) == 0)
        break;
    if (video_mode_ == MT9V_NUM_MODES) 
    {
      ROS_ERROR("Unknown video mode %s. Using 640x480x30", config_.video_mode.c_str());
      video_mode_ = 2;
      config_.video_mode = "640x480x30";
    }
  
    /// @todo add a check that width_ and height_ are set in camera_info in test bench
    width_ = MT9VModes[video_mode_].width;
    height_ = MT9VModes[video_mode_].height;
    imager_freq_ = MT9VModes[video_mode_].fps;

    // Distinguish between camera_.serial and config_.camera_.serial so
    // that when auto is selected, we will try to restart the previous
    // camera before probing again.

    desired_freq_ = config_.ext_trig ? config_.trig_rate : imager_freq_;
    
    if (!config_.auto_exposure && config_.exposure > 1 / desired_freq_)
    {
      ROS_WARN("Exposure (%f s) is greater frame period (%f s). Setting to 90%% of frame period.", 
          config_.exposure, 1 / desired_freq_);
      config_.exposure = 0.9 * 1 / desired_freq_;
    }
    if (!config_.auto_exposure && config_.exposure > 0.95 / desired_freq_)
    {
      ROS_WARN("Exposure (%f s) is greater than 95%% of frame period (%f s). You may miss frames.", 
          config_.exposure, 0.95 / desired_freq_);
    }
  }

  ~WGE100CameraDriver()
  {
//    wge100CamListDelAll(&camList);
    close();
  }

  int get_rmem_max()
  {
    int rmem_max;
    {
      std::ifstream f("/proc/sys/net/core/rmem_max");
      f >> rmem_max;
    }
    return rmem_max;
  }

  void doOpen()
  {
    assert(state_ == CLOSED);
    ROS_DEBUG("open()");
    
    int retval;
    
    // Check that rmem_max is large enough.
    rmem_max_ = get_rmem_max();
    if (rmem_max_ < RMEM_MAX_RECOMMENDED)
    {
      ROS_WARN("rmem_max is %i. Buffer overflows and packet loss may occur. Minimum recommended value is 20000000. Updates may not take effect until the driver is restarted. See http://pr.willowgarage.com/wiki/errors/Dropped_Frames_and_rmem_max for details.", rmem_max_);
    }

    double thresh_time = ros::Time::now().toSec() - 10;
    double last_time = last_image_time_ < last_camera_ok_time_ ? 
      last_camera_ok_time_ : last_image_time_;
    if (last_time < thresh_time)
    { // Haven't heard from the camera in a while, redo discovery.
      ROS_DEBUG("Redoing discovery.");
      const char *errmsg;
      int findResult = wge100FindByUrl(config_.camera_url.c_str(), &camera_, SEC_TO_USEC(0.2), &errmsg);

      if (findResult)
      {
        ROS_ERROR("Matching URL %s : %s", config_.camera_url.c_str(), errmsg);
        strncpy(camera_.cam_name, "unknown", sizeof(camera_.cam_name));
        camera_.serial = -1;
        return;
      }
      retval = wge100Configure(&camera_, camera_.ip_str, SEC_TO_USEC(0.5));
    }
    else
      retval = wge100Configure(&camera_, NULL, SEC_TO_USEC(0.5));

    retval = wge100Configure(&camera_, camera_.ip_str, SEC_TO_USEC(0.5));
    if (retval != 0) {
      if (retval == ERR_CONFIG_ARPFAIL) {
        ROS_WARN("Unable to create ARP entry (are you root?), continuing anyway");
      } else {
        ROS_FATAL("IP address configuration failed");
        return;
      }
    }
    camera_.serial = camera_.serial;
    hwinfo_ = camera_.hwinfo;

    ROS_INFO("Configured camera. Complete URLs: serial://%u@%s#%s name://%s@%s#%s",
             camera_.serial, camera_.ip_str, camera_.ifName, camera_.cam_name, camera_.ip_str, camera_.ifName);
      
    // We are going to receive the video on this host, so we need our own MAC address
    if ( wge100EthGetLocalMac(camera_.ifName, &localMac_) != 0 ) {
      ROS_FATAL("Unable to get local MAC address for interface %s", camera_.ifName);
      return;
    }

    // We also need our local IP address
    if ( wge100IpGetLocalAddr(camera_.ifName, &localIp_) != 0) {
      ROS_FATAL("Unable to get local IP address for interface %s", camera_.ifName);
      return;
    }
      
    // Select trigger mode.
    if ( wge100TriggerControl( &camera_, config_.ext_trig ? TRIG_STATE_EXTERNAL : TRIG_STATE_INTERNAL ) != 0) {
      ROS_FATAL("Trigger mode set error. Is %s accessible from interface %s? (Try running route to check.)", camera_.ip_str, camera_.ifName);
      return;
    }

    if (config_.ext_trig)
    {
      // Configure the triggering controller
      if (!config_.trig_controller.empty())
      {
        ROS_INFO("Configuring controller \"%s\" for triggering.", config_.trig_controller.c_str());
        trig_controller_cmd_ = config_.trig_controller + "/set_waveform";

        ROS_DEBUG("Setting trigger.");
//        trig_req_.running = 1;
//        trig_req_.rep_rate = config_.trig_rate; 
//        trig_req_.phase = config_.trig_phase;
//        trig_req_.active_low = 0;
//        trig_req_.pulsed = 1;
//        trig_req_.duty_cycle = 0; // Unused in pulsed mode.

//        trig_service_ = node_handle_.serviceClient<robot_mechanism_controllers::SetWaveform>(trig_controller_cmd_);
        // Retry a few times in case the service is just coming up.
      }
      else
      {
        ROS_WARN("External triggering is selected, but no \"trigger_controller\" was specified.");
      }
    }

    // Select a video mode
    if ( wge100ImagerModeSelect( &camera_, video_mode_ ) != 0) {
      ROS_FATAL("Mode select error");
      //node_handle_.shutdown();
      return;
    }

    uint16_t imager_version;
    if (wge100ReliableSensorRead(&camera_, 0x00, &imager_version, NULL))
    {
      ROS_ERROR("Unable to read imager version.");
      return;
    }
    bool is_027 = (camera_.serial / 100000) == 27;
    bool is_MT9V032 = imager_version != 0x1324;
    const char *imager_model = is_MT9V032 ? "MT9V032" : "MT9V034";
    if (camera_.serial != 0 && is_027 == is_MT9V032)
    {
      boost::recursive_mutex::scoped_lock lock(mutex_);
      setStatusMessagef("Camera has %s serial number, but has %s imager. This is a serious problem with the hardware.", 
          is_027 ? "027" : "non-027", imager_model);
      return;
    }
    
    if (wge100ReliableSensorWrite(&camera_, 0x18, 0x3e3a, NULL) || 
        wge100ReliableSensorWrite(&camera_, 0x15, 0x7f32, NULL) ||
        wge100ReliableSensorWrite(&camera_, 0x20, 0x01c1, NULL) ||
        wge100ReliableSensorWrite(&camera_, 0x21, 0x0018, NULL))
    {
      ROS_WARN("Error setting magic sensor register.");
    }
    if (wge100ReliableSensorWrite(&camera_, 0x70, 0x14, NULL) != 0)
    {
      ROS_WARN("Error turning off row-noise correction");
    }

    int bin_size = width_ > 320 ? 1 : 2;
    // Set horizontal blanking
    if ( wge100ReliableSensorWrite( &camera_, MT9V_REG_HORIZONTAL_BLANKING, MT9VModes[video_mode_].hblank * bin_size, NULL ) != 0)
    {
      ROS_WARN("Error setting horizontal blanking.");
    }

    if ( wge100ReliableSensorWrite( &camera_, MT9V_REG_VERTICAL_BLANKING, MT9VModes[video_mode_].vblank, NULL) != 0)
    {
      ROS_WARN("Error setting vertical blanking.");
    }

    /*
    // Set maximum course shutter width
    if ( wge100ReliableSensorWrite( camera_, 0xBD, 240, NULL ) != 0) {
      ROS_FATAL("Sensor write error");
      node_handle_.shutdown();
      return;
    }
    */

    if (wge100ReliableSensorWrite(&camera_, MT9V_REG_AGC_AEC_ENABLE, config_.auto_gain * 2 + config_.auto_exposure, NULL) != 0)
    {
      ROS_WARN("Error setting AGC/AEC mode. Exposure and gain may be incorrect.");
    }

    if (!config_.auto_gain)
    {
      if ( wge100ReliableSensorWrite( &camera_, MT9V_REG_ANALOG_GAIN, config_.gain, NULL) != 0)
      {
        ROS_WARN("Error setting analog gain.");
      }
    }

    if (!config_.auto_exposure)
    {
      uint16_t hblank = MT9VModes[video_mode_].hblank; 
      uint16_t hpix = width_ * bin_size; 
      double line_time = (hpix + hblank) / MT9V_CK_FREQ;
      ROS_DEBUG("Line time is %f microseconds. (hpix = %i, hblank = %i)", line_time * 1e6, hpix, hblank);
      int explines = config_.exposure / line_time;
      if (explines < 1) /// @TODO warning here?
        explines = 1;
      if (explines > 32767) /// @TODO warning here?
        explines = 32767;
      ROS_DEBUG("Setting exposure lines to %i.", explines);
      if ( wge100ReliableSensorWrite( &camera_, MT9V_REG_TOTAL_SHUTTER_WIDTH, explines, NULL) != 0)
      {
        ROS_WARN("Error setting exposure.");
      }
    }

    if ( wge100ReliableSensorWrite( &camera_, MT9V_COMPANDING_MODE, config_.companding ? 3 : 2, NULL) != 0)
    {
      ROS_WARN("Error setting companding mode.");
    }

    // Initialize frame time filter.
    frameTimeFilter_ = FrameTimeFilter(desired_freq_, 0.001, 0.5 / imager_freq_); 
    
    config_bord_service_ = ros::NodeHandle("~").advertiseService("board_config", &WGE100CameraDriver::boardConfig, this);
    set_camera_info_service_ = ros::NodeHandle("~").advertiseService("set_camera_info", &WGE100CameraDriver::setCameraInfo, this);
    
    state_ = OPENED;
    last_camera_ok_time_ = ros::Time::now().toSec(); // If we get here we are communicating with the camera well.
  }

  void doClose()
  {
    ROS_DEBUG("close()");
    assert(state_ == OPENED);
    config_bord_service_ = ros::ServiceServer();
    state_ = CLOSED;
  }

  void doStart()
  {
    ROS_DEBUG("start()");
    assert(state_ == OPENED);
    state_ = RUNNING;   
    image_thread_.reset(new boost::thread(boost::bind(&WGE100CameraDriver::imageThread, this)));
  }

  void doStop()
  {
    ROS_DEBUG("stop()");
    if (state_ != RUNNING) // RUNNING can exit asynchronously.
      return;
    
    /// @todo race condition here. Need to think more.

    state_ = OPENED;

    if (image_thread_ && !image_thread_->timed_join((boost::posix_time::milliseconds) 2000))
    {
      ROS_ERROR("image_thread_ did not die after two seconds. Pretending that it did. This is probably a bad sign.");
      lost_image_thread_count_++;
    }
    image_thread_.reset();
  }
  
  int setTestMode(uint16_t mode, diagnostic_updater::DiagnosticStatusWrapper &status)
  {
    if ( wge100ReliableSensorWrite( &camera_, 0x7F, mode, NULL ) != 0) {
      status.summary(2, "Could not set imager into test mode.");
      status.add("Writing imager test mode", "Fail");
      return 1;
    }
    else
    {
      status.add("Writing imager test mode", "Pass");
    }

    usleep(100000);
    uint16_t inmode;
    if ( wge100ReliableSensorRead( &camera_, 0x7F, &inmode, NULL ) != 0) {
      status.summary(2, "Could not read imager mode back.");
      status.add("Reading imager test mode", "Fail");
      return 1;
    }
    else
    {
      status.add("Reading imager test mode", "Pass");
    }
    
    if (inmode != mode) {
      status.summary(2, "Imager test mode read back did not match.");
      status.addf("Comparing read back value", "Fail (%04x != %04x)", inmode, mode);
      return 1;
    }
    else
    {
      status.add("Comparing read back value", "Pass");
    }
    
    return 0;
  }

  std::string getID()
  {
    if (camera_.serial)
      return "unknown";
    else
      return (boost::format("WGE100_%05u") % camera_.serial).str();
  }

private:
  void imageThread()
  {
    // Start video
    if (!trig_controller_cmd_.empty())
    {
//      trig_req_.running = 1;
//      if (!trig_service_.call(trig_req_, trig_rsp_))
//      {
//        ROS_DEBUG("Could not start trigger on first attempt. Sleeping and retrying.");
//        sleep(3); // Perhaps the trigger is just being brought up.
//        if (!trig_service_.call(trig_req_, trig_rsp_))
//        {
//          ROS_ERROR("Unable to set trigger controller.");
//          //node_handle_.shutdown();
//          goto end_image_thread;
//        }
//      }
    }
    frameTimeFilter_.reset_filter();
    ROS_INFO("Camera streaming.");
  
/*    if ( wge100StartVid( camera_, (uint8_t *)&(localMac_.sa_data[0]), inet_ntoa(localIp_), port) != 0 ) 
    {
      ROS_FATAL("Could not start camera streaming.");
      boost::mutex::scoped_lock lock(mutex_);
      state_ = OPENED;
      return;
    }
    // Receive video
    wge100VidReceive(camera_.ifName, port, height_, width_, &WGE100CameraDriver::frameHandler, this);*/
    
    wge100VidReceiveAuto( &camera_, height_, width_, &WGE100CameraDriver::frameHandler, this);
    
    /*// Stop Triggering
    if (!trig_controller_cmd_.empty())
    {   
      ROS_DEBUG("Stopping triggering.");
      trig_req_.running = 0;
      /// @todo need to turn on a node in the case where the node is
      //already down.
      //ros::Node shutdown_node("wge100_camera_node", ros::Node::ANONYMOUS_NAME | ros::Node::DONT_ADD_ROSOUT_APPENDER); // Need this because the node has been shutdown already
      if (!trig_service_.call(trig_req_, trig_rsp_))
      { // This probably means that the trigger controller was turned off,
        // so we don't really care.
        ROS_DEBUG("Was not able to stop triggering.");
      }
    }  */
/*stop_video:
    // Stop video
    boost::mutex::scoped_lock lock(mutex_);
    if (state_ == RUNNING)
    {
      state_ = OPENED;
      ROS_ERROR("Image thread exited unexpectedly.");
      
      if ( wge100StopVid(camera_) == 0 )
        ROS_ERROR("Video should have been stopped"); /// @todo get rid of this once things have stabilized.
    }
    else // Exited expectedly.
      if ( wge100StopVid(camera_) != 0)
        ROS_ERROR("Video Stop error");*/
    
//end_image_thread:
    state_ = OPENED; // FIXME This should be done with a held lock.
    ROS_DEBUG("Image thread exiting.");
  }

  void linkStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    if (ros::Time::now().toSec() - last_image_time_ > 5 / desired_freq_)
    {
      stat.summary(2, "Next frame is past due.");
    }
    else if (state_ != RUNNING)
    {
      stat.summary(1, "Frames are not streaming.");
    }
    else if (rmem_max_ < RMEM_MAX_RECOMMENDED)
    {
      stat.summaryf(1, "Frames are streaming, but the receive buffer is small (rmem_max=%u). Please increase rmem_max to %u to avoid dropped frames. For details: http://pr.willowgarage.com/wiki/errors/Dropped_Frames_and_rmem_max", rmem_max_, RMEM_MAX_RECOMMENDED);
    }
    else if (dropped_packet_event_)
    {
      stat.summary(1, "There have been dropped packets.");
      dropped_packet_event_ = 0;
    }
    else
    {
      stat.summary(0, "Frames are streaming.");
    }
    
    stat.addf("Missing image line frames", "%d", missed_line_count_);
    stat.addf("Missing EOF frames", "%d", missed_eof_count_);
    stat.addf("Dropped packet estimate", "%d", dropped_packets_);
    stat.addf("Massive frame losses", "%d", massive_frame_losses_);
    stat.addf("Losses of image thread", "%d", lost_image_thread_count_);
    stat.addf("First packet offset", "%d", config_.first_packet_offset);
    if (isClosed())
    {
      static const std::string not_opened = "not_opened";
      stat.add("Camera Serial #", not_opened);
      stat.add("Camera Name", not_opened);
      stat.add("Camera Hardware", not_opened);
      stat.add("Camera MAC", not_opened);
      stat.add("Interface", not_opened);
      stat.add("Camera IP", not_opened);
    }
    else
    {
      stat.add("Camera Serial #", camera_.serial);
      stat.add("Camera Name", camera_.cam_name);
      stat.add("Camera Hardware", hwinfo_);
      stat.addf("Camera MAC", "%02X:%02X:%02X:%02X:%02X:%02X", camera_.mac[0], camera_.mac[1], camera_.mac[2], camera_.mac[3], camera_.mac[4], camera_.mac[5]);
      stat.add("Interface", camera_.ifName);
      stat.add("Camera IP", camera_.ip_str);
    }
    stat.add("Image mode", config_.video_mode);
    stat.addf("Latest frame time", "%f", last_image_time_);
    stat.add("Latest frame #", last_frame_number_);
    stat.addf("Free-running frequency", "%f", imager_freq_);
    stat.add("External trigger controller", config_.trig_controller);
    stat.add("Trigger mode", config_.ext_trig ? "external" : "internal");
  }

  double getTriggeredFrameTime(double firstPacketTime)
  {
    /*// Assuming that there was no delay in receiving the first packet,
    // this should tell us the time at which the first trigger after the
    // image exposure occurred.
    double pulseStartTime = 0;
      controller::TriggerController::getTickStartTimeSec(firstPacketTime, trig_req_);

    // We can now compute when the exposure ended. By offsetting to the
    // falling edge of the pulse, going back one pulse duration, and going
    // forward one camera frame time.
    double exposeEndTime = pulseStartTime + 
      controller::TriggerController::getTickDurationSec(trig_req_) +
      1 / imager_freq_ -
      1 / config_.trig_rate;
    
    return exposeEndTime;*/
    return 0;
  }

  double getExternallyTriggeredFrameTime(double firstPacketTime)
  {
    // We can now compute when the exposure ended. By offsetting by the
    // empirically determined first packet offset, going back one pulse
    // duration, and going forward one camera frame time.
    
    double exposeEndTime = firstPacketTime - config_.first_packet_offset + 
      1 / imager_freq_ - 
      1 / config_.trig_rate;

    return exposeEndTime;
  }

  double getFreeRunningFrameTime(double firstPacketTime)
  {
    // This offset is empirical, but fits quite nicely most of the time.
    
    return firstPacketTime - config_.first_packet_offset;
  }
  
//    double nowTime = ros::Time::now().toSec();
//    static double lastExposeEndTime;
//    static double lastStartTime;
//    if (fabs(exposeEndTime - lastExposeEndTime - 1 / trig_rate_) > 1e-4) 
//      ROS_INFO("Mistimed frame #%u first %f first-pulse %f now-first %f pulse-end %f end-last %f first-last %f", eofInfo->header.frame_number, frameStartTime, frameStartTime - pulseStartTime, nowTime - pulseStartTime, pulseStartTime - exposeEndTime, exposeEndTime - lastExposeEndTime, frameStartTime - lastStartTime);
//    lastStartTime = frameStartTime;
//    lastExposeEndTime = exposeEndTime;
//    
//    static double lastImageTime;
//    static double firstFrameTime;
//    if (eofInfo->header.frame_number == 100)
//      firstFrameTime = imageTime;
//    ROS_INFO("Frame #%u time %f ofs %f ms delta %f Hz %f", eofInfo->header.frame_number, imageTime, 1000 * (imageTime - firstFrameTime - 1. / (29.5/* * 0.9999767*/) * (eofInfo->header.frame_number - 100)), imageTime - lastImageTime, 1. / (imageTime - lastImageTime));
//    lastImageTime = imageTime;
  
  int frameHandler(wge100FrameInfo *frame_info)
  {
    boost::recursive_mutex::scoped_lock(diagnostics_lock_);
    
    if (frame_info == NULL)
    {
      // The select call in the driver timed out.
      /// @TODO Do we want to support rates less than 1 Hz?
      ROS_WARN("No data have arrived for more than one second. Assuming that camera is no longer streaming.");
      state_ = OPENED;
    }

    if (!node_handle_.ok() && state_ == RUNNING)
      state_ = OPENED;

    if (state_ != RUNNING)
      return 1;

    // If we are not in triggered mode then use the arrival time of the
    // first packet as the image time.

    double unfilteredImageTime;
    double frameStartTime = frame_info->startTime.tv_sec + frame_info->startTime.tv_usec / 1e6;
    
    if (!config_.trig_controller.empty())
      unfilteredImageTime = getTriggeredFrameTime(frameStartTime);
    else if (config_.ext_trig)
      unfilteredImageTime = getExternallyTriggeredFrameTime(frameStartTime);
    else
      unfilteredImageTime = getFreeRunningFrameTime(frameStartTime);

    //static double lastunfiltered;
    //ROS_DEBUG("first_packet %f unfilteredImageTime %f frame_id %u deltaunf: %f", frameStartTime, unfilteredImageTime, frame_info->frame_number, unfilteredImageTime - lastunfiltered);
    //lastunfiltered = unfilteredImageTime;
    
    double imageTime = frameTimeFilter_.run(unfilteredImageTime, frame_info->frame_number);
    if (imageTime < 0) // Signals an early frame arrival.
    {
      imageTime = -imageTime;
      if (!config_.trig_controller.empty())
        misfire_blank_ = 1 + imager_freq_ / (imager_freq_ - desired_freq_);
    }

    //ROS_DEBUG("imageTime %f", imageTime);
    
    int16_t frame_delta = (frame_info->frame_number - last_frame_number_) & 0xFFFF;
    if (frame_delta > 1)
    {
      dropped_packet_event_ = true;
      ROS_WARN("Some frames were never seen. The dropped packet count will be incorrect.");
      massive_frame_losses_++;
    }

    // Check for frame with missing EOF.
    if (frame_info->eofInfo == NULL) {
      // We no longer use the eofInfo.
      missed_eof_count_++;
      dropped_packets_++;
      dropped_packet_event_++;
      ROS_WARN("Frame %u was missing EOF", frame_info->frame_number);
    }

    // Check for short packet (video lines were missing)
    if (frame_info->shortFrame) {
      missed_line_count_++;
      dropped_packets_ += frame_info->missingLines;
      dropped_packet_event_ = true;
      ROS_WARN("Short frame #%u (%zu video lines were missing, last was %zu)", frame_info->frame_number, 
          frame_info->missingLines, frame_info->lastMissingLine);
      return 0;
    }

    if (misfire_blank_ > 0)
    {
      ROS_WARN("Dropping frame #%u because of mistrigger event.", frame_info->frame_number);
      misfire_blank_--;
    }

    last_image_time_ = imageTime;
    last_frame_number_ = frame_info->frame_number;
    
    if (useFrame_(frame_info->width, frame_info->height, frame_info->frameData, ros::Time(imageTime)))
    {
      boost::recursive_mutex::scoped_lock lock(mutex_);
      if (state_ == RUNNING)
        state_ = OPENED;                       
      return 1;
    }

    return 0;
  }

  static int frameHandler(wge100FrameInfo *frameInfo, void *userData)
  {
    WGE100CameraDriver &fa_node = *(WGE100CameraDriver*)userData;
    return fa_node.frameHandler(frameInfo);
  }

  uint16_t intrinsicsChecksum(uint16_t *data, int words)
  {
    uint16_t sum = 0;
    for (int i = 0; i < words; i++)
      sum += htons(data[i]);
    return htons(0xFFFF - sum);
  }

  bool loadIntrinsics(sensor_msgs::CameraInfo &cam_info)
  {
    // Retrieve contents of user memory
    std::string calbuff(2 * FLASH_PAGE_SIZE, 0);

    if(wge100ReliableFlashRead(&camera_, FLASH_CALIBRATION_PAGENO, (uint8_t *) &calbuff[0], NULL) != 0 ||
        wge100ReliableFlashRead(&camera_, FLASH_CALIBRATION_PAGENO+1, (uint8_t *) &calbuff[FLASH_PAGE_SIZE], NULL) != 0)
    {
      ROS_ERROR("Error reading camera intrinsics.\n");
      return false;
    }

    uint16_t chk = intrinsicsChecksum((uint16_t *) &calbuff[0], calbuff.length() / 2);
    if (chk)
    {
      ROS_WARN("Camera intrinsics have a bad checksum. They have probably never been set.\n");
    }                                             

    std::string name;
    bool success = camera_calibration_parsers::parseCalibration(calbuff, "ini", name, cam_info);
    // @todo: rescale intrinsics as needed?
    if (success && (cam_info.width != width_ || cam_info.height != height_)) {
      ROS_WARN("Image resolution from intrinsics file does not match current video setting, "
               "intrinsics expect %ix%i but running at %ix%i", cam_info.width, cam_info.height, width_, height_);
    }
    return success;
  }

  bool saveIntrinsics(const sensor_msgs::CameraInfo &cam_info)
  {
    char calbuff[2 * FLASH_PAGE_SIZE];

    std::stringstream inifile;
    writeCalibrationIni(inifile, camera_.cam_name, cam_info);
    strncpy(calbuff, inifile.str().c_str(), 2 * FLASH_PAGE_SIZE - 2);

    uint16_t *chk = ((uint16_t *) calbuff) + FLASH_PAGE_SIZE - 1;
    *chk = 0;
    *chk = intrinsicsChecksum((uint16_t *) calbuff, sizeof(calbuff) / 2);

    ROS_INFO("Writing camera info:\n%s", calbuff);
    
    // Retrieve contents of user memory

    if(wge100ReliableFlashWrite(&camera_, FLASH_CALIBRATION_PAGENO, (uint8_t *) calbuff, NULL) != 0 ||
        wge100ReliableFlashWrite(&camera_, FLASH_CALIBRATION_PAGENO+1, (uint8_t *) calbuff + FLASH_PAGE_SIZE, NULL) != 0)
    {
      ROS_ERROR("Error writing camera intrinsics to flash.\n");
      return false;
    }
    else   
      ROS_INFO("Camera_info successfully written to camera.");

    return true;
  }

  bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp)
  {
    sensor_msgs::CameraInfo &cam_info = req.camera_info;
    
    if (state_ != RUNNING)
    {
      ROS_ERROR("set_camera_info service called while camera was not running. To avoid programming the wrong camera, set_camera_info can only be called while viewing the camera stream.");
      rsp.status_message = "Camera not running. Camera must be running to avoid setting the intrinsics of the wrong camera.";
      rsp.success = false;
      return 1;
    }

    if ((cam_info.width != width_ || cam_info.height != height_)) {
      ROS_ERROR("Image resolution of camera_info passed to set_camera_info service does not match current video setting, "
               "camera_info contains %ix%i but camera running at %ix%i", cam_info.width, cam_info.height, width_, height_);
      rsp.status_message = (boost::format("Camera_info resolution %ix%i does not match camera resolution %ix%i.")%cam_info.width%cam_info.height%width_%height_).str();
      rsp.success = false;
    }
    
    stop();
    rsp.success = saveIntrinsics(cam_info);
    if (!rsp.success)
      rsp.status_message = "Error writing camera_info to flash.";
    start();

    return 1;
  }

  bool boardConfig(wge100_camera::BoardConfig::Request &req, wge100_camera::BoardConfig::Response &rsp)
  {
    MACAddress mac;
    if (req.mac.size() != 6)
    {
      ROS_ERROR("board_config called with %zu bytes in MAC. Should be 6.", req.mac.size());
      rsp.success = 0;
      return 1;
    }
    for (int i = 0; i < 6; i++)
      mac[i] = req.mac[i];
    ROS_INFO("board_config called s/n #%i, and MAC %02x:%02x:%02x:%02x:%02x:%02x.", req.serial, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    stop();
    rsp.success = !wge100ConfigureBoard( &camera_, req.serial, &mac);

    if (rsp.success)
      ROS_INFO("board_config succeeded.");
    else
      ROS_INFO("board_config failed.");

    return 1;
  }
  
};

class WGE100CameraNode : public driver_base::DriverNode<WGE100CameraDriver>
{
public:
  WGE100CameraNode(ros::NodeHandle &nh) :
    driver_base::DriverNode<WGE100CameraDriver>(nh),
    cam_pub_(ros::NodeHandle("~").advertise<sensor_msgs::Image>("image_raw", 1), 
        diagnostic_,
        diagnostic_updater::FrequencyStatusParam(&driver_.desired_freq_, &driver_.desired_freq_, 0.05), 
        diagnostic_updater::TimeStampStatusParam()),
    camera_info_pub_(ros::NodeHandle("~").advertise<sensor_msgs::CameraInfo>("camera_info", 1),
        diagnostic_,
        diagnostic_updater::FrequencyStatusParam(&driver_.desired_freq_, &driver_.desired_freq_, 0.05), 
        diagnostic_updater::TimeStampStatusParam()),
    calibrated_(false)
  {
    driver_.useFrame_ = boost::bind(&WGE100CameraNode::publishImage, this, _1, _2, _3, _4);
    driver_.setPostOpenHook(boost::bind(&WGE100CameraNode::postOpenHook, this));
  }
  
private:  
  // Publications
  diagnostic_updater::DiagnosedPublisher<sensor_msgs::Image> cam_pub_;
  diagnostic_updater::DiagnosedPublisher<sensor_msgs::CameraInfo> camera_info_pub_;
  sensor_msgs::Image image_;
  sensor_msgs::CameraInfo camera_info_;
  
  // Calibration
  bool calibrated_;

  int publishImage(size_t width, size_t height, uint8_t *frameData, ros::Time t)
  {
    // Raw data is bayer BGGR pattern
    std::string image_encoding = isColor() ? sensor_msgs::image_encodings::BAYER_BGGR8 : sensor_msgs::image_encodings::MONO8;
    fillImage(image_, image_encoding, height, width, width, frameData);
    
    t = ros::Time::now(); // FIXME KILLME Big hack

    image_.header.stamp = t;
    cam_pub_.publish(image_);
    camera_info_.header.stamp = t;
    camera_info_pub_.publish(camera_info_);

    return 0;
  }
  
  bool isColor()
  {
    return !(driver_.camera_.serial >= 2700000 && driver_.camera_.serial < 2800000);
  }
  
  virtual void reconfigureHook(int level)
  {
    ROS_DEBUG("WGE100CameraNode::reconfigureHook called at level %x", level);
    if ((level | driver_base::SensorLevels::RECONFIGURE_CLOSE) == level)
    {
      image_.header.frame_id = driver_.config_.frame_id;
      camera_info_.header.frame_id = driver_.config_.frame_id;
    }
  }

  void postOpenHook()
  {
    // Try to load camera intrinsics from flash memory
    calibrated_ = driver_.loadIntrinsics(camera_info_);
    if (calibrated_)
      ROS_INFO("Loaded intrinsics from camera");
    else
    {
      ROS_WARN("Failed to load intrinsics from camera");
      camera_info_ = sensor_msgs::CameraInfo();
      camera_info_.width = driver_.width_;
      camera_info_.height = driver_.height_;
    }
    
    diagnostic_.setHardwareID(driver_.getID());
  }

  virtual void addDiagnostics()
  {
    // Set up diagnostics
    diagnostic_.add("Link Status", &driver_, &WGE100CameraDriver::linkStatus );
  }
  
  virtual void addRunningTests()
  {
    self_test_.add( "Streaming Test", this, &WGE100CameraNode::streamingTest);
  }
  
  virtual void addOpenedTests()
  {
  }
  
  virtual void addStoppedTests()
  {
    ROS_DEBUG("Adding wge100 camera video mode tests.");
    for (int i = 0; i < MT9V_NUM_MODES; i++)
    {
      diagnostic_updater::TaskFunction f = boost::bind(&WGE100CameraNode::videoModeTest, this, MT9VModes[i].name, _1);
      self_test_.add( str(boost::format("Test Pattern in mode %s")%MT9VModes[i].name), f );
    }
  }
  
  void streamingTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    cam_pub_.clear_window();
    sleep(5);

    cam_pub_.run(status);
  }

  class VideoModeTestFrameHandler
  {
    public:
      VideoModeTestFrameHandler(diagnostic_updater::DiagnosticStatusWrapper &status) : status_(status)
      {}

      int run(size_t width, size_t height, uint8_t *data, ros::Time stamp)
      {
        status_.add("Got a frame", "Pass");

        for (size_t y = 0; y < height; y++)
          for (size_t x = 0; x < width; x++, data++)
          {
            uint8_t expected;
            
            if (width > 320)
              expected = get_expected(x, y);
            else
              expected = (get_expected(2 * x, 2 * y) + get_expected(2 * x, 2 * y + 1)) / 2;
                                   
            if (*data == expected)
              continue;

            status_.summaryf(2, "Unexpected value in frame at x=%i y=%i expected=%i got=%i.", x, y, (int) expected, (int) *data);
            status_.addf("Frame content", "Fail: Unexpected value at (x=%i, y=%i, %hhi != %hhi)", x, y, expected, *data);
            return 1;
          }

        status_.addf("Frame content", "Pass");
        return 1;
      }

    private:
      static uint8_t get_expected(int x, int y)
      {
        if ((x + 1) / 2 + y < 500)
          return 14 + x / 4;
        else
          return 0;
      }
      diagnostic_updater::DiagnosticStatusWrapper &status_;
  };

  void videoModeTest(const std::string mode, diagnostic_updater::DiagnosticStatusWrapper& status) 
  {
    const Config old_config = driver_.config_;
    WGE100CameraDriver::UseFrameFunction oldUseFrame = driver_.useFrame_;

    Config new_config = driver_.config_;
    new_config.video_mode = mode;
    driver_.config_update(new_config);
    VideoModeTestFrameHandler callback(status);
    driver_.useFrame_ = boost::bind(&VideoModeTestFrameHandler::run, boost::ref(callback), _1, _2, _3, _4);

    status.name = mode + " Pattern Test";
    status.summary(0, "Passed"); // If nobody else fills this, then the test passed.

    driver_.open();

    if (driver_.setTestMode(0x3800, status))
      goto reset_state;

    driver_.start();  
    {
      int oldcount = driver_.lost_image_thread_count_;
      driver_.stop();
      if (oldcount < driver_.lost_image_thread_count_)
      {
        ROS_ERROR("Lost the image_thread. This should never happen.");
        status.summary(2, "Lost the image_thread. This should never happen.");
      }
    }
    driver_.close();

    if (driver_.setTestMode(0x0000, status))
      goto reset_state;

reset_state:
    driver_.close();
    driver_.useFrame_ = oldUseFrame;
    driver_.config_update(old_config);

    ROS_INFO("Exiting test %s with status %i: %s", status.name.c_str(), status.level, status.message.c_str());
  }
};

int main(int argc, char **argv)
{ 
  return driver_base::main<WGE100CameraNode>(argc, argv, "wge100_camera");
}
