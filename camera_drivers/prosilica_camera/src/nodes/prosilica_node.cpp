/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

// ROS communication
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <polled_camera/publication_server.h>

// Diagnostics
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <self_test/self_test.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include "prosilica_camera/ProsilicaCameraConfig.h"

// Standard libs
#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <sstream>

#include "prosilica/prosilica.h"
#include "prosilica/rolling_sum.h"

/// @todo Only stream when subscribed to
class ProsilicaNode
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher streaming_pub_;
  polled_camera::PublicationServer poll_srv_;
  ros::ServiceServer set_camera_info_srv_;

  // Camera
  boost::scoped_ptr<prosilica::Camera> cam_;
  prosilica::AcquisitionMode mode_; /// @todo Make this property of Camera
  bool running_;
  unsigned long max_data_rate_;

  // ROS messages
  sensor_msgs::Image img_;
  sensor_msgs::CameraInfo cam_info_;
  
  // Diagnostics
  ros::Timer diagnostic_timer_;
  self_test::TestRunner self_test_;
  diagnostic_updater::Updater diagnostic_;
  std::string hw_id_;
  int count_;
  double desired_freq_;
  static const int WINDOW_SIZE = 5; // remember previous 5s
  unsigned long frames_dropped_total_, frames_completed_total_;
  RollingSum<unsigned long> frames_dropped_acc_, frames_completed_acc_;
  unsigned long packets_missed_total_, packets_received_total_;
  RollingSum<unsigned long> packets_missed_acc_, packets_received_acc_;

  // So we don't get burned by auto-exposure
  unsigned long last_exposure_value_;
  int consecutive_stable_exposures_;

public:
  ProsilicaNode(const ros::NodeHandle& node_handle)
    : nh_(node_handle),
      it_(nh_),
      cam_(NULL), running_(false),
      self_test_(nh_),
      count_(0),
      frames_dropped_total_(0), frames_completed_total_(0),
      frames_dropped_acc_(WINDOW_SIZE),
      frames_completed_acc_(WINDOW_SIZE),
      packets_missed_total_(0), packets_received_total_(0),
      packets_missed_acc_(WINDOW_SIZE),
      packets_received_acc_(WINDOW_SIZE)
  {
    // Two-stage initialization: in the constructor we open the requested camera. Most
    // parameters controlling capture are set and streaming started in configure(), the
    // callback to dynamic_reconfig.
    prosilica::init();

    if (prosilica::numCameras() == 0)
      ROS_WARN("Found no cameras on local subnet");

    // Determine which camera to use. Opening by IP address is preferred, then guid. If both
    // parameters are set we open by IP and verify the guid. If neither are set we default
    // to opening the first available camera.
    ros::NodeHandle local_nh("~");
    unsigned long guid = 0;
    std::string guid_str;
    if (local_nh.getParam("guid", guid_str))
      guid = strtol(guid_str.c_str(), NULL, 0);

    std::string ip_str;
    if (local_nh.getParam("ip_address", ip_str)) {
      cam_.reset( new prosilica::Camera(ip_str.c_str()) );
      
      // Verify guid is the one expected
      unsigned long cam_guid = cam_->guid();
      if (guid != 0 && guid != cam_guid)
        throw prosilica::ProsilicaException(ePvErrBadParameter,
                                            "guid does not match expected");
      guid = cam_guid;
    }
    else {
      if (guid == 0) guid = prosilica::getGuid(0);
      cam_.reset( new prosilica::Camera(guid) );
    }
    hw_id_ = boost::lexical_cast<std::string>(guid);
    ROS_INFO("Found camera, guid = %s", hw_id_.c_str());
    diagnostic_.setHardwareID(hw_id_);

    // Try to load intrinsics from on-camera memory.
    loadIntrinsics();

    // Set up self tests and diagnostics.
    self_test_.add( "Info Test", this, &ProsilicaNode::infoTest );
    self_test_.add( "Attribute Test", this, &ProsilicaNode::attributeTest );
    self_test_.add( "Image Test", this, &ProsilicaNode::imageTest );
    
    diagnostic_.add( "Frequency Status", this, &ProsilicaNode::freqStatus );
    diagnostic_.add( "Frame Statistics", this, &ProsilicaNode::frameStatistics );
    diagnostic_.add( "Packet Statistics", this, &ProsilicaNode::packetStatistics );
    diagnostic_.add( "Packet Error Status", this, &ProsilicaNode::packetErrorStatus );

    diagnostic_timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&ProsilicaNode::runDiagnostics, this));

    // Service call for setting calibration.
    set_camera_info_srv_ = nh_.advertiseService("set_camera_info", &ProsilicaNode::setCameraInfo, this);
  }

  void configure(prosilica_camera::ProsilicaCameraConfig& config, uint32_t level)
  {
    ROS_DEBUG("Reconfigure request received");

    if (level >= (uint32_t)driver_base::SensorLevels::RECONFIGURE_STOP)
      stop();

    // Trigger mode
    if (config.trigger_mode == "streaming") {
      mode_ = prosilica::Continuous;
      /// @todo Tighter bound than this minimal check
      desired_freq_ = 1; // make sure we get _something_
    }
    else if (config.trigger_mode == "polled") {
      mode_ = prosilica::Triggered;
      desired_freq_ = 0;
    }
    else {
      ROS_ERROR("Invalid trigger mode '%s' in reconfigure request", config.trigger_mode.c_str());
    }

    // Exposure
    if (config.auto_exposure)
      cam_->setExposure(0, prosilica::Auto);
    else {
      unsigned us = config.exposure*1000000. + 0.5;
      cam_->setExposure(us, prosilica::Manual);
    }

    // Gain
    if (config.auto_gain)
      cam_->setGain(0, prosilica::Auto);
    else
      cam_->setGain(config.gain, prosilica::Manual);
    
    // White balance
    if (config.auto_whitebalance)
      cam_->setWhiteBalance(0, 0, prosilica::Auto);
    else
      cam_->setWhiteBalance(config.whitebalance_blue, config.whitebalance_red, prosilica::Manual);

    // Region of interest configuration
    tPvUint32 max_width, max_height, dummy; // @TODO Don't read these each time. They won't change.
    PvAttrRangeUint32(cam_->handle(), "Width", &dummy, &max_width);
    PvAttrRangeUint32(cam_->handle(), "Height", &dummy, &max_height);

    if (config.x_offset >= (int) max_width)
    {
      config.x_offset = max_width - 1;
      ROS_WARN("x_offset too large. Set to maximum value of %i", config.y_offset);
    }
    if (config.y_offset >= (int) max_height)
    {
      config.y_offset = max_height - 1;
      ROS_WARN("y_offset too large. Set to maximum value of %i", config.x_offset);
    }
    int x_offset = config.x_offset;
    int y_offset = config.y_offset;
    
    if (config.width > (int) max_width - x_offset)
    {
      config.width = max_width - x_offset;
      ROS_WARN("width too large. Set to maximum value of %i.", config.width);
    }
    if (config.height > (int) max_height - y_offset)
    {
      config.height = max_height - y_offset;
      ROS_WARN("height too large. Set to mayimum value of %i.", config.height);
    }
    int width = config.width;
    int height = config.height;

    // If width or height is 0, set it as large as possible
    if (!width) width = max_width - x_offset;
    if (!height) height = max_height - y_offset;

    cam_->setRoi(x_offset, y_offset, width, height);
    
    // TF frame
    img_.header.frame_id = cam_info_.header.frame_id = config.frame_id;

    if (level >= (uint32_t)driver_base::SensorLevels::RECONFIGURE_STOP)
      start();
  }

  ~ProsilicaNode()
  {
    stop();
    cam_.reset(); // must destroy Camera before calling prosilica::fini
    prosilica::fini();
  }

  void start()
  {
    if (running_) return;

    if (mode_ == prosilica::Triggered) {
      poll_srv_ = polled_camera::advertise(nh_, "request_image", &ProsilicaNode::pollCallback, this);
      // Auto-exposure tends to go wild the first few frames after startup
      // if (auto_expose) normalizeExposure();
    }
    else {
      assert(mode_ == prosilica::Continuous);
      cam_->setFrameCallback(boost::bind(&ProsilicaNode::publishImage, this, _1));
      streaming_pub_ = it_.advertiseCamera("image_raw", 1);
    }

    cam_->start(mode_);
    running_ = true;
  }

  void stop()
  {
    if (!running_) return;

    cam_->stop(); // Must stop camera before streaming_pub_.
    poll_srv_.shutdown();
    streaming_pub_.shutdown();
    
    running_ = false;
  }

  bool pollCallback(polled_camera::GetPolledImage::Request& req,
                    sensor_msgs::Image& image, sensor_msgs::CameraInfo& info)
  {
    static const unsigned long TIMEOUT = 500; //ms
    
    if (mode_ != prosilica::Triggered) {
      ROS_ERROR("Poll service called but camera is not in triggered mode");
      return false;
    }

    tPvFrame* frame = NULL;

    try {
      if (req.roi.x_offset || req.roi.y_offset || req.roi.width || req.roi.height) {
        cam_->setRoi(req.roi.x_offset, req.roi.y_offset, req.roi.width, req.roi.height);
      } else {
        cam_->setRoiToWholeFrame();
      }

      frame = cam_->grab(TIMEOUT);
    }
    catch (prosilica::ProsilicaException &e) {
      if (e.error_code == ePvErrBadSequence)
        throw; // not easily recoverable
      
      ROS_ERROR("Prosilica exception: %s\n\tx = %d, y = %d, width = %d, height = %d",
                e.what(), req.roi.x_offset, req.roi.y_offset, req.roi.width, req.roi.height);
      return false;
    }

    info = cam_info_;
    image.header.frame_id = img_.header.frame_id;
    return frame && processFrame(frame, image, info);
  }

  static bool frameToImage(tPvFrame* frame, sensor_msgs::Image &image)
  {
    // NOTE: 16-bit and Yuv formats not supported
    static const char* BAYER_ENCODINGS[] = { "bayer_rggb8", "bayer_gbrg8", "bayer_grbg8", "bayer_bggr8" };

    std::string encoding;
    if (frame->Format == ePvFmtMono8)       encoding = sensor_msgs::image_encodings::MONO8;
    else if (frame->Format == ePvFmtBayer8) 
    {
#if 1
      encoding = BAYER_ENCODINGS[frame->BayerPattern];
#else
      image.encoding = sensor_msgs::image_encodings::BGR8;
      image.height = frame->Height;
      image.width = frame->Width;
      image.step = frame->Width * 3;
      image.data.resize(frame->Height * (frame->Width * 3));
      PvUtilityColorInterpolate(frame, &image.data[2], &image.data[1], &image.data[0], 2, 0);
      return true;
#endif
    }
    else if (frame->Format == ePvFmtRgb24)  encoding = sensor_msgs::image_encodings::RGB8;
    else if (frame->Format == ePvFmtBgr24)  encoding = sensor_msgs::image_encodings::BGR8;
    else if (frame->Format == ePvFmtRgba32) encoding = sensor_msgs::image_encodings::RGBA8;
    else if (frame->Format == ePvFmtBgra32) encoding = sensor_msgs::image_encodings::BGRA8;
    else {
      ROS_WARN("Received frame with unsupported pixel format %d", frame->Format);
      return false;
    }

    uint32_t step = frame->ImageSize / frame->Height;
    return sensor_msgs::fillImage(image, encoding, frame->Height, frame->Width, step, frame->ImageBuffer);
  }
  
  bool processFrame(tPvFrame* frame, sensor_msgs::Image &img, sensor_msgs::CameraInfo &cam_info)
  {
    /// @todo Use time from frame?
    img.header.stamp = cam_info.header.stamp = ros::Time::now();
    
    if (!frameToImage(frame, img))
      return false;

    cam_info.roi.x_offset = frame->RegionX;
    cam_info.roi.y_offset = frame->RegionY;
    cam_info.roi.height = frame->Height;
    cam_info.roi.width = frame->Width;

    count_++;
    return true;
  }
  
  void publishImage(tPvFrame* frame)
  {
    if (processFrame(frame, img_, cam_info_))
      streaming_pub_.publish(img_, cam_info_);
  }

  void loadIntrinsics()
  {
    // Retrieve contents of user memory
    std::string buffer(prosilica::Camera::USER_MEMORY_SIZE, '\0');
    cam_->readUserMemory(&buffer[0], prosilica::Camera::USER_MEMORY_SIZE);

    // Parse calibration file
    std::string camera_name;
    if (camera_calibration_parsers::parseCalibrationIni(buffer, camera_name, cam_info_))
      ROS_INFO("Loaded calibration for camera '%s'", camera_name.c_str());
    else
      ROS_WARN("Failed to load intrinsics from camera");
  }

  bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp)
  {
    ROS_INFO("New camera info received");
    sensor_msgs::CameraInfo &info = req.camera_info;
    
    // Sanity check: the image dimensions should match the max resolution of the sensor.
    tPvUint32 width, height, dummy;
    PvAttrRangeUint32(cam_->handle(), "Width", &dummy, &width);
    PvAttrRangeUint32(cam_->handle(), "Height", &dummy, &height);
    if (info.width != width || info.height != height) {
      rsp.success = false;
      rsp.status_message = (boost::format("Camera_info resolution %ix%i does not match current video "
                                          "setting, camera running at resolution %ix%i.")
                            % info.width % info.height % width % height).str();
      ROS_ERROR(rsp.status_message.c_str());
      return true;
    }
    
    stop();

    std::string cam_name = "prosilica";
    cam_name += hw_id_;
    std::stringstream ini_stream;
    if (!camera_calibration_parsers::writeCalibrationIni(ini_stream, cam_name, info)) {
      rsp.status_message = "Error formatting camera_info for storage.";
      rsp.success = false;
    }
    else {
      std::string ini = ini_stream.str();
      if (ini.size() > prosilica::Camera::USER_MEMORY_SIZE) {
        rsp.success = false;
        rsp.status_message = "Unable to write camera_info to camera memory, exceeded storage capacity.";
      }
      else {
        try {
          cam_->writeUserMemory(ini.c_str(), ini.size());
          cam_info_ = info;
          rsp.success = true;
        }
        catch (prosilica::ProsilicaException &e) {
          rsp.success = false;
          rsp.status_message = e.what();
        }
      }
    }
    if (!rsp.success)
      ROS_ERROR(rsp.status_message.c_str());
    
    start();

    return true;
  }

  void normalizeCallback(tPvFrame* frame)
  {
    unsigned long exposure;
    cam_->getAttribute("ExposureValue", exposure);
    //ROS_WARN("Exposure value = %u", exposure);

    if (exposure == last_exposure_value_)
      consecutive_stable_exposures_++;
    else {
      last_exposure_value_ = exposure;
      consecutive_stable_exposures_ = 0;
    }
  }
  
  void normalizeExposure()
  {
    ROS_INFO("Normalizing exposure");
    /// @todo assert(stopped)

    last_exposure_value_ = 0;
    consecutive_stable_exposures_ = 0;
    cam_->setFrameCallback(boost::bind(&ProsilicaNode::normalizeCallback, this, _1));
    cam_->start(prosilica::Continuous);

    /// @todo thread safety
    while (consecutive_stable_exposures_ < 3)
      boost::this_thread::sleep(boost::posix_time::millisec(250));

    cam_->stop();
  }

  /////////////////
  // Diagnostics //
  /////////////////
  
  void runDiagnostics()
  {
    self_test_.checkTest();
    diagnostic_.update();
  }
  
  void freqStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    double freq = (double)(count_)/diagnostic_.getPeriod();

    if (freq < (.9*desired_freq_))
    {
      status.summary(2, "Desired frequency not met");
    }
    else
    {
      status.summary(0, "Desired frequency met");
    }

    status.add("Images in interval", count_);
    status.add("Desired frequency", desired_freq_);
    status.add("Actual frequency", freq);
    
    count_ = 0;
  }

  void frameStatistics(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    // Get stats from camera driver
    float frame_rate;
    unsigned long completed, dropped;
    cam_->getAttribute("StatFrameRate", frame_rate);
    cam_->getAttribute("StatFramesCompleted", completed);
    cam_->getAttribute("StatFramesDropped", dropped);

    // Compute rolling totals, percentages
    frames_completed_acc_.add(completed - frames_completed_total_);
    frames_completed_total_ = completed;
    unsigned long completed_recent = frames_completed_acc_.sum();
    
    frames_dropped_acc_.add(dropped - frames_dropped_total_);
    frames_dropped_total_ = dropped;
    unsigned long dropped_recent = frames_dropped_acc_.sum();

    float recent_ratio = float(completed_recent) / (completed_recent + dropped_recent);
    float total_ratio = float(completed) / (completed + dropped);

    // Set level based on recent % completed frames
    if (dropped_recent == 0) {
      status.summary(0, "No dropped frames");
    }
    else if (recent_ratio > 0.8f) {
      status.summary(1, "Some dropped frames");
    }
    else {
      status.summary(2, "Excessive proportion of dropped frames");
    }

    status.add("Camera Frame Rate", frame_rate);
    status.add("Recent % Frames Completed", recent_ratio * 100.0f);
    status.add("Overall % Frames Completed", total_ratio * 100.0f);
    status.add("Frames Completed", completed);
    status.add("Frames Dropped", dropped);
  }

  void packetStatistics(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    // Get stats from camera driver
    unsigned long received, missed, requested, resent;
    cam_->getAttribute("StatPacketsReceived", received);
    cam_->getAttribute("StatPacketsMissed", missed);
    cam_->getAttribute("StatPacketsRequested", requested);
    cam_->getAttribute("StatPacketsResent", resent);

    // Compute rolling totals, percentages
    packets_received_acc_.add(received - packets_received_total_);
    packets_received_total_ = received;
    unsigned long received_recent = packets_received_acc_.sum();
    
    packets_missed_acc_.add(missed - packets_missed_total_);
    packets_missed_total_ = missed;
    unsigned long missed_recent = packets_missed_acc_.sum();

    float recent_ratio = float(received_recent) / (received_recent + missed_recent);
    float total_ratio = float(received) / (received + missed);

    if (missed_recent == 0) {
      status.summary(0, "No missed packets");
    }
    else if (recent_ratio > 0.99f) {
      status.summary(1, "Some missed packets");
    }
    else {
      status.summary(2, "Excessive proportion of missed packets");
    }

    // Adjust data rate
    unsigned long data_rate = 0;
    unsigned long max_data_rate = cam_->getMaxDataRate();
    if (max_data_rate < prosilica::Camera::GIGE_MAX_DATA_RATE)
      status.mergeSummary(1, "Max data rate is lower than expected for a GigE port");
    try {
      cam_->getAttribute("StreamBytesPerSecond", data_rate);

      /// @todo Something that doesn't oscillate
      float multiplier = 1.0f;
      if (recent_ratio == 1.0f) {
        multiplier = 1.1f;
      } else if (recent_ratio < 0.99f) {
        multiplier = 0.9f;
      }
      if (multiplier != 1.0f) {
        unsigned long new_data_rate = std::min((unsigned long)(multiplier * data_rate + 0.5), max_data_rate);
        new_data_rate = std::max(new_data_rate, max_data_rate/1000);
        if (data_rate != new_data_rate) {
          data_rate = new_data_rate;
          cam_->setAttribute("StreamBytesPerSecond", data_rate);
          ROS_DEBUG("Changed data rate to %lu bytes per second", data_rate);
        }
      }
    }
    catch (prosilica::ProsilicaException &e) {
      if (e.error_code == ePvErrUnplugged)
        throw;
      ROS_ERROR("Exception occurred: '%s'\n"
                "Possible network issue. Attempting to reset data rate to the current maximum.",
                e.what());
      data_rate = max_data_rate;
      cam_->setAttribute("StreamBytesPerSecond", data_rate);
    }
    
    status.add("Recent % Packets Received", recent_ratio * 100.0f);
    status.add("Overall % Packets Received", total_ratio * 100.0f);
    status.add("Received Packets", received);
    status.add("Missed Packets", missed);
    status.add("Requested Packets", requested);
    status.add("Resent Packets", resent);
    status.add("Data Rate (bytes/s)", data_rate);
    status.add("Max Data Rate (bytes/s)", max_data_rate);
  }

  void packetErrorStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    unsigned long erroneous;
    cam_->getAttribute("StatPacketsErroneous", erroneous);

    if (erroneous == 0) {
      status.summary(0, "No erroneous packets");
    } else {
      status.summary(2, "Possible camera hardware failure");
    }

    status.add("Erroneous Packets", erroneous);
  }

  ////////////////
  // Self tests //
  ////////////////

  // Try to load camera name, etc. Should catch gross communication failure.
  void infoTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Info Test";

    self_test_.setID(hw_id_);

    std::string camera_name;
    try {
      cam_->getAttribute("CameraName", camera_name);
      status.summary(0, "Connected to Camera");
      status.add("Camera Name", camera_name);
    }
    catch (prosilica::ProsilicaException& e) {
      status.summary(2, e.what());
    }
  }

  // Test validity of all attribute values.
  void attributeTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Attribute Test";

    tPvAttrListPtr list_ptr;
    unsigned long list_length;

    if (PvAttrList(cam_->handle(), &list_ptr, &list_length) == ePvErrSuccess) {
      status.summary(0, "All attributes in valid range");
      for (unsigned int i = 0; i < list_length; ++i) {
        const char* attribute = list_ptr[i];
        tPvErr e = PvAttrIsValid(cam_->handle(), attribute);
        if (e != ePvErrSuccess) {
          status.summary(2, "One or more invalid attributes");
          if (e == ePvErrOutOfRange)
            status.add(attribute, "Out of range");
          else if (e == ePvErrNotFound)
            status.add(attribute, "Does not exist");
          else
            status.addf(attribute, "Unexpected error code %u", e);
        }
      }
    }
    else {
      status.summary(2, "Unable to retrieve attribute list");
    }
  }

  // Try to capture an image.
  void imageTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Image Capture Test";

    try {
      if (mode_ != prosilica::Triggered) {
        tPvUint32 start_completed, current_completed;
        cam_->getAttribute("StatFramesCompleted", start_completed);
        for (int i = 0; i < 6; ++i) {
          boost::this_thread::sleep(boost::posix_time::millisec(500));
          cam_->getAttribute("StatFramesCompleted", current_completed);
          if (current_completed > start_completed) {
            status.summary(0, "Captured a frame, see diagnostics for detailed stats");
            return;
          }
        }

        // Give up after 3s
        status.summary(2, "No frames captured over 3s in continuous mode");
        return;
      }

      // In triggered mode, try to grab a frame
      cam_->setRoiToWholeFrame();
      tPvFrame* frame = cam_->grab(500);
      if (frame)
	{
	  status.summary(0, "Successfully triggered a frame capture");
	}
      else
	{
	  status.summary(2, "Attempted to grab a frame, but received NULL");
	}
	
    }
    catch (prosilica::ProsilicaException &e) {
      status.summary(2, e.what());
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "prosilica_driver");

  /// @todo If server is in the try-catch block or pn is not a shared_ptr, we hang on
  /// exception. Why???
  typedef dynamic_reconfigure::Server<prosilica_camera::ProsilicaCameraConfig> Server;
  Server server;

  try {
    ros::NodeHandle nh("camera");
    //ProsilicaNode pn(nh);
    boost::shared_ptr<ProsilicaNode> pn(new ProsilicaNode(nh));
    
    Server::CallbackType f = boost::bind(&ProsilicaNode::configure, pn, _1, _2);
    server.setCallback(f);
    
    ros::spin();
  }
  catch (std::runtime_error &e) {
    ROS_FATAL("Uncaught exception: '%s', aborting.", e.what());
    ROS_BREAK();
  }

  return 0;
}
