#! /usr/bin/env python
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

# Author: Blaise Gassend

# WGE100 camera configuration, non camera-specific settings.

PACKAGE='wge100_camera'
import roslib; roslib.load_manifest(PACKAGE)

from driver_base.msg import SensorLevels
from dynamic_reconfigure.parameter_generator import *

def add_others(gen):
	#       Name                         Type      Reconfiguration level             Description                                                                                                              Default    Min   Max
	#gen.add("exit_on_fault",             bool_t,   SensorLevels.RECONFIGURE_RUNNING, "Indicates if the driver should exit when an error occurs.",                                                             False) 
	gen.add("video_mode",                str_t,    SensorLevels.RECONFIGURE_CLOSE,   "Sets the camera video mode.",                                                                                           "640x480x30")
	gen.add("auto_exposure",             bool_t,   SensorLevels.RECONFIGURE_CLOSE,   "Sets the camera exposure duration to automatic. Causes the @b exposure setting to be ignored.",                         True)
	gen.add("exposure",                  double_t, SensorLevels.RECONFIGURE_CLOSE,   "Maximum camera exposure time in seconds. The valid range depends on the video mode.",                                   0.01,       0,   0.1)
	gen.add("max_exposure",              double_t, SensorLevels.RECONFIGURE_CLOSE,   "Maximum exposure time in seconds. Zero for automatic.",                                                                 0,          0,    .1)
	gen.add("auto_gain",                 bool_t,   SensorLevels.RECONFIGURE_CLOSE,   "Sets the analog gain to automatic. Causes the @b gain setting to be ignored.",                                          True)
	gen.add("companding",                bool_t,   SensorLevels.RECONFIGURE_CLOSE,   "Turns on companding",                                                                                                   True)
	gen.add("gain",                      int_t,    SensorLevels.RECONFIGURE_CLOSE,   "The camera analog gain.",                                                                                               32,         16,   64)
	gen.add("brightness",                int_t,    SensorLevels.RECONFIGURE_CLOSE,   "The camera brightness for automatic gain/exposure.",                                                                    58,         1,    64)
	gen.add("frame_id",                  str_t,    SensorLevels.RECONFIGURE_RUNNING, "Sets the TF frame from which the camera is publishing.",                                                                "")
	gen.add("first_packet_offset",       double_t, SensorLevels.RECONFIGURE_RUNNING, "Offset between the end of exposure and the minimal arrival time for the first frame packet.",                           0.0025,     0,   .02)
	gen.add("ext_trig",                  bool_t,   SensorLevels.RECONFIGURE_CLOSE,   "Set camera to trigger from the external trigger input.",                                                                False)
        gen.add("rising_edge_trig",          bool_t,   SensorLevels.RECONFIGURE_CLOSE,   "Indicates that the camera should trigger on rising edges.",                                                            False)
        gen.add("trig_controller",           str_t,    SensorLevels.RECONFIGURE_CLOSE,   "Sets the trigger controller from which an externally trigged camera operates.",                                         "")
	gen.add("trig_rate",                 double_t, SensorLevels.RECONFIGURE_CLOSE,   "Sets the triggering rate in externally triggered mode.",                                                                30,         1,  100)
	gen.add("mirror_x",                  bool_t,   SensorLevels.RECONFIGURE_CLOSE,   "Mirrors the image left to right.",                                                                                      False)
	gen.add("mirror_y",                  bool_t,   SensorLevels.RECONFIGURE_CLOSE,   "Mirrors the image top to bottom.",                                                                                      False)
	gen.add("rotate_180",                bool_t,   SensorLevels.RECONFIGURE_CLOSE,   "Rotates the image 180 degrees. Acts in addition to mirror_?",                                                           False)
	gen.add("enable_alternate",          bool_t,   SensorLevels.RECONFIGURE_CLOSE,   "Enable the alternate imager register set. The camera trigger signal selects the register set to use for each image.",   False)
	gen.add("auto_exposure_alternate",   bool_t,   SensorLevels.RECONFIGURE_CLOSE,   "Sets the alternate camera exposure duration to automatic. Causes the @b exposure_alternate setting to be ignored.",     True)
	gen.add("exposure_alternate",        double_t, SensorLevels.RECONFIGURE_CLOSE,   "Alternate camera exposure in seconds. The valid range depends on the video mode.",                                      0.01,       0,    .1)
	gen.add("max_exposure_alternate",    double_t, SensorLevels.RECONFIGURE_CLOSE,   "Alternate maximum exposure time in seconds. Zero for automatic.",                                                                 0,          0,    .1)
	gen.add("auto_gain_alternate",       bool_t,   SensorLevels.RECONFIGURE_CLOSE,   "Sets the alternate analog gain to automatic. Causes the @b gain_alternate setting to be ignored.",                      True)
	gen.add("gain_alternate",            int_t,    SensorLevels.RECONFIGURE_CLOSE,   "The alternate camera analog gain.",                                                                                     32,         16,   64)
	gen.add("companding_alternate",      bool_t,   SensorLevels.RECONFIGURE_CLOSE,   "Turns on companding for the alternate imager register set",                                                             True)
