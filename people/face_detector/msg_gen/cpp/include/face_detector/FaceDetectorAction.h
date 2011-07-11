/* Auto-generated by genmsg_cpp for file /home/reza/git/people/face_detector/msg/FaceDetectorAction.msg */
#ifndef FACE_DETECTOR_MESSAGE_FACEDETECTORACTION_H
#define FACE_DETECTOR_MESSAGE_FACEDETECTORACTION_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "face_detector/FaceDetectorActionGoal.h"
#include "face_detector/FaceDetectorActionResult.h"
#include "face_detector/FaceDetectorActionFeedback.h"

namespace face_detector
{
template <class ContainerAllocator>
struct FaceDetectorAction_ : public ros::Message
{
  typedef FaceDetectorAction_<ContainerAllocator> Type;

  FaceDetectorAction_()
  : action_goal()
  , action_result()
  , action_feedback()
  {
  }

  FaceDetectorAction_(const ContainerAllocator& _alloc)
  : action_goal(_alloc)
  , action_result(_alloc)
  , action_feedback(_alloc)
  {
  }

  typedef  ::face_detector::FaceDetectorActionGoal_<ContainerAllocator>  _action_goal_type;
   ::face_detector::FaceDetectorActionGoal_<ContainerAllocator>  action_goal;

  typedef  ::face_detector::FaceDetectorActionResult_<ContainerAllocator>  _action_result_type;
   ::face_detector::FaceDetectorActionResult_<ContainerAllocator>  action_result;

  typedef  ::face_detector::FaceDetectorActionFeedback_<ContainerAllocator>  _action_feedback_type;
   ::face_detector::FaceDetectorActionFeedback_<ContainerAllocator>  action_feedback;


private:
  static const char* __s_getDataType_() { return "face_detector/FaceDetectorAction"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "665c888633df000242196f7098a55805"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
FaceDetectorActionGoal action_goal\n\
FaceDetectorActionResult action_result\n\
FaceDetectorActionFeedback action_feedback\n\
\n\
================================================================================\n\
MSG: face_detector/FaceDetectorActionGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
FaceDetectorGoal goal\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: face_detector/FaceDetectorGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#goal\n\
\n\
================================================================================\n\
MSG: face_detector/FaceDetectorActionResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
FaceDetectorResult result\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalStatus\n\
GoalID goal_id\n\
uint8 status\n\
uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n\
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n\
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n\
                            #   and has since completed its execution (Terminal State)\n\
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n\
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n\
                            #    to some failure (Terminal State)\n\
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n\
                            #    because the goal was unattainable or invalid (Terminal State)\n\
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n\
                            #    and has not yet completed execution\n\
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n\
                            #    but the action server has not yet confirmed that the goal is canceled\n\
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n\
                            #    and was successfully cancelled (Terminal State)\n\
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n\
                            #    sent over the wire by an action server\n\
\n\
#Allow for the user to associate a string with GoalStatus for debugging\n\
string text\n\
\n\
\n\
================================================================================\n\
MSG: face_detector/FaceDetectorResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#result\n\
people_msgs/PositionMeasurement[] face_positions\n\
\n\
================================================================================\n\
MSG: people_msgs/PositionMeasurement\n\
Header          header\n\
string          name\n\
string          object_id\n\
geometry_msgs/Point  pos\n\
float64         reliability\n\
float64[9]      covariance\n\
byte            initialization\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: face_detector/FaceDetectorActionFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
FaceDetectorFeedback feedback\n\
\n\
================================================================================\n\
MSG: face_detector/FaceDetectorFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#feedback\n\
\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, action_goal);
    ros::serialization::serialize(stream, action_result);
    ros::serialization::serialize(stream, action_feedback);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, action_goal);
    ros::serialization::deserialize(stream, action_result);
    ros::serialization::deserialize(stream, action_feedback);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(action_goal);
    size += ros::serialization::serializationLength(action_result);
    size += ros::serialization::serializationLength(action_feedback);
    return size;
  }

  typedef boost::shared_ptr< ::face_detector::FaceDetectorAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::face_detector::FaceDetectorAction_<ContainerAllocator>  const> ConstPtr;
}; // struct FaceDetectorAction
typedef  ::face_detector::FaceDetectorAction_<std::allocator<void> > FaceDetectorAction;

typedef boost::shared_ptr< ::face_detector::FaceDetectorAction> FaceDetectorActionPtr;
typedef boost::shared_ptr< ::face_detector::FaceDetectorAction const> FaceDetectorActionConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::face_detector::FaceDetectorAction_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::face_detector::FaceDetectorAction_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace face_detector

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::face_detector::FaceDetectorAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "665c888633df000242196f7098a55805";
  }

  static const char* value(const  ::face_detector::FaceDetectorAction_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x665c888633df0002ULL;
  static const uint64_t static_value2 = 0x42196f7098a55805ULL;
};

template<class ContainerAllocator>
struct DataType< ::face_detector::FaceDetectorAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "face_detector/FaceDetectorAction";
  }

  static const char* value(const  ::face_detector::FaceDetectorAction_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::face_detector::FaceDetectorAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
FaceDetectorActionGoal action_goal\n\
FaceDetectorActionResult action_result\n\
FaceDetectorActionFeedback action_feedback\n\
\n\
================================================================================\n\
MSG: face_detector/FaceDetectorActionGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
FaceDetectorGoal goal\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: face_detector/FaceDetectorGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#goal\n\
\n\
================================================================================\n\
MSG: face_detector/FaceDetectorActionResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
FaceDetectorResult result\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalStatus\n\
GoalID goal_id\n\
uint8 status\n\
uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n\
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n\
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n\
                            #   and has since completed its execution (Terminal State)\n\
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n\
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n\
                            #    to some failure (Terminal State)\n\
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n\
                            #    because the goal was unattainable or invalid (Terminal State)\n\
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n\
                            #    and has not yet completed execution\n\
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n\
                            #    but the action server has not yet confirmed that the goal is canceled\n\
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n\
                            #    and was successfully cancelled (Terminal State)\n\
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n\
                            #    sent over the wire by an action server\n\
\n\
#Allow for the user to associate a string with GoalStatus for debugging\n\
string text\n\
\n\
\n\
================================================================================\n\
MSG: face_detector/FaceDetectorResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#result\n\
people_msgs/PositionMeasurement[] face_positions\n\
\n\
================================================================================\n\
MSG: people_msgs/PositionMeasurement\n\
Header          header\n\
string          name\n\
string          object_id\n\
geometry_msgs/Point  pos\n\
float64         reliability\n\
float64[9]      covariance\n\
byte            initialization\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: face_detector/FaceDetectorActionFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
FaceDetectorFeedback feedback\n\
\n\
================================================================================\n\
MSG: face_detector/FaceDetectorFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#feedback\n\
\n\
\n\
";
  }

  static const char* value(const  ::face_detector::FaceDetectorAction_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::face_detector::FaceDetectorAction_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.action_goal);
    stream.next(m.action_result);
    stream.next(m.action_feedback);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct FaceDetectorAction_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::face_detector::FaceDetectorAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::face_detector::FaceDetectorAction_<ContainerAllocator> & v) 
  {
    s << indent << "action_goal: ";
s << std::endl;
    Printer< ::face_detector::FaceDetectorActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
s << std::endl;
    Printer< ::face_detector::FaceDetectorActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
s << std::endl;
    Printer< ::face_detector::FaceDetectorActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};


} // namespace message_operations
} // namespace ros

#endif // FACE_DETECTOR_MESSAGE_FACEDETECTORACTION_H

