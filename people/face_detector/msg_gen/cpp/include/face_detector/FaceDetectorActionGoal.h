/* Auto-generated by genmsg_cpp for file /home/reza/git/people/face_detector/msg/FaceDetectorActionGoal.msg */
#ifndef FACE_DETECTOR_MESSAGE_FACEDETECTORACTIONGOAL_H
#define FACE_DETECTOR_MESSAGE_FACEDETECTORACTIONGOAL_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "face_detector/FaceDetectorGoal.h"

namespace face_detector
{
template <class ContainerAllocator>
struct FaceDetectorActionGoal_ : public ros::Message
{
  typedef FaceDetectorActionGoal_<ContainerAllocator> Type;

  FaceDetectorActionGoal_()
  : header()
  , goal_id()
  , goal()
  {
  }

  FaceDetectorActionGoal_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , goal_id(_alloc)
  , goal(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef  ::actionlib_msgs::GoalID_<ContainerAllocator>  _goal_id_type;
   ::actionlib_msgs::GoalID_<ContainerAllocator>  goal_id;

  typedef  ::face_detector::FaceDetectorGoal_<ContainerAllocator>  _goal_type;
   ::face_detector::FaceDetectorGoal_<ContainerAllocator>  goal;


private:
  static const char* __s_getDataType_() { return "face_detector/FaceDetectorActionGoal"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "4b30be6cd12b9e72826df56b481f40e0"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
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
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, header);
    ros::serialization::serialize(stream, goal_id);
    ros::serialization::serialize(stream, goal);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, goal_id);
    ros::serialization::deserialize(stream, goal);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(goal_id);
    size += ros::serialization::serializationLength(goal);
    return size;
  }

  typedef boost::shared_ptr< ::face_detector::FaceDetectorActionGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::face_detector::FaceDetectorActionGoal_<ContainerAllocator>  const> ConstPtr;
}; // struct FaceDetectorActionGoal
typedef  ::face_detector::FaceDetectorActionGoal_<std::allocator<void> > FaceDetectorActionGoal;

typedef boost::shared_ptr< ::face_detector::FaceDetectorActionGoal> FaceDetectorActionGoalPtr;
typedef boost::shared_ptr< ::face_detector::FaceDetectorActionGoal const> FaceDetectorActionGoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::face_detector::FaceDetectorActionGoal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::face_detector::FaceDetectorActionGoal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace face_detector

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::face_detector::FaceDetectorActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4b30be6cd12b9e72826df56b481f40e0";
  }

  static const char* value(const  ::face_detector::FaceDetectorActionGoal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4b30be6cd12b9e72ULL;
  static const uint64_t static_value2 = 0x826df56b481f40e0ULL;
};

template<class ContainerAllocator>
struct DataType< ::face_detector::FaceDetectorActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "face_detector/FaceDetectorActionGoal";
  }

  static const char* value(const  ::face_detector::FaceDetectorActionGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::face_detector::FaceDetectorActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
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
";
  }

  static const char* value(const  ::face_detector::FaceDetectorActionGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::face_detector::FaceDetectorActionGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::face_detector::FaceDetectorActionGoal_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::face_detector::FaceDetectorActionGoal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.goal_id);
    stream.next(m.goal);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct FaceDetectorActionGoal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::face_detector::FaceDetectorActionGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::face_detector::FaceDetectorActionGoal_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "goal_id: ";
s << std::endl;
    Printer< ::actionlib_msgs::GoalID_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_id);
    s << indent << "goal: ";
s << std::endl;
    Printer< ::face_detector::FaceDetectorGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};


} // namespace message_operations
} // namespace ros

#endif // FACE_DETECTOR_MESSAGE_FACEDETECTORACTIONGOAL_H

