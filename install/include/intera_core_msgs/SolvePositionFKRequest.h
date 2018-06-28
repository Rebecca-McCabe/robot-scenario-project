// Generated by gencpp from file intera_core_msgs/SolvePositionFKRequest.msg
// DO NOT EDIT!


#ifndef INTERA_CORE_MSGS_MESSAGE_SOLVEPOSITIONFKREQUEST_H
#define INTERA_CORE_MSGS_MESSAGE_SOLVEPOSITIONFKREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <sensor_msgs/JointState.h>

namespace intera_core_msgs
{
template <class ContainerAllocator>
struct SolvePositionFKRequest_
{
  typedef SolvePositionFKRequest_<ContainerAllocator> Type;

  SolvePositionFKRequest_()
    : configuration()
    , tip_names()  {
    }
  SolvePositionFKRequest_(const ContainerAllocator& _alloc)
    : configuration(_alloc)
    , tip_names(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::sensor_msgs::JointState_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::sensor_msgs::JointState_<ContainerAllocator> >::other >  _configuration_type;
  _configuration_type configuration;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _tip_names_type;
  _tip_names_type tip_names;





  typedef boost::shared_ptr< ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SolvePositionFKRequest_

typedef ::intera_core_msgs::SolvePositionFKRequest_<std::allocator<void> > SolvePositionFKRequest;

typedef boost::shared_ptr< ::intera_core_msgs::SolvePositionFKRequest > SolvePositionFKRequestPtr;
typedef boost::shared_ptr< ::intera_core_msgs::SolvePositionFKRequest const> SolvePositionFKRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace intera_core_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'intera_core_msgs': ['/home/rebecca/catkin_ws/src/intera_common/intera_core_msgs/msg', '/home/rebecca/catkin_ws/devel/share/intera_core_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "14c88cbabc4e4d6c183969e91f5e56ca";
  }

  static const char* value(const ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x14c88cbabc4e4d6cULL;
  static const uint64_t static_value2 = 0x183969e91f5e56caULL;
};

template<class ContainerAllocator>
struct DataType< ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "intera_core_msgs/SolvePositionFKRequest";
  }

  static const char* value(const ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
sensor_msgs/JointState[] configuration\n\
\n\
\n\
string[] tip_names\n\
\n\
\n\
================================================================================\n\
MSG: sensor_msgs/JointState\n\
# This is a message that holds data to describe the state of a set of torque controlled joints. \n\
#\n\
# The state of each joint (revolute or prismatic) is defined by:\n\
#  * the position of the joint (rad or m),\n\
#  * the velocity of the joint (rad/s or m/s) and \n\
#  * the effort that is applied in the joint (Nm or N).\n\
#\n\
# Each joint is uniquely identified by its name\n\
# The header specifies the time at which the joint states were recorded. All the joint states\n\
# in one message have to be recorded at the same time.\n\
#\n\
# This message consists of a multiple arrays, one for each part of the joint state. \n\
# The goal is to make each of the fields optional. When e.g. your joints have no\n\
# effort associated with them, you can leave the effort array empty. \n\
#\n\
# All arrays in this message should have the same size, or be empty.\n\
# This is the only way to uniquely associate the joint name with the correct\n\
# states.\n\
\n\
\n\
Header header\n\
\n\
string[] name\n\
float64[] position\n\
float64[] velocity\n\
float64[] effort\n\
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
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.configuration);
      stream.next(m.tip_names);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SolvePositionFKRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::intera_core_msgs::SolvePositionFKRequest_<ContainerAllocator>& v)
  {
    s << indent << "configuration[]" << std::endl;
    for (size_t i = 0; i < v.configuration.size(); ++i)
    {
      s << indent << "  configuration[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::sensor_msgs::JointState_<ContainerAllocator> >::stream(s, indent + "    ", v.configuration[i]);
    }
    s << indent << "tip_names[]" << std::endl;
    for (size_t i = 0; i < v.tip_names.size(); ++i)
    {
      s << indent << "  tip_names[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.tip_names[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // INTERA_CORE_MSGS_MESSAGE_SOLVEPOSITIONFKREQUEST_H
