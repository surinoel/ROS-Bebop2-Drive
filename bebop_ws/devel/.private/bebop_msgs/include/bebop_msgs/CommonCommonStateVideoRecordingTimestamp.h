// Generated by gencpp from file bebop_msgs/CommonCommonStateVideoRecordingTimestamp.msg
// DO NOT EDIT!


#ifndef BEBOP_MSGS_MESSAGE_COMMONCOMMONSTATEVIDEORECORDINGTIMESTAMP_H
#define BEBOP_MSGS_MESSAGE_COMMONCOMMONSTATEVIDEORECORDINGTIMESTAMP_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace bebop_msgs
{
template <class ContainerAllocator>
struct CommonCommonStateVideoRecordingTimestamp_
{
  typedef CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator> Type;

  CommonCommonStateVideoRecordingTimestamp_()
    : header()
    , startTimestamp(0)
    , stopTimestamp(0)  {
    }
  CommonCommonStateVideoRecordingTimestamp_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , startTimestamp(0)
    , stopTimestamp(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint64_t _startTimestamp_type;
  _startTimestamp_type startTimestamp;

   typedef uint64_t _stopTimestamp_type;
  _stopTimestamp_type stopTimestamp;





  typedef boost::shared_ptr< ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator> const> ConstPtr;

}; // struct CommonCommonStateVideoRecordingTimestamp_

typedef ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<std::allocator<void> > CommonCommonStateVideoRecordingTimestamp;

typedef boost::shared_ptr< ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp > CommonCommonStateVideoRecordingTimestampPtr;
typedef boost::shared_ptr< ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp const> CommonCommonStateVideoRecordingTimestampConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace bebop_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'bebop_msgs': ['/home/nim/bebop_ws/src/bebop_autonomy/bebop_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2af0a016042e73c407f0fbf66360342c";
  }

  static const char* value(const ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2af0a016042e73c4ULL;
  static const uint64_t static_value2 = 0x07f0fbf66360342cULL;
};

template<class ContainerAllocator>
struct DataType< ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bebop_msgs/CommonCommonStateVideoRecordingTimestamp";
  }

  static const char* value(const ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# CommonCommonStateVideoRecordingTimestamp\n\
# auto-generated from up stream XML files at\n\
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml\n\
# To check upstream commit hash, refer to last_build_info file\n\
# Do not modify this file by hand. Check scripts/meta folder for generator files.\n\
#\n\
# SDK Comment: Current or last video recording timestamp.\\n Timestamp in milliseconds since 00:00:00 UTC on 1 January 1970.\\n **Please note that values dont persist after drone reboot**\n\
\n\
Header header\n\
\n\
# Timestamp in milliseconds since 00:00:00 UTC on 1 January 1970.\n\
uint64 startTimestamp\n\
# Timestamp in milliseconds since 00:00:00 UTC on 1 January 1970. 0 mean that video is still recording.\n\
uint64 stopTimestamp\n\
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

  static const char* value(const ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.startTimestamp);
      stream.next(m.stopTimestamp);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CommonCommonStateVideoRecordingTimestamp_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bebop_msgs::CommonCommonStateVideoRecordingTimestamp_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "startTimestamp: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.startTimestamp);
    s << indent << "stopTimestamp: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.stopTimestamp);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEBOP_MSGS_MESSAGE_COMMONCOMMONSTATEVIDEORECORDINGTIMESTAMP_H
