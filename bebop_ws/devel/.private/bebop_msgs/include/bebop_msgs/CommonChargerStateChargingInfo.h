// Generated by gencpp from file bebop_msgs/CommonChargerStateChargingInfo.msg
// DO NOT EDIT!


#ifndef BEBOP_MSGS_MESSAGE_COMMONCHARGERSTATECHARGINGINFO_H
#define BEBOP_MSGS_MESSAGE_COMMONCHARGERSTATECHARGINGINFO_H


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
struct CommonChargerStateChargingInfo_
{
  typedef CommonChargerStateChargingInfo_<ContainerAllocator> Type;

  CommonChargerStateChargingInfo_()
    : header()
    , phase(0)
    , rate(0)
    , intensity(0)
    , fullChargingTime(0)  {
    }
  CommonChargerStateChargingInfo_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , phase(0)
    , rate(0)
    , intensity(0)
    , fullChargingTime(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _phase_type;
  _phase_type phase;

   typedef uint8_t _rate_type;
  _rate_type rate;

   typedef uint8_t _intensity_type;
  _intensity_type intensity;

   typedef uint8_t _fullChargingTime_type;
  _fullChargingTime_type fullChargingTime;



  enum {
    phase_UNKNOWN = 0u,
    phase_CONSTANT_CURRENT_1 = 1u,
    phase_CONSTANT_CURRENT_2 = 2u,
    phase_CONSTANT_VOLTAGE = 3u,
    phase_CHARGED = 4u,
    phase_DISCHARGING = 5u,
    rate_UNKNOWN = 0u,
    rate_SLOW = 1u,
    rate_MODERATE = 2u,
    rate_FAST = 3u,
  };


  typedef boost::shared_ptr< ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator> const> ConstPtr;

}; // struct CommonChargerStateChargingInfo_

typedef ::bebop_msgs::CommonChargerStateChargingInfo_<std::allocator<void> > CommonChargerStateChargingInfo;

typedef boost::shared_ptr< ::bebop_msgs::CommonChargerStateChargingInfo > CommonChargerStateChargingInfoPtr;
typedef boost::shared_ptr< ::bebop_msgs::CommonChargerStateChargingInfo const> CommonChargerStateChargingInfoConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6cde652314f80f4da435ec6429960e6d";
  }

  static const char* value(const ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6cde652314f80f4dULL;
  static const uint64_t static_value2 = 0xa435ec6429960e6dULL;
};

template<class ContainerAllocator>
struct DataType< ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bebop_msgs/CommonChargerStateChargingInfo";
  }

  static const char* value(const ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# CommonChargerStateChargingInfo\n\
# auto-generated from up stream XML files at\n\
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml\n\
# To check upstream commit hash, refer to last_build_info file\n\
# Do not modify this file by hand. Check scripts/meta folder for generator files.\n\
#\n\
# SDK Comment: Charging information.\n\
\n\
Header header\n\
\n\
# The current charging phase.\n\
uint8 phase_UNKNOWN=0  # The charge phase is unknown or irrelevant.\n\
uint8 phase_CONSTANT_CURRENT_1=1  # First phase of the charging process. The battery is charging with constant current.\n\
uint8 phase_CONSTANT_CURRENT_2=2  # Second phase of the charging process. The battery is charging with constant current, with a higher voltage than the first phase.\n\
uint8 phase_CONSTANT_VOLTAGE=3  # Last part of the charging process. The battery is charging with a constant voltage.\n\
uint8 phase_CHARGED=4  # The battery is fully charged.\n\
uint8 phase_DISCHARGING=5  # The battery is discharging; Other arguments refers to the last charge.\n\
uint8 phase\n\
# The charge rate. If phase is DISCHARGING, refers to the last charge.\n\
uint8 rate_UNKNOWN=0  # The charge rate is not known.\n\
uint8 rate_SLOW=1  # Slow charge rate.\n\
uint8 rate_MODERATE=2  # Moderate charge rate.\n\
uint8 rate_FAST=3  # Fast charge rate.\n\
uint8 rate\n\
# The charging intensity, in dA. (12dA = 1,2A) ; If phase is DISCHARGING, refers to the last charge. Equals to 0 if not known.\n\
uint8 intensity\n\
# The full charging time estimated, in minute. If phase is DISCHARGING, refers to the last charge. Equals to 0 if not known.\n\
uint8 fullChargingTime\n\
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

  static const char* value(const ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.phase);
      stream.next(m.rate);
      stream.next(m.intensity);
      stream.next(m.fullChargingTime);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CommonChargerStateChargingInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bebop_msgs::CommonChargerStateChargingInfo_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "phase: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.phase);
    s << indent << "rate: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.rate);
    s << indent << "intensity: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.intensity);
    s << indent << "fullChargingTime: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.fullChargingTime);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEBOP_MSGS_MESSAGE_COMMONCHARGERSTATECHARGINGINFO_H
