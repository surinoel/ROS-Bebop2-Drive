// Generated by gencpp from file bebop_msgs/CommonCommonStateProductModel.msg
// DO NOT EDIT!


#ifndef BEBOP_MSGS_MESSAGE_COMMONCOMMONSTATEPRODUCTMODEL_H
#define BEBOP_MSGS_MESSAGE_COMMONCOMMONSTATEPRODUCTMODEL_H


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
struct CommonCommonStateProductModel_
{
  typedef CommonCommonStateProductModel_<ContainerAllocator> Type;

  CommonCommonStateProductModel_()
    : header()
    , model(0)  {
    }
  CommonCommonStateProductModel_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , model(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _model_type;
  _model_type model;



  enum {
    model_RS_TRAVIS = 0u,
    model_RS_MARS = 1u,
    model_RS_SWAT = 2u,
    model_RS_MCLANE = 3u,
    model_RS_BLAZE = 4u,
    model_RS_ORAK = 5u,
    model_RS_NEWZ = 6u,
    model_JS_MARSHALL = 7u,
    model_JS_DIESEL = 8u,
    model_JS_BUZZ = 9u,
    model_JS_MAX = 10u,
    model_JS_JETT = 11u,
    model_JS_TUKTUK = 12u,
    model_SW_BLACK = 13u,
    model_SW_WHITE = 14u,
  };


  typedef boost::shared_ptr< ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator> const> ConstPtr;

}; // struct CommonCommonStateProductModel_

typedef ::bebop_msgs::CommonCommonStateProductModel_<std::allocator<void> > CommonCommonStateProductModel;

typedef boost::shared_ptr< ::bebop_msgs::CommonCommonStateProductModel > CommonCommonStateProductModelPtr;
typedef boost::shared_ptr< ::bebop_msgs::CommonCommonStateProductModel const> CommonCommonStateProductModelConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace bebop_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'bebop_msgs': ['/home/nim/catkin_ws/src/bebop_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5577551dc33e452626f964eb7a27a391";
  }

  static const char* value(const ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5577551dc33e4526ULL;
  static const uint64_t static_value2 = 0x26f964eb7a27a391ULL;
};

template<class ContainerAllocator>
struct DataType< ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bebop_msgs/CommonCommonStateProductModel";
  }

  static const char* value(const ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# CommonCommonStateProductModel\n\
# auto-generated from up stream XML files at\n\
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml\n\
# To check upstream commit hash, refer to last_build_info file\n\
# Do not modify this file by hand. Check scripts/meta folder for generator files.\n\
#\n\
# SDK Comment: Product sub-model.\\n This can be used to customize the UI depending on the product.\n\
\n\
Header header\n\
\n\
# The Model of the product.\n\
uint8 model_RS_TRAVIS=0  # Travis (RS taxi) model.\n\
uint8 model_RS_MARS=1  # Mars (RS space) model\n\
uint8 model_RS_SWAT=2  # SWAT (RS SWAT) model\n\
uint8 model_RS_MCLANE=3  # Mc Lane (RS police) model\n\
uint8 model_RS_BLAZE=4  # Blaze (RS fire) model\n\
uint8 model_RS_ORAK=5  # Orak (RS carbon hydrofoil) model\n\
uint8 model_RS_NEWZ=6  # New Z (RS wooden hydrofoil) model\n\
uint8 model_JS_MARSHALL=7  # Marshall (JS fire) model\n\
uint8 model_JS_DIESEL=8  # Diesel (JS SWAT) model\n\
uint8 model_JS_BUZZ=9  # Buzz (JS space) model\n\
uint8 model_JS_MAX=10  # Max (JS F1) model\n\
uint8 model_JS_JETT=11  # Jett (JS flames) model\n\
uint8 model_JS_TUKTUK=12  # Tuk-Tuk (JS taxi) model\n\
uint8 model_SW_BLACK=13  # Swing black model\n\
uint8 model_SW_WHITE=14  # Swing white model\n\
uint8 model\n\
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

  static const char* value(const ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.model);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CommonCommonStateProductModel_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bebop_msgs::CommonCommonStateProductModel_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "model: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.model);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEBOP_MSGS_MESSAGE_COMMONCOMMONSTATEPRODUCTMODEL_H
