// Generated by gencpp from file extractor/feature.msg
// DO NOT EDIT!


#ifndef EXTRACTOR_MESSAGE_FEATURE_H
#define EXTRACTOR_MESSAGE_FEATURE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>

namespace extractor
{
template <class ContainerAllocator>
struct feature_
{
  typedef feature_<ContainerAllocator> Type;

  feature_()
    : header()
    , position()
    , diameter(0.0)  {
    }
  feature_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , position(_alloc)
    , diameter(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef double _diameter_type;
  _diameter_type diameter;




  typedef boost::shared_ptr< ::extractor::feature_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::extractor::feature_<ContainerAllocator> const> ConstPtr;

}; // struct feature_

typedef ::extractor::feature_<std::allocator<void> > feature;

typedef boost::shared_ptr< ::extractor::feature > featurePtr;
typedef boost::shared_ptr< ::extractor::feature const> featureConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::extractor::feature_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::extractor::feature_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace extractor

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'extractor': ['/home/kevin/Documents/catkin_ws/src/extractor/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::extractor::feature_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::extractor::feature_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::extractor::feature_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::extractor::feature_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::extractor::feature_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::extractor::feature_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::extractor::feature_<ContainerAllocator> >
{
  static const char* value()
  {
    return "77a2cae90db3b4aa44ca6145d57ccfd6";
  }

  static const char* value(const ::extractor::feature_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x77a2cae90db3b4aaULL;
  static const uint64_t static_value2 = 0x44ca6145d57ccfd6ULL;
};

template<class ContainerAllocator>
struct DataType< ::extractor::feature_<ContainerAllocator> >
{
  static const char* value()
  {
    return "extractor/feature";
  }

  static const char* value(const ::extractor::feature_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::extractor::feature_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
geometry_msgs/Point position\n\
float64 diameter\n\
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
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::extractor::feature_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::extractor::feature_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.position);
      stream.next(m.diameter);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct feature_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::extractor::feature_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::extractor::feature_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "diameter: ";
    Printer<double>::stream(s, indent + "  ", v.diameter);
  }
};

} // namespace message_operations
} // namespace ros

#endif // EXTRACTOR_MESSAGE_FEATURE_H
