// Generated by gencpp from file homo_msgs/HomographyResult.msg
// DO NOT EDIT!


#ifndef HOMO_MSGS_MESSAGE_HOMOGRAPHYRESULT_H
#define HOMO_MSGS_MESSAGE_HOMOGRAPHYRESULT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace homo_msgs
{
template <class ContainerAllocator>
struct HomographyResult_
{
  typedef HomographyResult_<ContainerAllocator> Type;

  HomographyResult_()
    : header()
    , homography()  {
      homography.assign(0.0);
  }
  HomographyResult_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , homography()  {
  (void)_alloc;
      homography.assign(0.0);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef boost::array<float, 9>  _homography_type;
  _homography_type homography;





  typedef boost::shared_ptr< ::homo_msgs::HomographyResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::homo_msgs::HomographyResult_<ContainerAllocator> const> ConstPtr;

}; // struct HomographyResult_

typedef ::homo_msgs::HomographyResult_<std::allocator<void> > HomographyResult;

typedef boost::shared_ptr< ::homo_msgs::HomographyResult > HomographyResultPtr;
typedef boost::shared_ptr< ::homo_msgs::HomographyResult const> HomographyResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::homo_msgs::HomographyResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::homo_msgs::HomographyResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::homo_msgs::HomographyResult_<ContainerAllocator1> & lhs, const ::homo_msgs::HomographyResult_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.homography == rhs.homography;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::homo_msgs::HomographyResult_<ContainerAllocator1> & lhs, const ::homo_msgs::HomographyResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace homo_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::homo_msgs::HomographyResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::homo_msgs::HomographyResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::homo_msgs::HomographyResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::homo_msgs::HomographyResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::homo_msgs::HomographyResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::homo_msgs::HomographyResult_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::homo_msgs::HomographyResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "18f3159e548aca86612999d64fb8f5fa";
  }

  static const char* value(const ::homo_msgs::HomographyResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x18f3159e548aca86ULL;
  static const uint64_t static_value2 = 0x612999d64fb8f5faULL;
};

template<class ContainerAllocator>
struct DataType< ::homo_msgs::HomographyResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "homo_msgs/HomographyResult";
  }

  static const char* value(const ::homo_msgs::HomographyResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::homo_msgs::HomographyResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"float32[9] homography\n"
"\n"
"\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::homo_msgs::HomographyResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::homo_msgs::HomographyResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.homography);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HomographyResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::homo_msgs::HomographyResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::homo_msgs::HomographyResult_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "homography[]" << std::endl;
    for (size_t i = 0; i < v.homography.size(); ++i)
    {
      s << indent << "  homography[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.homography[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // HOMO_MSGS_MESSAGE_HOMOGRAPHYRESULT_H