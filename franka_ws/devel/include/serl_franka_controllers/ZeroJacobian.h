// Generated by gencpp from file serl_franka_controllers/ZeroJacobian.msg
// DO NOT EDIT!


#ifndef SERL_FRANKA_CONTROLLERS_MESSAGE_ZEROJACOBIAN_H
#define SERL_FRANKA_CONTROLLERS_MESSAGE_ZEROJACOBIAN_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace serl_franka_controllers
{
template <class ContainerAllocator>
struct ZeroJacobian_
{
  typedef ZeroJacobian_<ContainerAllocator> Type;

  ZeroJacobian_()
    : zero_jacobian()  {
      zero_jacobian.assign(0.0);
  }
  ZeroJacobian_(const ContainerAllocator& _alloc)
    : zero_jacobian()  {
  (void)_alloc;
      zero_jacobian.assign(0.0);
  }



   typedef boost::array<double, 42>  _zero_jacobian_type;
  _zero_jacobian_type zero_jacobian;





  typedef boost::shared_ptr< ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator> const> ConstPtr;

}; // struct ZeroJacobian_

typedef ::serl_franka_controllers::ZeroJacobian_<std::allocator<void> > ZeroJacobian;

typedef boost::shared_ptr< ::serl_franka_controllers::ZeroJacobian > ZeroJacobianPtr;
typedef boost::shared_ptr< ::serl_franka_controllers::ZeroJacobian const> ZeroJacobianConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator1> & lhs, const ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator2> & rhs)
{
  return lhs.zero_jacobian == rhs.zero_jacobian;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator1> & lhs, const ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace serl_franka_controllers

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator> >
{
  static const char* value()
  {
    return "573da0494fbe019a7da2ae31329663cf";
  }

  static const char* value(const ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x573da0494fbe019aULL;
  static const uint64_t static_value2 = 0x7da2ae31329663cfULL;
};

template<class ContainerAllocator>
struct DataType< ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator> >
{
  static const char* value()
  {
    return "serl_franka_controllers/ZeroJacobian";
  }

  static const char* value(const ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[42] zero_jacobian\n"
;
  }

  static const char* value(const ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.zero_jacobian);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ZeroJacobian_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::serl_franka_controllers::ZeroJacobian_<ContainerAllocator>& v)
  {
    s << indent << "zero_jacobian[]" << std::endl;
    for (size_t i = 0; i < v.zero_jacobian.size(); ++i)
    {
      s << indent << "  zero_jacobian[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.zero_jacobian[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // SERL_FRANKA_CONTROLLERS_MESSAGE_ZEROJACOBIAN_H
