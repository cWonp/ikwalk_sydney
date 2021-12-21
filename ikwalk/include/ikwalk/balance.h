// Generated by gencpp from file set_walk/balance.msg
// DO NOT EDIT!


#ifndef SET_WALK_MESSAGE_BALANCE_H
#define SET_WALK_MESSAGE_BALANCE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace set_walk
{
template <class ContainerAllocator>
struct balance_
{
  typedef balance_<ContainerAllocator> Type;

  balance_()
    : pitch_GP(0.0)
    , pitch_GI(0.0)
    , pitch_GD(0.0)
    , pitch_ELIMIT(0.0)
    , pitch_OLIMIT(0.0)
    , pitch_neg_Target(0.0)
    , pitch_pos_Target(0.0)
    , roll_GP(0.0)
    , roll_GI(0.0)
    , roll_GD(0.0)
    , roll_ELIMIT(0.0)
    , roll_OLIMIT(0.0)
    , roll_neg_Target(0.0)
    , roll_pos_Target(0.0)
    , pid_onoff(0)
    , pelvic_amp(0.0)
    , knee_amp(0.0)
    , ankle_amp(0.0)
    , roll_pelvic_amp(0.0)
    , roll_ankle_amp(0.0)
    , pid_roll_onoff(0)
    , amp_time(0.0)
    , time_pos(0.0)
    , time_neg(0.0)
    , time_con_onoff(0)
    , linear_increase_value(0.0)
    , linear_decrease_value(0.0)
    , user_time_ent(0.0)  {
    }
  balance_(const ContainerAllocator& _alloc)
    : pitch_GP(0.0)
    , pitch_GI(0.0)
    , pitch_GD(0.0)
    , pitch_ELIMIT(0.0)
    , pitch_OLIMIT(0.0)
    , pitch_neg_Target(0.0)
    , pitch_pos_Target(0.0)
    , roll_GP(0.0)
    , roll_GI(0.0)
    , roll_GD(0.0)
    , roll_ELIMIT(0.0)
    , roll_OLIMIT(0.0)
    , roll_neg_Target(0.0)
    , roll_pos_Target(0.0)
    , pid_onoff(0)
    , pelvic_amp(0.0)
    , knee_amp(0.0)
    , ankle_amp(0.0)
    , roll_pelvic_amp(0.0)
    , roll_ankle_amp(0.0)
    , pid_roll_onoff(0)
    , amp_time(0.0)
    , time_pos(0.0)
    , time_neg(0.0)
    , time_con_onoff(0)
    , linear_increase_value(0.0)
    , linear_decrease_value(0.0)
    , user_time_ent(0.0)  {
  (void)_alloc;
    }



   typedef double _pitch_GP_type;
  _pitch_GP_type pitch_GP;

   typedef double _pitch_GI_type;
  _pitch_GI_type pitch_GI;

   typedef double _pitch_GD_type;
  _pitch_GD_type pitch_GD;

   typedef double _pitch_ELIMIT_type;
  _pitch_ELIMIT_type pitch_ELIMIT;

   typedef double _pitch_OLIMIT_type;
  _pitch_OLIMIT_type pitch_OLIMIT;

   typedef double _pitch_neg_Target_type;
  _pitch_neg_Target_type pitch_neg_Target;

   typedef double _pitch_pos_Target_type;
  _pitch_pos_Target_type pitch_pos_Target;

   typedef double _roll_GP_type;
  _roll_GP_type roll_GP;

   typedef double _roll_GI_type;
  _roll_GI_type roll_GI;

   typedef double _roll_GD_type;
  _roll_GD_type roll_GD;

   typedef double _roll_ELIMIT_type;
  _roll_ELIMIT_type roll_ELIMIT;

   typedef double _roll_OLIMIT_type;
  _roll_OLIMIT_type roll_OLIMIT;

   typedef double _roll_neg_Target_type;
  _roll_neg_Target_type roll_neg_Target;

   typedef double _roll_pos_Target_type;
  _roll_pos_Target_type roll_pos_Target;

   typedef int32_t _pid_onoff_type;
  _pid_onoff_type pid_onoff;

   typedef double _pelvic_amp_type;
  _pelvic_amp_type pelvic_amp;

   typedef double _knee_amp_type;
  _knee_amp_type knee_amp;

   typedef double _ankle_amp_type;
  _ankle_amp_type ankle_amp;

   typedef double _roll_pelvic_amp_type;
  _roll_pelvic_amp_type roll_pelvic_amp;

   typedef double _roll_ankle_amp_type;
  _roll_ankle_amp_type roll_ankle_amp;

   typedef int32_t _pid_roll_onoff_type;
  _pid_roll_onoff_type pid_roll_onoff;

   typedef double _amp_time_type;
  _amp_time_type amp_time;

   typedef double _time_pos_type;
  _time_pos_type time_pos;

   typedef double _time_neg_type;
  _time_neg_type time_neg;

   typedef int32_t _time_con_onoff_type;
  _time_con_onoff_type time_con_onoff;

   typedef double _linear_increase_value_type;
  _linear_increase_value_type linear_increase_value;

   typedef double _linear_decrease_value_type;
  _linear_decrease_value_type linear_decrease_value;

   typedef double _user_time_ent_type;
  _user_time_ent_type user_time_ent;





  typedef boost::shared_ptr< ::set_walk::balance_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::set_walk::balance_<ContainerAllocator> const> ConstPtr;

}; // struct balance_

typedef ::set_walk::balance_<std::allocator<void> > balance;

typedef boost::shared_ptr< ::set_walk::balance > balancePtr;
typedef boost::shared_ptr< ::set_walk::balance const> balanceConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::set_walk::balance_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::set_walk::balance_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace set_walk

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'set_walk': ['/home/robit/catkin_ws/src/set_walk/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::set_walk::balance_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::set_walk::balance_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::set_walk::balance_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::set_walk::balance_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::set_walk::balance_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::set_walk::balance_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::set_walk::balance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5783c9d6e4c1721a439a82c521368b5c";
  }

  static const char* value(const ::set_walk::balance_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5783c9d6e4c1721aULL;
  static const uint64_t static_value2 = 0x439a82c521368b5cULL;
};

template<class ContainerAllocator>
struct DataType< ::set_walk::balance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "set_walk/balance";
  }

  static const char* value(const ::set_walk::balance_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::set_walk::balance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 pitch_GP\n\
float64 pitch_GI\n\
float64 pitch_GD\n\
float64 pitch_ELIMIT\n\
float64 pitch_OLIMIT\n\
float64 pitch_neg_Target\n\
float64 pitch_pos_Target\n\
float64 roll_GP\n\
float64 roll_GI\n\
float64 roll_GD\n\
float64 roll_ELIMIT\n\
float64 roll_OLIMIT\n\
float64 roll_neg_Target\n\
float64 roll_pos_Target\n\
int32 pid_onoff\n\
float64 pelvic_amp\n\
float64 knee_amp\n\
float64 ankle_amp\n\
float64 roll_pelvic_amp\n\
float64 roll_ankle_amp\n\
int32 pid_roll_onoff\n\
float64 amp_time\n\
float64 time_pos\n\
float64 time_neg\n\
int32 time_con_onoff\n\
float64 linear_increase_value \n\
float64 linear_decrease_value\n\
float64 user_time_ent\n\
";
  }

  static const char* value(const ::set_walk::balance_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::set_walk::balance_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pitch_GP);
      stream.next(m.pitch_GI);
      stream.next(m.pitch_GD);
      stream.next(m.pitch_ELIMIT);
      stream.next(m.pitch_OLIMIT);
      stream.next(m.pitch_neg_Target);
      stream.next(m.pitch_pos_Target);
      stream.next(m.roll_GP);
      stream.next(m.roll_GI);
      stream.next(m.roll_GD);
      stream.next(m.roll_ELIMIT);
      stream.next(m.roll_OLIMIT);
      stream.next(m.roll_neg_Target);
      stream.next(m.roll_pos_Target);
      stream.next(m.pid_onoff);
      stream.next(m.pelvic_amp);
      stream.next(m.knee_amp);
      stream.next(m.ankle_amp);
      stream.next(m.roll_pelvic_amp);
      stream.next(m.roll_ankle_amp);
      stream.next(m.pid_roll_onoff);
      stream.next(m.amp_time);
      stream.next(m.time_pos);
      stream.next(m.time_neg);
      stream.next(m.time_con_onoff);
      stream.next(m.linear_increase_value);
      stream.next(m.linear_decrease_value);
      stream.next(m.user_time_ent);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct balance_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::set_walk::balance_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::set_walk::balance_<ContainerAllocator>& v)
  {
    s << indent << "pitch_GP: ";
    Printer<double>::stream(s, indent + "  ", v.pitch_GP);
    s << indent << "pitch_GI: ";
    Printer<double>::stream(s, indent + "  ", v.pitch_GI);
    s << indent << "pitch_GD: ";
    Printer<double>::stream(s, indent + "  ", v.pitch_GD);
    s << indent << "pitch_ELIMIT: ";
    Printer<double>::stream(s, indent + "  ", v.pitch_ELIMIT);
    s << indent << "pitch_OLIMIT: ";
    Printer<double>::stream(s, indent + "  ", v.pitch_OLIMIT);
    s << indent << "pitch_neg_Target: ";
    Printer<double>::stream(s, indent + "  ", v.pitch_neg_Target);
    s << indent << "pitch_pos_Target: ";
    Printer<double>::stream(s, indent + "  ", v.pitch_pos_Target);
    s << indent << "roll_GP: ";
    Printer<double>::stream(s, indent + "  ", v.roll_GP);
    s << indent << "roll_GI: ";
    Printer<double>::stream(s, indent + "  ", v.roll_GI);
    s << indent << "roll_GD: ";
    Printer<double>::stream(s, indent + "  ", v.roll_GD);
    s << indent << "roll_ELIMIT: ";
    Printer<double>::stream(s, indent + "  ", v.roll_ELIMIT);
    s << indent << "roll_OLIMIT: ";
    Printer<double>::stream(s, indent + "  ", v.roll_OLIMIT);
    s << indent << "roll_neg_Target: ";
    Printer<double>::stream(s, indent + "  ", v.roll_neg_Target);
    s << indent << "roll_pos_Target: ";
    Printer<double>::stream(s, indent + "  ", v.roll_pos_Target);
    s << indent << "pid_onoff: ";
    Printer<int32_t>::stream(s, indent + "  ", v.pid_onoff);
    s << indent << "pelvic_amp: ";
    Printer<double>::stream(s, indent + "  ", v.pelvic_amp);
    s << indent << "knee_amp: ";
    Printer<double>::stream(s, indent + "  ", v.knee_amp);
    s << indent << "ankle_amp: ";
    Printer<double>::stream(s, indent + "  ", v.ankle_amp);
    s << indent << "roll_pelvic_amp: ";
    Printer<double>::stream(s, indent + "  ", v.roll_pelvic_amp);
    s << indent << "roll_ankle_amp: ";
    Printer<double>::stream(s, indent + "  ", v.roll_ankle_amp);
    s << indent << "pid_roll_onoff: ";
    Printer<int32_t>::stream(s, indent + "  ", v.pid_roll_onoff);
    s << indent << "amp_time: ";
    Printer<double>::stream(s, indent + "  ", v.amp_time);
    s << indent << "time_pos: ";
    Printer<double>::stream(s, indent + "  ", v.time_pos);
    s << indent << "time_neg: ";
    Printer<double>::stream(s, indent + "  ", v.time_neg);
    s << indent << "time_con_onoff: ";
    Printer<int32_t>::stream(s, indent + "  ", v.time_con_onoff);
    s << indent << "linear_increase_value: ";
    Printer<double>::stream(s, indent + "  ", v.linear_increase_value);
    s << indent << "linear_decrease_value: ";
    Printer<double>::stream(s, indent + "  ", v.linear_decrease_value);
    s << indent << "user_time_ent: ";
    Printer<double>::stream(s, indent + "  ", v.user_time_ent);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SET_WALK_MESSAGE_BALANCE_H
