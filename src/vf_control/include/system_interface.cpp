#ifndef __SYSTEM_INTERFACE_HPP__
#define __SYSTEM_INTERFACE_HPP__
// this is the class that controls the haptic interface using its API,
// you can redefine it to adapt the library to your device
#include "raptor_api_interfaces/msg/in_virtuose_force.hpp"
#include "raptor_api_interfaces/msg/in_virtuose_pose.hpp"
#include "raptor_api_interfaces/msg/in_virtuose_speed.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_force.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_physical_pose.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_pose.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_speed.hpp"
#include "raptor_api_interfaces/msg/out_virtuose_status.hpp"
#include "raptor_api_interfaces/srv/virtuose_impedance.hpp"
#include "raptor_api_interfaces/srv/virtuose_reset.hpp"
#endif  // __SYSTEM_INTERFACE_HPP__