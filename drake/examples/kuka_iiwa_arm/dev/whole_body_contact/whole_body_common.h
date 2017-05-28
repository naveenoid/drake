# pragma once

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace whole_body_contact {
//
//// These constants are used in several locations in the Monolithic
//// pick and place demo.
//const char kIiwaUrdf[] =
//    "/manipulation/models/iiwa_description/urdf/"
//        "iiwa14_polytope_collision.urdf";
//const char kIiwaEndEffectorName[] = "iiwa_link_ee";


// Different states for the pick and place task.
enum PickAndPlaceState {
  APPROACH_PICK_PREGRASP,
  APPROACH_PICK,
  GRASP,
//  LIFT_FROM_PICK,
//  APPROACH_PLACE_PREGRASP,
//  APPROACH_PLACE,
//  PLACE,
//  LIFT_FROM_PLACE,
//  DONE,
};


} // namespace whole_body_contact
} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake