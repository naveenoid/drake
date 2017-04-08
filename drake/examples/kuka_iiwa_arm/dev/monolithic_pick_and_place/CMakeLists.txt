if(lcm_FOUND)

  add_subdirectory(action_primitives)

  add_library_with_exports(LIB_NAME pickAndPlaceMonolithic SOURCE_FILES
      demo_diagram_builder.cc
      state_machine_system.cc
      iiwa_state_feedback_plan.cc
      synchronous_world_state.cc)
  target_link_libraries(pickAndPlaceMonolithic
      actionPrimitives
      drakeCommon
      drakeIK
      drakeKukaIiwaArmCommon
      drakeKukaIiwaWorld
      drakeKukaIiwaArmIK
      drakeLcmSystem
      drakeLCMTypes
      drakeLCMUtil
      drakeRBM
      drakeRigidBodyPlant
      drakeSchunkWsg
      drakeSystemPrimitives
      drakeTrajectories
      drakeMultibodyParsers
      robotlocomotion-lcmtypes-cpp)
  drake_install_libraries(pickAndPlaceMonolithic)
  drake_install_headers(
      demo_diagram_builder.h
      iiwa_state_feedback_plan.h
      synchronous_world_state.h
      state_machine_system.h)
  drake_install_pkg_config_file(pick-and-place-monolithic
      TARGET pickAndPlaceMonolithic
      LIBS -lpickAndPlaceMonolithic
      REQUIRES drake-common drake-ik drake-kuka-iiwa-arm-common
      drake-lcm-system drake-lcmtypes-cpp drake-lcm-utils drake-rbm
      drake-system-primitives robotlocomotion-lcmtypes-cpp drake-kuka-iiwa-world
      action-primitives)

  add_executable(monolithic_pick_and_place_demo
      monolithic_pick_and_place_demo.cc)
  target_link_libraries(monolithic_pick_and_place_demo
      drakeKukaIiwaArmCommon
      drakeKukaIiwaArmIK
      drakeLCMTypes
      drakeLCMUtil
      drakeRigidBodyPlant
      pickAndPlaceCommon
      pickAndPlaceMonolithic
      robotlocomotion-lcmtypes-cpp)

endif()