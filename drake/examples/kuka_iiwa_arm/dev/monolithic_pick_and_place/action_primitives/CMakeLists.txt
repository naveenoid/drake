if(lcm_FOUND)
  add_library_with_exports(LIB_NAME actionPrimitives SOURCE_FILES
      action_primitives_base.cc
      gripper_action.cc
      iiwa_move.cc)
  target_link_libraries(actionPrimitives
      drakeCommon
      drakeIK
      drakeKukaIiwaArmCommon
      drakeKukaIiwaArmIK
      drakeLcmSystem
      drakeLCMTypes
      drakeLCMUtil
      drakeRBM
      drakeMultibodyParsers
      pickAndPlaceCommon
      robotlocomotion-lcmtypes-cpp)
  drake_install_libraries(actionPrimitives)
  drake_install_headers(
      action_primitives_common.h
      action_primitives_base.h
      gripper_action.h
      iiwa_move.h)
drake_install_pkg_config_file(action-primitives
      TARGET actionPrimitives
      LIBS -lactionPrimitives
      REQUIRES drake-common drake-ik drake-kuka-iiwa-arm-common
      drake-lcm-system drake-lcmtypes-cpp drake-lcm-utils drake-rbm
      pick-and-place-common
      robotlocomotion-lcmtypes-cpp)
endif()