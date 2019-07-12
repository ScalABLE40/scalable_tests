cmake_minimum_required(VERSION 2.8.3)
project(scalable_individual_tests)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(rostest/skiros2.test)
  add_rostest(rostest/task_manager.test)
  add_rostest(rostest/iiwa_sim.test)
  add_rostest(rostest/vfk_msb.test)
endif()
