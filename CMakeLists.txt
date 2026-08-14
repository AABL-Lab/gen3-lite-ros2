cmake_minimum_required(VERSION 3.8)
project(gen3-lite-ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/joy_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joy_teleop
)

install(PROGRAMS
  src/joint_playback.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_playback
)

install(PROGRAMS
  src/joint_recorder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_recorder
)

install(PROGRAMS
  src/behavior_recorder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME behavior_recorder
)

install(PROGRAMS
  src/joy_web_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joy_web_bridge
)

install(PROGRAMS
  src/emoji_web_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME emoji_web_bridge
)

install(DIRECTORY
  arrows
  emojis
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
