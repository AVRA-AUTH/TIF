cmake_minimum_required(VERSION 3.8)
project(asv_exhibition)

find_package(ament_cmake REQUIRED)

# Install directories (launch, models, worlds)
install(DIRECTORY
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/obstacle_spawner.py
  scripts/game_manager.py
  scripts/asv_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
