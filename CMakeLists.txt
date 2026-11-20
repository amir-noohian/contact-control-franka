cmake_minimum_required(VERSION 3.10)
project(contact_control_franka LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED)

add_executable(joint_positions src/joint_positions.cpp)
target_link_libraries(joint_positions PRIVATE Franka::Franka)

add_executable(franka_state src/franka_state.cpp)
target_link_libraries(franka_state PRIVATE Franka::Franka)

add_executable(cartesian_move src/cartesian_move.cpp)
target_link_libraries(cartesian_move PRIVATE Franka::Franka)

add_executable(external_force src/external_force.cpp)
target_link_libraries(external_force PRIVATE Franka::Franka)

add_executable(joint_move src/joint_move.cpp)
target_link_libraries(joint_move PRIVATE Franka::Franka)

add_executable(computed_torque src/computed_torque.cpp)
target_link_libraries(computed_torque PRIVATE Franka::Franka)
target_compile_definitions(computed_torque PRIVATE FRANKA_PROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
