cmake_minimum_required(VERSION 3.23)
project(Arduino C CXX)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(Arduino
        L298N_motor_template.ino
        PID_Control_Template_2019.ino)


set_target_properties(Arduino PROPERTIES LINKER_LANGUAGE CXX)
