cmake_minimum_required(VERSION 3.18)
include(FetchContent)

project("ed_reader")

set_property(GLOBAL PROPERTY CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

FetchContent_Declare( fftlib 
    GIT_REPOSITORY https://github.com/epicyclism/fftlib
    GIT_TAG origin/master
    )
FetchContent_MakeAvailable(fftlib)
include_directories(${fftlib_SOURCE_DIR}/fftlib)

add_executable(ed_reader main.cpp basic_file.h gpio_help.h)
target_link_libraries(ed_reader gpiodcxx fftlib)

add_executable(calib_aq calibrate.cpp basic_file.h gpio_help.h)
target_link_libraries(calib_aq gpiodcxx)

add_executable(tacho tacho.cpp gpio_help.h)
target_link_libraries(tacho gpiodcxx pthread)

add_executable(polar polar.cpp gpio_help.h)
target_link_libraries(polar gpiodcxx pthread)
