cmake_minimum_required(VERSION 3.16)
project(stereo_calibration_loader LANGUAGES CXX)
find_package(OpenCV 4 REQUIRED)
add_executable(load_calibration load_calibration.cpp)
target_compile_features(load_calibration PRIVATE cxx_std_17)
target_link_libraries(load_calibration PRIVATE ${OpenCV_LIBS})

