include_directories(../include)

add_executable(simple_2d_tracking simple_2d_tracking.cpp)
add_executable(simple_2d_tracking_tags simple_2d_tracking_tags.cpp)


find_package( OpenCV REQUIRED )

target_link_libraries(simple_2d_tracking rays_bayes_tracking ${OpenCV_LIBS})
target_link_libraries(simple_2d_tracking_tags rays_bayes_tracking ${OpenCV_LIBS})
