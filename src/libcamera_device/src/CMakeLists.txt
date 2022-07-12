cmake_minimum_required(VERSION 3.0.2)

add_subdirectory(core)
add_subdirectory(encoder)
add_subdirectory(preview)
add_subdirectory(image)
add_subdirectory(post_processing_stages)

# Add node executable.
add_executable(${PROJECT_NAME}_node camera_node.cpp camera_messenger.cpp)
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} libcamera_app)