cmake_minimum_required(VERSION 2.8.3)
project(marine_radar_layer)

find_package(catkin REQUIRED COMPONENTS
    costmap_2d
    dynamic_reconfigure
    pluginlib
    roscpp
    marine_msgs
    geometry_msgs
    tf2_geometry_msgs
)

generate_dynamic_reconfigure_options(cfg/MarineRadarLayer.cfg)

catkin_package(
    CATKIN_DEPENDS
        costmap_2d
        dynamic_reconfigure
        pluginlib
        roscpp
        marine_msgs
        geometry_msgs
        tf2_geometry_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/marine_radar_layer.cpp)
add_dependencies(${PROJECT_NAME} ${marine_radar_layer_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES costmap_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
