
add_executable(pos_ctrl_interface
src/main.cpp
#src/takeoff.cpp
)

target_include_directories(pos_ctrl_interface PUBLIC include)

target_link_libraries(pos_ctrl_interface PRIVATE
comm_lib
)

# Third party dependencies
target_link_libraries(pos_ctrl_interface PRIVATE
#Eigen3::Eigen
MAVSDK::mavsdk
fastcdr 
fastrtps
)



