
add_executable(velocity_ctrl_interface
src/main.cpp
)

target_include_directories(velocity_ctrl_interface PUBLIC include)

target_link_libraries(velocity_ctrl_interface PRIVATE
comm_lib
)

# Third party dependencies
target_link_libraries(velocity_ctrl_interface PRIVATE
MAVSDK::mavsdk
fastcdr 
fastrtps
)



