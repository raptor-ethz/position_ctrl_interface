
add_executable(square_traj square_traj.cpp)


target_link_libraries(square_traj PRIVATE
comm_lib
fastcdr 
fastrtps
)


