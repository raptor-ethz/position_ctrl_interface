#this is necessary to stop double-building of libraries in raptor-core
if(NOT DEFINED IS_PART_OF_RAPTOR_CORE)
    add_subdirectory(communication_lib)
endif()