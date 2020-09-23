enable_language(CXX)

set(SOURCES jsk_elmo_shm.cpp ../jsk_common/jsk_elmo.cpp servo_shm.cpp display_shm.cpp list_display.cpp)
set(CMAKE_C_FLAGS "-O3 -Wall -Wextra -std=c++11")
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -std=c++11")
include_directories(../)

add_executable(jsk_elmo_shm ${SOURCES})
target_link_libraries(jsk_elmo_shm soem rt ncurses tinfo)
install(TARGETS jsk_elmo_shm DESTINATION bin)

add_library(eus_shm_interface SHARED euslisp/eus_shm_interface.cpp servo_shm.cpp)
target_link_libraries(eus_shm_interface pthread)
set_target_properties(eus_shm_interface PROPERTIES COMPILE_FLAG "-fPIC" PREFIX "" SUFFIX ".so" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/jsk_sample/elmo_shm/euslisp)
