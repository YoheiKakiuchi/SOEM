enable_language(CXX)

set(SOURCES jsk_elmo_shm.cpp ../jsk_common/jsk_elmo.cpp servo_shm.cpp display_shm.cpp list_display.cpp)
set(CMAKE_C_FLAGS "-O3 -Wall -Wextra -std=c++11")
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -std=c++11")
include_directories(../)
add_executable(jsk_elmo_shm ${SOURCES})
target_link_libraries(jsk_elmo_shm soem rt ncurses tinfo)
install(TARGETS jsk_elmo_shm DESTINATION bin)


