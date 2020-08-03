enable_language(CXX)

set(SOURCES jsk_elmo_torque_sensor.cpp ../jsk_common/jsk_keyboard.cpp ../jsk_common/jsk_elmo.cpp)
set(CMAKE_C_FLAGS "-O3 -Wall -Wextra -std=c++11")
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -std=c++11")
include_directories(../)
add_executable(jsk_elmo_torque_sensor ${SOURCES})
target_link_libraries(jsk_elmo_torque_sensor soem rt)
install(TARGETS jsk_elmo_torque_sensor DESTINATION bin)


