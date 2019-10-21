
set(SOURCES elmo_test.c)
set(CMAKE_C_FLAGS "-O3 -Wall -Wextra")
add_executable(elmo_test ${SOURCES})
target_link_libraries(elmo_test soem)
install(TARGETS elmo_test DESTINATION bin)


