enable_language(CXX)

set(SOURCES elmo_test.cpp)
set(CMAKE_C_FLAGS "-O3 -Wall -Wextra -std=c++11")
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -std=c++11")
add_executable(elmo_test ${SOURCES})
target_link_libraries(elmo_test soem rt)
install(TARGETS elmo_test DESTINATION bin)


