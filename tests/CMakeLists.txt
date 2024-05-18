# Tests for the system library
add_executable(test_system test_system.cpp)
target_link_libraries(test_system system_lib Boost::unit_test_framework)
add_test(NAME test_system COMMAND test_system)

# Tests for the observer library
add_executable(test_observer test_observer.cpp)
target_link_libraries(test_observer observer_lib Boost::unit_test_framework)
add_test(NAME test_observer COMMAND test_observer)

# Tests for the controller library
add_executable(test_controller test_controller.cpp)
target_link_libraries(test_controller controller_lib system_lib Boost::unit_test_framework)
add_test(NAME test_controller COMMAND test_controller)

# Tests for the simulator library
add_executable(test_simulator test_simulator.cpp)
target_link_libraries(test_simulator simulator_lib Boost::unit_test_framework)

