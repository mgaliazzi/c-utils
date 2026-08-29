add_library(mjg_pid STATIC src/mjg_pid.c)
target_include_directories(mjg_pid PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
cutils_set_warnings(mjg_pid)

# Namespaced alias for in-tree and FetchContent consumers. There are
# deliberately no install() or export() rules: the supported way to take
# this component is to copy include/mjg_pid.h and src/mjg_pid.c.
add_library(c-utils::pid ALIAS mjg_pid)

if(CUTILS_BUILD_TESTS)
    add_executable(test_pid tests/test_pid.c)
    target_link_libraries(test_pid PRIVATE mjg_pid)
    cutils_set_warnings(test_pid)
    add_test(NAME pid COMMAND test_pid)
endif()

if(CUTILS_BUILD_EXAMPLES)
    add_executable(pid_heater examples/heater.c)
    target_link_libraries(pid_heater PRIVATE mjg_pid)
    cutils_set_warnings(pid_heater)
endif()
