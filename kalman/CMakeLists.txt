add_library(mjg_kalman STATIC src/mjg_kalman.c)
target_include_directories(mjg_kalman PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
cutils_set_warnings(mjg_kalman)

# Namespaced alias for in-tree and FetchContent consumers. There are
# deliberately no install() or export() rules: the supported way to take
# this component is to copy include/mjg_kalman.h and src/mjg_kalman.c.
add_library(c-utils::kalman ALIAS mjg_kalman)

if(CUTILS_BUILD_TESTS)
    add_executable(test_kalman tests/test_kalman.c)
    target_link_libraries(test_kalman PRIVATE mjg_kalman)
    cutils_set_warnings(test_kalman)
    add_test(NAME kalman COMMAND test_kalman)
endif()

if(CUTILS_BUILD_EXAMPLES)
    add_executable(kalman_tracker examples/tracker.c)
    target_link_libraries(kalman_tracker PRIVATE mjg_kalman)
    cutils_set_warnings(kalman_tracker)

    add_executable(kalman_observer examples/observer.c)
    target_link_libraries(kalman_observer PRIVATE mjg_kalman)
    cutils_set_warnings(kalman_observer)
endif()
