add_library(mjg_fsm STATIC src/mjg_fsm.c)
target_include_directories(mjg_fsm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
cutils_set_warnings(mjg_fsm)

# Namespaced alias for in-tree and FetchContent consumers. There are
# deliberately no install() or export() rules: the supported way to take
# this component is to copy include/mjg_fsm.h and src/mjg_fsm.c.
add_library(c-utils::fsm ALIAS mjg_fsm)

if(CUTILS_BUILD_TESTS)
    add_executable(test_fsm tests/test_fsm.c)
    target_link_libraries(test_fsm PRIVATE mjg_fsm)
    cutils_set_warnings(test_fsm)
    add_test(NAME fsm COMMAND test_fsm)
endif()

if(CUTILS_BUILD_EXAMPLES)
    add_executable(fsm_traffic_light examples/traffic_light.c)
    target_link_libraries(fsm_traffic_light PRIVATE mjg_fsm)
    cutils_set_warnings(fsm_traffic_light)
endif()
