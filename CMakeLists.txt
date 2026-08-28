cmake_minimum_required(VERSION 3.15)

project(c-utils LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(CUTILS_BUILD_FSM      "Build the state machine component" ON)
option(CUTILS_BUILD_TESTS    "Build component tests"             ON)
option(CUTILS_BUILD_EXAMPLES "Build component examples"          ON)
option(CUTILS_WERROR         "Treat warnings as errors"          OFF)

# Applied per target so each component stays self-contained and no global
# flag state leaks between them.
function(cutils_set_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4)
        if(CUTILS_WERROR)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
        if(CUTILS_WERROR)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
endfunction()

if(CUTILS_BUILD_TESTS)
    enable_testing()
endif()

if(CUTILS_BUILD_FSM)
    add_subdirectory(fsm)
endif()
