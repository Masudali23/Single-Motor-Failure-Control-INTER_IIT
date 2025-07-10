px4_add_module(
    MODULE modules__motor_failure_detect_custom
    MAIN motor_failure_detect_custom
    SRCS
    	motor_failure_detect_custom.cpp
        motor_failure_detect_custom.hpp

    DEPENDS
        drivers__device
        mathlib
)
