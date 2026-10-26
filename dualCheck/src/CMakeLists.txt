set(controller_SRC
  DualCheck.cpp
)

set(controller_HDR
  DualCheck.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DDualCheck_EXPORTS")

