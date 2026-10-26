
add_fsm_state_simple(Idle)
add_fsm_state_simple(Independent)
add_fsm_state_simple(Collaborative)


# Compile CooperativeGrasp implementation into the Collaborative state.
target_sources(Collaborative PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../tasks/cooperativeTask/cooperative.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../tasks/cooperativeTask/inputWrenchTask.cpp
)

add_fsm_data_directory(data)
