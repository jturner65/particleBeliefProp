###############################################
# apps/particleBeliefProp
file(GLOB particleBeliefProp_srcs "*.cpp")
file(GLOB particleBeliefProp_hdrs "*.h")
add_executable(particleBeliefProp ${particleBeliefProp_srcs} ${particleBeliefProp_hdrs})
target_link_libraries(particleBeliefProp dart)
set_target_properties(particleBeliefProp PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
