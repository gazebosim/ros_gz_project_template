add_library(BasicSystem
  SHARED
  BasicSystem.cpp)

target_link_libraries(BasicSystem
  PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
)

target_include_directories(BasicSystem
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

install(TARGETS BasicSystem DESTINATION lib/${PROJECT_NAME})
