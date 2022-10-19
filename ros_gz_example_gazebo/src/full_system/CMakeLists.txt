add_library(FullSystem
  SHARED
  FullSystem.cpp)

target_link_libraries(
  FullSystem
  PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
)

target_include_directories(FullSystem
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

install(TARGETS FullSystem DESTINATION lib/${PROJECT_NAME})
