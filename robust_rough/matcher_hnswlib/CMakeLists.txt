add_library(matcher_hnswlib INTERFACE)

target_include_directories(matcher_hnswlib INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
)
