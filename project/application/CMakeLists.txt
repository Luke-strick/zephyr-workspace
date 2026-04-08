cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/display_ui
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/ahrs
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/nav
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tracker_app)

target_sources(app PRIVATE
  src/main.c
  src/config.c
  src/data_engine.c
  src/data_processor.c
  src/lora_comms.c
  src/display_engine.c
)

target_include_directories(app PRIVATE src)
