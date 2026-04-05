cmake_minimum_required(VERSION 3.20.0)

# Register display_ui as a Zephyr extra module so its Kconfig symbols
# and CMake library are picked up automatically.
list(APPEND ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/display_ui
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(display_sample)

target_sources(app PRIVATE src/main.c)
