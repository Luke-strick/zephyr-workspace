zephyr_library()
zephyr_library_sources(src/nav.c)
zephyr_include_directories(include)
zephyr_library_link_libraries(m)
