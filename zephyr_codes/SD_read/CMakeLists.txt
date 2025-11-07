cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dev_MicMaster_SDFast)

# Include the `include/` directory for headers
target_include_directories(app PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add all source files in `src/`
file(GLOB APP_SOURCES src/*.c)

target_sources(app PRIVATE ${APP_SOURCES})

# NORDIC SDK APP END
zephyr_library_include_directories(.)