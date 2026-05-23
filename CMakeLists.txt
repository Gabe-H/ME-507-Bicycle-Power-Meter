cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Power_Meter_Breadboard)

# add_subdirectory(lib/icm20948)
add_subdirectory(lib/icm40609)
add_subdirectory(lib/max17048)

target_sources(app PRIVATE src/main.c)
