cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ME-507-Power-Meter)

# add_subdirectory(lib/icm20948)
add_subdirectory(lib/icm40609)
add_subdirectory(lib/max17048)
add_subdirectory(lib/ble)

target_sources(app PRIVATE
    src/main.c
    src/app_ipc.c
    src/imu_thread.c
    src/adc_thread.c
    src/filter_thread.c
    src/battery_thread.c
    src/ble_thread.c
)
