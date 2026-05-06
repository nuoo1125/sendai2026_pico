add_executable(sendai2026_pico sendai2026_pico.cpp)

pico_set_program_name(sendai2026_pico "sendai2026_pico")
pico_set_program_version(sendai2026_pico "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(sendai2026_pico 0)
pico_enable_stdio_usb(sendai2026_pico 0)
target_include_directories(sendai2026_pico PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)
target_include_directories(sendai2026_pico PRIVATE ../lib)
target_link_libraries(sendai2026_pico  gyro VL53L0X servo interface motor pico_bootsel_via_double_reset mcp3208) 
target_link_libraries(sendai2026_pico pico_stdlib hardware_pwm hardware_spi hardware_i2c hardware_pio hardware_clocks hardware_uart)

pico_add_extra_outputs(sendai2026_pico)

