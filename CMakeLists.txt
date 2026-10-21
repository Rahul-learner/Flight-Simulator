cmake_minimum_required(VERSION 3.13)

# Include Raspberry Pi Pico SDK functions
include(pico_sdk_import.cmake)

# Overclock Pico default clock to 250 MHz so all peripheral baud rates calculate accurately
set(PICO_DEFAULT_CPU_FREQ_MHZ 250)

project(Koyal_Flight_Controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable
add_executable(Koyal_Flight_Controller
    Koyal_Flight_Controller.cpp
)

# Pull in Raspberry Pi Pico hardware libraries
target_link_libraries(Koyal_Flight_Controller
    pico_stdlib
    hardware_gpio
    hardware_i2c
    hardware_clocks
    hardware_uart
)

# Enable USB serial output for PC console and disable UART stdio (UART0 is dedicated to ESP8266)
pico_enable_stdio_usb(Koyal_Flight_Controller 1)
pico_enable_stdio_uart(Koyal_Flight_Controller 0)

# Create map/bin/hex/uf2 file outputs
pico_add_extra_outputs(Koyal_Flight_Controller)
