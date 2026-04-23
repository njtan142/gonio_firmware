# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ssd1306_test)

# Build SPIFFS image for the external 8MB flash chip.
# After "idf.py build", flash it to the external chip with:
#   esptool.py --chip esp32c3 --port <PORT> --spi-connection 4,5,6,7 write_flash 0x0 build/spiffs_ext.bin
idf_build_get_property(python PYTHON)
idf_build_get_property(idf_path IDF_PATH)
add_custom_target(spiffs_image_ext ALL
    COMMAND ${python} ${idf_path}/components/spiffs/spiffsgen.py
            8388608 ${CMAKE_SOURCE_DIR}/spiffs_image ${CMAKE_BINARY_DIR}/spiffs_ext.bin
    COMMENT "Generating SPIFFS image -> build/spiffs_ext.bin"
)