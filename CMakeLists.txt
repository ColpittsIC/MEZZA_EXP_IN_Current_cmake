# file-format: 1.0.0
cmake_minimum_required(VERSION 3.20)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)

include(cmake/target.cmake)
include(cmake/${TOOLCHAIN_FILE})

project(MEZZA_EXP_IN_Current)
add_executable(MEZZA_EXP_IN_Current)
include(cmake/files.cmake)
include(cmake/flags.cmake)
include(cmake/components.cmake)

# Stubs to be able to add compiler flag overrides, e.g. -Os
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
  # Add additional source files here
  # SPI2 is not (yet) managed by the STM32CubeMX2 codegen tooling in this
  # project, so its mx_spi2.c/.h were added by hand under generated/hal/,
  # following the same mx_<periph>.c/.h pattern as the other peripherals -
  # wired in here instead of generated/hal/STM32_HAL_Driver_codegen.cmake.
  generated/hal/mx_spi2.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
  # Add additional include directories here
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC
  # Add additional defined symbols here
)

target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC
  # Add additional compiler flags here
)

target_link_options(${CMAKE_PROJECT_NAME} PUBLIC
  # Add additional linker flags here
)

target_link_directories(${CMAKE_PROJECT_NAME} PUBLIC
  # Add additional linker search directories here
)

target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC
  # Add additional linker libraries here
)

# Generate .hex and .bin images alongside the .elf after each build
set(FW_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/build/${CMAKE_BUILD_TYPE}")
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex "${FW_OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.elf" "${FW_OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.hex"
  COMMAND ${CMAKE_OBJCOPY} -O binary "${FW_OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.elf" "${FW_OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.bin"
  COMMENT "Generating ${CMAKE_PROJECT_NAME}.hex and ${CMAKE_PROJECT_NAME}.bin"
)
set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_CLEAN_FILES "${FW_OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.hex")
set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_CLEAN_FILES "${FW_OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.bin")
