cmake_minimum_required(VERSION 3.16)
project(SoapyICR8600 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SoapySDR CONFIG)
if (NOT SoapySDR_FOUND)
    message(FATAL_ERROR "SoapySDR development package not found.")
endif()

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    get_filename_component(_soapy_root "${SoapySDR_DIR}/../../.." ABSOLUTE)
    set(CMAKE_INSTALL_PREFIX "${_soapy_root}" CACHE PATH "install prefix" FORCE)
    message(STATUS "Defaulting CMAKE_INSTALL_PREFIX to SoapySDR's own install root: ${CMAKE_INSTALL_PREFIX}")
endif()

set(SWIFT_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/SwiftCore" CACHE PATH "Path to the ICR8600Core SwiftPM package")
set(SWIFT_BUILD_CONFIG "debug" CACHE STRING "Swift build configuration to link against (debug or release)")

set(SWIFT_CORE_LIB_DIR "${SWIFT_CORE_DIR}/.build/${SWIFT_BUILD_CONFIG}")
set(SWIFT_CORE_HEADER_DIR "${SWIFT_CORE_DIR}/Sources/CICR8600Core/include")
set(SWIFT_CORE_STATIC_LIB "${SWIFT_CORE_LIB_DIR}/libICR8600Core.a")

if (NOT EXISTS "${SWIFT_CORE_HEADER_DIR}/icr8600_core.h")
    message(FATAL_ERROR "icr8600_core.h not found at ${SWIFT_CORE_HEADER_DIR}")
endif()

execute_process(
    COMMAND xcrun --find swift
    OUTPUT_VARIABLE SWIFT_EXE_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
get_filename_component(SWIFT_BIN_DIR "${SWIFT_EXE_PATH}" DIRECTORY)
set(SWIFT_RUNTIME_LIB_DIR "${SWIFT_BIN_DIR}/../lib/swift/macosx")

SOAPY_SDR_MODULE_UTIL(
    TARGET icr8600Support
    SOURCES SoapyICR8600.cpp
    LIBRARIES
        ICR8600Core
        "-framework IOUSBHost"
        "-framework IOKit"
        "-framework Foundation"
)

target_include_directories(icr8600Support PRIVATE ${SWIFT_CORE_HEADER_DIR})
target_link_directories(icr8600Support PRIVATE ${SWIFT_CORE_LIB_DIR})
target_compile_definitions(icr8600Support PRIVATE
    ICR8600_DEFAULT_FIRMWARE_PATH="${CMAKE_INSTALL_PREFIX}/share/SoapyIC-R8600/spt_seq.json"
)

if (EXISTS "${SWIFT_CORE_STATIC_LIB}")
    set_target_properties(icr8600Support PROPERTIES
        LINK_DEPENDS "${SWIFT_CORE_STATIC_LIB}")
else()
    message(WARNING "libICR8600Core.a not found at ${SWIFT_CORE_STATIC_LIB}; build SwiftCore first.")
endif()

set_target_properties(icr8600Support PROPERTIES
    INSTALL_RPATH "${SWIFT_CORE_LIB_DIR};${SWIFT_RUNTIME_LIB_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE
)

install(FILES README.md LICENSE DESTINATION share/SoapyIC-R8600)
