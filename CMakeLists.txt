cmake_minimum_required(VERSION 3.12)
project(ping-viewer LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CXX_STANDARD_REQUIRED ON)

set(QT_MIN_VERSION "5.15.0")

set(CMAKE_AUTOMOC ON)
set(AUTOMOC_MOC_OPTIONS -Muri=org.bluerobotics.pingviewer)

set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

if (MSVC)
  add_compile_options(/FS)  # allow concurrent PDB writes
endif()

# Defaults
include("cmake/default.cmake")
# Compiler information
include("cmake/compiler.cmake")
# Update submodules if necessary and load our GIT variables
include("cmake/git.cmake")
# Helper module
include("cmake/helper.cmake")

add_compile_definitions(
    QT_USE_QSTRINGBUILDER
    QT_NO_CAST_TO_ASCII
    QT_STRICT_ITERATORS
    QT_NO_CAST_FROM_BYTEARRAY
    QT_USE_FAST_OPERATOR_PLUS
)

find_package(Qt5 ${QT_MIN_VERSION} REQUIRED NO_MODULE COMPONENTS
    Charts
    Concurrent
    Core
    Network
    Qml
    Quick
    QuickControls2
    SerialPort
    Svg
    Test
    Widgets
)

message(STATUS "Qt Version: ${Qt5_VERSION}")

# global include directories
include_directories(
    lib/fmt/fmt/include/
    lib/libcintelhex/include
    lib/maddy/maddy/include/
    lib/mavlink/c_library_v2/
    lib/mavlink/c_library_v2/minimal
    lib/mavlink/c_library_v2/common
    lib/ping-cpp/ping-cpp/src/message/
)

add_subdirectory(lib/fmt/fmt)

### --- Add this block to make qml/ visible in Qt Creator ---
file(GLOB_RECURSE QML_FILES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/qml/*.qml
    ${CMAKE_CURRENT_SOURCE_DIR}/qml/*.js
)

# Create a dummy target only for IDE display
add_custom_target(qml_files ALL SOURCES ${QML_FILES})

# Group them nicely in Qt Creator's tree
source_group("QML" FILES ${QML_FILES})
### ---

add_subdirectory(src)
