#-------------------------------------------------------------------------------
# exampleApp executables setup

project(RadiumExampleApps)

# short convenience target from cmake/cleanCpppExtensions.cpp
add_custom_target(test-run)
add_custom_target(${PROJECT_NAME})

# These variable are normally set by the find_package(Radium) for client applications
set(RADIUM_ROOT_DIR "${CMAKE_INSTALL_PREFIX}/")
set(RADIUM_RESOURCES_DIR "${CMAKE_INSTALL_PREFIX}/Resources")
set(RADIUM_PLUGINS_DIR "${CMAKE_INSTALL_PREFIX}/Plugins")

# must find the Qt installation directory. This will be set by RadiumConfig.cmake after install
# TODO : find a way to that more efficiently
find_dependency(Qt5 COMPONENTS Core REQUIRED)
get_target_property(QtCore_Dll Qt5::Core LOCATION)
get_filename_component(QtDlls_location "${QtCore_Dll}" DIRECTORY)
include(radium_setup_functions)

add_executable(CoreExampleApp
    CoreExampleApp/CoreExampleApp.cpp)
target_link_libraries(CoreExampleApp PUBLIC Core)
configure_radium_app(NAME CoreExampleApp)
add_dependencies(${PROJECT_NAME} CoreExampleApp)

#if (RADIUM_GENERATE_LIB_ENGINE)
find_package(Qt5 COMPONENTS Core Widgets OpenGL Xml REQUIRED)
find_package(OpenGL REQUIRED)
set(CMAKE_AUTOMOC ON)
add_executable(HelloRadium MACOSX_BUNDLE
    HelloRadium/main.cpp
    HelloRadium/minimalapp.cpp
    HelloRadium/minimalapp.hpp
    HelloRadium/minimalradium.cpp
    HelloRadium/minimalradium.hpp)
target_include_directories(HelloRadium PRIVATE HelloRadium)
target_link_libraries(HelloRadium PUBLIC
    Core Engine GuiBase
    Qt5::Core Qt5::Widgets Qt5::OpenGL Qt5::Xml
    OpenGL::GL)
add_dependencies(${PROJECT_NAME} HelloRadium)
message(STATUS " Installing HelloRadium into ${CMAKE_INSTALL_PREFIX}")
configure_radium_app(NAME HelloRadium)

add_executable(DrawPrimitivesDemo MACOSX_BUNDLE
    DrawPrimitivesApp/main.cpp
    DrawPrimitivesApp/minimalapp.cpp
    DrawPrimitivesApp/minimalapp.hpp
    DrawPrimitivesApp/minimalradium.cpp
    DrawPrimitivesApp/minimalradium.hpp)
target_include_directories(DrawPrimitivesDemo PRIVATE DrawPrimitivesApp)
target_link_libraries(DrawPrimitivesDemo PUBLIC
    Core Engine GuiBase
    Qt5::Core Qt5::Widgets Qt5::OpenGL Qt5::Xml
    OpenGL::GL)
add_dependencies(${PROJECT_NAME} DrawPrimitivesDemo)
message(STATUS " Installing DrawPrimitivesDemo into ${CMAKE_INSTALL_PREFIX}")
configure_radium_app(NAME DrawPrimitivesDemo)

add_executable(SimpleAnimationApp MACOSX_BUNDLE
    SimpleAnimationApp/main.cpp
    SimpleAnimationApp/minimalapp.cpp
    SimpleAnimationApp/minimalapp.hpp
    SimpleAnimationApp/minimalradium.cpp
    SimpleAnimationApp/minimalradium.hpp)
target_include_directories(SimpleAnimationApp PRIVATE SimpleAnimationApp)
target_link_libraries(SimpleAnimationApp PUBLIC
    Core Engine GuiBase
    Qt5::Core Qt5::Widgets Qt5::OpenGL Qt5::Xml
    OpenGL::GL)
add_dependencies(${PROJECT_NAME} SimpleAnimationApp)
message(STATUS " Installing SimpleAnimationApp into ${CMAKE_INSTALL_PREFIX}")
configure_radium_app(NAME SimpleAnimationApp)
#endif ()
