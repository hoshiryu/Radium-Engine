set(ra_core_target Core)

project( ${ra_core_target}
         LANGUAGES CXX
         VERSION ${RADIUM_VERSION})

option( RADIUM_QUIET "Disable Radium Log messages" OFF )

set(RA_VERSION_CPP "${CMAKE_CURRENT_BINARY_DIR}/Version.cpp")
configure_file (
"${CMAKE_CURRENT_SOURCE_DIR}/Utils/Version.cpp.in"
"${RA_VERSION_CPP}"
)
# If you want to have date and time of the build, your targets has to depend on this.
# This will force recompilation of version.o and thus forcing gcc to update __DATE__ macro.
if(RADIUM_UPDATE_VERSION)
add_custom_target(versionFileTouchForRebuild
COMMAND ${CMAKE_COMMAND} -E touch "${RA_VERSION_CPP}")
else()
  add_custom_target(versionFileTouchForRebuild)
endif()

find_package( Threads REQUIRED )

include( filelistCore )

add_library(
    ${ra_core_target} SHARED
    ${core_sources}
    ${core_headers}
    ${core_inlines}
    ${RA_VERSION_CPP}
)
addExternalFolder( Core ${CMAKE_CURRENT_SOURCE_DIR}/external )

find_package(Eigen3 3.3
    REQUIRED
    NO_DEFAULT_PATH )

find_package(OpenMesh
    REQUIRED
    COMPONENTS Core Tools
    NO_DEFAULT_PATH )

find_package(cpplocate
        REQUIRED
        NO_DEFAULT_PATH)

add_dependencies ( ${ra_core_target} versionFileTouchForRebuild) # We want precise time of build in version
target_include_directories(${ra_core_target} PUBLIC
    $<BUILD_INTERFACE:${OPENMESH_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${EIGEN_INCLUDE_DIR}> )
target_link_libraries(${ra_core_target} PUBLIC OpenMeshCore OpenMeshTools Threads::Threads Eigen3::Eigen cpplocate::cpplocate)

target_compile_definitions(${ra_core_target} PRIVATE "-DRA_CORE_EXPORTS")
if (WIN_32)
    target_compile_definitions(${ra_core_target} PUBLIC "-D_USE_MATH_DEFINES") ##OpenMesh
endif ()
if (${RADIUM_QUIET})
    target_compile_definitions(${ra_core_target} PUBLIC "-DRA_NO_LOG")
    message(STATUS "${PROJECT_NAME} : Radium Logs disabled")
endif ()

configureTargetWithDefaultRaSettings(${ra_core_target})
configure_radium_library(
    TARGET ${ra_core_target}
    TARGET_DIR "Core"
    PACKAGE_DIR ${ConfigPackageLocation}
    FILES "${core_headers};${core_inlines}"
)
