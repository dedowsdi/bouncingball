cmake_minimum_required(VERSION 3.8)

project(bouncingball VERSION 0.0 LANGUAGES CXX C)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(DEFAULT_BUILD_TYPE "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)

    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(BULLET_USE_DOUBLE_PRECISION "Use double precision bullet" OFF)
if (BULLET_USE_DOUBLE_PRECISION)
    add_definitions( -DBT_USE_DOUBLE_PRECISION)
    set( BULLET_DOUBLE_DEF "-DBT_USE_DOUBLE_PRECISION")
endif (BULLET_USE_DOUBLE_PRECISION)

option(BUILD_TEST "Build test" OFF)

find_package(OpenGL REQUIRED)
find_package(OpenAL REQUIRED)
find_package(Bullet REQUIRED)
find_package(Fontconfig REQUIRED)
find_package(OpenSceneGraph REQUIRED COMPONENTS
    osgViewer
    osgText
    osgAnimation
    osgParticle
    osgFX
    osgShadow
    osgWidget
    osgGA
    osgDB
    osgUtil
    )
find_package(PkgConfig REQUIRED)
pkg_check_modules(FREEALUT REQUIRED freealut)

include_directories(
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
    )

set(SRC
    src/AL.cpp
    src/ALBuffer.cpp
    src/ALContext.cpp
    src/ALListener.cpp
    src/ALSource.cpp
    src/Atom.cpp
    src/BulletClip.cpp
    src/DB.cpp
    src/DebugDrawer.cpp
    src/DebugHandler.cpp
    src/Game.cpp
    src/Math.cpp
    src/MotionState.cpp
    src/OsgFactory.cpp
    src/OsgQuery.cpp
    src/ReaderWriterAL.cpp
    src/SoundObjectManager.cpp
    src/ToyCopyOp.cpp
    src/main.cpp
    )

add_executable(bouncingball ${SRC})

target_link_libraries(bouncingball
    PRIVATE
    ${BULLET_LIBRARIES}
    ${OPENSCENEGRAPH_LIBRARIES}
    OpenGL::GL
    ${FREEALUT_LIBRARIES}
    ${OPENAL_LIBRARY}
    )

target_include_directories(bouncingball
    PRIVATE
    include
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${OPENAL_INCLUDE_DIR}
    ${FREEALUT_INCLUDE_DIRS}
    )

target_compile_options(bouncingball PRIVATE $<$<AND:$<CONFIG:Debug>,$<CXX_COMPILER_ID:GNU>>:-gdwarf -g3>)
target_compile_options(bouncingball PRIVATE $<$<CONFIG:Debug>:-DDEBUG>)
