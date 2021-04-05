# minimum required cmake version
cmake_minimum_required( VERSION 3.5 FATAL_ERROR )

# project name
project( HPM )

# set default CMAKE_BUILD_TYPE to Release if not set
if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE )
endif()

# set c++11
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

# compile everything position independent (even static libraries)
set( CMAKE_POSITION_INDEPENDENT_CODE TRUE )

# set verbose compile options
# set( CMAKE_VERBOSE_MAKEFILE ON )

# use folders in IDEs for projects (e.g. lib sample app test)
set_property( GLOBAL PROPERTY USE_FOLDERS ON )

if( UNIX OR MINGW )
  set( CMAKE_C_FLAGS_RELEASE "-g -O3 -fomit-frame-pointer -Wall -Wno-unused-function -Wno-unused-but-set-variable -Wno-unused-variable -Wno-attributes -Werror -Wno-strict-overflow -std=c99 -DLINUX -DX86_64 -msse4.2" )
  set( CMAKE_CXX_FLAGS_RELEASE "-g -O3 -fomit-frame-pointer -Wall -Werror -std=c++11 -Wno-unused-function -Wno-unused-but-set-variable -Wno-unused-variable -Wno-attributes -Wno-strict-overflow -Wno-sign-compare -Wno-unused-value -Wno-unknown-pragmas -DLINUX -DX86_64 -msse4.2" )

  set( CMAKE_C_FLAGS_DEBUG "-g -O3 -fomit-frame-pointer -Wall -Wno-unused-function -Wno-unused-but-set-variable -Wno-unused-variable -Wno-attributes -Werror -Wno-strict-overflow -std=c99 -DLINUX -DX86_64 -msse4.2" )
  set( CMAKE_CXX_FLAGS_DEBUG "-g -O3 -fomit-frame-pointer -Wall -Werror -std=c++11 -Wno-unused-function -Wno-unused-but-set-variable -Wno-unused-variable -Wno-attributes -Wno-strict-overflow -Wno-sign-compare -Wno-unused-value -Wno-unknown-pragmas -DLINUX -DX86_64 -msse4.2" )
endif()

if( MSVC )
  add_compile_options( "/wd4996" )
  string( REPLACE "/DNDEBUG" "" CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}" )
  string( REPLACE "/DNDEBUG" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}" )
endif()

# add needed subdirectories
add_subdirectory( "src" )
add_subdirectory( "app" )
