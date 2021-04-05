# library
set( COM_LIB_NAME CommonLib )
set( ENC_LIB_NAME EncoderLib )
set( DEC_LIB_NAME DecoderLib )

# get source files
file( GLOB COM_SRC_FILES "com*.c*" )
file( GLOB ENC_SRC_FILES "enc*.c*" )
file( GLOB DEC_SRC_FILES "dec*.c*" )

# get include files
file( GLOB COM_INC_FILES "../inc/com*.h" )
file( GLOB ENC_INC_FILES "../inc/enc*.h" )
file( GLOB DEC_INC_FILES "../inc/dec*.h" )

# library
add_library( ${COM_LIB_NAME} STATIC ${COM_SRC_FILES} ${COM_INC_FILES} )
add_library( ${ENC_LIB_NAME} STATIC ${ENC_SRC_FILES} ${ENC_INC_FILES} )
add_library( ${DEC_LIB_NAME} STATIC ${DEC_SRC_FILES} ${DEC_INC_FILES} )

target_compile_definitions( ${COM_LIB_NAME} PUBLIC )
target_compile_definitions( ${ENC_LIB_NAME} PUBLIC )
target_compile_definitions( ${DEC_LIB_NAME} PUBLIC )

target_include_directories( ${COM_LIB_NAME} PUBLIC ../inc )
target_include_directories( ${ENC_LIB_NAME} PUBLIC ../inc )
target_include_directories( ${DEC_LIB_NAME} PUBLIC ../inc )

target_link_libraries( ${ENC_LIB_NAME} CommonLib )
target_link_libraries( ${DEC_LIB_NAME} CommonLib )

# set the folder where to place the projects
if( UNIX OR MINGW )
  set_source_files_properties(enc_esao.c PROPERTIES COMPILE_FLAGS "-mavx -mavx2")
  set_source_files_properties(com_esao.c PROPERTIES COMPILE_FLAGS "-mavx -mavx2")
endif()

set_target_properties( ${COM_LIB_NAME} PROPERTIES FOLDER lib )
set_target_properties( ${DEC_LIB_NAME} PROPERTIES FOLDER lib )
set_target_properties( ${ENC_LIB_NAME} PROPERTIES FOLDER lib )
