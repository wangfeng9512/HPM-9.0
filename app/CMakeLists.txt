# executable
set( ENC_EXE_NAME app_encoder )
set( DEC_EXE_NAME app_decoder )
set( MER_EXE_NAME app_bitstream_merge )

# add executable
add_executable( ${ENC_EXE_NAME} app_encoder.c )
add_executable( ${DEC_EXE_NAME} app_decoder.c )
add_executable( ${MER_EXE_NAME} app_bitstream_merge.c )

include_directories(${CMAKE_CURRENT_BINARY_DIR})

if( CMAKE_COMPILER_IS_GNUCC )
  set( ADDITIONAL_LIBS ${ADDITIONAL_LIBS} m )
endif()

target_link_libraries( ${ENC_EXE_NAME} CommonLib EncoderLib ${ADDITIONAL_LIBS} )
target_link_libraries( ${DEC_EXE_NAME} CommonLib DecoderLib ${ADDITIONAL_LIBS} )
target_link_libraries( ${MER_EXE_NAME} CommonLib DecoderLib EncoderLib ${ADDITIONAL_LIBS} )

# set the folder where to place the projects
set_target_properties( ${ENC_EXE_NAME}  PROPERTIES FOLDER app LINKER_LANGUAGE CXX )
set_target_properties( ${DEC_EXE_NAME}  PROPERTIES FOLDER app LINKER_LANGUAGE CXX )
set_target_properties( ${MER_EXE_NAME}  PROPERTIES FOLDER app LINKER_LANGUAGE CXX )
