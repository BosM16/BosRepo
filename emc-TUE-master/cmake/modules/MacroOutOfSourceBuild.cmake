# Ensures that we do an out of source build

MACRO(MACRO_ENSURE_OUT_OF_SOURCE_BUILD MSG)
    # Is the source-dir equal to binary-dir -> must be false
    STRING(COMPARE EQUAL "${CMAKE_SOURCE_DIR}" "${CMAKE_BINARY_DIR}" insource)
    
    # parent directory of source-dir
    GET_FILENAME_COMPONENT(PARENTDIR ${CMAKE_SOURCE_DIR} PATH) # use DIRECTORY in cmake version > 2.8.12
    # Is the source-dir equal to parent-dir of source-dir -> must be false
    STRING(COMPARE EQUAL "${CMAKE_SOURCE_DIR}" "${PARENTDIR}" insourcesubdir)
    
    # checks if you build in another directory
    IF(insource OR insourcesubdir)
        MESSAGE(FATAL_ERROR "${MSG}")

    ENDIF(insource OR insourcesubdir)
ENDMACRO(MACRO_ENSURE_OUT_OF_SOURCE_BUILD)

