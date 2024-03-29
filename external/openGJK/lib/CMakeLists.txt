# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
#                                    #####        # #    #                #
#        ####  #####  ###### #    # #     #       # #   #                 #
#       #    # #    # #      ##   # #             # #  #                  #
#       #    # #    # #####  # #  # #  ####       # ###                   #
#       #    # #####  #      #  # # #     # #     # #  #                  #
#       #    # #      #      #   ## #     # #     # #   #                 #
#        ####  #      ###### #    #  #####   #####  #    #                #
#                                                                         #
#           Mattia Montanari    |   University of Oxford 2018             #
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #


project (openGJKlib)

set(CMAKE_C_STANDARD 11)

# SELECT USER OPTIONS
option(VERSION_ACCURATE  "Reduce speed to maximise accuracy (OFF)"
        OFF )

# APPLY USER OPTIONS
IF(VERSION_ACCURATE)
    set(USE_PREDICATES          ON)
    set(openGJK_VERSION "Accurate")
ELSE()
    set(USE_PREDICATES         OFF)
    set(openGJK_VERSION     "Fast")
ENDIF()

# COMPILE
message( "[${CMAKE_PROJECT_NAME}] Compiling ..")
message(STATUS "Version     (Accurate,Fast): " ${openGJK_VERSION}  )
message(STATUS "Build type  (Debug,Release): " ${CMAKE_BUILD_TYPE} )

# Select source files
set( SOURCE_FILES src/openGJK.c )
set( SOURCE_HEADS include/openGJK/openGJK.h)

IF(USE_PREDICATES)
    # for adpative floating-point artim.
    set( SOURCE_FILES ${SOURCE_FILES} ext/predicates.c)
    set( SOURCE_HEADS ${SOURCE_HEADS} ext/predicates.h)
    # Add flag for adpative floating-point artim.
    add_definitions(-DADAPTIVEFP)
ENDIF()

# Create the (dynamic) library
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${SOURCE_HEADS})
add_definitions(-DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=TRUE -DBUILD_SHARED_LIBS=TRUE)

# Link include file
target_include_directories( ${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)

IF(USE_PREDICATES)
    # for adpative floating-point artim.
    target_include_directories( ${PROJECT_NAME}
            PUBLIC ${PROJECT_SOURCE_DIR}/ext
            )
ENDIF()

# Report
message( ".. DONE!")
