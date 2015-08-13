cmake_minimum_required (VERSION 2.6)

#
# проверка Unicode (UTF-8):
# 偉大和強大的俄語。
# اللغة الروسية العظمى والأقوياء.
# Η μεγάλη και πανίσχυρη ρωσική γλώσσα.
# השפה הרוסיה הגדולה והעצומה.
# 大いなるロシア語。
# 위대하고 강력한 러시아어 언어입니다.
# Wielki i potężny język rosyjski.
#

set( PROJECT_NAME "LiveWireOCV" )
project ( ${PROJECT_NAME} )

###############################################
set(CMAKE_BUILD_TYPE "Debug")
if(NOT CMAKE_BUILD_TYPE)
#    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(PLATFORM "x32")
if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
    set(PLATFORM "x64")
endif()

set(BUILD_PREFIX "${CMAKE_BUILD_TYPE}_${PLATFORM}")
message("build prefix=[${BUILD_PREFIX}]")

###############################################
set( TARGET_1     "LiveWire_OCV_Test"           )
set( TARGET_2     "LiveWire_OCV_Test2"          )
set( TARGET_3     "LiveWire_OCV_DragContour"    )

###############################################
find_package( OpenCV REQUIRED )

set( PROJECT_INCLUDE_DIR
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIR}
)

include_directories( ${PROJECT_INCLUDE_DIR} )

set(LIB_LWOCV "lwocv")
add_library(${LIB_LWOCV} STATIC "${LIB_LWOCV}.cpp" "${LIB_LWOCV}.h")

###############################################
add_executable("${TARGET_1}_${BUILD_PREFIX}" "${TARGET_1}.cpp")
target_link_libraries("${TARGET_1}_${BUILD_PREFIX}" ${LIB_LWOCV} ${OpenCV_LIBS})

add_executable("${TARGET_2}_${BUILD_PREFIX}" "${TARGET_2}.cpp")
target_link_libraries("${TARGET_2}_${BUILD_PREFIX}" ${LIB_LWOCV} ${OpenCV_LIBS})

add_executable("${TARGET_3}_${BUILD_PREFIX}" "${TARGET_3}.cpp")
target_link_libraries("${TARGET_3}_${BUILD_PREFIX}" ${LIB_LWOCV} ${OpenCV_LIBS})

##get_cmake_property(_variableNames VARIABLES)
##foreach (_variableName ${_variableNames})
##    message(STATUS "${_variableName}=${${_variableName}}")
##endforeach()
