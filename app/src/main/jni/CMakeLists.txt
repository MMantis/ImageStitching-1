#Added 2 path definitions to support 20160825 additions
set(PROJECT_PATH /home/kunato/ImageStitching)
set(CV_PATH /home/kunato/project-lib/OpenCV-3-android-sdk/)
set(EIGEN_PATH /usr/local/include/eigen3/)
set(GLOG_PATH /home/kunato/project-lib/ceres-solver/internal/ceres/miniglog/)
set(CERES_PATH /home/kunato/project-lib/ceres-solver/)

#Added by the IDE on project create
cmake_minimum_required(VERSION 3.4.1)

#Two sets suggested by Bruno Alexandre Krinski 20160825
set(CMAKE_VERBOSE_MAKEFILE on)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

#Addition suggested by Bruno Alexandre Krinski 20160825
include_directories(${CV_PATH}/sdk/native/jni/include)
include_directories(${EIGEN_PATH})
include_directories(${GLOG_PATH})
include_directories(${CERES_PATH}/include)

#Addition suggested by Bruno Alexandre Krinski 20160825
add_library( lib_opencv SHARED IMPORTED )

#Added by IDE on project create
add_library( native SHARED
${PROJECT_PATH}/app/src/main/jni/stitching.h
${PROJECT_PATH}/app/src/main/jni/stitching.cpp
${PROJECT_PATH}/app/src/main/jni/tracking.cpp
${PROJECT_PATH}/app/src/main/jni/composer.h
${PROJECT_PATH}/app/src/main/jni/composer.cpp
${PROJECT_PATH}/app/src/main/jni/util.h
${PROJECT_PATH}/app/src/main/jni/util.cpp
${PROJECT_PATH}/app/src/main/jni/matcher.h
${PROJECT_PATH}/app/src/main/jni/matcher.cpp
${PROJECT_PATH}/app/src/main/jni/BundleCV.h
${PROJECT_PATH}/app/src/main/jni/BundleCV.cpp
${PROJECT_PATH}/app/src/main/jni/BundleCeres.h
${PROJECT_PATH}/app/src/main/jni/BundleCeres.cpp
)






#Addition suggested by Bruno Alexandre Krinski 20160825
set_target_properties(lib_opencv PROPERTIES IMPORTED_LOCATION ${PROJECT_PATH}/app/src/main/jniLibs/${ANDROID_ABI}/libopencv_java3.so)



#Added by IDE on project create
find_library( log-lib log )

#Added by IDE on project create, Removed and replace with additional parameter suggested by Bruno Alexandre Krinski 20160825
#target_link_libraries( native-lib $\{log-lib} )

target_link_libraries( native log lib_opencv ${CERES_PATH}/libs/${ANDROID_ABI}/libceres.a)