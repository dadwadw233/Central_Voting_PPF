cmake_minimum_required(VERSION 3.5)
project(central_voting)
set(CMAKE_CXX_STANDARD 17)

find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})

link_directories($PCL_LIBRARY_DIRS)

add_definitions(${PCL_DEFINITIONS})

add_executable(central_voting
        main.cpp
        PPFEstimation.cpp
        PPFEstimation.h
        CentralVoting.h
        CentralVoting.cpp
        SmartDownSample.cpp
        SmartDownSample.h
        HashMap.cpp
        HashMap.h
        PPFRegistration.cpp PPFRegistration.h
        add_gauss_noise.cpp add_gauss_noise.h)

target_link_libraries(central_voting ${PCL_LIBRARIES} tbb)



