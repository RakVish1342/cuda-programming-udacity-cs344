############################################################################
# <summary> CMakeLists.txt for OpenCV and CUDA. </summary>
# <date>    2012-02-07          </date>
# <author>  Quan Tran Minh. edit by Johannes Kast, Michael Sarahan </author>
# <email>   quantm@unist.ac.kr  kast.jo@googlemail.com msarahan@gmail.com</email>
############################################################################

cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(cs344)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

link_libraries(${OpenCV_LIBS} )

set (EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin/")

if(CUDA_FOUND)

  ## Original command. Uses clang for compilation. Has flags for different GPU hardware.
  # set(CUDA_NVCC_FLAGS "
  # -ccbin /usr/bin/clang; 
  # -gencode;arch=compute_30,code=sm_30;  
  # -gencode;arch=compute_35,code=sm_35;)

  # nvcc set to use g++ for compilation rather than clang.
  # gencode, arch and code flags set for Alienware 15 R3 Nvidia GTX 1070 host GPU.
  set(CUDA_NVCC_FLAGS "
  -ccbin /usr/bin/g++;
  -gencode;arch=compute_61,code=sm_61;")

  # add -Wextra compiler flag for gcc compilations
  if (UNIX)
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-Xcompiler -Wextra")
    #set(CMAKE_CXX_FLAGS "-stdlib=libstdc++") # This is a line that is relevant to clang. Commenting out since g++ is compiler being used now.
  endif (UNIX)

  # add debugging to CUDA NVCC flags.  For NVidia's NSight tools.
  set(CUDA_NVCC_FLAGS_DEBUG ${CUDA_NVCC_FLAGS_DEBUG} "-G")

  add_subdirectory (ProblemSets/ProblemSet1/)
else(CUDA_FOUND)
  message("CUDA is not installed on this system.")
endif()
