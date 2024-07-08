cmake_minimum_required(VERSION 3.14)

project(go2 VERSION 1.0.0 LANGUAGES C CXX)

find_package (Threads REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(pinocchio REQUIRED)
include_directories(${PINOCCHIO_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

find_package(NLopt REQUIRED)
set(NLopt_LIBRARIES /usr/local/lib/libnlopt.so.0.11.1 ) 

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -O3 -pthread")

set(BITBOT_DEPENDENCY_USE_PROXY OFF)
set(BITBOT_DEPENDENCY_USE_LOCAL_FILE TRUE)
set(BITBOT_DEPENDENCY_LOCAL_FILE_PATH ${CMAKE_SOURCE_DIR}/dependencies)

include(bitbot.cmake)

# 添加头文件所在目录
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(/usr/local/include/QuadProg++ /usr/local/include/qpOASES)

BitbotAddDependency(
  NAME           bitbot_mujoco
  FILE_NAME      "bitbot_mujoco-main.zip"
  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
  USE_LOCAL_FILE TRUE
)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

#将src文件夹中的所有cpp文件加入
set(SOURCES
        external/quadprog/QuadProg.cpp
        external/quadprog/aind.c
        external/quadprog/daxpy.c
        external/quadprog/ddot.c
        external/quadprog/dpofa.c
        external/quadprog/dscal.c
        external/quadprog/f2c_lite.c
        external/quadprog/util.c
        external/quadprog/solve.QP.c
        external/quadprog/solve.QP.compact.c
        src/BallanceController.cpp
        src/ClassManager.cpp
        src/common.cpp
        src/Dynamic.cpp
        src/Dynamic_arm.cpp
        src/FiniteStateMachine.cpp
        src/HoQp.cpp
        src/Info.cpp
        src/MPC_solver.cpp
        src/MujocoInterface.cpp
        src/NMPC_solver_DRBM_rpy.cpp
        src/NMPC_solver_DRBM.cpp
        src/NMPC_solver_rpy.cpp
        src/NMPC_solver.cpp
        src/ObjectController.cpp
        src/Robot_Parameters.cpp
        src/SaveLog.cpp
        src/StateEstimator.cpp
        src/SwingLegController.cpp
        src/test.cpp
        src/TrajectoryGenerator.cpp
        src/UserCmd.cpp
        src/WholeBodyController_arm.cpp
        src/WholeBodyController.cpp
        src/WholeBodyPlanner.cpp
        src/EEF.cpp
        main.cpp
        user_func.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} 
${PINOCCHIO_LIBRARIES}
${NLopt_LIBRARIES}
BitbotMujoco
libquadprog.a
/usr/local/lib/libqpOASES.a 
Threads::Threads 
Eigen3::Eigen
pinocchio::pinocchio
yaml-cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
