cmake_minimum_required(VERSION 3.22)
project(roboteam_ai)

###### UTILS #######
file(GLOB_RECURSE UTILS_SRC
        "${PROJECT_SOURCE_DIR}/src/utilities/*.cpp"
)
add_library(roboteam_ai_utils ${UTILS_SRC})

target_link_libraries(roboteam_ai_utils
        PUBLIC Qt5::Network
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
        PRIVATE roboteam_ai_gui
)
target_include_directories(roboteam_ai_utils
        PRIVATE include/roboteam_ai
)
target_compile_options(roboteam_ai_utils PRIVATE "${COMPILER_FLAGS}")

###### SKILLS #######
file(GLOB_RECURSE SKILLS_SRC
        "${PROJECT_SOURCE_DIR}/src/stp/skills/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/stp/Skill.cpp"
)
add_library(roboteam_ai_skills ${SKILLS_SRC})

target_include_directories(roboteam_ai_skills
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_skills
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
)
target_compile_options(roboteam_ai_skills PRIVATE "${COMPILER_FLAGS}")

###### TACTICS #######
file(GLOB_RECURSE TACTICS_SRC
     "${PROJECT_SOURCE_DIR}/src/stp/tactics/*.cpp"
     "${PROJECT_SOURCE_DIR}/src/stp/Tactic.cpp"
)
add_library(roboteam_ai_tactics ${TACTICS_SRC})

target_include_directories(roboteam_ai_tactics
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_tactics
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
        PRIVATE roboteam_ai_skills
)
target_compile_options(roboteam_ai_tactics PRIVATE "${COMPILER_FLAGS}")

###### ROLES #######
file(GLOB_RECURSE ROLES_SRC
     "${PROJECT_SOURCE_DIR}/src/stp/Role.cpp"
     "${PROJECT_SOURCE_DIR}/src/stp/roles/*.cpp"
)
add_library(roboteam_ai_roles ${ROLES_SRC})

target_include_directories(roboteam_ai_roles
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_roles
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
        PRIVATE roboteam_ai_tactics
)
target_compile_options(roboteam_ai_roles PRIVATE "${COMPILER_FLAGS}")

###### PLAYS #######
file(GLOB_RECURSE PLAYS_SRC
     "${PROJECT_SOURCE_DIR}/src/stp/Play.cpp"
     "${PROJECT_SOURCE_DIR}/src/stp/PlayDecider.cpp"
     "${PROJECT_SOURCE_DIR}/src/stp/PlayEvaluator.cpp"
     "${PROJECT_SOURCE_DIR}/src/stp/plays/*.cpp"
)
add_library(roboteam_ai_plays ${PLAYS_SRC})

target_include_directories(roboteam_ai_plays
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_plays
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
        PRIVATE roboteam_ai_utils
        PRIVATE roboteam_ai_world
        PRIVATE roboteam_ai_evaluation
        PRIVATE roboteam_ai_control
        PRIVATE roboteam_ai_computation
        PRIVATE roboteam_ai_roles
)
target_compile_options(roboteam_ai_plays PRIVATE "${COMPILER_FLAGS}")

###### EVALUATION #######
file(GLOB_RECURSE EVALUATIONS_SRC
     "${PROJECT_SOURCE_DIR}/src/stp/evaluations/*.cpp"
)
add_library(roboteam_ai_evaluation ${EVALUATIONS_SRC})

target_include_directories(roboteam_ai_evaluation
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_evaluation
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
)
target_compile_options(roboteam_ai_evaluation PRIVATE "${COMPILER_FLAGS}")

###### COMPUTATION #######
file(GLOB_RECURSE COMPUTATIONS_SRC
     "${PROJECT_SOURCE_DIR}/src/stp/computations/*.cpp"
)
add_library(roboteam_ai_computation ${COMPUTATIONS_SRC})

target_include_directories(roboteam_ai_computation
    PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_computation
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
        PRIVATE roboteam_ai_evaluation
)
target_compile_options(roboteam_ai_computation PRIVATE "${COMPILER_FLAGS}")

###### CONTROL #######
file(GLOB_RECURSE CONTROL_SRC
     "${PROJECT_SOURCE_DIR}/src/control/*.cpp"
)
add_library(roboteam_ai_control ${CONTROL_SRC})

target_include_directories(roboteam_ai_control
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_control
        PRIVATE ruckig
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
        PUBLIC Qt5::Network
)
target_compile_features(roboteam_ai_control PUBLIC cxx_std_17)
target_compile_options(roboteam_ai_control PRIVATE "${COMPILER_FLAGS}")

###### INTERFACE #######
add_library(roboteam_ai_gui
        ${PROJECT_SOURCE_DIR}/src/gui/networking/InterfaceGateway.cpp
        ${PROJECT_SOURCE_DIR}/src/gui/Out.cpp

        ${PROJECT_SOURCE_DIR}/src/gui/networking/InterfacePublisher.cpp
        ${PROJECT_SOURCE_DIR}/src/gui/networking/InterfaceSubscriber.cpp

        ${PROJECT_SOURCE_DIR}/src/interface/api/Output.cpp
)
target_include_directories(roboteam_ai_gui
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_gui
        PUBLIC Qt5::Network
        PRIVATE ixwebsocket
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
)
target_compile_options(roboteam_ai_gui PRIVATE "${COMPILER_FLAGS}")

###### WORLD #######
add_library(roboteam_ai_world
        ${PROJECT_SOURCE_DIR}/src/world/FieldComputations.cpp
        ${PROJECT_SOURCE_DIR}/src/world/Ball.cpp
        ${PROJECT_SOURCE_DIR}/src/world/Robot.cpp
        ${PROJECT_SOURCE_DIR}/src/world/World.cpp
        ${PROJECT_SOURCE_DIR}/src/world/WorldData.cpp
        ${PROJECT_SOURCE_DIR}/src/world/views/WorldDataView.cpp
        ${PROJECT_SOURCE_DIR}/src/world/views/RobotView.cpp
        ${PROJECT_SOURCE_DIR}/src/world/views/BallView.cpp
)
target_include_directories(roboteam_ai_world
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai_world
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
        PRIVATE roboteam_ai_control
)
target_compile_options(roboteam_ai_world PRIVATE "${COMPILER_FLAGS}")

###### MAIN #######
add_executable(roboteam_ai
        ${PROJECT_SOURCE_DIR}/src/STPManager.cpp
        ${PROJECT_SOURCE_DIR}/src/roboteam_ai.cpp
)
target_include_directories(roboteam_ai
        PRIVATE include/roboteam_ai
)
target_link_libraries(roboteam_ai
        PRIVATE ixwebsocket
        PRIVATE Qt5::Charts
        PRIVATE roboteam_ai_computation
        PRIVATE roboteam_ai_control
        PRIVATE roboteam_ai_evaluation
        PRIVATE roboteam_ai_gui
        PRIVATE roboteam_ai_plays
        PRIVATE roboteam_ai_roles
        PRIVATE roboteam_ai_skills
        PRIVATE roboteam_ai_tactics
        PRIVATE roboteam_ai_utils
        PRIVATE roboteam_ai_world
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
)
target_compile_options(roboteam_ai PRIVATE "${COMPILER_FLAGS}")

###### TESTHELPERS #######
add_library(testHelpers
        ${PROJECT_SOURCE_DIR}/test/helpers/WorldHelper.h
        ${PROJECT_SOURCE_DIR}/test/helpers/WorldHelper.cpp
        ${PROJECT_SOURCE_DIR}/test/helpers/FieldHelper.h
        ${PROJECT_SOURCE_DIR}/test/helpers/FieldHelper.cpp)

target_include_directories(testHelpers
        PRIVATE include/roboteam_ai)

target_link_libraries(testHelpers
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
        PRIVATE roboteam_ai_utils
        )

target_compile_options(testHelpers PRIVATE "${COMPILER_FLAGS}")

###### TEST #######
add_executable(roboteam_ai_tests
        ${PROJECT_SOURCE_DIR}/src/STPManager.cpp
        ${PROJECT_SOURCE_DIR}/test/main.cpp
        ${PROJECT_SOURCE_DIR}/test/HelperTests/FieldHelperTest.cpp
        ${PROJECT_SOURCE_DIR}/test/UtilTests/RefereeTest.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/BallTests.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/RobotTests.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/FieldComputationTest.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/HistoryRetrievalTest.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/WhichRobotHasBallTest.cpp
        ${PROJECT_SOURCE_DIR}/test/WorldTests/WorldResetTests.cpp
        ${PROJECT_SOURCE_DIR}/test/StpTests/TacticTests.cpp
        ${PROJECT_SOURCE_DIR}/test/StpTests/CorrectPlayTest.cpp
        ${PROJECT_SOURCE_DIR}/test/ControlTests/BBTrajectory/BBTrajectory1DTest.cpp
        ${PROJECT_SOURCE_DIR}/test/PassingTests/PassingTests.cpp
        )

target_include_directories(roboteam_ai_tests
        PRIVATE include/roboteam_ai
        PRIVATE test
        )
target_link_libraries(roboteam_ai_tests
        PRIVATE GTest::gtest
        PRIVATE GTest::gmock
        PRIVATE Qt5::Charts
        PRIVATE ixwebsocket
        PRIVATE roboteam_ai_computation
        PRIVATE roboteam_ai_control
        PRIVATE roboteam_ai_evaluation
        PRIVATE roboteam_ai_gui
        PRIVATE roboteam_ai_plays
        PRIVATE roboteam_ai_roles
        PRIVATE roboteam_ai_skills
        PRIVATE roboteam_ai_tactics
        PRIVATE roboteam_ai_utils
        PRIVATE roboteam_ai_world
        PRIVATE roboteam_networking
        PRIVATE roboteam_utils
        PRIVATE testHelpers
)

target_compile_options(roboteam_ai_tests PRIVATE "${COMPILER_FLAGS}")

gtest_discover_tests(roboteam_ai_tests)

###### POSITIONSCORINGTEST #######
# This is a random world with 22 robots and prints the best location, not useful for automated testing. Uncomment when needed
# add_executable(positionScoringTest
#         ${PROJECT_SOURCE_DIR}/test/PositionScoringTests/PositionScoringTest.cpp
#         )

# target_include_directories(positionScoringTest
#         PRIVATE test
#         PRIVATE include/roboteam_ai
#         )

# target_link_libraries(positionScoringTest
#         PRIVATE Qt5::Charts
#         PRIVATE roboteam_ai_computation
#         PRIVATE roboteam_ai_control
#         PRIVATE roboteam_ai_evaluation
#         PRIVATE roboteam_ai_gui
#         PRIVATE roboteam_ai_plays
#         PRIVATE roboteam_ai_roles
#         PRIVATE roboteam_ai_skills
#         PRIVATE roboteam_ai_tactics
#         PRIVATE roboteam_ai_utils
#         PRIVATE roboteam_ai_world
#         PRIVATE roboteam_networking
#         PRIVATE roboteam_utils
#         PRIVATE testHelpers
# )
