cmake_minimum_required(VERSION 2.8.3)
project(spatial_relation_classifier)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)
find_package( PCL 1.4 REQUIRED COMPONENTS visualization filters segmentation  )
find_package( OpenCV REQUIRED )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spatial_relation_classifier
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(src/params)
include_directories(src/DatabaseHandlingModule/ObjectInformationModule/header)
include_directories(src/DatabaseHandlingModule/SceneInformationModule/header)
include_directories(src/DatabaseHandlingModule/DatabaseInformationModule/header)
include_directories(src/DatabaseHandlingModule/ApisConvertionModule/ApiConvertKTHDBModule/header)
include_directories(src/DatabaseHandlingModule/ApisConvertionModule/ApiConvertSimulationDBModule/header)
include_directories(src/utils/header)
include_directories(src/FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionDatabaseSingleObject/header)
include_directories(src/FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionDatabaseObjectPair/header)
include_directories(src/FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionSceneSingleObject/header)
include_directories(src/FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionSceneObjectPair/header)
include_directories(src/FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionSingleObject/header)
include_directories(src/FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionObjectPair/header)
include_directories(src/FeatureExtractionModule/DatabaseSingleObjectFeatureModule/header)
include_directories(src/FeatureExtractionModule/DatabaseObjectPairFeatureModule/header)
include_directories(src/FeatureExtractionModule/SceneSingleObjectFeatureModule/header)
include_directories(src/FeatureExtractionModule/SceneObjectPairFeatureModule/header)
include_directories(src/FeatureExtractionModule/SingleObjectFeatureModule/header)
include_directories(src/FeatureExtractionModule/ObjectPairFeatureModule/header)
include_directories(src/ApisStatisticalToolModule/ArrangeFeatureTraining/header)
include_directories(src/ApisStatisticalToolModule/Training/header)
include_directories(src/ApisStatisticalToolModule/StatisticalToolModule/header)
include_directories(src/ApisStatisticalToolModule/ModelTrainedIO/header)
include_directories(src/ApisStatisticalToolModule/ArrangeFeatureTestScene/header)
include_directories(src/ApisStatisticalToolModule/Test/header)
include_directories(src/ApiDatabaseFrequencies/header)
include_directories(src/ApisStatisticalToolModule/ApiGraph/header)
include_directories(src/EvaluationFrameworkModule/ApiConvertionResultsTestConfusionMatrix/header)
include_directories(src/EvaluationFrameworkModule/ConfusionMatrix/header)
include_directories(src/EvaluationFrameworkModule/Evaluation/header)
include_directories(src/EvaluationFrameworkModule/CrossValidation/header)
include_directories(src/DatabaseHandlingModule/ApisConvertionModule/ApiConvertServiceFormat/header
include_directories(src/DatabaseHandlingModule/ApisConvertionModule/ApiConvertRealWorldDBModule/header)
)

## Declare a cpp library
# add_library(spatial_relation_classifier
#   src/${PROJECT_NAME}/spatial_relation_classifier.cpp
# )
add_library( SceneClassification src/DatabaseHandlingModule/ObjectInformationModule/impl/Object.cpp 
src/DatabaseHandlingModule/SceneInformationModule/impl/SceneInformation.cpp 
src/DatabaseHandlingModule/ApisConvertionModule/ApiConvertKTHDBModule/impl/ApiConvertKTHDB.cpp 
src/DatabaseHandlingModule/ApisConvertionModule/ApiConvertSimulationDBModule/impl/ApiConvertSimulationDB.cpp 
src/DatabaseHandlingModule/DatabaseInformationModule/impl/DatabaseInformation.cpp  
src/utils/impl/utils.cpp 
src/FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionDatabaseSingleObject/impl/ApiFeatureExtractionDatabaseSingleObject.cpp 
src/FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionDatabaseObjectPair/impl/ApiFeatureExtractionDatabaseObjectPair.cpp 
src/FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionSceneSingleObject/impl/ApiFeatureExtractionSceneSingleObject.cpp 
src/FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionSceneObjectPair/impl/ApiFeatureExtractionSceneObjectPair.cpp 
src/FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionSingleObject/impl/ApiFeatureExtractionSingleObject.cpp
src/FeatureExtractionModule/ApisFeatureExtraction/ApiFeatureExtractionObjectPair/impl/ApiFeatureExtractionObjectPair.cpp 
src/FeatureExtractionModule/DatabaseSingleObjectFeatureModule/impl/DatabaseSingleObjectFeature.cpp 
src/FeatureExtractionModule/DatabaseObjectPairFeatureModule/impl/DatabaseObjectPairFeature.cpp 
src/FeatureExtractionModule/SceneSingleObjectFeatureModule/impl/SceneSingleObjectFeature.cpp 
src/FeatureExtractionModule/SceneObjectPairFeatureModule/impl/SceneObjectPairFeature.cpp 
src/FeatureExtractionModule/SingleObjectFeatureModule/impl/SingleObjectFeature.cpp 
src/FeatureExtractionModule/ObjectPairFeatureModule/impl/ObjectPairFeature.cpp 
src/ApisStatisticalToolModule/ArrangeFeatureTraining/impl/ArrangeFeatureTraining.cpp 
src/ApisStatisticalToolModule/Training/impl/Training.cpp 
src/ApisStatisticalToolModule/StatisticalToolModule/impl/StatisticalTool.cpp 
src/ApisStatisticalToolModule/ModelTrainedIO/impl/ModelTrainedIO.cpp 
src/ApisStatisticalToolModule/ArrangeFeatureTestScene/impl/ArrangeFeatureTestScene.cpp 
src/ApisStatisticalToolModule/Test/impl/Test.cpp 
src/ApiDatabaseFrequencies/impl/ApiStatisticsDatabase.cpp 
src/ApisStatisticalToolModule/ApiGraph/impl/ApiGraph.cpp 
src/EvaluationFrameworkModule/ApiConvertionResultsTestConfusionMatrix/impl/ApiConvertionResultsTestConfusionMatrix.cpp 
src/EvaluationFrameworkModule/ConfusionMatrix/impl/ConfusionMatrix.cpp 
src/EvaluationFrameworkModule/Evaluation/impl/Evaluation.cpp 
src/EvaluationFrameworkModule/CrossValidation/impl/CrossValidation.cpp
 src/DatabaseHandlingModule/ApisConvertionModule/ApiConvertServiceFormat/impl/ApiConvertServiceFormat.cpp
src/DatabaseHandlingModule/ApisConvertionModule/ApiConvertRealWorldDBModule/impl/ApiConvertRealWorldDB.cpp 
)


## Declare a cpp executable
# add_executable(spatial_relation_classifier_node src/spatial_relation_classifier_node.cpp)
add_executable(test src/TestCases/TestCase1/impl/test.cpp)
add_executable(test2 src/TestCases/TestCaseModelTrainedIO/impl/test2.cpp)
add_executable(test3 src/TestCases/TestCaseSimulation/impl/test3.cpp)
add_executable(test4 src/TestCases/TestCaseTrainingTest/impl/test4.cpp)
add_executable(test5 src/TestCases/TestApiGraph/impl/test5.cpp)
add_executable(test6 src/TestCases/TestCrossValidation/impl/test6.cpp)
add_executable(spatial_relation_classifier src/TestCases/ROSService/spatial_relation_classifier.cpp)
add_executable(runTraining src/TestCases/RunTraining/impl/runTraining.cpp)
add_executable(runTrainingRealWorld src/TestCases/RunTrainingRealWorld/impl/runTrainingRealWorld.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(spatial_relation_classifier_node spatial_relation_classifier_generate_messages_cpp)
#add_dependencies(object_class_prediction_service strands_qsr_spatial_relation_estimator_generate_messages_cpp)


## Specify libraries to link a library or executable target against
# target_link_libraries(spatial_relation_classifier_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries( spatial_relation_classifier ${catkin_LIBRARIES})
target_link_libraries( test SceneClassification)
target_link_libraries( test2 SceneClassification)
target_link_libraries( test3 SceneClassification)
target_link_libraries( test4 SceneClassification)
target_link_libraries( test5 SceneClassification)
target_link_libraries( test6 SceneClassification)
target_link_libraries( spatial_relation_classifier SceneClassification)
target_link_libraries( runTraining SceneClassification)
target_link_libraries( runTrainingRealWorld SceneClassification)
target_link_libraries( SceneClassification ${OpenCV_LIBS} )
target_link_libraries( test ${OpenCV_LIBS} )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS spatial_relation_classifier spatial_relation_classifier_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_spatial_relation_classifier.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
