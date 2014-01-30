/*
 * object_classification_service.cpp
 *
 *  Created on: Jan 10, 2014
 *      Author: marina
 */


#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <iostream>
#include <fstream>
#include "utils.hpp"
#include "SceneInformation.hpp"
#include "ApiConvertKTHDB.hpp"
#include "DatabaseInformation.hpp"
#include "ApiFeatureExtractionDatabaseSingleObject.hpp"
#include "ApiFeatureExtractionDatabaseObjectPair.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "ArrangeFeatureTraining.hpp"
#include "DatabaseSingleObjectFeature.hpp"
#include "DatabaseObjectPairFeature.hpp"
#include "Training.hpp"
#include "ModelTrainedIO.hpp"
#include "ArrangeFeatureTestScene.hpp"
#include "Test.hpp"
#include "ApiStatisticsDatabase.hpp"
#include "ApiGraph.hpp"
#include "ConfusionMatrix.hpp"
#include "ApiConvertionResultsTestConfusionMatrix.hpp"
#include "Evaluation.hpp"
#include "ApiConvertServiceFormat.hpp"

#include "ros/ros.h"
#include "strands_qsr_msgs/GetGroupClassification.h"
#include "strands_qsr_msgs/BBox.h"
#include "strands_qsr_msgs/ObjectClassification.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"

#define DEBUG 1

typedef map<int, double> mapCategoryConfidence;

using namespace std;

bool handle_group_estimate(strands_qsr_msgs::GetGroupClassification::Request  & req,
         strands_qsr_msgs::GetGroupClassification::Response & res) {

	SceneInformation testScene;

	// Extracts the fields from the ROS service request and stores them into c++ data structures
	vector<string> objectInstanceNameList = req.object_id;
	vector<strands_qsr_msgs::BBox> bboxListInput = req.bbox;
	vector<geometry_msgs::Pose> poseListInput = req.pose;

	cout << "Extract fields of request " << endl;

	// Converts the Pose field of the request
	vector<pcl::PointXYZ> poseList;
	for(int i = 0; i < poseListInput.size(); i++) {
		pcl::PointXYZ point;
		point.x = poseListInput.at(i).position.x;
		point.y = poseListInput.at(i).position.y;
		point.z = poseListInput.at(i).position.z;
		poseList.push_back(point);
	}

	cout << "Convert the bbox field of the request " << endl;

	// Converts the bbox field of the request
	vector<vector<pcl::PointXYZ> > bboxList;
	for (int i = 0; i < bboxListInput.size(); i++) {

		strands_qsr_msgs::BBox currentBbox = bboxListInput.at(i);

		vector<pcl::PointXYZ> currentBboxConverted;

		for (int j = 0; j < currentBbox.point.size(); j++) {
			pcl::PointXYZ point;
			point.x = currentBbox.point.at(j).x;
			point.y = currentBbox.point.at(j).y;
			point.z = currentBbox.point.at(j).z;

			currentBboxConverted.push_back(point);
		}
		bboxList.push_back(currentBboxConverted);
	}


	vector<string> categoryListString = req.type;

	vector<strands_qsr_msgs::ObjectClassification> classificationListInupt = req.group_classification;

	cout << "Convert data MsgToIDS" << endl;

	map<string, mapCategoryConfidence> msgMap = Test::convertObjectClassificationMsgToIDS(classificationListInupt);

	vector<int> categoryListInt = convertStringToIntCategoryLabelVector(categoryListString);


	// Parses the scene and converts the data into the internal data structure
	cout << "Parse scene" << endl;

	ApiConvertServiceFormat::parseScene(objectInstanceNameList, bboxList, poseList, testScene);

	// // Feature extraction: single object and object pair features

	cout << "Extract features" << endl;

	SceneSingleObjectFeature sceneSof;
	SceneObjectPairFeature sceneOpf;
	ApiFeatureExtractionSceneSingleObject::extractNoReference(testScene, sceneSof);
	ApiFeatureExtractionSceneObjectPair::extract(testScene, sceneOpf);

	// // Arranges features of test scene

	ArrangeFeatureTestScene arrangeFeaturesTest;
	arrangeFeaturesTest.arrangeTestFeatures(sceneSof, sceneOpf);

	// // Testing

	Test testingScene;

	// Loads the stored trained models and parameters

	// string storingFolder = "./src/params";
	// string storingFolder = "src/paramsRealWorld";
	// string storingFolder = "/home/marina/catkin_strands_qsr_ws/src/strands_qsr/spatial_relation_classifier/src/params";
	string storingFolder = "/home/marina/catkin_strands_qsr_ws/src/strands_qsr/spatial_relation_classifier/src/paramsRealWorld";
	ModelTrainedIO::loadTrainedGMMsFile(storingFolder, testingScene);
	ModelTrainedIO::loadfrequencies(storingFolder, testingScene);

	int normalizationOption = 0;

	// PERCEPTION input:
	// 1 to take perception confidence into account, 0 for not considering perception
	int perceptionOption = 0;   

	vector<strands_qsr_msgs::ObjectClassification> results;

	// Prepares the input for the voting strategy
	vector<vector<double> > votingTable;

	results = testingScene.voting(arrangeFeaturesTest, normalizationOption, votingTable, categoryListInt, categoryListString, msgMap, perceptionOption);



	// ***********************************************************************************
        // Prints results
	map<string, mapCategoryConfidence> outMap = Test::convertObjectClassificationMsgToIDS(results);


	for (std::map<string, mapCategoryConfidence>::iterator it=outMap.begin(); it!=outMap.end(); ++it) {

 		std::cout << it->first << endl; 
		
		mapCategoryConfidence categoryConfidenceMap = it->second; 

		for (std::map<int, double>::iterator it2=categoryConfidenceMap.begin(); it2!=categoryConfidenceMap.end(); ++it2) {
			int cat = it2->first;
			double score = it2->second;
			cout << "Category number: " << cat << "    Score:  " << score << endl; 
		}
	}
	// ****************************************************************************************

	res.group_classification = results;
	return true;
}


int main(int argc, char **argv) {	

  // the last parameter is a string containing the name of the ROS node
  ros::init(argc, argv, "spatial_relation_classifier_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("spatial_relation_classifier", handle_group_estimate);
  ROS_INFO("Ready to estimate the test scene");
  ros::spin();
  
  return 0;
}

