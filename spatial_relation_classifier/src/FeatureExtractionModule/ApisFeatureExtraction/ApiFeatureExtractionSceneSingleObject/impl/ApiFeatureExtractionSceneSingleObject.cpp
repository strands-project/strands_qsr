/*
 * ApiFeatureExtractionSceneSingleObject.cpp
 *
 *  Created on: Nov 18, 2013
 *      Author: marina
 */


#include "ApiFeatureExtractionSceneSingleObject.hpp"
#define TESTFLAG 0


void ApiFeatureExtractionSceneSingleObject::extract(SceneInformation & scene, SceneSingleObjectFeature & out) {

	vector<Object> objectList = scene.getObjectList();
	pcl::PointXYZ centroid = scene.getReferenceCentroid();


	// for each object in the scene
	for (vector<Object>::iterator it = objectList.begin(); it != objectList.end(); it++ ) {

		SingleObjectFeature currentObjectFeature;
		ApiFeatureExtractionSingleObject fe;

		fe.extractFeatures( *it, centroid, currentObjectFeature);

		out.addSingleObjectFeature(currentObjectFeature);

    }

}


void ApiFeatureExtractionSceneSingleObject::extractNoReference(SceneInformation & scene, SceneSingleObjectFeature & out) {

	if (TESTFLAG) {
		cout << "in ApiFeatureExtractionSceneSingleObject 00" << endl;
	}
	vector<Object> objectList = scene.getObjectList();
	//pcl::PointXYZ centroid = scene.getReferenceCentroid();

	if (TESTFLAG) {
		cout << "in ApiFeatureExtractionSceneSingleObject 0" << endl;
	}


	// for each object in the scene
	for (vector<Object>::iterator it = objectList.begin(); it != objectList.end(); it++ ) {

		if (TESTFLAG) {
			cout << "in ApiFeatureExtractionSceneSingleObject 1: new object" << endl;
		}

		SingleObjectFeature currentObjectFeature;
		ApiFeatureExtractionSingleObject fe;

		if (TESTFLAG) {
			cout << "in ApiFeatureExtractionSceneSingleObject 2" << endl;
		}

		fe.extractFeaturesNoReference( *it,  currentObjectFeature);


		if (TESTFLAG) {
			cout << "in ApiFeatureExtractionSceneSingleObject 3" << endl;
		}

		out.addSingleObjectFeature(currentObjectFeature);

    }

}
