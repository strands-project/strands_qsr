/*
 * ApiFeatureExtractionDatabaseSingleObject.cpp
 *
 *  Created on: Nov 23, 2013
 *      Author: marina
 */


#include "ApiFeatureExtractionDatabaseSingleObject.hpp"
#define TESTFLAG 0

/*
 * Input:
 * a database ("DatabaseInformation" object)
 * Output:
 * the features of "single object" organized at the database-level ("DatabaseSingleObjectFeature" object)
 *  */
void ApiFeatureExtractionDatabaseSingleObject::extract(DatabaseInformation & database, DatabaseSingleObjectFeature & out) {

	if (TESTFLAG) {
		cout << "in ApiFeatureExtractionDatabaseSingleObject 0" << endl;
	}

	// gets the list of all the scenes in the database
	vector<SceneInformation> sceneList = database.getSceneList();

	// for each scene in the database
	for (vector<SceneInformation>::iterator it = sceneList.begin(); it != sceneList.end(); it++ ) {

		if (TESTFLAG) {
			cout << "in ApiFeatureExtractionDatabaseSingleObject 1: a new scene" << endl;
		}

		SceneSingleObjectFeature currentObjectFeature;

		// extracts the features in the current scene (features organized at the scene level)
		// ApiFeatureExtractionSceneSingleObject::extract( *it, currentObjectFeature );
		ApiFeatureExtractionSceneSingleObject::extractNoReference( *it, currentObjectFeature );
		if (TESTFLAG) {
			cout << "in ApiFeatureExtractionDatabaseSingleObject 2" << endl;
		}


		// adds the features organized at the scene level to the features organized at the database level in output
		out.addSceneSingleObjectFeature(currentObjectFeature);

    }

	int numberOfCategories = database.getNumberOfCategories();
	out.setNumberOfCategories(numberOfCategories);


}

