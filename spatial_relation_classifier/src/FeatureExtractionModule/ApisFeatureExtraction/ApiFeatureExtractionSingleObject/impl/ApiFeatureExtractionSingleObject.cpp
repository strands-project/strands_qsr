/*
 * ApiSingleObjectFeature.cpp
 *
 *  Created on: Nov 18, 2013
 *      Author: marina
 */

#include "ApiFeatureExtractionSingleObject.hpp"

#define DEBUG 0
#define TESTFLAG 0

void ApiFeatureExtractionSingleObject::extractFeatures(Object & inputObject , pcl::PointXYZ refCentroid , SingleObjectFeature & out){

	int objectID =  inputObject.getActualObjectID();
	int instanceID = inputObject.getInstanceID();
	string instanceName = inputObject.getInstanceName();

	out.setObjectID(objectID);
	out.setInstanceID(instanceID);
	out.setInstanceName(instanceName);

	computePose(inputObject);

	computeAngle2dCentroid(inputObject, refCentroid);

	computeAngle2d(inputObject);

	computeVolumeSize(inputObject);

	computeSizeProjectedX(inputObject);

	computeSizeProjectedY(inputObject);

	computeSizeProjectedZ(inputObject);

	out.setPose(poseX, poseY, poseZ);
	out.setAngle2dCentroid(angle2d);
	out.setAngle2d(angle2dCentroid);
	out.setVolume(volumeSize);
	out.setSizeProjectedX(sizeProjectedX);
	out.setSizeProjectedY(sizeProjectedY);
	out.setSizeProjectedZ(sizeProjectedZ);

}

void ApiFeatureExtractionSingleObject::extractFeaturesNoReference(Object & inputObject ,SingleObjectFeature & out){


	if (TESTFLAG) {
		cout << "in ApiFeatureExtractionSingleObject 0" << endl;
	}
	int objectID =  inputObject.getActualObjectID();
	int instanceID = inputObject.getInstanceID();
	string instanceName = inputObject.getInstanceName();

	if (TESTFLAG) {
		cout << "in ApiFeatureExtractionSingleObject 1" << endl;
	}

	out.setObjectID(objectID);
	out.setInstanceID(instanceID);
	out.setInstanceName(instanceName);

	if (TESTFLAG) {
		cout << "in ApiFeatureExtractionSingleObject 2" << endl;
	}

	computePose(inputObject);

	if (TESTFLAG) {
		cout << "in ApiFeatureExtractionSingleObject 3" << endl;
	}

	//computeAngle2dCentroid(inputObject, refCentroid);

	computeAngle2d(inputObject);
	if (TESTFLAG) {
		cout << "in ApiFeatureExtractionSingleObject 4" << endl;
	}

	//computeVolumeSize(inputObject);

	if (TESTFLAG) {
		cout << "in ApiFeatureExtractionSingleObject 5" << endl;
	}

	computeSizeProjectedX(inputObject);
	if (TESTFLAG) {
		cout << "in ApiFeatureExtractionSingleObject 6" << endl;
	}

	computeSizeProjectedY(inputObject);
	if (TESTFLAG) {
		cout << "in ApiFeatureExtractionSingleObject 7" << endl;
	}

	computeSizeProjectedZ(inputObject);

	if (TESTFLAG) {
		cout << "in ApiFeatureExtractionSingleObject 8" << endl;
	}

	out.setPose(poseX, poseY, poseZ);
	out.setAngle2dCentroid(angle2d);
	// out.setAngle2d(angle2dCentroid);
	// out.setVolume(volumeSize);
	out.setSizeProjectedX(sizeProjectedX);
	out.setSizeProjectedY(sizeProjectedY);
	out.setSizeProjectedZ(sizeProjectedZ);

}

void ApiFeatureExtractionSingleObject::computePose(Object & inputObject) {

	  pcl::PointXYZ objectPose = inputObject.getCentroid();
	  poseX = objectPose.x;
	  poseY = objectPose.y;
	  poseZ = objectPose.z;

	if (TESTFLAG) {
		cout << "in ApiFeatureExtractionSingleObject : pose = " << poseX << "  " << poseZ << "  " << poseZ << endl;
	}

}


void ApiFeatureExtractionSingleObject::computeAngle3d (Object & inputObject , pcl::PointXYZ refCentroid) {

  // Eigen::Vector4f VectorObjectPose4f(pose.x, pose.y, 0, 0);

  pcl::PointXYZ objectCentroid = inputObject.getCentroid();
  Eigen::Vector4f v0;
  v0[0] = objectCentroid.x;
  v0[1] = objectCentroid.y;
  v0[2] = objectCentroid.z;
  v0[3] = 1; // ?
  Eigen::Vector4f v1(1, 0, 0, 1);
  angle3d = pcl::getAngle3D (v0, v1);

}


void ApiFeatureExtractionSingleObject::computeAngle2dCentroid(Object & inputObject , pcl::PointXYZ refCentroid) {

  float dx_object_desk = (inputObject.getCentroid()).x - refCentroid.x;
  float dy_object_desk = (inputObject.getCentroid()).y - refCentroid.y;
 //float l_object_desk = sqrt( dx_object_desk * dx_object_desk + dy_object_desk * dy_object_desk );
  float slope_object_desk = ( dy_object_desk ) / ( dx_object_desk );
  float angle_object_desk = atan2 (dy_object_desk, dx_object_desk);

  if ( angle_object_desk < 0 ) {
    angle_object_desk = angle_object_desk + 2 * M_PI;
  }

 // float angle2 = (slope_startModel - slope_vertix) / (1.0 + (slope_startModel * slope_vertix));

  if (DEBUG) {
    cout << "The angle between desk centroid and object centroid is : " << angle_object_desk
          << " = in degrees " << (angle_object_desk * 180 / 3.14) << endl;
    cout << " when desk centroid x = " << refCentroid.x << " and object centroid x = "
         << (inputObject.getCentroid()).x << endl;
    cout << " when desk centroid y = " << refCentroid.y << " and object centroid y = "
         << (inputObject.getCentroid()).y << endl;
  }

  angle2dCentroid = angle_object_desk;
}


void ApiFeatureExtractionSingleObject::computeAngle2d(Object & inputObject) {

  float dx_object_desk = (inputObject.getCentroid()).x - 0.0;
  float dy_object_desk = (inputObject.getCentroid()).y - 0.0;
 //float l_object_desk = sqrt( dx_object_desk * dx_object_desk + dy_object_desk * dy_object_desk );
  float slope_object_desk = ( dy_object_desk ) / ( dx_object_desk );
  float angle_object_desk = atan2 (dy_object_desk, dx_object_desk);
  if ( angle_object_desk < 0 ) {
    angle_object_desk = angle_object_desk + 2 * M_PI;
  }

  if (DEBUG) {
    cout << "The angle between desk CORNER and object centroid is : " << angle_object_desk
          << " = in degrees " << (angle_object_desk * 180 / 3.14) << endl;
    cout << " when object centroid x = "
         << (inputObject.getCentroid()).x << endl;
    cout << " when object centroid y = "
         << (inputObject.getCentroid()).y << endl;
  }

  angle2d = angle_object_desk;
}


void ApiFeatureExtractionSingleObject::computeVolumeSize(Object & inputObject) {

	if (TESTFLAG) {
		cout << "in ApiFeatureExtractionSingleObject volume 0" << endl;

	}

  pcl::PointCloud<pcl::PointXYZ> internalBoundingBox = inputObject.getBoundingBox();

  //float dim1 = pcl::euclideanDistance(internalBoundingBox.points[1], internalBoundingBox.points[0]);
	if (TESTFLAG) {
		cout << "in ApiFeatureExtractionSingleObject volume 1" << endl;
		cout << "size of bounding box " <<  internalBoundingBox.size() << endl;
		cout << internalBoundingBox.points[1].x << endl;
	}

  float dim1 = pcl::euclideanDistance(internalBoundingBox.points[1], internalBoundingBox.points[0]);

	if (TESTFLAG) {
		cout << "in ApiFeatureExtractionSingleObject volume 2" << endl;
	}
  float dim2 = pcl::euclideanDistance(internalBoundingBox.points[3], internalBoundingBox.points[0]);
  float dim3 = pcl::euclideanDistance(internalBoundingBox.points[4], internalBoundingBox.points[0]);


  volumeSize = (dim1 * dim2 * dim3);

  if (TESTFLAG) {
    cout << "The volume size of current object is: " << volumeSize << endl;
  }

}


void  ApiFeatureExtractionSingleObject::computeSizeProjectedX(Object & inputObject) {
  pcl::PointCloud<pcl::PointXYZ> internalBoundingBox = inputObject.getBoundingBox();
  vector<float> verticesProjectedX;
  for (int i = 0; i < 8; i++) {
    verticesProjectedX.push_back(internalBoundingBox.points[i].x) ;

  }
  std::sort (verticesProjectedX.begin(), verticesProjectedX.end());
  float minX = verticesProjectedX[0];
  float maxX = verticesProjectedX[7];
  float _size = abs ( maxX - minX );
  sizeProjectedX =  _size ;

  if (DEBUG) {
    cout << "The volume size of current object projected onto X is: " << _size << endl;
    cout << "Difference of " << maxX << " and " << minX << endl;
  }
}


void  ApiFeatureExtractionSingleObject::computeSizeProjectedY(Object & inputObject) {
  pcl::PointCloud<pcl::PointXYZ> internalBoundingBox = inputObject.getBoundingBox();
  vector<float> verticesProjectedY;
  for (int i = 0; i < 8; i++) {
    verticesProjectedY.push_back(internalBoundingBox.points[i].y) ;

  }
  std::sort (verticesProjectedY.begin(), verticesProjectedY.end());
  float minY = verticesProjectedY[0];
  float maxY = verticesProjectedY[7];
  float _size = abs ( maxY - minY );
  sizeProjectedY = _size;

  if (DEBUG) {
    cout << "The volume size of current object projected onto y is: " << _size << endl;
  }
}


void  ApiFeatureExtractionSingleObject::computeSizeProjectedZ(Object & inputObject) {
  pcl::PointCloud<pcl::PointXYZ> internalBoundingBox = inputObject.getBoundingBox();
  vector<float> verticesProjectedZ;
  for (int i = 0; i < 8; i++) {
    verticesProjectedZ.push_back(internalBoundingBox.points[i].z) ;

  }
  std::sort (verticesProjectedZ.begin(), verticesProjectedZ.end());
  float minZ = verticesProjectedZ[0];
  float maxZ = verticesProjectedZ[7];
  float _size = abs ( maxZ - minZ );
  sizeProjectedZ = _size;

  if (DEBUG) {
    cout << "The volume size of current object projected onto z is: " << _size << endl;
  }
}




