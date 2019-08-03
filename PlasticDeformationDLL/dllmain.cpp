#include <cmath>
//#include <vector>
#include "LinearAlgebra.h"

#define DLL_EXPORT __declspec(dllexport)

using namespace std;

struct ColliderData {
	float* colliderPositions;
	float* colliderSizes;
	int* colliderTypes;
	int colliderCount;
};

extern "C" {
	// tet mesh transforms
//	float* tetMeshPosition;
//	float* tetMeshRotation;

	//vertex data
	float* previousVertexArray;
	float* vertexArray;
	int vertexCount;
	//collider data
	ColliderData collData;
	//constraint data

	// collision data
	int currentCollidingVertexCount = 0;
	int* collidingVertices;

#pragma region collider specific collision handling
	//Checks collision between a point and a box (3D)
	bool pointBoxCollision(float* point, float* colliderPosition, float* colliderSize) {
		for (int i = 0; i < 3; i++) {
			if (std::abs(colliderPosition[i] - point[i]) > std::abs(colliderSize[i]/2.0f))
				return false;
		}
		return true;
	}

	float* boxCollisionProjection(float*point, float*prevPoint, float*colliderPos, float*colliderSize) {
		float* result = new float[3];
		float* min = new float[3];
		float* max = new float[3];
		float* planePoint = new float[3];

		//determine min and max point of AABB
		for (int i = 0; i < 3; i++) {
			min[i] = colliderPos[i] - (colliderSize[i] / 2);
			max[i] = colliderPos[i] + (colliderSize[i] / 2);
		}
		 
		//determine sector of previous point in relation to collider
		int* prevPointSector = new int[3]{ 0, 0, 0 };
		for (int i = 0; i < 3; i++) {
			if (prevPoint[i] < min[i]) {
				prevPointSector[i] = -1;
				planePoint[i] = min[i];
			}else if (prevPoint[i] < max[i]) {
				prevPointSector[i] = 1;
				planePoint[i] = max[i];
			}
		}

		//determine planes for 


		return result;
	}

#pragma endregion collider specific collision handling methods
	
#pragma region general collision handling
	bool collisionCheck(float* point, float* colliderPos, float* colliderSize, int colliderType) {
		switch (colliderType) {
		case -1: 
			return false;
			break;
		case 1: //box
			return pointBoxCollision(point, colliderPos, colliderSize);
			break;
		}
	}

	float* collisionProjection(float* point, float* prevPoint, float* colliderPos, float* colliderSize, int colliderType) {
		switch (colliderType) {
		case 1:	//box
			return boxCollisionProjection(point, prevPoint, colliderPos, colliderSize);
			break;
		}
	}

	// TODO: differentiate the type of collision check depending on the type
	// Returns the count of vertices inside colliders
	void getCollidingVertices() {
		int collCount = 0;
		float* vertex = new float[3];
		float* prevVertex = new float[3];
		float* collPos = new float[3];
		float* collSize = new float[3];
		int collType = -1;
		float* collidingVertices = new float[vertexCount * 3];

		//go over all vertices
		for (int i = 0; i < vertexCount; i++) {
			// get current vertex
			vertex[0] = vertexArray[i * 3];
			vertex[1] = vertexArray[i * 3 + 1];
			vertex[2] = vertexArray[i * 3 + 2];
			// get corresponding previous vertex
			prevVertex[0] = previousVertexArray[i * 3];
			prevVertex[1] = previousVertexArray[i * 3 + 1];
			prevVertex[2] = previousVertexArray[i * 3 + 2];

			//go over all colliders
			for (int j = 0; j < collData.colliderCount; j++) {
				// get current collider data
				collType = collData.colliderTypes[j];
				for (int g = 0; g < 3; g++) {
					collPos[g] = collData.colliderPositions[j * 3 + g];
					collSize[g] = collData.colliderSizes[j * 3 + g];
				}
				//collision check
				if (collisionCheck(vertex, collPos, collSize, collType)) {
					collCount++;
					vertex = collisionProjection(vertex, previousVertexArray, collPos, collSize, collType);
					//set new vertex data
					vertexArray[i * 3] = vertex[0];
					vertexArray[i * 3 + 1] = vertex[1];
					vertexArray[i * 3 + 2] = vertex[2];
				}
			}
		}
		currentCollidingVertexCount = collCount;
	}

#pragma endregion general collision handling

	/*
	//transforms the vertices/colliders according to the tet mesh transformation
	
	void tansformTetMesh() {

		float rotationMatrix[3][3];

		float rotX = tetMeshRotation[0];
		float rotY = tetMeshRotation[1];
		float rotZ = tetMeshRotation[2];

		rotationMatrix[0][0] = cos(rotY)*sin(rotZ);
		rotationMatrix[0][1] = cos(rotX)*cos(rotZ) - sin(rotX)*sin(rotY)*sin(rotZ);
		rotationMatrix[0][2] = sin(rotX)*sin(rotZ)- cos(rotX)* sin(rotY)*cos(rotZ);

		rotationMatrix[1][0] = -cos(rotY)*sin(rotZ);
		rotationMatrix[1][1] = cos(rotX)*cos(rotZ)-sin(rotX)*sin(rotY)*sin(rotZ);
		rotationMatrix[1][2] = sin(rotX)*cos(rotZ) + cos(rotX)*sin(rotY)*sin(rotZ);
							   ;
		rotationMatrix[0][0] = ;
		rotationMatrix[0][1] = ;
		rotationMatrix[0][2] = ;

		//rotate
		for (int i = 0; i < vertexArrayLength; i++) {

		}
		//move
		for (int i = 0; i < vertexArrayLength; i++) {

		}
	}*/

	/*DLL_EXPORT void setTetMeshTransform(float* position, float* rotation) {
		tetMeshPosition = new float[3];
		tetMeshRotation = new float[3];
		
		for (int i = 0; i < 3; i++) {
			tetMeshPosition = position;
			tetMeshRotation = rotation;
		}
	}*/

#pragma region Exports

#pragma region Setters
	DLL_EXPORT void dll_setVertices(float* vertices, int vertCount) {
		vertexCount = vertCount;
		vertexArray = new float[vertCount*3];
		previousVertexArray = new float[vertCount * 3];
		for (int i = 0; i < vertCount*3; i++) {
			previousVertexArray[i] = vertexArray[i];
			vertexArray[i] = vertices[i];
		}
	}


	DLL_EXPORT void dll_setConstraints() {

	}

	DLL_EXPORT void dll_setColliders(float* colliderPositions, float* colliderSizes, int* colliderTypes, int colliderCount) {
		collData.colliderCount = colliderCount;
		collData.colliderPositions = new float[colliderCount*3];
		collData.colliderSizes = new float[colliderCount*3];
		collData.colliderTypes = new int[colliderCount];

		for (int i = 0; i < colliderCount*3; i++) {
			collData.colliderPositions[i] = colliderPositions[i];
			collData.colliderSizes[i] = colliderSizes[i];
		}
		for (int i = 0; i < colliderCount; i++) {
			collData.colliderTypes[i] = colliderTypes[i];
		}
	}
#pragma endregion Setters

#pragma region Getters
	DLL_EXPORT void dll_getVertices(int* outputArray) {
		memcpy(outputArray, vertexArray, vertexCount * 3 * sizeof(float));
	}

	DLL_EXPORT void dll_getColliders(int* positionOutput, int* sizeOutput, int* typeOutput) {
		memcpy(positionOutput, collData.colliderPositions, collData.colliderCount * 3 * sizeof(float));
		memcpy(sizeOutput, collData.colliderSizes, collData.colliderCount * 3 * sizeof(float));
		memcpy(typeOutput, collData.colliderTypes, collData.colliderCount * sizeof(int));
	}
	
	DLL_EXPORT int dll_getCollisionCount() {
		return currentCollidingVertexCount;
	}
#pragma endregion Getters

#pragma region Calculations
	DLL_EXPORT void dll_collisionRepsonses() {
		getCollidingVertices();
	}
#pragma endregion Calculations

#pragma endregion Exports
}