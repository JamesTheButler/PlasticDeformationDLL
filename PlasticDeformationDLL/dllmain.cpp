#include <cmath>
#include <vector>

#define DLL_EXPORT __declspec(dllexport)

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


	//Checks collision between a point and a box (3D)
	bool pointBoxCollision(float* point, float* colliderPosition, float* colliderSize) {
		for (int i = 0; i < 3; i++) {
			if (std::abs(colliderPosition[i] - point[i]) > std::abs(colliderSize[i]/2.0f))
				return false;
		}
		return true;
	}

	// TODO: differentiate the type of collision check depending on the type
	// Returns the count of vertices inside colliders
	void getCollidingVertices() {
		int collCount = 0;
		float* vertex = new float[3];
		float* collPos = new float[3];
		float* collSize = new float[3];
		float* collidingVertices = new float[vertexCount * 3];
		for (int i = 0; i < vertexCount; i++) {
			// get current vertex
			vertex[0] = vertexArray[i * 3];
			vertex[1] = vertexArray[i * 3 + 1];
			vertex[2] = vertexArray[i * 3 + 2];
			for (int j = 0; j < collData.colliderCount; j++) {
				//set type
				// get current collider data
				for (int g = 0; g < 3; g++) {
					collPos[g] = collData.colliderPositions[j * 3 + g];
					collSize[g] = collData.colliderSizes[j * 3 + g];
				}
				//collision check
			// TODO: switch collData.collTypes[j]
				if (pointBoxCollision(vertex, collPos, collSize)) {
					collCount++;
				}
			}
		}
		currentCollidingVertexCount = collCount;
	}

	//transforms the vertices/colliders according to the tet mesh transformation
	/*void tansformTetMesh() {

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

	/// SETTERS
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

	/// GETTERS
	DLL_EXPORT void dll_getVertices(int* outputArray) {
		std::memcpy(outputArray, vertexArray, vertexCount * 3 * sizeof(float));
	}

	DLL_EXPORT void dll_getColliders(int* positionOutput, int* sizeOutput, int* typeOutput) {
		std::memcpy(positionOutput, collData.colliderPositions, collData.colliderCount * 3 * sizeof(float));
		std::memcpy(sizeOutput, collData.colliderSizes, collData.colliderCount * 3 * sizeof(float));
		std::memcpy(typeOutput, collData.colliderTypes, collData.colliderCount * sizeof(int));
	}
	
	DLL_EXPORT int dll_getCollisionCount() {
		return currentCollidingVertexCount;
	}

	///Calculations
	DLL_EXPORT void dll_collisionRepsonses() {
		getCollidingVertices();
	}
}