#include <cmath>

#define DLL_EXPORT __declspec(dllexport)

extern "C" {
	//vertex data
	float* vertexArray;
	int vertexArrayLength;

	//collider data

	//constraint data

	bool isReadyForSimulation;

	struct colliderData {
		float* colliderPositions;
		float* colliderSizes;
		int* colliderTypes;
		int constraintCount;
	};

	//Checks collision between a point and a box (3D)
	bool pointBoxCollision(float* point, float* colliderPosition, float* colliderSize) {
		for (int i = 0; i < 3; i++) {
			if (std::abs(colliderPosition[i] - point[i]) > std::abs(colliderPosition[i] - (colliderSize[i]/2.0f)))
				return false;
		}
		return true;
	}
	
	DLL_EXPORT void setVertices(float* vertices, int elementCount) {
		vertexArrayLength = elementCount;
		vertexArray = new float[elementCount];
		for (int i = 0; i < elementCount; i++) {
			vertexArray[i] = vertices[i];
		}
	}

	DLL_EXPORT void setConstraints() {

	}

	DLL_EXPORT void setColliders(float* colliderPositions, float* colliderSizes, int* colliderTypes,int colliderCount) {
		for (int i = 0; i < colliderCount; i++) {
			this.colliderPositions = colliderPositions;
		}
	}

	/*DLL_EXPORT void setVertices(float* verts, int elementCount, float &result, int &elementCountRes) {
		float res = 0;
		for (int i = 0; i < elementCount; i++) {
			res += verts[i];
		}
		result = res;
		elementCountRes = elementCount;
	}
	*/

	// TODO: add int array for type if collider -> differentiate the type of collision check depending on the type
	DLL_EXPORT void getVertexCountInColliders(float* verts, int elementCount, float* colliderPositions, float* colliderSizes, int colliderCount, int &collisionCount) {
		int collCount = 0;
		float* vertex = new float[3];
		float* collPos = new float[3];
		float* collSize = new float[3];
		for (int i = 0; i < elementCount; i++) {
			// get current vertex
			vertex[0] = verts[i * 3];
			vertex[1] = verts[i * 3 + 1];
			vertex[2] = verts[i * 3 + 2];
			for (int j = 0; j < colliderCount; j++) {
				// get current collider data
				collPos[0] = colliderPositions[j * 3];
				collPos[1] = colliderPositions[j * 3 + 1];
				collPos[2] = colliderPositions[j * 3 + 2];
				collSize[0] = colliderSizes[j * 3];
				collSize[1] = colliderSizes[j * 3 + 1];
				collSize[2] = colliderSizes[j * 3 + 2];

				//collision check
				// switch collType[j]
				if (pointBoxCollision(vertex, collPos, collSize)) {
					collCount++;
				}
			}
		}
		collisionCount = collCount;
	}
}