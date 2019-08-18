#include <cmath>
#include <cstring>
#include <gmtl/Intersection.h>
#include <gmtl/Quat.h>

#define DLL_EXPORT __declspec(dllexport)

using namespace std;
using namespace gmtl;

struct ColliderData {
	Point3f* colliderPositions;
	Vec3f* colliderSizes;
	int* colliderTypes;
	int colliderCount;
};

// tet mesh transforms
Vec3f tetMeshPosition = Vec3f();
Quatf tetMeshRotation = Quatf();

//vertex data
Point3f* previousVertexArray;
Point3f* vertexArray;
int vertexCount;

//collider data
ColliderData collData;

//constraint data

// collision data
int currentCollidingVertexCount = 0;
int* collidingVertices;

bool isEqualQuatf(Quatf q1, Quatf q2) {
	for (int i = 0; i < 4; i++)
		if (q1.mData[i] != q2.mData[i]) return false;
	return true;
}

void teardown() {
	tetMeshPosition = Vec3f();
	tetMeshRotation = Quatf();
	previousVertexArray = new Point3f();
	vertexArray = new Point3f();
	vertexCount = 0;
	collData = ColliderData();
	currentCollidingVertexCount = 0;
}

AABoxf makeAABox(Point3f position, Vec3f size) {
	Point3f min = position - (size / 2.0f);
	Point3f max = position + (size / 2.0f);
	return AABoxf(min, max);
}

#pragma region collisions
bool doesCollide(Point3f vertex, Point3f colliderPos, Vec3f colliderSize, int colliderType) {
	switch (colliderType) {
	case -1: // default/unset
		return false;
	case 1: {//box
		AABoxf aabb = makeAABox(colliderPos, colliderSize);
		return intersect(aabb, vertex);
	}
	default:
		return false;
	}
}

Point3f collisionProjection(Point3f vertex, Point3f previousVertex, Point3f colliderPos, Vec3f colliderSize, int colliderType) {
	// generate line between previous and current vertex
	/*LineSegf line = LineSegf(previousVertex, vertex);
	switch (colliderType) {
	case 1: {//box
		AABoxf aabb = makeAABox(colliderPos, colliderSize);
		float t_in = 0.0f, t_out = 0.0f;
		unsigned int numHits = 0;
		intersect(aabb, line, numHits, t_in, t_out);
		return previousVertex + (vertex - previousVertex) * t_in;
	}
	default:
		return vertex;
	}*/


	// generate ray from current to previous vertex
	Rayf line = Rayf(vertex, previousVertex - vertex);

	switch (colliderType) {
	case 1: {//box
		AABoxf aabb = makeAABox(colliderPos, colliderSize);

		float t_in = 0.0f, t_out = 0.0f;
		unsigned int numHits = 0;
		intersect(aabb, line, numHits, t_in, t_out);
		return vertex + (previousVertex - vertex) * t_out;
	}
	default:
		return vertex;
	}
}

void collisionHandling() {
	int collCount = 0;
	Point3f collPos = Point3f();
	Vec3f collSize = Vec3f();
	int collType = -1;

	//go over all colliders
	for (int i = 0; i < collData.colliderCount; i++) {
		// get current collider data
		collType = collData.colliderTypes[i];
		collPos = collData.colliderPositions[i];
		collSize = collData.colliderSizes[i];

		//go over all vertices
		for (int j = 0; j < vertexCount; j++) {
			//collision handling
			if (doesCollide(vertexArray[j], collPos, collSize, collType)) {
				collCount++;
				vertexArray[j] = collisionProjection(vertexArray[j], previousVertexArray[j], collPos, collSize, collType);
			}
		}
	}
	currentCollidingVertexCount = collCount;
}
#pragma endregion collisions

#pragma region Setters
void setVertices(float* vertices, int vertCount) {
	vertexCount = vertCount;
	vertexArray = new Point3f[vertCount];
	previousVertexArray = new Point3f[vertCount];
	for (int i = 0; i < vertCount; i++) {
		for (int j = 0; j < 3; j++) {
			previousVertexArray[i] = vertexArray[i];
			vertexArray[i].mData[j] = vertices[i * 3 + j];
		}
	}
}

void setColliders(float* colliderPositions, float* colliderSizes, int* colliderTypes, int colliderCount) {
	collData.colliderCount = colliderCount;
	collData.colliderPositions = new Point3f[colliderCount];
	collData.colliderSizes = new Vec3f[colliderCount];
	collData.colliderTypes = new int[colliderCount];

	for (int i = 0; i < colliderCount; i++) {
		collData.colliderTypes[i] = colliderTypes[i];
		for (int j = 0; j < 3; j++) {
			collData.colliderPositions[i].mData[j] = colliderPositions[i * 3 + j];
			collData.colliderSizes[i].mData[j] = colliderSizes[i * 3 + j];
		}
	}
}

void applyTetMeshTransformation(float* translation, float* rotation) {
	Point3f trans;
	Quatf rot;
	// create usable data objects
	for (int i = 0; i < 3; i++) { trans.mData[i] = translation[i]; }
	for (int i = 0; i < 4; i++) { rot.mData[i] = rotation[i]; }

	//determine differnces between old and new transfoms
	bool positionIsEqual = true, rotationIsEqual = true;
	positionIsEqual = (trans == tetMeshPosition);
	rotationIsEqual = isEqualQuatf(rot, tetMeshRotation);

	//apply transforms
	if (!positionIsEqual || !rotationIsEqual) {
		for (int i = 0; i < vertexCount; i++) {
			//undo old position
			vertexArray[i] += trans - tetMeshPosition;
			/*if (!rotationIsEqual) {
				// undo old rotation
				// apply new rotation
			}*/
			// apply new position
			//vertexArray[i] += trans;
		}
		tetMeshPosition = trans;
		tetMeshRotation = rot;
	}
}

#pragma endregion Setters

#pragma region Getters
Vec3f* getUntransformedVertices() {
	Vec3f* untransformedVertices = new Vec3f[vertexCount];
	// copy and move and rotate each vertex
	for (int i = 0; i < vertexCount; i++) {
		untransformedVertices[i] = vertexArray[i] /*- tetMeshPosition*/;
		// TODO: undo rotation per vertex
		//untransformedVertices[i] = vertexArray[i];
	}
	return untransformedVertices;
}
#pragma endregion Getters

// EXPORT FUNCTIONS
extern "C" {
#pragma region Setters
	DLL_EXPORT void dll_setVertices(float* vertices, int vertCount) {
		setVertices(vertices, vertCount);
	}

	DLL_EXPORT void dll_setConstraints() {
	}

	DLL_EXPORT void dll_setColliders(float* colliderPositions, float* colliderSizes, int* colliderTypes, int colliderCount) {
		setColliders(colliderPositions, colliderSizes, colliderTypes, colliderCount);
	}

	DLL_EXPORT void dll_setTetMeshTransforms(float* translation, float* rotation) {
		applyTetMeshTransformation(translation, rotation);
	}

#pragma endregion Setters

#pragma region Getters
	DLL_EXPORT void dll_getVertices(int* outputArray) {
		//memcpy(outputArray, getUntransformedVertices()->mData, vertexCount * 3 * sizeof(float));
		memcpy(outputArray, vertexArray->mData, vertexCount * 3 * sizeof(float));
	}

	DLL_EXPORT void dll_getColliders(int* positionOutput, int* sizeOutput, int* typeOutput) {
		memcpy(positionOutput, collData.colliderPositions->mData, collData.colliderCount * 3 * sizeof(float));
		memcpy(sizeOutput, collData.colliderSizes->mData, collData.colliderCount * 3 * sizeof(float));
		memcpy(typeOutput, collData.colliderTypes, collData.colliderCount * sizeof(int));
	}

	DLL_EXPORT int dll_getCollisionCount() {
		return currentCollidingVertexCount;
	}

	DLL_EXPORT void dll_getTetMeshTransforms(int* translationOutput, int* rotationOutput) {
		memcpy(translationOutput, tetMeshPosition.mData, 3 * sizeof(float));
		memcpy(rotationOutput, tetMeshRotation.mData.mData, 4 * sizeof(float));
	}
#pragma endregion Getters

#pragma region Calculations
	DLL_EXPORT void dll_collisionHandling() {
		collisionHandling();
	}
#pragma endregion Calculations

	DLL_EXPORT void dll_teardown() {
		teardown();
	}
}