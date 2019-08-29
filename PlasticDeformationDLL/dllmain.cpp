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

//Solver data
int _iterationCount = -1;

// tet mesh transforms
Vec3f _tetMeshPosition = Vec3f();
Quatf _tetMeshRotation = Quatf();

//vertex data
Point3f* _previousVertexArray;
Point3f* _vertexArray;
int _vertexCount;

//collider data
ColliderData _collData;

//constraint data

// collision data
int _currentCollidingVertexCount = 0;
int* _collidingVertices;

bool isEqualQuatf(Quatf q1, Quatf q2) {
	for (int i = 0; i < 4; i++)
		if (q1.mData[i] != q2.mData[i]) return false;
	return true;
}

void teardown() {
	_tetMeshPosition = Vec3f();
	_tetMeshRotation = Quatf();
	_previousVertexArray = new Point3f();
	_vertexArray = new Point3f();
	_vertexCount = 0;
	_collData = ColliderData();
	_currentCollidingVertexCount = 0;
}

AABoxf makeAABox(Point3f position, Vec3f size) {
	Point3f min = position - (size / 2.0f);
	Point3f max = position + (size / 2.0f);
	return AABoxf(min, max);
}

void solve() {
	// get previous verts

	// generate collision constraints

	for (int i = 0; i < _iterationCount; i++) {
		//for each constraint
		//	constraint.solve();

	}
	// write velocity update 


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
	for (int i = 0; i < _collData.colliderCount; i++) {
		// get current collider data
		collType = _collData.colliderTypes[i];
		collPos = _collData.colliderPositions[i];
		collSize = _collData.colliderSizes[i];

		//go over all vertices
		for (int j = 0; j < _vertexCount; j++) {
			//collision handling
			if (doesCollide(_vertexArray[j], collPos, collSize, collType)) {
				collCount++;
				_vertexArray[j] = collisionProjection(_vertexArray[j], _previousVertexArray[j], collPos, collSize, collType);
			}
		}
	}
	_currentCollidingVertexCount = collCount;
}
#pragma endregion collisions

#pragma region Setters
void setVertices(float* vertices, int vertCount) {
	_vertexCount = vertCount;
	_vertexArray = new Point3f[vertCount];
	_previousVertexArray = new Point3f[vertCount];
	for (int i = 0; i < vertCount; i++) {
		for (int j = 0; j < 3; j++) {
			_previousVertexArray[i] = _vertexArray[i];
			_vertexArray[i].mData[j] = vertices[i * 3 + j];
		}
	}
}

void setColliders(float* colliderPositions, float* colliderSizes, int* colliderTypes, int colliderCount) {
	_collData.colliderCount = colliderCount;
	_collData.colliderPositions = new Point3f[colliderCount];
	_collData.colliderSizes = new Vec3f[colliderCount];
	_collData.colliderTypes = new int[colliderCount];

	for (int i = 0; i < colliderCount; i++) {
		_collData.colliderTypes[i] = colliderTypes[i];
		for (int j = 0; j < 3; j++) {
			_collData.colliderPositions[i].mData[j] = colliderPositions[i * 3 + j];
			_collData.colliderSizes[i].mData[j] = colliderSizes[i * 3 + j];
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
	positionIsEqual = (trans == _tetMeshPosition);
	rotationIsEqual = isEqualQuatf(rot, _tetMeshRotation);

	//apply transforms
	if (!positionIsEqual || !rotationIsEqual) {
		for (int i = 0; i < _vertexCount; i++) {
			//undo old position
			_vertexArray[i] += trans - _tetMeshPosition;
			/*if (!rotationIsEqual) {
				// undo old rotation
				// apply new rotation
			}*/
			// apply new position
			//vertexArray[i] += trans;
		}
		_tetMeshPosition = trans;
		_tetMeshRotation = rot;
	}
}

void setIterationCount(int iterationCount) {
	_iterationCount = iterationCount;
}
#pragma endregion Setters

#pragma region Getters
Vec3f* getUntransformedVertices() {
	Vec3f* untransformedVertices = new Vec3f[_vertexCount];
	// copy and move and rotate each vertex
	for (int i = 0; i < _vertexCount; i++) {
		untransformedVertices[i] = _vertexArray[i] /*- tetMeshPosition*/;
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

	DLL_EXPORT void dll_setIterationCount(int iterationCount) {
		setIterationCount(iterationCount);
	}

#pragma endregion Setters

#pragma region Getters
	DLL_EXPORT void dll_getVertices(int* outputArray) {
		//memcpy(outputArray, getUntransformedVertices()->mData, vertexCount * 3 * sizeof(float));
		memcpy(outputArray, _vertexArray->mData, _vertexCount * 3 * sizeof(float));
	}

	DLL_EXPORT void dll_getColliders(int* positionOutput, int* sizeOutput, int* typeOutput) {
		memcpy(positionOutput, _collData.colliderPositions->mData, _collData.colliderCount * 3 * sizeof(float));
		memcpy(sizeOutput, _collData.colliderSizes->mData, _collData.colliderCount * 3 * sizeof(float));
		memcpy(typeOutput, _collData.colliderTypes, _collData.colliderCount * sizeof(int));
	}

	DLL_EXPORT int dll_getCollisionCount() {
		return _currentCollidingVertexCount;
	}

	DLL_EXPORT void dll_getTetMeshTransforms(int* translationOutput, int* rotationOutput) {
		memcpy(translationOutput, _tetMeshPosition.mData, 3 * sizeof(float));
		memcpy(rotationOutput, _tetMeshRotation.mData.mData, 4 * sizeof(float));
	}
#pragma endregion Getters

#pragma region Calculations
	DLL_EXPORT void dll_collisionHandling() {
		collisionHandling();
	}

	DLL_EXPORT void dll_solve() {
		solve();
	}
#pragma endregion Calculations

	DLL_EXPORT void dll_teardown() {
		teardown();
	}
}