#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>
using namespace std;

#include <glm/vec3.hpp>
#include <glm/gtx/transform.hpp>
using namespace glm;

#include <tbb/parallel_for.h>
 #include <tbb/blocked_range.h>
using namespace tbb;

#include "Ray.h"
#include "AABox.h"
#include "Sphere.h"
#include "OBox.h"
#include "Intersection.h"
#include "Misc.h"

#define DLL_EXPORT __declspec(dllexport)

const int size_t = 32;

struct ColliderData {
	vec3* colliderPositions;
	vec3* colliderSizes;
	float* colliderOuterSphereRadius;
	int* colliderTypes;
	int colliderCount;
};

struct ConstraintData {
	//array of vertex indeces
	int* vertexIds;
	// starting index of the vertexID array
	int* vertexStartIndeces;
	// number of vertex indeces for each constraint
	int* vertexIdArrayLengths;
	float* currentValues;
	float* restValues;
	int* constraintTypes;
	int constraintCount;
};

// holds solver data to prevent re-initialization
struct SolverData {
	vec3 delta, distance, unitVector;
	int startId;
};

//Solver data
int _iterationCount = -1;
SolverData _solverData;

// tet mesh transforms
vec3 _tetMeshPosition = vec3();
vec3 _tetMeshRotation = vec3();
// outer circle of tet mesh
Sphere _tetMeshOuterSphere = Sphere();

//vertex data
vec3* _previousVertexArray;
vec3* _vertexArray;
int _vertexCount;

ColliderData _collData;
ConstraintData _constraintData;

// collision data
int _currentCollidingVertexCount = 0;
int _closeColliderCount = 0;
int* _collidingVertices;

void init() {
	_iterationCount = 0;
	_solverData.startId = 0;
	_solverData.delta = vec3();
	_solverData.distance = vec3();
	_solverData.unitVector = vec3();
}

void teardown() {
	_tetMeshPosition = vec3();
	_tetMeshRotation = vec3();
	_previousVertexArray = new vec3();
	_vertexArray = new vec3();
	_vertexCount = 0;
	_collData = ColliderData();
	_constraintData = ConstraintData();
	_solverData = SolverData();
	_currentCollidingVertexCount = 0;
	_closeColliderCount = 0;
}

#pragma region collisions
bool doesCollide(vec3 vertex, vec3 colliderPos, vec3 colliderSize, int colliderType) {
	switch (colliderType) {
	case -1: // default/unset
		return false;
	case 1: {//box
		AABox aabb = AABox(colliderPos, colliderSize);
		return intersect(aabb, vertex);
	}
	default:
		return false;
	}
}

vec3 collisionProjection(vec3 vertex, vec3 previousVertex, vec3 colliderPos, vec3 colliderSize, int colliderType) {
	// generate ray from current to previous vertex
	Ray line = Ray(vertex, previousVertex - vertex);
	switch (colliderType) {
	case 1: {//box
		AABox aabb = AABox(colliderPos, colliderSize);
		vec3 in, out;
		intersect(aabb, line, in, out);
		return out;
	}
	default:
		return vertex;
	}
}

void collisionCheck() {
	vec3 collPos = vec3();
	vec3 collSize = vec3();
	Sphere colliderOuterSphere = Sphere();
	//Sphere tetMeshOuterSphere = Sphere(_tetMeshOuterSphere);
	//	Sphere vertexOuterSphere = Sphere();
	int collType = -1;
	for (int j = 0; j < _vertexCount; j++) {
		//	vertexOuterSphere._center = _tetMeshPosition;
		//	vertexOuterSphere._radius = length(_vertexArray[j]);
		// coarse check (before applying rotation to vertex)
		// define possible positions of rotated vertex with a description as a circle
		//	if (intersect(vertexOuterSphere, colliderOuterSphere)) {
			//collision handling
			//transform vertex
		vec3 vertex = rotate(_vertexArray[j], _tetMeshRotation) + _tetMeshPosition;
		if (doesCollide(vertex, collPos, collSize, collType)) {
			_currentCollidingVertexCount++;
		}
		//	}
	}
}

void collisionHandling() {
	int collCount = 0;
	vec3 collPos = vec3();
	vec3 collSize = vec3();
	int collType = -1;
	_closeColliderCount = 0;
	Sphere colliderOuterSphere = Sphere();
	Sphere tetMeshOuterSphere = Sphere(_tetMeshOuterSphere);
	Sphere vertexOuterSphere = Sphere();

	vec3 tempVert = vec3();

	tetMeshOuterSphere._center += _tetMeshPosition;

	//go over all colliders
	for (int i = 0; i < _collData.colliderCount; i++) {
		// get current collider data
		collType = _collData.colliderTypes[i];
		collPos = _collData.colliderPositions[i];
		collSize = _collData.colliderSizes[i];

		// skip collision handling when collider is not within range of tet mesh (check whether outer circles of collider and car intersect)
		colliderOuterSphere._center = collPos;
		colliderOuterSphere._radius = _collData.colliderOuterSphereRadius[i];

		if (!intersect(tetMeshOuterSphere, colliderOuterSphere))
			continue;
		_closeColliderCount++;

		parallel_for(blocked_range<int>(0, 10), collisionCheck());

		//go over all vertices
		/*for (int j = 0; j < _vertexCount; j++) {
			vertexOuterSphere._center = _tetMeshPosition;
			vertexOuterSphere._radius = length(_vertexArray[j]);
			// coarse check (before applying rotation to vertex)
			// define possible positions of rotated vertex with a description as a circle
			if(intersect(vertexOuterSphere, colliderOuterSphere)) {
				//collision handling
				//transform vertex
				vec3 vertex = rotate(_tetMeshRotation, _vertexArray[j]) + _tetMeshPosition;
				if (doesCollide(vertex, collPos, collSize, collType)) {
					collCount++;
				//	tempVert = _vertexArray[j];
				//	_vertexArray[j] = collisionProjection(vertex, _previousVertexArray[j], collPos, collSize, collType);
				//	_previousVertexArray[j] = tempVert;
				}
			}
		}*/
	}
	_currentCollidingVertexCount = collCount;
}
#pragma endregion collisions

#pragma region solveing
void solveConstraint(int index) {
	// if (restValue == currentValue) return;
	switch (_constraintData.constraintTypes[index]) {
	case -1:
		break;
	case 0: { // distance
		// determine distance
		/*_solverData.startId = _constraintData.vertexStartIndeces[index];
		_solverData.distance = vec3(_vertexArray[_solverData.startId] - _vertexArray[_solverData.startId + 1]);
		_solverData.unitVector = _solverData.distance / length(_solverData.distance);
		_solverData.delta = 0.5f * (length(_solverData.distance) - _constraintData.restValues[index]) * _solverData.unitVector;
		// update vertex array
		_vertexArray[_solverData.startId] -= delta;
		_vertexArray[_solverData.startId + 1] += delta;*/
		break;
	}
	case 1: // angle
		break;
	case 2: // volume
		break;
	}
}

void solveConstraints() {
	for (int i = 0; i < _constraintData.constraintCount; i++) {
		solveConstraint(i);
	}
}

void solve() {
	//handle collisions
	collisionHandling();

	// solve constraints
	for (int i = 0; i < _iterationCount; i++) {
		solveConstraints();
	}

	// write velocity update
		//???
}
#pragma endregion solver

#pragma region Setters
void setVertices(float* vertices, int vertCount) {
	_vertexCount = vertCount;
	_vertexArray = new vec3[vertCount];
	_previousVertexArray = new vec3[vertCount];
	for (int i = 0; i < vertCount; i++) {
		_previousVertexArray[i] = _vertexArray[i];
		_vertexArray[i].x = vertices[i * 3];
		_vertexArray[i].y = vertices[i * 3 + 1];
		_vertexArray[i].z = vertices[i * 3 + 2];
	}
}

void setColliders(float* colliderPositions, float* colliderSizes, int* colliderTypes, int colliderCount) {
	_collData.colliderCount = colliderCount;
	_collData.colliderPositions = new vec3[colliderCount];
	_collData.colliderSizes = new vec3[colliderCount];
	_collData.colliderTypes = new int[colliderCount];
	_collData.colliderOuterSphereRadius = new float[colliderCount];

	for (int i = 0; i < colliderCount; i++) {
		_collData.colliderTypes[i] = colliderTypes[i];
		_collData.colliderPositions[i].x = colliderPositions[i * 3];
		_collData.colliderPositions[i].y = colliderPositions[i * 3 + 1];
		_collData.colliderPositions[i].z = colliderPositions[i * 3 + 2];
		_collData.colliderSizes[i].x = colliderSizes[i * 3];
		_collData.colliderSizes[i].y = colliderSizes[i * 3 + 1];
		_collData.colliderSizes[i].z = colliderSizes[i * 3 + 2];
		_collData.colliderOuterSphereRadius[i] = length(_collData.colliderSizes[i])/2.0f;
	}
}

void setConstraints(int* vertexIds, int* vertexStartIndeces, int* vertexIdArrayLengths, float* currentValues, float* restValues, int* constraintTypes, int constraintCount) {
	_constraintData.constraintCount = constraintCount;
	_constraintData.vertexStartIndeces = new int[constraintCount];
	_constraintData.vertexIdArrayLengths = new int[constraintCount];
	_constraintData.currentValues = new float[constraintCount];
	int vertexIdCount = vertexStartIndeces[constraintCount - 1] + vertexIdArrayLengths[constraintCount - 1];
	_constraintData.vertexIds = new int[vertexIdCount-1];
	_constraintData.restValues = new float[constraintCount];
	_constraintData.constraintTypes = new int[constraintCount];
	
	for (int i = 0; i < constraintCount; i++) {
		_constraintData.vertexIds[i] = vertexIds[i];
		_constraintData.vertexStartIndeces[i] = vertexStartIndeces[i];
		_constraintData.vertexIdArrayLengths[i] = vertexIdArrayLengths[i];
		_constraintData.currentValues[i] = currentValues[i];
		_constraintData.restValues[i] = restValues[i];
		_constraintData.constraintTypes[i] = constraintTypes[i];
	}
	for (int i = constraintCount; i < vertexIdCount; i++) {
		_constraintData.vertexIds[i] = vertexIds[i];
	}
}

void setTetMeshTransforms(float* translation, float* rotation) {
	_tetMeshPosition = vec3(translation[0], translation[1], translation[2]);
	_tetMeshRotation = vec3(rotation[0], rotation[1], rotation[2]);
}

void setOuterSphereData(float* center, float radius) {
	_tetMeshOuterSphere._center = vec3(center[0], center[1], center[2]);
	_tetMeshOuterSphere._radius = radius;
}

void applyTetMeshTransformation(float* translation, float* rotation) {
	vec3 trans;
	vec3 rot;
	trans = vec3(translation[0], translation[1], translation[2]);
	rot = vec3(rotation[0], rotation[1], rotation[2]);

	//determine differnces between old and new transfoms
	bool positionIsDifferent, rotationIsDifferent;
	positionIsDifferent = (trans != _tetMeshPosition);
	rotationIsDifferent = (rot != _tetMeshRotation);

	//apply transforms
	if (positionIsDifferent || rotationIsDifferent) {
		for (int i = 0; i < _vertexCount; i++) {
			//undo old position
			_vertexArray[i] -= _tetMeshPosition;
			if (rotationIsDifferent) {
				// undo old rotation
				//_vertexArray[i] = undoRotation(_tetMeshRotation, _vertexArray[i]);
				// apply new rotation
				//_vertexArray[i] = rotate(rot, _vertexArray[i]);
			}
			// apply new position
			_vertexArray[i] += trans;
		}
	}
	setTetMeshTransforms(translation, rotation);
}

void setIterationCount(int iterationCount) {
	_iterationCount = iterationCount;
}
#pragma endregion Setters

// EXPORT FUNCTIONS
extern "C" {
#pragma region Setters
	DLL_EXPORT void dll_setVertices(float* vertices, int vertCount) {
		setVertices(vertices, vertCount);
	}

	DLL_EXPORT void dll_setConstraints(int* vertexIds, int* vertexStartIndeces, int* vertexIndexArrayLength, float* currentValues, float* restValues, int* colliderTypes, int constraintCount) {
		setConstraints(vertexIds, vertexStartIndeces, vertexIndexArrayLength, currentValues, restValues, colliderTypes, constraintCount);
	}

	DLL_EXPORT void dll_setColliders(float* colliderPositions, float* colliderSizes, int* colliderTypes, int colliderCount) {
		setColliders(colliderPositions, colliderSizes, colliderTypes, colliderCount);
	}

	DLL_EXPORT void dll_setOuterSphereData(float* position, float radius) {
		setOuterSphereData(position, radius);
	}

	DLL_EXPORT void dll_setTetMeshTransforms(float* translation, float* rotation) {
		//applyTetMeshTransformation(translation, rotation);
		setTetMeshTransforms(translation, rotation);
	}

	DLL_EXPORT void dll_setIterationCount(int iterationCount) {
		setIterationCount(iterationCount);
	}
#pragma endregion Setters

#pragma region Getters
	DLL_EXPORT void dll_getVertices(int* outputArray) {
		memcpy(outputArray, getVectorData(_vertexArray, _vertexCount), _vertexCount * 3 * sizeof(float));
	}

	DLL_EXPORT void dll_getColliders(int* positionOutput, int* sizeOutput, int* typeOutput) {
		memcpy(positionOutput, getVectorData(_collData.colliderPositions, _collData.colliderCount) , _collData.colliderCount * 3 * sizeof(float));
		memcpy(sizeOutput, getVectorData(_collData.colliderSizes, _collData.colliderCount), _collData.colliderCount * 3 * sizeof(float));
		memcpy(typeOutput, _collData.colliderTypes, _collData.colliderCount * sizeof(int));
	}

	DLL_EXPORT void dll_getConstraintTypes(int* outputArray) {
		memcpy(outputArray, _constraintData.constraintTypes, _constraintData.constraintCount * sizeof(int));
	}

	DLL_EXPORT int dll_getCollisionCount() {
		return _currentCollidingVertexCount;
		//return _closeColliderCount;
	}

	DLL_EXPORT void dll_getTetMeshTransforms(int* translationOutput, int* rotationOutput) {
		memcpy(translationOutput, getVectorData(_tetMeshPosition), 3 * sizeof(float));
		memcpy(rotationOutput, getVectorData(_tetMeshRotation), 3 * sizeof(float));
	}

	DLL_EXPORT int dll_getCloseColliderCount() {
		return _closeColliderCount;
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

	DLL_EXPORT void dll_init() {
		init();
	}

	DLL_EXPORT void dll_teardown() {
		teardown();
	}
}