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

#include "AABox.h"

#include "Intersection.h"
#include "Misc.h"

#define DLL_EXPORT __declspec(dllexport)

struct ColliderData {
	vec3* colliderPositions;
	vec3* colliderSizes;
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
	float* vertexDeltas;
	int* constraintsPerVertex;
	int iterationCount = -1;
};

//Solver data
SolverData _solverData;

// tet mesh transforms
vec3 _tetMeshPosition = vec3();
vec3 _tetMeshRotation = vec3();
// outer circle of tet mesh

//vertex data
vec3* _vertexArray;
vec3* _tempVertexArray;
int _vertexCount;

ColliderData _collData;
ConstraintData _constraintData;

// collision data
int _collidingVertexCount = 0;
int* _collidingVertices;

int _debugInt = 0;
float _debugFloat = 0.0f;

float _plasticityFactor = 0.0f;

void init() {
	_solverData.iterationCount = 0;
	_solverData.startId = 0;
	_solverData.delta = vec3();
	_solverData.distance = vec3();
	_solverData.unitVector = vec3();
}

// TODO: clean up correctly
void teardown() {
	_tetMeshPosition = vec3();
	_tetMeshRotation = vec3();
	_tempVertexArray = new vec3();
	_vertexArray = new vec3();
	//delete[] _tempVertexArray;
	//delete[] _vertexArray;
	_vertexCount = 0;
	_collData = ColliderData();
	_constraintData = ConstraintData();
	_solverData = SolverData();
	_collidingVertexCount = 0;
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
// todo: remove (old)
/*vec3 collisionProjection(vec3 vertex, vec3 previousVertex, vec3 colliderPos, vec3 colliderSize, int colliderType) {
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
}*/
/*
void collisionCheck() {
	vec3 collPos = vec3();
	vec3 collSize = vec3();
	int collType = -1;
	for (int j = 0; j < _vertexCount; j++) {
		vec3 vertex = rotate(_vertexArray[j], _tetMeshRotation) + _tetMeshPosition;
		if (doesCollide(vertex, collPos, collSize, collType)) {
			_currentCollidingVertexCount++;
			// todo: project
		}
	}
}*/
#pragma endregion collisions

#pragma region solver
/// projects a point that is inside a box onto the closest plane
vec3 projectOrthogonal(vec3 vertex, AABox box) {
	//find closest plane
	vec3 diffToMax = abs(box.getMax() - vertex);
	vec3 diffToMin = abs(box.getMin() - vertex);

	vec3 closestPoint = vec3();
	closestPoint.x = abs(diffToMax.x) < abs(diffToMin.x) ? box.getMax().x : box.getMin().x;
	closestPoint.y = abs(diffToMax.y) < abs(diffToMin.y) ? box.getMax().y : box.getMin().y;
	closestPoint.z = abs(diffToMax.z) < abs(diffToMin.z) ? box.getMax().z : box.getMin().z;

	vec3 diff = vertex - closestPoint;
	vec3 projectedVertex = vertex;

	if (diff.x < diff.y && diff.x < diff.z)
		projectedVertex.x = closestPoint.x;
	else if (diff.y < diff.x && diff.y < diff.z)
		projectedVertex.y = closestPoint.y;
	else
		projectedVertex.z = closestPoint.z;
	return projectedVertex;
}

vec3 projectOrthogonal(vec3 vertex, vec3 collPos, vec3 collSize, int collType) {
	switch (collType) {
	default:
		return projectOrthogonal(vertex, AABox(collPos, collSize));
	}
}

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
		// parallel for each vertex: add delta/count of constraints
		solveConstraint(i);
	}
}


void sequentialProject(vec3 collPos, vec3 collSize, int collType) {
	_collidingVertexCount = 0;
	for(int i=0; i<_vertexCount; i++) {
		vec3 vertex = rotate(_vertexArray[i], _tetMeshRotation) + _tetMeshPosition;
		if (doesCollide(vertex, collPos, collSize, collType)) {
			_collidingVertexCount++;
			vertex = projectOrthogonal(vertex, collPos, collSize, collType);
			// transform it back into local space
			_tempVertexArray[i] = revertRotation(vertex - _tetMeshPosition, _tetMeshRotation);
		}
	}
	_debugInt = _collidingVertexCount;
}

void sequentialSolveConstraints() {

}

void parallelProject(vec3 collPos, vec3 collSize, int collType) {
	_collidingVertexCount = 0;
	parallel_for((size_t)0, (size_t)_vertexCount, (size_t)1, [=](size_t i) {
		vec3 vertex = rotate(_vertexArray[i], _tetMeshRotation) + _tetMeshPosition;
		if (doesCollide(vertex, collPos, collSize, collType)) {
			_collidingVertexCount++;
			vertex = projectOrthogonal(vertex, collPos, collSize, collType);
			// transform it back into local space
			_tempVertexArray[i] = revertRotation(vertex - _tetMeshPosition, _tetMeshRotation);
			//_vertexArray[i] = revertRotation(vertex - _tetMeshPosition, _tetMeshRotation);
		}
	});
	_debugInt = _collidingVertexCount;
}

void parallelSolveConstraints() {
	// for each changed vertex
	//		get all constraints that influence vertex
	//		determine deltaArray [p]
	//			get current length and rest length of each constraint
	//			rest_new = plasticityFactor * length_current + (1-plasticityFactor)*rest_old;
	//			delta = 
	//		apply deltaArray [p]
	//			vec[i] += delta[i] / constraintPerVec[i];

	// r_new = r_0 * plasticityFactor + r_curr * (1-plasticityFactor)
}

void getCollisionResult(int colliderId) {
	_debugInt = 0;
	
	vec3 collPos = _collData.colliderPositions[colliderId];
	vec3 collSize = _collData.colliderSizes[colliderId];
	int collType = _collData.colliderTypes[colliderId];
	
	// for int i to iterationCount

	//sequentialProject(collPos, collSize, collType);
	//sequentialSolveConstraints();

	parallelProject(collPos, collSize, collType);
	parallelSolveConstraints();

	//solveConstraints();
}
#pragma endregion solver

#pragma region Setters
void setVertices(float* vertices, int vertCount) {
	_vertexCount = vertCount;
	_vertexArray = new vec3[vertCount];
	_tempVertexArray = new vec3[vertCount];
	for (int i = 0; i < vertCount; i++) {
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

	for (int i = 0; i < colliderCount; i++) {
		_collData.colliderTypes[i] = colliderTypes[i];
		_collData.colliderPositions[i].x = colliderPositions[i * 3];
		_collData.colliderPositions[i].y = colliderPositions[i * 3 + 1];
		_collData.colliderPositions[i].z = colliderPositions[i * 3 + 2];
		_collData.colliderSizes[i].x = colliderSizes[i * 3];
		_collData.colliderSizes[i].y = colliderSizes[i * 3 + 1];
		_collData.colliderSizes[i].z = colliderSizes[i * 3 + 2];
	}
}

// TODO: replace by constraint generation
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

// TODO: old
/*
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
}*/

void setIterationCount(int iterationCount) {
	_solverData.iterationCount = iterationCount;
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
		return _collidingVertexCount;
	}

	DLL_EXPORT void dll_getTetMeshTransforms(int* translationOutput, int* rotationOutput) {
		memcpy(translationOutput, getVectorData(_tetMeshPosition), 3 * sizeof(float));
		memcpy(rotationOutput, getVectorData(_tetMeshRotation), 3 * sizeof(float));
	}

#pragma endregion Getters

#pragma region Calculations
	DLL_EXPORT void dll_getCollisionResult(int collId) {
		getCollisionResult(collId);
	}
#pragma endregion Calculations

	DLL_EXPORT void dll_init() {
		init();
	}

	DLL_EXPORT void dll_teardown() {
		teardown();
	}

#pragma region tests
	DLL_EXPORT void dll_testVectorRotation(float* outputRotated, float* outputUnrotated, float* rotationOutput, float* vector, float* rotation) {
		vec3 v = vec3(vector[0], vector[1], vector[2]);
		vec3 r = vec3(rotation[0], rotation[1], rotation[2]);
		memcpy(outputRotated, getVectorData(rotate(v, r)), 3 * sizeof(float));
		memcpy(outputUnrotated, getVectorData(revertRotation(revertRotation(v, r), r)), 3 * sizeof(float));
		memcpy(rotationOutput, getVectorData(r), 3 * sizeof(float));
	}

	DLL_EXPORT bool dll_testVertexAABoxIntersection(float* vertex, float* cPos, float* cSize) {
		return intersect(AABox(vec3(cPos[0], cPos[1], cPos[2]), vec3(cSize[0], cSize[1], cSize[2])), vec3(vertex[0], vertex[1], vertex[2]));
	}

	DLL_EXPORT int dll_getDebugInt() {
		return _debugInt;
	}

	DLL_EXPORT float dll_getDebugFloat() {
		return _debugFloat;
	}

#pragma endregion tests
}

