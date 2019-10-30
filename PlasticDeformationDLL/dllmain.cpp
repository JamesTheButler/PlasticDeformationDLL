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
#include "ConstraintData.h"
#include "ColliderData.h"

#define DLL_EXPORT __declspec(dllexport)

// tet mesh transforms
vec3 _tetMeshPosition = vec3();
vec3 _tetMeshRotation = vec3();

//vertex data
vector<vec3> _vertices;
vector<int> _tetrahedra;
vector<vec3> _tempVertices;
vector<int> _surfaceVertexIndeces;
vector<int> _surfaceTriangles;

ConstraintData _constraintData;
ColliderData _collData;

int _collidingVertexCount = 0;
int _iterationCount = 0;
float _plasticityFactor = 0.0f;

int _debugInt = 0;
float _debugFloat = 0.0f;

void init() {
	_iterationCount = 1;
}

// TODO: clean up correctly
void teardown() {
	_tetMeshPosition = vec3();
	_tetMeshRotation = vec3();
	_collData = ColliderData();
	_constraintData = ConstraintData();
	_collidingVertexCount = 0;

	vector<vec3>().swap(_vertices);
	vector<int>().swap(_tetrahedra);
	vector<vec3>().swap(_tempVertices);
	vector<int>().swap(_surfaceVertexIndeces);
	vector<int>().swap(_surfaceTriangles);
}

#pragma region solver
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

// Projects a point that is inside a box onto the closest plane
vec3 projectOrthogonal(vec3 vertex, AABox box) {
	//find closest plane
	vec3 diffToMax = abs(box.getMax() - vertex);
	vec3 diffToMin = abs(box.getMin() - vertex);

	vec3 closestPoint = vec3();
	closestPoint.x = abs(diffToMax.x) < abs(diffToMin.x) ? box.getMax().x : box.getMin().x;
	closestPoint.y = abs(diffToMax.y) < abs(diffToMin.y) ? box.getMax().y : box.getMin().y;
	closestPoint.z = abs(diffToMax.z) < abs(diffToMin.z) ? box.getMax().z : box.getMin().z;

	vec3 diff = abs(vertex - closestPoint);
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

void parallelProject(vec3 collPos, vec3 collSize, int collType) {
	_collidingVertexCount = 0;
	parallel_for((size_t)0, (size_t)(_vertices.size()-1), (size_t)1, [=](size_t i) {
		vec3 vertex = rotate(_vertices.data()[i], _tetMeshRotation) + _tetMeshPosition;
		if (doesCollide(vertex, collPos, collSize, collType)) {
			_collidingVertexCount++;
			vertex = projectOrthogonal(vertex, collPos, collSize, collType);
			// transform it back into local space
			//_tempVertices.at(i) = revertRotation(vertex - _tetMeshPosition, _tetMeshRotation);
			_vertices.at(i) = revertRotation(vertex - _tetMeshPosition, _tetMeshRotation);
		}
	});
	_debugInt = _collidingVertexCount;
}

void parallelSolveConstraints() {
	// for each changed vertex [p]
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
	
	for (int i = 0; i < _iterationCount; i++) {
		parallelProject(collPos, collSize, collType);
		parallelSolveConstraints();
	}
}
#pragma endregion solver

#pragma region Setters
void setColliders(float* colliderPositions, float* colliderSizes, int* colliderTypes, int colliderCount) {
	_collData.setColliderCount(colliderCount);
	vec3 tempVec3;
	for (int i = 0; i < colliderCount; i++) {
		// pos
		tempVec3.x = colliderPositions[i * 3];
		tempVec3.y = colliderPositions[i * 3 + 1];
		tempVec3.z = colliderPositions[i * 3 + 2];
		_collData.colliderPositions.push_back(tempVec3);
		// size
		tempVec3.x = colliderSizes[i * 3];
		tempVec3.y = colliderSizes[i * 3 + 1];
		tempVec3.z = colliderSizes[i * 3 + 2];
		_collData.colliderSizes.push_back(tempVec3);
		// type
		_collData.colliderTypes.push_back(colliderTypes[i]);
	}
	_debugFloat = _collData.colliderPositions[0].y;
	_debugInt = _collData.colliderTypes[1];
}

void setTetMeshTransforms(float* translation, float* rotation) {
	_tetMeshPosition = vec3(translation[0], translation[1], translation[2]);
	_tetMeshRotation = vec3(rotation[0], rotation[1], rotation[2]);
}

void setIterationCount(int iterationCount) {
	_iterationCount = iterationCount;
}

// Finds index of each element of the subset in the superset and returns a vector of the indeces within the superset.
vector<int> indexSubsetVertices(vector<vec3> subSet, vector<vec3> superSet) {
	vector<int> indeces;
	indeces.reserve(subSet.size());
	std::vector<vec3>::iterator it;
	for (auto i = 0; i < subSet.size(); i++) {
		it = std::find(superSet.begin(), superSet.end(), subSet[i]);
		if (it != superSet.end())
			indeces.push_back(std::distance(superSet.begin(), it));
		else
			indeces.push_back(-1);
	}
	return indeces;
}

void generateConstraints(vector<int> tetrahedra) {
	_constraintData.setConstraintCount(tetrahedra.size() * 6);
	vec3 tetVertices[4];
	int tetVertexIDs[4];
	vector<int> tempIds(2,0);
	int vertexID = 0;
	//for each tetrahdedron
	for (int tetId = 0; tetId < tetrahedra.size()/4; tetId++) {
		//get 4 vertices + their ids
		for (int i = 0; i < 4; i++) {
			vertexID = tetrahedra[tetId * 4 + i];
			tetVertices[i] = _vertices[vertexID];
			tetVertexIDs[i] = vertexID;
		}
		// generate distance constraints along each tetrahedron edge
		tempIds[0] = tetVertexIDs[0];
		tempIds[1] = tetVertexIDs[1];
		_constraintData.addConstraint(tempIds, distance(tetVertices[0], tetVertices[1]), DISTANCE);
		tempIds[0] = tetVertexIDs[0];
		tempIds[1] = tetVertexIDs[2];
		_constraintData.addConstraint(tempIds, distance(tetVertices[0], tetVertices[2]), DISTANCE);
		tempIds[0] = tetVertexIDs[0];
		tempIds[1] = tetVertexIDs[3];
		_constraintData.addConstraint(tempIds, distance(tetVertices[0], tetVertices[3]), DISTANCE);
		tempIds[0] = tetVertexIDs[1];
		tempIds[1] = tetVertexIDs[2];
		_constraintData.addConstraint(tempIds, distance(tetVertices[1], tetVertices[2]), DISTANCE);
		tempIds[0] = tetVertexIDs[1];
		tempIds[1] = tetVertexIDs[3];
		_constraintData.addConstraint(tempIds, distance(tetVertices[1], tetVertices[3]), DISTANCE);
		tempIds[0] = tetVertexIDs[2];
		tempIds[1] = tetVertexIDs[3];
		_constraintData.addConstraint(tempIds, distance(tetVertices[2], tetVertices[3]), DISTANCE);
	}
	// clean up
	vector<int>().swap(tempIds);
}

void setTetMeshData(float* vertices, int vertexCount, int* tetrahedra, int tetCount, float* surfaceVertices, int surfaceVertCount, int* surfaceTriangles, int surfaceTriCount) {
	// vertices
	_vertices.reserve(vertexCount);
	_tempVertices.reserve(vertexCount);
	vec3 temporaryVertex;
	for (int i = 0; i < vertexCount; i++) {
		temporaryVertex.x = vertices[i * 3];
		temporaryVertex.y = vertices[i * 3 + 1];
		temporaryVertex.z = vertices[i * 3 + 2];
		_vertices.push_back(temporaryVertex);
		_tempVertices.push_back(vec3(0, 0, 0));

	}
	// tetrahedra
	_tetrahedra.reserve(tetCount);
	for (int i = 0; i < tetCount; i++) {
		_tetrahedra.push_back(tetrahedra[i]);
	}
	// surface triangles
	_surfaceTriangles.reserve(surfaceTriCount);
	for (int i = 0; i < surfaceTriCount; i++) {
		_surfaceTriangles.push_back(surfaceTriangles[i]);
	}
	// surface Vertices
	vector<vec3> tempSurfaceVertices;
	tempSurfaceVertices.reserve(surfaceVertCount);
	for (int i = 0; i < surfaceVertCount; i++) {
		temporaryVertex.x = surfaceVertices[i * 3];
		temporaryVertex.y = surfaceVertices[i * 3 + 1];
		temporaryVertex.z = surfaceVertices[i * 3 + 2];
		tempSurfaceVertices.push_back(temporaryVertex);
	}
	// indexing
	_surfaceVertexIndeces.reserve(surfaceVertCount);
	_surfaceVertexIndeces = indexSubsetVertices(tempSurfaceVertices, _vertices);
	// set up vertex-to-constrain look up
	_constraintData.constraintsPerVertex.reserve(vertexCount);
	vector<int> filler(0);
	for (int i = 0; i < vertexCount; i++) {
		_constraintData.constraintsPerVertex.push_back(filler);
	}
	// generate constraints
	generateConstraints(_tetrahedra);
	//clean up
	vector<vec3>().swap(tempSurfaceVertices);
}
#pragma endregion Setters

// Writes the vector of surface vertices by de-referencing the surface vertex indeces.
void getSurfaceVertices(vector<vec3> &surfaceVertices) {
	surfaceVertices.reserve(_surfaceVertexIndeces.size());
	for (int i = 0; i < _surfaceVertexIndeces.size(); i++) {
		surfaceVertices.push_back(_vertices[_surfaceVertexIndeces[i]]);
	}
}

// EXPORT FUNCTIONS
extern "C" {
#pragma region Setters
	DLL_EXPORT void dll_setIterationCount(int iterationCount) {
		setIterationCount(iterationCount);
	}
	DLL_EXPORT void dll_setTetMeshData(float* vertices, int vertexCount, int* tetrahedra, int tetCount, float* surfaceVertices, int surfaceVertCount, int* surfaceTriangles, int surfaceTriCount) {
		setTetMeshData(vertices, vertexCount, tetrahedra, tetCount, surfaceVertices, surfaceVertCount, surfaceTriangles, surfaceTriCount);
	}
	DLL_EXPORT void dll_setColliders(float* colliderPositions, float* colliderSizes, int* colliderTypes, int colliderCount) {
		setColliders(colliderPositions, colliderSizes, colliderTypes, colliderCount);
	}
	DLL_EXPORT void dll_setTetMeshTransforms(float* translation, float* rotation) {
		setTetMeshTransforms(translation, rotation);
	}
#pragma endregion Setters
#pragma region Getters
	DLL_EXPORT int dll_getVertexCount() {
		return _vertices.size();
	}
	DLL_EXPORT void dll_getVertices(int* vertexOutput) {
		vector<float> result;
		getVectorData(_tempVertices, result);
		memcpy(vertexOutput, result.data(), _tempVertices.size() * 3 * sizeof(float));
	}
	DLL_EXPORT float dll_getSurfVertexCount() {
		return _surfaceVertexIndeces.size();
	}
	DLL_EXPORT void dll_getSurfVertIndeces(int* output) {
		memcpy(output, _surfaceVertexIndeces.data(), _surfaceVertexIndeces.size() * sizeof(int));
	}
	DLL_EXPORT void dll_getSurfVertices(int* output) {
		// Get all surface vertices
		vector<vec3> surfVertices;
		getSurfaceVertices(surfVertices);
		// get float list of vector data
		vector<float> result;
		getVectorData(surfVertices, result);
		memcpy(output, result.data(), result.size() * sizeof(float));
	}
	DLL_EXPORT void dll_getSurfTriangles(int* output) {
		memcpy(output, _surfaceTriangles.data(), _surfaceTriangles.size()* sizeof(int));
	}
	DLL_EXPORT void dll_getColliders(int* positionOutput, int* sizeOutput, int* typeOutput) {
		vector<float> posResult;
		getVectorData(_collData.colliderPositions, posResult);
		memcpy(positionOutput, posResult.data(), _collData.colliderCount * 3 * sizeof(float));

		vector<float> sizeResult;
		getVectorData(_collData.colliderSizes, sizeResult);
		memcpy(sizeOutput, sizeResult.data(), _collData.colliderCount * 3 * sizeof(float));

		memcpy(typeOutput, _collData.colliderTypes.data(), _collData.colliderCount * sizeof(int));
	}
	DLL_EXPORT void dll_getConstraints(int* vertexIndecesOutput) {

	}
	DLL_EXPORT void dll_getConstraintTypes(int* outputArray) {
		memcpy(outputArray, _constraintData.constraintTypes.data(), _constraintData.constraintCount * sizeof(int));
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
#pragma region setup/setdown
	DLL_EXPORT void dll_init() {
		init();
	}
	DLL_EXPORT void dll_teardown() {
		teardown();
	}
#pragma endregion setup/setdown
#pragma region tests
	DLL_EXPORT int dll_getDebugInt() {
		return _debugInt;
	}
	DLL_EXPORT float dll_getDebugFloat() {
		return _debugFloat;
	}
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
#pragma endregion tests
}