#include <cmath>
#include <cstring>
#include <vector>
#include <chrono>
#include <algorithm>
using namespace std;

#include <glm/vec3.hpp>
#include <glm/gtx/transform.hpp>
using namespace glm;

#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/blocked_range.h>
using namespace tbb;

#include "AABox.h"
#include "Intersection.h"
#include "Misc.h"
#include "DistanceConstraintData.h"
#include "VolumeConstraintData.h"
#include "ColliderData.h"
#include "BarycentricMapping.h"
#include "Types.h"

#define DLL_EXPORT __declspec(dllexport)

// tet mesh transforms
vec3 _tetMeshPosition = vec3();
vec3 _tetMeshRotation = vec3();

//vertex data
vector<vec3> _vertices;
vector<int> _tetrahedra;
vector<int> _surfaceVertexIndeces;
vector<int> _surfaceTriangles;

DistanceConstraintData _distanceConstraintData;
VolumeConstraintData _volumeConstraintData;
ColliderData _collData;
vector<vec3> _distanceDeltas;
vector<vec3> _volumeDeltas;

int _solverDeltaTime = 0.0f;

int _collidingVertexCount = 0;
int _iterationCount = 0;
float _plasticityFactor = 0.0f;

int _debugInt = 0;
float _debugFloat = 0.0f;

void init() {
	_iterationCount = 1;
}

// TODO: clean up correctly
// TODO: optimize (pass-by-reference, const (if unchanged)
void teardown() {
	_tetMeshPosition = vec3();
	_tetMeshRotation = vec3();
	_collData = ColliderData();
	_distanceConstraintData.cleanUp();
	_volumeConstraintData.cleanUp();
	_distanceConstraintData = DistanceConstraintData();
	_volumeConstraintData = VolumeConstraintData();
	_collidingVertexCount = 0;

	vector<vec3>().swap(_vertices);
	vector<int>().swap(_tetrahedra);
	vector<int>().swap(_surfaceVertexIndeces);
	vector<int>().swap(_surfaceTriangles);
	vector<vec3>().swap(_distanceDeltas);
	vector<vec3>().swap(_volumeDeltas);
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

// Projects a vertex orthogonally onto a colliders surface.
vec3 projectOrthogonal(vec3 vertex, vec3 collPos, vec3 collSize, int collType) {
	switch (collType) {
	default:
		return projectOrthogonal(vertex, AABox(collPos, collSize));
	}
}

void projectVertices(vec3 collPos, vec3 collSize, int collType) {
	_collidingVertexCount = 0;
	parallel_for((size_t)0, (size_t)(_vertices.size()-1), (size_t)1, [=](size_t i) {
		vec3 vertex = rotate(_vertices.data()[i], _tetMeshRotation) + _tetMeshPosition;
		if (doesCollide(vertex, collPos, collSize, collType)) {
			_collidingVertexCount++;
			vertex = projectOrthogonal(vertex, collPos, collSize, collType);
			// transform it back into local space
			_vertices[i] = revertRotation(vertex - _tetMeshPosition, _tetMeshRotation);
		}
	});
}

float linearInterpolation(float val1, float val2, float t) {
	return t * val1 + (1 - t) * val2;
}

void solveVolumeConstraints() {
	//clear deltas
	vector<vec3>(_vertices.size(), vec3(0, 0, 0)).swap(_volumeDeltas);
	//calc deltas
	parallel_for((size_t)0, (size_t)(_volumeConstraintData.constraintCount - 1), (size_t)1, [=](size_t i) {
		// get actual vertices
		vec3 verts[4];
		verts[0] = _vertices[get<0>(_volumeConstraintData.vertexIds[i])];
		verts[1] = _vertices[get<1>(_volumeConstraintData.vertexIds[i])];
		verts[2] = _vertices[get<2>(_volumeConstraintData.vertexIds[i])]; 
		verts[3] = _vertices[get<3>(_volumeConstraintData.vertexIds[i])];

		float newRest = linearInterpolation(getTetrahedronVolume(verts[0], verts[1], verts[2], verts[3]), _volumeConstraintData.restValues[i],_plasticityFactor);
		float volumeDifference = _volumeConstraintData.restValues[i] - newRest;
		//_volumeConstraintData.restValues[i] = newRest;
		// determine displacement vectors per vertex (normals of opposing faces)
		vec3 p0p1 = verts[1] - verts[0];
		vec3 p0p2 = verts[2] - verts[0];
		vec3 p0p3 = verts[3] - verts[0];
		vec3 grad[4];
		grad[1] = cross(p0p2, p0p3) / 6.f;
		grad[2] = cross(p0p3, p0p1) / 6.f;
		grad[3] = cross(p0p1, p0p2) / 6.f;
		grad[0] = (grad[1] + grad[2] + grad[3]) * -1.f;
		// determine deltas
		// denominator of the scaling factor
		float sum_squared_grad_p = dot(grad[0], grad[0]) + dot(grad[1], grad[1]) + dot(grad[2], grad[2]) + dot(grad[3], grad[3]);
		
		float displacement = 0;
		if (sum_squared_grad_p > 0.00001f) {
			displacement = (volumeDifference / sum_squared_grad_p);
		} else {
			displacement = 0;
		}
		_volumeDeltas[get<0>(_volumeConstraintData.vertexIds[i])] += grad[0] * displacement;
		_volumeDeltas[get<1>(_volumeConstraintData.vertexIds[i])] += grad[1] * displacement;
		_volumeDeltas[get<2>(_volumeConstraintData.vertexIds[i])] += grad[2] * displacement;
		_volumeDeltas[get<3>(_volumeConstraintData.vertexIds[i])] += grad[3] * displacement;
	});
	// apply deltas
	parallel_for((size_t)0, (size_t)(_vertices.size() - 1), (size_t)1, [=](size_t i) {
		_vertices[i] += _volumeDeltas[i] / (float)_volumeConstraintData.constraintsPerVertex[i];
	});
	// update rest values
	parallel_for((size_t)0, (size_t)(_volumeConstraintData.constraintCount - 1), (size_t)1, [=](size_t i) {
		vec3 verts[4];
		verts[0] = _vertices[get<0>(_volumeConstraintData.vertexIds[i])];
		verts[1] = _vertices[get<1>(_volumeConstraintData.vertexIds[i])];
		verts[2] = _vertices[get<2>(_volumeConstraintData.vertexIds[i])];
		verts[3] = _vertices[get<3>(_volumeConstraintData.vertexIds[i])];
		_volumeConstraintData.restValues[i] = getTetrahedronVolume(verts[0], verts[1], verts[2], verts[3]);
	});
}

void solveDistanceConstraints() {
	// clear deltas
	vector<vec3>(_vertices.size(), vec3(0, 0, 0)).swap(_distanceDeltas);
	//calc deltas
	parallel_for((size_t)0, (size_t)(_distanceConstraintData.constraintCount - 1), (size_t)1, [=](size_t i) {
		int id1 = _distanceConstraintData.vertexIds[i].first;
		int id2 = _distanceConstraintData.vertexIds[i].second;
		float currentDistance = distance(_vertices[id1], _vertices[id2]);
		float newRest = linearInterpolation(_distanceConstraintData.restValues[i], currentDistance, _plasticityFactor);
		vec3 delta = ((_vertices[id1] - _vertices[id2]) / currentDistance) * (newRest - currentDistance) / 2.0f;
		_distanceConstraintData.restValues[i] = newRest;
		_distanceDeltas[id1] += -delta;
		_distanceDeltas[id2] += +delta;
	});
	// apply deltas
	parallel_for((size_t)0, (size_t)(_vertices.size() - 1), (size_t)1, [=](size_t i) {
		_vertices[i] += _distanceDeltas[i] / (float)_distanceConstraintData.constraintCountPerVertex[i];
	});
	// update rest values
	parallel_for((size_t)0, (size_t)(_distanceConstraintData.constraintCount - 1), (size_t)1, [=](size_t i) {
		int id1 = _distanceConstraintData.vertexIds[i].first;
		int id2 = _distanceConstraintData.vertexIds[i].second;
		_distanceConstraintData.restValues[i] = distance(_vertices[id1], _vertices[id2]);
	});
}

void solveConstraints() {
	solveVolumeConstraints();
	solveDistanceConstraints();
}

void getCollisionResult(int colliderId) {
	auto startTime = chrono::high_resolution_clock::now();
	vec3 collPos = _collData.colliderPositions[colliderId];
	vec3 collSize = _collData.colliderSizes[colliderId];
	int collType = _collData.colliderTypes[colliderId];
	
	for (int i = 0; i < _iterationCount; i++) {
		projectVertices(collPos, collSize, collType);
		solveConstraints();
	}

	chrono::duration<float> duration = chrono::high_resolution_clock::now() - startTime;
	_solverDeltaTime = std::chrono::duration_cast<chrono::milliseconds>(duration).count();
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
}

void setTetMeshTransforms(float* translation, float* rotation) {
	_tetMeshPosition = vec3(translation[0], translation[1], translation[2]);
	_tetMeshRotation = vec3(rotation[0], rotation[1], rotation[2]);
}

void setIterationCount(int iterationCount) {
	_iterationCount = iterationCount;
}

void setPlasticity(float plasticity) {
	_plasticityFactor = plasticity;
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
	_distanceConstraintData.vertexIds.reserve((size_t)tetrahedra.size() * 6/4);
	_distanceConstraintData.restValues.reserve((size_t)tetrahedra.size() * 6/4);
	_volumeConstraintData.vertexIds.reserve((size_t)tetrahedra.size()/4);
	_volumeConstraintData.restValues.reserve((size_t)tetrahedra.size()/4);
	_volumeConstraintData.constraintCount = (int)tetrahedra.size()/4;
	vec3 tetVertices[4];
	int tetVertexIDs[4];
	int vertexID = 0; 
	
	//for each tetrahdedron
	for (int tetId = 0; tetId < tetrahedra.size()/4; tetId++) {
		//get 4 vertices and their ids
		for (int i = 0; i < 4; i++) {
			vertexID = tetrahedra[tetId * 4 + i];
			tetVertices[i] = _vertices[vertexID];
			tetVertexIDs[i] = vertexID;
		}
		// generate distance constraints along each tetrahedron edge
		_distanceConstraintData.addConstraint(pair<int, int>(tetVertexIDs[0], tetVertexIDs[1]), distance(tetVertices[0], tetVertices[1]));
		_distanceConstraintData.addConstraint(pair<int, int>(tetVertexIDs[0], tetVertexIDs[2]), distance(tetVertices[0], tetVertices[2]));
		_distanceConstraintData.addConstraint(pair<int, int>(tetVertexIDs[0], tetVertexIDs[3]), distance(tetVertices[0], tetVertices[3]));
		_distanceConstraintData.addConstraint(pair<int, int>(tetVertexIDs[1], tetVertexIDs[2]), distance(tetVertices[1], tetVertices[2]));
		_distanceConstraintData.addConstraint(pair<int, int>(tetVertexIDs[1], tetVertexIDs[3]), distance(tetVertices[1], tetVertices[3]));
		_distanceConstraintData.addConstraint(pair<int, int>(tetVertexIDs[2], tetVertexIDs[3]), distance(tetVertices[2], tetVertices[3]));
		// generate volume constraint per tetrahedron
		_volumeConstraintData.addConstraint(tuple<int, int, int, int>(tetVertexIDs[0], tetVertexIDs[1], tetVertexIDs[2], tetVertexIDs[3]), 
			getTetrahedronVolume(tetVertices[0], tetVertices[1], tetVertices[2], tetVertices[3]));
	}
	// resize data constrainers and set constraint count
	_distanceConstraintData.vertexIds.resize(_distanceConstraintData.vertexIds.size());
	_distanceConstraintData.restValues.resize(_distanceConstraintData.vertexIds.size());
	_distanceConstraintData.constraintCount = _distanceConstraintData.vertexIds.size();
}

void setTetMeshData(float* vertices, int vertexCount, int* tetrahedra, int tetCount, float* surfaceVertices, int surfaceVertCount, int* surfaceTriangles, int surfaceTriCount) {
	// vertices
	_vertices.reserve(vertexCount);
	vec3 temporaryVertex;
	for (int i = 0; i < vertexCount; i++) {
		temporaryVertex.x = vertices[i * 3];
		temporaryVertex.y = vertices[i * 3 + 1];
		temporaryVertex.z = vertices[i * 3 + 2];
		_vertices.push_back(temporaryVertex);
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
	_distanceConstraintData.constraintCountPerVertex.resize(vertexCount, 0);
	_distanceConstraintData.constraintsPerVertex.resize(vertexCount, vector<int>(0));
	_volumeConstraintData.constraintsPerVertex.resize(vertexCount, 0);
	// generate constraints
	generateConstraints(_tetrahedra);
	_distanceDeltas.resize(vertexCount, vec3());
	_volumeDeltas.resize(vertexCount, vec3());
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
	DLL_EXPORT void dll_setPlasticity(float plasticity) {
		setPlasticity(plasticity);
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
		return (int)_vertices.size();
	}
	DLL_EXPORT void dll_getVertices(int* vertexOutput) {
		vector<float> result;
		getVectorData(_vertices, result);
		memcpy(vertexOutput, result.data(), _vertices.size() * 3 * sizeof(float));
	}
	DLL_EXPORT int dll_getSurfVertexCount() {
		return (int)_surfaceVertexIndeces.size();
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
	DLL_EXPORT int dll_getDistanceConstraintCount() {
		return (int)_distanceConstraintData.constraintCount;
	}
	DLL_EXPORT int dll_getVolumeConstraintCount() {
		return (int)_volumeConstraintData.constraintCount;
	}
	DLL_EXPORT void dll_getConstraintRestValues(int* output) {
		memcpy(output, _distanceConstraintData.restValues.data(), (int)_distanceConstraintData.restValues.size() * sizeof(int));
	}
	DLL_EXPORT int dll_getCollisionCount() {
		return _collidingVertexCount;
	}
	DLL_EXPORT void dll_getTetMeshTransforms(int* translationOutput, int* rotationOutput) {
		memcpy(translationOutput, getVectorData(_tetMeshPosition), 3 * sizeof(float));
		memcpy(rotationOutput, getVectorData(_tetMeshRotation), 3 * sizeof(float));
	}
	DLL_EXPORT void dll_getDeltas(int* output) {
		memcpy(output, _distanceDeltas.data(), _distanceDeltas.size() * sizeof(float));
	}
	DLL_EXPORT int dll_getDeltasCount() {
		return (int)_distanceDeltas.size();
	}
	DLL_EXPORT int dll_getSolverDeltaTime() {
		return _solverDeltaTime;
	}
#pragma endregion Getters
#pragma region Calculations
	DLL_EXPORT void dll_getCollisionResult(int collId) {
		getCollisionResult(collId);
	}
	DLL_EXPORT void dll_project(int collId) {
		vec3 collPos = _collData.colliderPositions[collId];
		vec3 collSize = _collData.colliderSizes[collId];
		int collType = _collData.colliderTypes[collId];
		projectVertices(collPos, collSize, collType);
	}
	DLL_EXPORT void dll_solveConstraints() {
		solveConstraints();
	}
	DLL_EXPORT void dll_solveDistanceConstraints() {
		solveDistanceConstraints();
	}
	DLL_EXPORT void dll_solveVolumeConstraints() {
		solveVolumeConstraints();
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