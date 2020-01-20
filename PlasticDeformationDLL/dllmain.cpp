#include <cmath>
#include <cstring>
#include <vector>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <fstream>
using namespace std;

#include <glm/gtx/transform.hpp>
using namespace glm;

#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/blocked_range.h>
using namespace tbb;

//#include "Profiler/profile.h"
#include "Logger.h"

#include "AABox.h"
#include "Intersection.h"
#include "Misc.h"

#include "DistanceConstraintData.h"
#include "VolumeConstraintData.h"
#include "ColliderData.h"
#include "BarycentricMapping.h"
#include "FileReader.h"
#include "FileWriter.h"

#define DLL_EXPORT __declspec(dllexport)

// tet mesh transforms
vec3 _tetMeshPosition = vec3();
vec3 _tetMeshRotation = vec3();

//vertex data
vector<vec3> _vertices;
vector<ivec4> _tetrahedra;
vector<vec3> _surfaceVertices;	// vertices of original model
vector<int> _surfaceVertexToTetVertexMap;
vector<vec4> _barycentricCoordinates;
vector<int> _barycentricTetIds;

DistanceConstraintData _distanceConstraintData;
VolumeConstraintData _volumeConstraintData;
vector<vec3> _distanceDeltas;
vector<vec3> _volumeDeltas;
ColliderData _collData;

string _projectPath;
string _tetrahedralizationPath;
string _logPath;
string _fileName;

float _solverDeltaTime = 0;

int _collidingVertexCount = 0;
int _iterationCount = 0;
float _plasticityFactor = 0.0f;

int _debugInt = 0;
float _debugFloat = 0.0f;

void writeProfilerToFile() {
	/*std::ofstream op(_filePath + _fileName + ".profile");
	realm::GetProfiler()->GetRootNode()->DisplayFlatStats(op);*/
}

// TODO: clean up correctly
// TODO: optimize (pass-by-reference, const (if unchanged))
void teardown() {
	_tetMeshPosition = vec3();
	_tetMeshRotation = vec3();
	_collData = ColliderData();
	_distanceConstraintData = DistanceConstraintData();
	_volumeConstraintData = VolumeConstraintData();
	_collidingVertexCount = 0;

	vector<vec3>().swap(_vertices);
	vector<ivec4>().swap(_tetrahedra);
	vector<vec3>().swap(_surfaceVertices);
	vector<int>().swap(_surfaceVertexToTetVertexMap);
	vector<vec4>().swap(_barycentricCoordinates);
	vector<int>().swap(_barycentricTetIds);
	vector<vec3>().swap(_distanceDeltas);
	vector<vec3>().swap(_volumeDeltas);
}

void loadTetMeshData(string fileName) {

}

#pragma region solver
bool doesCollide(const vec3& vertex, const vec3& colliderPos, const vec3& colliderSize, const int colliderType) {
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
// TODO: Ootimize (pass aabox by const reference)
vec3 projectOrthogonal(const vec3& vertex, const AABox& box) {
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
vec3 projectOrthogonal(const vec3& vertex, const vec3& collPos, const vec3& collSize, const int collType) {
	switch (collType) {
	default:
		AABox aabox = AABox(collPos, collSize);
		return projectOrthogonal(vertex, aabox);
	}
}

void projectVertices(const vec3& collPos, const vec3& collSize, const int collType) {
	_collidingVertexCount = 0;
	parallel_for((size_t)0, (size_t)(_vertices.size()-1), (size_t)1, [=](size_t i) {
		vec3 vertex = geometry::rotate(_vertices.data()[i], _tetMeshRotation) + _tetMeshPosition;
		if (doesCollide(vertex, collPos, collSize, collType)) {
			_collidingVertexCount++;
			vertex = projectOrthogonal(vertex, collPos, collSize, collType);
			// transform it back into local space
			_vertices[i] = geometry::revertRotation(vertex - _tetMeshPosition, _tetMeshRotation);
		}
	});
}

void solveVolumeConstraints() {
	//clear deltas
	vector<vec3>(_vertices.size(), vec3(0, 0, 0)).swap(_volumeDeltas);
	//calc deltas
	parallel_for((size_t)0, _volumeConstraintData.constraintCount - 1, (size_t)1, [=](size_t i) {
		// get actual vertices
		vec3 verts[4];
		verts[0] = _vertices[get<0>(_volumeConstraintData.vertexIds[i])];
		verts[1] = _vertices[get<1>(_volumeConstraintData.vertexIds[i])];
		verts[2] = _vertices[get<2>(_volumeConstraintData.vertexIds[i])]; 
		verts[3] = _vertices[get<3>(_volumeConstraintData.vertexIds[i])];

		float volumeDifference = _volumeConstraintData.restValues[i] - geometry::getTetrahedronVolume(verts[0], verts[1], verts[2], verts[3]);
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
	parallel_for((size_t)0, _volumeConstraintData.constraintCount - 1, (size_t)1, [=](size_t i) {
		vec3 verts[4];
		verts[0] = _vertices[get<0>(_volumeConstraintData.vertexIds[i])];
		verts[1] = _vertices[get<1>(_volumeConstraintData.vertexIds[i])];
		verts[2] = _vertices[get<2>(_volumeConstraintData.vertexIds[i])];
		verts[3] = _vertices[get<3>(_volumeConstraintData.vertexIds[i])];
		_volumeConstraintData.restValues[i] = misc::lerp(geometry::getTetrahedronVolume(verts[0], verts[1], verts[2], verts[3]), _volumeConstraintData.restValues[i], _plasticityFactor);
	});
}

void solveDistanceConstraints() {
	// clear deltas
	vector<vec3>(_vertices.size(), vec3(0, 0, 0)).swap(_distanceDeltas);
	//calc deltas
	parallel_for((size_t)0, _distanceConstraintData.constraintCount - 1, (size_t)1, [=](size_t i) {
		int id1 = _distanceConstraintData.vertexIds[i].first;
		int id2 = _distanceConstraintData.vertexIds[i].second;
		float currentDistance = distance(_vertices[id1], _vertices[id2]);
		vec3 delta = ((_vertices[id1] - _vertices[id2]) / currentDistance) * (_distanceConstraintData.restValues[i] - currentDistance) / 2.0f;
		_distanceDeltas[id1] += -delta;
		_distanceDeltas[id2] += +delta;
	});
	// apply deltas
	parallel_for((size_t)0, (size_t)(_vertices.size() - 1), (size_t)1, [=](size_t i) {
		_vertices[i] += _distanceDeltas[i] / (float)_distanceConstraintData.constraintCountPerVertex[i];
	});
	// update rest values
	parallel_for((size_t)0, _distanceConstraintData.constraintCount - 1, (size_t)1, [=](size_t i) {
		int id1 = _distanceConstraintData.vertexIds[i].first;
		int id2 = _distanceConstraintData.vertexIds[i].second;
		_distanceConstraintData.restValues[i] = misc::lerp(distance(_vertices[id1], _vertices[id2]), _distanceConstraintData.restValues[i], _plasticityFactor);
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

//TODO: parallelize
void generateConstraints(
	const vector<vec3> &vertices,
	const vector<ivec4> &tetrahedra, 
	DistanceConstraintData &distanceConstraintData, 
	VolumeConstraintData &volumeConstraintData) {

	logger::log("--generateConstraints");
	int vertexCount = vertices.size();
	// set up vertex-to-constrain look up
	distanceConstraintData.constraintCountPerVertex.resize(vertexCount, 0);
	distanceConstraintData.constraintsPerVertex.resize(vertexCount, vector<int>(0));
	volumeConstraintData.constraintsPerVertex.resize(vertexCount, 0);

	// generate constraints
	size_t tetCount = tetrahedra.size();
	distanceConstraintData.vertexIds.reserve(tetCount * 6);
	distanceConstraintData.restValues.reserve(tetCount * 6);
	volumeConstraintData.vertexIds.reserve(tetCount);
	volumeConstraintData.restValues.reserve(tetCount);
	volumeConstraintData.constraintCount = tetCount;
	vec3 vertex[4];
	int tetVertexIDs[4];
	//for each tetrahdedron
	//parallel_for((size_t)0, tetCount, (size_t)1, [&](size_t &tetId) {
	for (int tetId = 0; tetId < tetCount; tetId++) {
		//get 4 vertices and their ids
		ivec4 tet = tetrahedra[tetId];
		vertex[0] = vertices[tet[0]];
		vertex[1] = vertices[tet[1]];
		vertex[2] = vertices[tet[2]];
		vertex[3] = vertices[tet[3]];
		tetVertexIDs[0] = tet[0];
		tetVertexIDs[1] = tet[1];
		tetVertexIDs[2] = tet[2];
		tetVertexIDs[3] = tet[3];		
		// generate distance constraints along each tetrahedron edge
		distanceConstraintData.addConstraint(pair<int, int>(tetVertexIDs[0], tetVertexIDs[1]), distance(vertex[0], vertex[1]));
		distanceConstraintData.addConstraint(pair<int, int>(tetVertexIDs[0], tetVertexIDs[2]), distance(vertex[0], vertex[2]));
		distanceConstraintData.addConstraint(pair<int, int>(tetVertexIDs[0], tetVertexIDs[3]), distance(vertex[0], vertex[3]));
		distanceConstraintData.addConstraint(pair<int, int>(tetVertexIDs[1], tetVertexIDs[2]), distance(vertex[1], vertex[2]));
		distanceConstraintData.addConstraint(pair<int, int>(tetVertexIDs[1], tetVertexIDs[3]), distance(vertex[1], vertex[3]));
		distanceConstraintData.addConstraint(pair<int, int>(tetVertexIDs[2], tetVertexIDs[3]), distance(vertex[2], vertex[3]));
		// generate volume constraint per tetrahedron
		volumeConstraintData.addConstraint(tuple<int, int, int, int>(tetVertexIDs[0], tetVertexIDs[1], tetVertexIDs[2], tetVertexIDs[3]),
			geometry::getTetrahedronVolume(vertex[0], vertex[1], vertex[2], vertex[3]));
	}//);
	// resize data constrainers and set constraint count
	distanceConstraintData.vertexIds.resize(distanceConstraintData.vertexIds.size());
	distanceConstraintData.restValues.resize(distanceConstraintData.vertexIds.size());
	distanceConstraintData.constraintCount = distanceConstraintData.vertexIds.size();
	
	logger::log("\t -D-constraints:" + to_string(_distanceConstraintData.constraintCount) + ",\t V-constraints: " + to_string(_volumeConstraintData.constraintCount));
}

void setSurfaceVertices(float* surfaceVertices, int surfaceVertCount) {
	logger::log("--setSurfaceVertices");
	logger::log("\t -surface verts:" + to_string(surfaceVertCount));
	// surface Vertices
	_surfaceVertices.resize(surfaceVertCount, vec3(0,0,0));
	parallel_for((size_t)0, (size_t)surfaceVertCount, (size_t)1, [&](size_t &i) {
		vec3 vertex;
		vertex.x = surfaceVertices[i * 3];
		vertex.y = surfaceVertices[i * 3 + 1];
		vertex.z = surfaceVertices[i * 3 + 2];
		_surfaceVertices[i] = vertex;
	});
}

//TODO: old -> remove
/* void setTetMeshData(
	float* vertices,
	int vertexCount,
	int* tetrahedra,
	int tetCount) {

	logger::log("--setTetMeshData ");
	logger::log("\t -verts:"+to_string(vertexCount)+", \t tets:" +to_string(tetCount));
	// vertices
	_vertices.resize(vertexCount, vec3(0,0,0));
	parallel_for((size_t)0, (size_t)vertexCount, (size_t)1, [&](size_t i) {
		vec3 temporaryVertex;
		temporaryVertex.x = vertices[i * 3];
		temporaryVertex.y = vertices[i * 3 + 1];
		temporaryVertex.z = vertices[i * 3 + 2];
		_vertices[i] = temporaryVertex;
	});
	// tetrahedra
	_tetrahedra.resize(tetCount, ivec4());
	parallel_for((size_t)0, (size_t)tetCount, (size_t)1, [&](size_t i) {
		ivec4 tet;
		tet[0] = tetrahedra[i * 4];
		tet[1] = tetrahedra[i * 4 + 1];
		tet[2] = tetrahedra[i * 4 + 2];
		tet[3] = tetrahedra[i * 4 + 3];
		_tetrahedra[i] = tet;
	});
}*/

bool init() {
	logger::log("--init");
	// generate file paths
	string tetMeshFilePath = _tetrahedralizationPath + _fileName + ".obj.mesh";
	string surfaceFilePath = _tetrahedralizationPath + _fileName + ".tetMesh";
	logger::log("\t-reading files.. "+tetMeshFilePath + " " + surfaceFilePath);
	// read tet mesh file
	logger::log("\t-parsing tet mesh data file");
	if (!fileReader::fileExists(tetMeshFilePath)) {
		logger::logError("\t\t" + tetMeshFilePath + " does not exist");
		return false;
	}
	fileReader::parseFile_obj_mesh(tetMeshFilePath, _vertices, _tetrahedra);
	//generate constraints
	generateConstraints(
		_vertices,
		_tetrahedra,
		_distanceConstraintData,
		_volumeConstraintData);
	_distanceDeltas.resize(_vertices.size(), vec3());
	_volumeDeltas.resize(_vertices.size(), vec3());
	logger::log("\t-constraints generated.. ");
	logger::log("\t-parsing surface mesh data file");
	// Read / generate surface data file
	if (fileReader::fileExists(surfaceFilePath)) {		// read surface mapping file
		logger::log("\t\t surface data file exists parsed");
		fileReader::parseFile_tetmesh(
			surfaceFilePath,
			_surfaceVertexToTetVertexMap,
			_barycentricCoordinates,
			_barycentricTetIds);
		logger::log("\t\t" + surfaceFilePath + " parsed");
	}
	else {
		logger::log("\t\t" + surfaceFilePath + " does not exist");
		// find tetmesh vertices that map directly to surface vertices
		vectorFuncs::indexSubsetVertices(_surfaceVertices, _vertices, _surfaceVertexToTetVertexMap);
		// generate barycentric mapping for other surfaceVertices
		bcmapping::findBaryCentricCoordinatedForVerticesWithMapping(
			_surfaceVertices,
			_vertices,
			_tetrahedra,
			_surfaceVertexToTetVertexMap,
			_barycentricCoordinates,
			_barycentricTetIds);
		fileWriter::writeTetMeshDataToFile(surfaceFilePath, _surfaceVertexToTetVertexMap, _barycentricCoordinates, _barycentricTetIds);
		logger::log("\t\t" + surfaceFilePath + " generated");
	}
	return true;
}
#pragma endregion Setters

// EXPORT FUNCTIONS
extern "C" {
#pragma region Setters
	DLL_EXPORT void dll_setIterationCount(int iterationCount) {
		setIterationCount(iterationCount);
	}
	DLL_EXPORT void dll_setPlasticity(float plasticity) {
		setPlasticity(plasticity);
	}
	DLL_EXPORT void dll_setSurfaceVertices(float* surfaceVertices, int surfaceVertCount) 		{
		setSurfaceVertices(surfaceVertices, surfaceVertCount);
	}
	//TODO: old -> remove
	/*DLL_EXPORT void dll_setTetMeshData(float* vertices, int vertexCount, int* tetrahedra, int tetCount) {
		setTetMeshData(vertices, vertexCount, tetrahedra, tetCount);
	}*/
	DLL_EXPORT void dll_setColliders(float* colliderPositions, float* colliderSizes, int* colliderTypes, int colliderCount) {
		setColliders(colliderPositions, colliderSizes, colliderTypes, colliderCount);
	}
	DLL_EXPORT void dll_setTetMeshTransforms(float* translation, float* rotation) {
		setTetMeshTransforms(translation, rotation);
	}
	DLL_EXPORT void dll_setFileName(const char* name, int charCount) {
		//_fileName = name;
		_fileName = vectorFuncs::charPtrToString(name, charCount);
	}
	DLL_EXPORT void dll_setFilePath(const char* path, int charCount) {
		//_filePath = path;
		_projectPath = vectorFuncs::charPtrToString(path, charCount);
		_tetrahedralizationPath = _projectPath + "Tetrahedralization/";
		_logPath = _projectPath + "Logs/";
		logger::setFilePath(_logPath);
	}
#pragma endregion Setters
#pragma region Getters
	DLL_EXPORT int dll_getVertexCount() {
		return (int)_vertices.size();
	}
	DLL_EXPORT void dll_getVertices(int* vertexOutput) {
		vector<float> result;
		vectorFuncs::getVectorData(_vertices, result);
		memcpy(vertexOutput, result.data(), _vertices.size() * 3 * sizeof(float));
	}
	DLL_EXPORT int dll_getTetrahedronCount() {
		return (int)_vertices.size();
	}
	DLL_EXPORT void dll_getTetrahedra(int* tetOutput) {
		vector<int> result;
		vectorFuncs::getVectorData(_tetrahedra, result);
		memcpy(tetOutput, result.data(), _tetrahedra.size() * 4 * sizeof(int));
	}
	DLL_EXPORT int dll_getSurfaceVertexCount() {
		return (int)_surfaceVertices.size();
	}
	DLL_EXPORT void dll_getSurfaceVertices(int* output) {
		// get vertex positions for barycentric mapping
		bcmapping::updateSurfaceVerticesWithMapping(_surfaceVertices, _vertices, _tetrahedra,_surfaceVertexToTetVertexMap, _barycentricCoordinates, _barycentricTetIds);
		//updateSurfaceVertices(_surfaceVertices, _vertices, _tetrahedra, _barycentricCoordinates, _barycentricTetIds);
		// get float list of vector data
		vector<float> result;
		vectorFuncs::getVectorData(_surfaceVertices, result);
		memcpy(output, result.data(), _surfaceVertices.size() * 3 * sizeof(float));
	}
	DLL_EXPORT void dll_getBarycentricCoords(int* barycentricCoordOutput) {
		vector<float> baryCoordResult;
		vectorFuncs::getVectorData(_barycentricCoordinates, baryCoordResult);
		memcpy(barycentricCoordOutput, baryCoordResult.data(), _barycentricCoordinates.size() * 4 * sizeof(float));
	}
	DLL_EXPORT int dll_getBarycentricCoordCount() {
		return (int)_barycentricCoordinates.size();
	}
	DLL_EXPORT void dll_getBarycentricTetIds(int* barycentricTetIdOutput) {
		memcpy(barycentricTetIdOutput, _barycentricTetIds.data(), _barycentricTetIds.size() * 4 * sizeof(int));
	}
	DLL_EXPORT int dll_getBarycentricTetIdCount() {
		return (int)_barycentricTetIds.size();
	}
	DLL_EXPORT void dll_getColliders(int* positionOutput, int* sizeOutput, int* typeOutput) {
		vector<float> posResult;
		vectorFuncs::getVectorData(_collData.colliderPositions, posResult);
		memcpy(positionOutput, posResult.data(), _collData.colliderCount * 3 * sizeof(float));

		vector<float> sizeResult;
		vectorFuncs::getVectorData(_collData.colliderSizes, sizeResult);
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
		memcpy(translationOutput, vectorFuncs::getVectorData(_tetMeshPosition), 3 * sizeof(float));
		memcpy(rotationOutput, vectorFuncs::getVectorData(_tetMeshRotation), 3 * sizeof(float));
	}
	DLL_EXPORT void dll_getDeltas(int* output) {
		memcpy(output, _distanceDeltas.data(), _distanceDeltas.size() * sizeof(float));
	}
	DLL_EXPORT int dll_getDeltasCount() {
		return (int)_distanceDeltas.size();
	}
	DLL_EXPORT float dll_getSolverDeltaTime() {
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
	DLL_EXPORT void dll_toggleLoggingOn() {
		logger::setActive(true);
	}
	DLL_EXPORT void dll_toggleLoggingOff() {
		logger::setActive(false);
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
		memcpy(outputRotated, vectorFuncs::getVectorData(geometry::rotate(v, r)), 3 * sizeof(float));
		memcpy(outputUnrotated, vectorFuncs::getVectorData(geometry::revertRotation(geometry::revertRotation(v, r), r)), 3 * sizeof(float));
		memcpy(rotationOutput, vectorFuncs::getVectorData(r), 3 * sizeof(float));
	}
	DLL_EXPORT bool dll_testVertexAABoxIntersection(float* vertex, float* cPos, float* cSize) {
		return intersect(AABox(vec3(cPos[0], cPos[1], cPos[2]), vec3(cSize[0], cSize[1], cSize[2])), vec3(vertex[0], vertex[1], vertex[2]));
	}
#pragma endregion tests
}