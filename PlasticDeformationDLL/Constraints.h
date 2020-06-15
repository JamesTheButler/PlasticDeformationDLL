#pragma once

#include <set>
#include <tuple>
#include <vector>

#include <glm/gtx/transform.hpp>
#include <tbb/parallel_for.h>

#include "Misc.h"

using namespace std;
using namespace glm;
using namespace tbb;

class DistanceConstraintData {
public:
	static const int N = 21;

	struct DistanceConstraint {
		ivec2 vertex_ids;
		float rest_value;
	};

	DistanceConstraint all_constraints[N];

	ivec2 vertex_ids[N];
	float rest_value[N];




	vector<ivec2> vertexIds;
	vector<float> restValues;
	vector<int> constraintCountPerVertex;
	vector<vector<int>> constraintsPerVertex;		//needed to check for duplicates in generation

	size_t constraintCount;

	~DistanceConstraintData() {
		cleanUp();
	}

	void cleanUp() {
		vector<ivec2>().swap(vertexIds);
		vector<float>().swap(restValues);
		vector<int>().swap(constraintCountPerVertex);
		vector<vector<int>>().swap(constraintsPerVertex);
	}

	void setConstraintCount() {
		constraintCount = vertexIds.size();
		vertexIds.resize(constraintCount);
		restValues.resize(constraintCount);
	}

	ivec2 sortVertexIds(ivec2 vertexIds) {
		if (vertexIds.x > vertexIds.y) {
			int temp = vertexIds.y;
			vertexIds.y = vertexIds.x;
			vertexIds.x = temp;
		}
		return vertexIds;
	}

	bool doesConstraintExist(ivec2 constraint) {
		for (int constraintId : constraintsPerVertex[constraint.x]) {
			if (vertexIds[constraintId] == constraint)	// if exists: skip adding
				return true;
		}
		return false;
	}

	void addConstraint(ivec2 vertexIDs, float restValue) {
		// sort pair
		vertexIDs = sortVertexIds(vertexIDs);
		// add data, if constraint does not exist yet
		if (doesConstraintExist(vertexIDs))
			return;
		int constraintId = (int)vertexIds.size();
		vertexIds.push_back(vertexIDs);
		restValues.push_back(restValue);
		// setup vertex-constraint lookup & constraints per vertex
		constraintCountPerVertex[vertexIDs.x]++;
		constraintCountPerVertex[vertexIDs.y]++;
		constraintsPerVertex[vertexIDs.x].push_back(constraintId);
		constraintsPerVertex[vertexIDs.y].push_back(constraintId);
	}

	void addConstraint(int id, ivec2 vertexIDs, float restValue) {
		// sort pair
		vertexIDs = sortVertexIds(vertexIDs);
		// add data, if constraint does not exist yet
		if (doesConstraintExist(vertexIDs))
			return;
		int constraintId = (int)vertexIds.size();
		vertexIds[id] = vertexIDs;
		restValues[id] = restValue;
		// setup vertex-constraint lookup & constraints per vertex
		constraintCountPerVertex[vertexIDs.x]++;
		constraintCountPerVertex[vertexIDs.y]++;
		constraintsPerVertex[vertexIDs.x].push_back(constraintId);
		constraintsPerVertex[vertexIDs.y].push_back(constraintId);
	}

	// Returns flat array of the data of vertexIds.
	int* getVertexIdData() {
		vector<int> flat;
		flat.reserve(vertexIds.size() * 2);
		for (auto& v : vertexIds) {
			flat.push_back(v.x);
			flat.push_back(v.y);
		}
		return flat.data();
	}
};

class VolumeConstraintData {
public:
	vector<ivec4> vertexIds;
	vector<float> restValues;
	vector<int> constraintCountPerVertex;
	size_t constraintCount;

	~VolumeConstraintData() {
		cleanUp();
	}

	void cleanUp() {
		vector<ivec4>().swap(vertexIds);
		vector<float>().swap(restValues);
		vector<int>().swap(constraintCountPerVertex);
	}

	void addConstraint(int id, ivec4 vertexIDs, float restValue) {
		vertexIds[id] = vertexIDs;
		restValues[id] = restValue;
		constraintCountPerVertex[vertexIDs.x]++;
		constraintCountPerVertex[vertexIDs.y]++;
		constraintCountPerVertex[vertexIDs.z]++;
		constraintCountPerVertex[vertexIDs.w]++;
	}

	void addConstraint(ivec4 vertexIDs, float restValue) {
		vertexIds.push_back(vertexIDs);
		restValues.push_back(restValue);
		constraintCountPerVertex[vertexIDs.x]++;
		constraintCountPerVertex[vertexIDs.y]++;
		constraintCountPerVertex[vertexIDs.z]++;
		constraintCountPerVertex[vertexIDs.w]++;
	}

	// Returns flat array of the data of vertexIds.
	int* getVertexIdData() {
		vector<int> flat;
		flat.reserve(vertexIds.size() * 4);
		for (auto& v : vertexIds) {
			flat.push_back(v.x);
			flat.push_back(v.y);
			flat.push_back(v.z);
			flat.push_back(v.w);
		}
		return flat.data();
	}
};

void generateConstraints_S(
	const vector<vec3> &vertices,
	const vector<ivec4> &tetrahedra,
	DistanceConstraintData &distanceConstraintData,
	VolumeConstraintData &volumeConstraintData) {

	int vertexCount = vertices.size();
	// set up vertex-to-constrain look up
	distanceConstraintData.constraintCountPerVertex.resize(vertexCount, 0);
	distanceConstraintData.constraintsPerVertex.resize(vertexCount, vector<int>(0));
	volumeConstraintData.constraintCountPerVertex.resize(vertexCount, 0);

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
		distanceConstraintData.addConstraint(ivec2(tetVertexIDs[0], tetVertexIDs[1]), distance(vertex[0], vertex[1]));
		distanceConstraintData.addConstraint(ivec2(tetVertexIDs[0], tetVertexIDs[2]), distance(vertex[0], vertex[2]));
		distanceConstraintData.addConstraint(ivec2(tetVertexIDs[0], tetVertexIDs[3]), distance(vertex[0], vertex[3]));
		distanceConstraintData.addConstraint(ivec2(tetVertexIDs[1], tetVertexIDs[2]), distance(vertex[1], vertex[2]));
		distanceConstraintData.addConstraint(ivec2(tetVertexIDs[1], tetVertexIDs[3]), distance(vertex[1], vertex[3]));
		distanceConstraintData.addConstraint(ivec2(tetVertexIDs[2], tetVertexIDs[3]), distance(vertex[2], vertex[3]));
		// generate volume constraint per tetrahedron
		volumeConstraintData.addConstraint(ivec4(tetVertexIDs[0], tetVertexIDs[1], tetVertexIDs[2], tetVertexIDs[3]),
			geometry::getTetrahedronVolume(vertex[0], vertex[1], vertex[2], vertex[3]));
	}
	// resize data constrainers and set constraint count
	distanceConstraintData.vertexIds.resize(distanceConstraintData.vertexIds.size());
	distanceConstraintData.restValues.resize(distanceConstraintData.vertexIds.size());
	distanceConstraintData.constraintCount = distanceConstraintData.vertexIds.size();
}

// NOT FUNCTIONAL (id of distance constraint not known due to duplicate removal)
void generateConstraints_parallel(
	const vector<vec3> &vertices,
	const vector<ivec4> &tetrahedra,
	DistanceConstraintData &distanceConstraintData,
	VolumeConstraintData &volumeConstraintData) {
/*
	int vertexCount = vertices.size();
	// set up vertex-to-constrain look up
	distanceConstraintData.constraintCountPerVertex.resize(vertexCount, 0);
	distanceConstraintData.constraintsPerVertex.resize(vertexCount, vector<int>(0));
	volumeConstraintData.constraintCountPerVertex.resize(vertexCount, 0);

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
	parallel_for((size_t)0, tetCount, (size_t)1, [&](size_t &tetId) {
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
		distanceConstraintData.addConstraint(tetId * 6    , ivec2(tetVertexIDs[0], tetVertexIDs[1]), distance(vertex[0], vertex[1]));
		distanceConstraintData.addConstraint(tetId * 6 + 1, ivec2(tetVertexIDs[0], tetVertexIDs[2]), distance(vertex[0], vertex[2]));
		distanceConstraintData.addConstraint(tetId * 6 + 2, ivec2(tetVertexIDs[0], tetVertexIDs[3]), distance(vertex[0], vertex[3]));
		distanceConstraintData.addConstraint(tetId * 6 + 3, ivec2(tetVertexIDs[1], tetVertexIDs[2]), distance(vertex[1], vertex[2]));
		distanceConstraintData.addConstraint(tetId * 6 + 4, ivec2(tetVertexIDs[1], tetVertexIDs[3]), distance(vertex[1], vertex[3]));
		distanceConstraintData.addConstraint(tetId * 6 + 5, ivec2(tetVertexIDs[2], tetVertexIDs[3]), distance(vertex[2], vertex[3]));
		// generate volume constraint per tetrahedron
		volumeConstraintData.addConstraint(tetId, ivec4(tetVertexIDs[0], tetVertexIDs[1], tetVertexIDs[2], tetVertexIDs[3]),
			geometry::getTetrahedronVolume(vertex[0], vertex[1], vertex[2], vertex[3]));
	});
	// resize data constrainers and set constraint count
	distanceConstraintData.vertexIds.resize(distanceConstraintData.vertexIds.size());
	distanceConstraintData.restValues.resize(distanceConstraintData.vertexIds.size());
	distanceConstraintData.constraintCount = distanceConstraintData.vertexIds.size();
	*/
}