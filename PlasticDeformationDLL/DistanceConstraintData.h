#pragma once
#include <set>
#include <tuple>
#include <vector>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

using namespace std;
using namespace glm;

class DistanceConstraintData {
public:
	vector<ivec2> vertexIds;
	vector<float> restValues;
	vector<int> constraintCountPerVertex;
	vector<vector<int>> constraintsPerVertex;
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
	
	// Returns flat array of the data of vertexIds.
	int* getVertexIdData() {
		vector<int> flat;
		flat.reserve(vertexIds.size()*2);
		for (auto& v : vertexIds) {
			flat.push_back(v.x);
			flat.push_back(v.y);
		}
		return flat.data();
	}
};