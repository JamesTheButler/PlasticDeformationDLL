#pragma once
#include <set>
#include <tuple>
#include <vector>
#include <glm/vec3.hpp>

using namespace std;
using namespace glm;

class DistanceConstraintData {
public:
	vector<pair<int, int>> vertexIds;
	vector<float> restValues;
	vector<int> constraintCountPerVertex;
	vector<vector<int>> constraintsPerVertex;
	size_t constraintCount;

	~DistanceConstraintData() {
		cleanUp();
	}

	void cleanUp() {
		vector<pair<int, int>>().swap(vertexIds);
		vector<float>().swap(restValues);
		vector<int>().swap(constraintCountPerVertex);
		vector<vector<int>>().swap(constraintsPerVertex);
	}

	void setConstraintCount() {
		constraintCount = vertexIds.size();
		vertexIds.resize(constraintCount);
		restValues.resize(constraintCount);
	}
	
	pair<int, int> sortVertexIds(pair<int, int> vertexIds) {
		if (vertexIds.first > vertexIds.second) {
			int temp = vertexIds.second;
			vertexIds.second = vertexIds.first;
			vertexIds.first = temp;
		}
		return vertexIds;
	}

	bool doesConstraintExist(pair<int, int> constraint) {
		for (int constraintId : constraintsPerVertex[constraint.first]) {
			if (vertexIds[constraintId] == constraint)	// if exists: skip adding
				return true;
		}
		return false;
	}

	void addConstraint(pair<int, int> vertexIDs, float restValue) {
		// sort pair
		vertexIDs = sortVertexIds(vertexIDs);
		// add data, if constraint does not exist yet
		if (doesConstraintExist(vertexIDs))
			return;
		int constraintId = (int)vertexIds.size();
		vertexIds.push_back(vertexIDs);
		restValues.push_back(restValue);
		// setup vertex-constraint lookup & constraints per vertex
		constraintCountPerVertex[vertexIDs.first]++;
		constraintCountPerVertex[vertexIDs.second]++;
		constraintsPerVertex[vertexIDs.first].push_back(constraintId);
		constraintsPerVertex[vertexIDs.second].push_back(constraintId);
	}
	
	// Returns flat array of the data of vertexIds.
	int* getVertexIdData() {
		vector<int> flat;
		flat.reserve(vertexIds.size()*2);
		for (auto& v : vertexIds) {
			flat.push_back(v.first);
			flat.push_back(v.second);
		}
		return flat.data();
	}
};