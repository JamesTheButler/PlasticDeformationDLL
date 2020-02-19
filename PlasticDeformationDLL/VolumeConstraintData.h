#pragma once
#include <set>
#include <tuple>
#include <vector>

using namespace std;

class VolumeConstraintData {
public:
	vector<ivec4> vertexIds;
	vector<float> restValues;
	vector<int> constraintsPerVertex;
	size_t constraintCount;

	~VolumeConstraintData() {
		cleanUp();
	}

	void cleanUp() {
		vector<ivec4>().swap(vertexIds);
		vector<float>().swap(restValues);
		vector<int>().swap(constraintsPerVertex);
	}

	void addConstraint(ivec4 vertexIDs, float restValue) {
		vertexIds.push_back(vertexIDs);
		restValues.push_back(restValue);
		constraintsPerVertex[vertexIDs.x]++;
		constraintsPerVertex[vertexIDs.y]++;
		constraintsPerVertex[vertexIDs.z]++;
		constraintsPerVertex[vertexIDs.w]++;
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