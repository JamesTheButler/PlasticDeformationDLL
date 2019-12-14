#pragma once
#include <set>
#include <tuple>

class VolumeConstraintData {
public:
	vector<tuple<int, int, int, int>> vertexIds;
	vector<float> restValues;
	vector<int> constraintsPerVertex;
	size_t constraintCount;

	~VolumeConstraintData() {
		cleanUp();
	}

	void cleanUp() {
		vector<tuple<int, int, int, int>>().swap(vertexIds);
		vector<float>().swap(restValues);
		vector<int>().swap(constraintsPerVertex);
	}

	void addConstraint(tuple<int, int, int, int> vertexIDs, float restValue) {
		vertexIds.push_back(vertexIDs);
		restValues.push_back(restValue);
		constraintsPerVertex[get<0>(vertexIDs)]++;
		constraintsPerVertex[get<1>(vertexIDs)]++;
		constraintsPerVertex[get<2>(vertexIDs)]++;
		constraintsPerVertex[get<3>(vertexIDs)]++;
	}

	// Returns flat array of the data of vertexIds.
	int* getVertexIdData() {
		vector<int> flat;
		flat.reserve(vertexIds.size() * 4);
		for (auto& v : vertexIds) {
			flat.push_back(get<0>(v));
			flat.push_back(get<1>(v));
			flat.push_back(get<2>(v));
			flat.push_back(get<3>(v));
		}
		return flat.data();
	}
};