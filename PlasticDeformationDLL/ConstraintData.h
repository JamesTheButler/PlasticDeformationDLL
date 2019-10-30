#pragma once
enum EConstraintType {
	DEFAULT = -1,
	DISTANCE = 0,
	BENDING = 1,
	VOLUME = 2
};

class ConstraintData {
public:
	int constraintCount;
	vector<vector<int>> vertexIds;
	vector<float> currentValues;
	vector<float> restValues;
	vector<EConstraintType> constraintTypes;
	vector<vector<int>> constraintsPerVertex;

	int getConstraintCount() {
		return constraintTypes.size();
	}

	void setConstraintCount(int newConstraintCount) {
		constraintCount = newConstraintCount;
		vertexIds.reserve(constraintCount);
		currentValues.reserve(constraintCount);
		restValues.reserve(constraintCount);
		constraintTypes.reserve(constraintCount);
	}

	void addConstraint(vector<int> vertIDs, float restValue, EConstraintType constraintType) {
		int constraintID = vertIDs.size();
		vertexIds.push_back(vertIDs);
		currentValues.push_back(restValue);
		restValues.push_back(restValue);
		constraintTypes.push_back(constraintType);
		for (int i = 0; i < vertIDs.size(); i++) {
			constraintsPerVertex.at(vertIDs[i]).push_back(constraintID);
		}
	}

	// Returns total count of vertexIds.
	int getVertexIdSize() {
		int flatSize = 0;
		for (auto& v : vertexIds) {
			flatSize += v.size();
		}
		return flatSize;
	}

	// Returns flat array of the data of vertexIds.
	int* getVertexIdData() {
		vector<int> flat;
		flat.reserve(getVertexIdSize());
		for (auto& v : vertexIds)
			for (auto& elem : v)
				flat.push_back(elem);
		return flat.data();
	}
};