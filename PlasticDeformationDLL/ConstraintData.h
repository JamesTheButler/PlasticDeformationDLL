#pragma once
enum EConstraintType {
	DEFAULT = -1,
	DISTANCE = 0,
	BENDING = 1,
	VOLUME = 2
};
//TODO: remove constraint count and current values
class ConstraintData {
public:
	vector<vector<int>> vertexIds;
	vector<float> restValues;
	vector<EConstraintType> constraintTypes;
	vector<vector<int>> constraintsPerVertexOld;
	vector<int> constraintsPerVertex;

	int getConstraintCount() {
		return (int)constraintTypes.size();
	}

	void setConstraintCount(int newConstraintCount) {
		vertexIds.reserve(newConstraintCount);
		restValues.reserve(newConstraintCount);
		constraintTypes.reserve(newConstraintCount);
	}

	void addConstraint(vector<int> vertIDs, float restValue, EConstraintType constraintType) {
		//sort for faster searching
		sort(vertIDs.begin(), vertIDs.end());
		// check if constraint exists already
		for (int constraintId : constraintsPerVertexOld[vertIDs[0]]) {
			if (vertexIds[constraintId] == vertIDs)	// if exists: skip adding
				return;
		}
		// add constraint
		constraintsPerVertex[vertIDs[0]]++;
		constraintsPerVertex[vertIDs[1]]++;
		int constraintID = (int)vertexIds.size();
		vertexIds.push_back(vertIDs);
		restValues.push_back(restValue);
		constraintTypes.push_back(constraintType);
		for (int i = 0; i < vertIDs.size(); i++) {
			constraintsPerVertexOld[vertIDs[i]].push_back(constraintID);
		}
	}

	// Returns total count of vertexIds.
	int getVertexIdSize() {
		int flatSize = 0;
		for (auto& v : vertexIds) {
			flatSize += (int)v.size();
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