#pragma once
#include <vector>
#include <glm/vec3.hpp>

using namespace std;
using namespace glm;

struct ColliderData {
	vector<vec3> colliderPositions;
	vector<vec3> colliderSizes;
	vector<int> colliderTypes;
	int colliderCount;

	void setColliderCount(int count) {
		colliderCount = colliderCount;
		colliderPositions.reserve(colliderCount);
		colliderSizes.reserve(colliderCount);
		colliderTypes.reserve(colliderCount);
	}
};