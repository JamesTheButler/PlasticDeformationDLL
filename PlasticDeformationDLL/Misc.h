#pragma once
#include <vector>
#include <glm/gtx/transform.hpp>
#include <tbb/parallel_for.h>
#include "Logger.h"

using namespace std;
using namespace glm;
using namespace tbb;

namespace vectorFuncs {
	// Gets a vec3 vectors data as a concurrent float array [x1, y1, z1, x2, y2, z2, ...].
	void getVectorData(const vector<vec3> &vectors, vector<float> &result) {
		result.reserve(vectors.size() * 3);
		for (int i = 0; i < vectors.size(); i++) {
			result.push_back(vectors[i].x);
			result.push_back(vectors[i].y);
			result.push_back(vectors[i].z);
		}
	}

	// Gets a vec4 vectors data as a concurrent float array.
	void getVectorData(const vector<vec4> &vectors, vector<float> &result) {
		result.reserve(vectors.size() * 4);
		for (int i = 0; i < vectors.size(); i++) {
			result.push_back(vectors[i].x);
			result.push_back(vectors[i].y);
			result.push_back(vectors[i].z);
			result.push_back(vectors[i].w);
		}
	}

	// Gets a vec3 vectors data as a concurrent float array.
	void getVectorData(const vector<ivec3> &vectors, vector<int> &result) {
		result.reserve(vectors.size() * 3);
		for (int i = 0; i < vectors.size(); i++) {
			result.push_back(vectors[i].x);
			result.push_back(vectors[i].y);
			result.push_back(vectors[i].z);
		}
	}

	// Gets a vec4 vectors data as a concurrent uint array.
	void getVectorData(const vector<ivec4> &vectors, vector<int> &result) {
		result.reserve(vectors.size() * 4);
		for (int i = 0; i < vectors.size(); i++) {
			result.push_back((int)vectors[i].x);
			result.push_back((int)vectors[i].y);
			result.push_back((int)vectors[i].z);
			result.push_back((int)vectors[i].w);
		}
	}

	// Gets a vec3's data as a concurrent float array [x, y, z].
	float* getVectorData(const vec3 &vector) {
		float result[3]{ vector.x, vector.y, vector.z };
		return result;
	}
	
	string charPtrToString (const char* chars, int count) {
		string s = "";
		for (int i = 0; i < count; i++) {
			s += chars[i];
		}
		return s;
	}

	// Finds the ID of a vec3 'v' in a list of vectors 'set'. Allowed difference is specified by 'delta'. If v does not exist in 'set', returns -1.
	int findIdOfSimilar(const vec3& v, const vector<vec3>& set, float delta) {
		int id = -1;
		parallel_for((size_t)0, set.size(), [&](size_t &i) {
			if ((abs(set[i].x - v.x) < delta) && (abs(set[i].y - v.y) < delta) && (abs(set[i].z - v.z) < delta)) {
				id = i;
				task::self().cancel_group_execution();
			}
		});
		return id;
	}

	// For each sub set vec3, the function finds the id of the identical vec3 in the super set. If no identical vector3 exists, the id will be set to -1.
	void indexSubsetVertices(const vector<vec3> &subSet, vector<vec3>& superSet, vector<int>& indeces) {
		logger::log("--indexSubsetVertices");
		logger::log("\t -subset count:"+ to_string(subSet.size())+",\t superSet count: "+ to_string(superSet.size()));
		indeces.resize(subSet.size(), -1);
		parallel_for((size_t)0, subSet.size(), [&](size_t &i) {
			indeces[i] = findIdOfSimilar(subSet[i], superSet, 0.01f);
		});
	}

	// Generates a vertex vectors by using a vertex vector and a map (generated by indexSupersetVertices())
	void mapVertices(const vector<int>& map, const vector<vec3>& superSet, vector<vec3>& mappedVerts) {
		mappedVerts.resize(map.size(), vec3(0,0,0));
		parallel_for((size_t)0, map.size(), [&](size_t &i) {
			mappedVerts[i] = superSet[map[i]];
		});
	}
}

namespace geometry{
	// Reverts the rotation of a vector around vec3.null according to unity's rotation scheme.
	vec3 revertRotation(const vec3 &vector, const vec3 &eulerAngles) {
		mat4 xRot = glm::rotate(eulerAngles[0] * pi<float>() / 180.0f, vec3(1, 0, 0));
		mat4 yRot = glm::rotate(eulerAngles[1] * pi<float>() / 180.0f, vec3(0, 1, 0));
		mat4 zRot = glm::rotate(eulerAngles[2] * pi<float>() / 180.0f, vec3(0, 0, 1));
		return vec3(inverse(yRot*xRot*zRot)*vec4(vector, 0));
	}

	// Rotate a vector around vec3.null according to unity's rotation scheme.
	vec3 rotate(const vec3 &vector, const vec3 &eulerAngles) {
		mat4 xRot = glm::rotate(eulerAngles[0] * pi<float>() / 180.0f, vec3(1, 0, 0));
		mat4 yRot = glm::rotate(eulerAngles[1] * pi<float>() / 180.0f, vec3(0, 1, 0));
		mat4 zRot = glm::rotate(eulerAngles[2] * pi<float>() / 180.0f, vec3(0, 0, 1));
		return vec3(yRot*xRot*zRot*vec4(vector, 0));
	}

	// Returns volume of tetrahedron specified by the four corners p1-p4.
	float getTetrahedronVolume(const vec3 &p1, const vec3 &p2, const vec3 &p3, const vec3 &p4) {
		return abs(dot(cross(p2 - p1, p3 - p1), p4 - p1) / 6.0f);
	}
}

namespace misc {
	// Linear interpolation between val1 and val2.
	float lerp(const float val1, const float val2, float t) {
		t = clamp(t, 0.0f, 1.0f);
		return (1.0f - t) * val1 + t * val2;
	}
}