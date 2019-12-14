#pragma once

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

// Gets a vec3 vectors data as a concurrent float array [x1, y1, z1, x2, y2, z2, ...].
void getVectorData(const vector<vec3> &vectors, vector<float> &result) {
	result.reserve(vectors.size() * 3);
	for (int i = 0; i < vectors.size(); i++) {
		result.push_back(vectors[i].x);
		result.push_back(vectors[i].y);
		result.push_back(vectors[i].z);
	}
}

// Gets a vec3's data as a concurrent float array [x, y, z].
float* getVectorData(const vec3 &vector) {
	float result[3]{ vector.x, vector.y, vector.z };
	return result;
}

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

float getTetrahedronVolume(const vec3 &p1, const vec3 &p2, const vec3 &p3, const vec3 &p4) {
	return abs(dot(cross(p2 - p1, p3 - p1), p4 - p1) / 6.0f);
}

// Linear interpolation between val1 and val2.
float lerp(const float val1, const float val2, const float t) {
	return t * val1 + (1 - t) * val2;
}
