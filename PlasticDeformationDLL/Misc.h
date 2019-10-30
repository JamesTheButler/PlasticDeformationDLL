#pragma once
// Gets a vec3 vectors data as a concurrent float array [x1, y1, z1, x2, y2, z2, ...].
void getVectorData(vector<vec3> vectors, vector<float> &result) {
	result.reserve(vectors.size() * 3);
	for (int i = 0; i < vectors.size(); i++) {
		result.push_back(vectors[i].x);
		result.push_back(vectors[i].y);
		result.push_back(vectors[i].z);
	}
}

// Gets a vec3's data as a concurrent float array [x, y, z].
float* getVectorData(vec3 vector) {
	float result[3]{ vector.x, vector.y, vector.z };
	return result;
}

// Reverts the rotation of a vector around vec3.null according to unity's rotation scheme.
vec3 revertRotation(vec3 vector, vec3 eulerAngles) {
	mat4 xRot = glm::rotate(eulerAngles[0] * pi<float>() / 180.0f, vec3(1, 0, 0));
	mat4 yRot = glm::rotate(eulerAngles[1] * pi<float>() / 180.0f, vec3(0, 1, 0));
	mat4 zRot = glm::rotate(eulerAngles[2] * pi<float>() / 180.0f, vec3(0, 0, 1));
	return vec3(inverse(yRot*xRot*zRot)*vec4(vector, 0));
}

// Rotate a vector around vec3.null according to unity's rotation scheme.
vec3 rotate(vec3 vector, vec3 eulerAngles) {
	mat4 xRot = glm::rotate(eulerAngles[0] * pi<float>() / 180.0f, vec3(1, 0, 0));
	mat4 yRot = glm::rotate(eulerAngles[1] * pi<float>() / 180.0f, vec3(0, 1, 0));
	mat4 zRot = glm::rotate(eulerAngles[2] * pi<float>() / 180.0f, vec3(0, 0, 1));
	return vec3(yRot*xRot*zRot*vec4(vector, 0));
}