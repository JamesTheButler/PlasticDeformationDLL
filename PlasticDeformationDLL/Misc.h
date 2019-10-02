#pragma once

float* getVectorData(vec3* vectors, int vectorCount) {
	float* result = new float[vectorCount * 3];
	for (int i = 0; i < vectorCount; i++) {
		result[i * 3] = vectors[i].x;
		result[i * 3 + 1] = vectors[i].y;
		result[i * 3 + 2] = vectors[i].z;
	}
	return result;
}

float* getVectorData(vec3 vector) {
	return new float[3]{ vector.x, vector.y, vector.z };
}

vec3 revertRotation(vec3 vector, vec3 eulerAngles) {
	mat4 xRot = glm::rotate(eulerAngles[0] * pi<float>() / 180.0f, vec3(1, 0, 0));
	mat4 yRot = glm::rotate(eulerAngles[1] * pi<float>() / 180.0f, vec3(0, 1, 0));
	mat4 zRot = glm::rotate(eulerAngles[2] * pi<float>() / 180.0f, vec3(0, 0, 1));
	return vec3(inverse(yRot*xRot*zRot)*vec4(vector, 0));
}

vec3 rotate(vec3 vector, vec3 eulerAngles) {
	mat4 xRot = glm::rotate(eulerAngles[0] * pi<float>() / 180.0f, vec3(1, 0, 0));
	mat4 yRot = glm::rotate(eulerAngles[1] * pi<float>() / 180.0f, vec3(0, 1, 0));
	mat4 zRot = glm::rotate(eulerAngles[2] * pi<float>() / 180.0f, vec3(0, 0, 1));
	return vec3(yRot*xRot*zRot*vec4(vector, 0));
}