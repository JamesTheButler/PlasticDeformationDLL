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

enum RotationOrder {
	XYZ,
	ZYX
};

vec3 rotate(vec3 eulerAngles, vec3 vector, RotationOrder order) {
	mat4 xRot, yRot, zRot;
	xRot = glm::rotate(eulerAngles[0], vec3(1, 0, 0));
	yRot = glm::rotate(eulerAngles[1], vec3(0, 1, 0));
	zRot = glm::rotate(eulerAngles[2], vec3(0, 0, 1));
	switch (order) {
	case XYZ:
		vector = vec3(zRot*yRot*xRot*vec4(vector, 0));
		break;
	case ZYX:
		vector = vec3(xRot*yRot*zRot*vec4(vector, 0));
		break;
	}
	return vector;
}

vec3 undoRotation(vec3 eulerAngles, vec3 vector) {
	return rotate(vec3(-eulerAngles.x, -eulerAngles.y, -eulerAngles.z), vector, ZYX);
}