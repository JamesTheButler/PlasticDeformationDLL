#pragma once
#include <glm/vec3.hpp>
#include "Ray.h"
#include "AABox.h"
#include "Misc.h"

using namespace glm;

int _currentCollidingVertexCount = 0;

bool doesCollide(vec3 vertex, vec3 colliderPos, vec3 colliderSize, int colliderType) {
	switch (colliderType) {
	case -1: // default/unset
		return false;
	case 1: {//box
		//AABox aabb = makeAABox(colliderPos, colliderSize);
		//return intersect(aabb, vertex);
		return false;
	}
	default:
		return false;
	}
}

vec3 collisionProjection(vec3 vertex, vec3 previousVertex, vec3 colliderPos, vec3 colliderSize, int colliderType) {
	// generate ray from current to previous vertex
	Ray line = Ray(vertex, previousVertex - vertex);
	switch (colliderType) {
	case 1: {//box
		AABox aabb = makeAABox(colliderPos, colliderSize);
		float t_in = 0.0f, t_out = 0.0f;
		unsigned int numHits = 0;
		//intersect(aabb, line, numHits, t_in, t_out);
		return vertex + (previousVertex - vertex) * t_out;
	}
	default:
		return vertex;
	}
}

void collisionHandling(ColliderData) {
	int collCount = 0;
	vec3 collPos = vec3();
	vec3 collSize = vec3();
	int collType = -1;

	//go over all colliders
	for (int i = 0; i < _collData.colliderCount; i++) {
		// get current collider data
		collType = _collData.colliderTypes[i];
		collPos = _collData.colliderPositions[i];
		collSize = _collData.colliderSizes[i];

		//go over all vertices
		for (int j = 0; j < _vertexCount; j++) {
			//collision handling
			if (doesCollide(_vertexArray[j], collPos, collSize, collType)) {
				collCount++;
				_vertexArray[j] = collisionProjection(_vertexArray[j], _previousVertexArray[j], collPos, collSize, collType);
			}
		}
	}
	_currentCollidingVertexCount = collCount;
}