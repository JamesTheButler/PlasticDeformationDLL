#pragma once
#include <glm/vec3.hpp>
using namespace glm;

class AABB {
public:
	AABB() : _position(0,0,0), _size(0,0,0) {}
	AABB(vec3 pos, vec3 size) : _position(pos), _size(size) {}

	vec3 _position, _size;

	vec3 getMin() const { return _position - (_size / 2.0f); }
	vec3 getMax() const { return _position + (_size / 2.0f); }
};

class OBB {
public:
	OBB() : _position(0, 0, 0), _rotation(0,0,0), _size(0, 0, 0) {}
	OBB(vec3 pos, vec3 _rotation, vec3 size) : _position(pos), _rotation(pos), _size(size) {}

	vec3 _position, _rotation, _size;
};