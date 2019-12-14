#pragma once

class AABox {
public:
	AABox() : _position(0,0,0), _size(0,0,0) {}
	AABox(vec3 pos, vec3 size) : _position(pos), _size(size) {}

	vec3 _position, _size;

	vec3 getMin() const { return _position - (_size / 2.0f); }
	vec3 getMax() const { return _position + (_size / 2.0f); }
};