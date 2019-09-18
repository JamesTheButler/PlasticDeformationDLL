#pragma once

class Ray {
public:
	vec3 _origin, _direction;

	Ray() : _origin(0,0,0), _direction(0,0,0){}
	Ray(vec3 origin, vec3 direction) : _origin(origin), _direction(direction) {}
};