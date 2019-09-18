#pragma once

class Sphere {
public:
	vec3 _center;
	float _radius;

	Sphere() : _center(0,0,0), _radius(0.0f){}
	Sphere(vec3 center, float radius) : _center(center), _radius(radius){}
};