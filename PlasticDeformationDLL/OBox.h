#pragma once

class OBox {
public:
	vec3 _center;
	vec3 _halfSides[3];

	OBox() : _center(0, 0, 0), _halfSides{ {0,0,0},{0,0,0},{0,0,0} } {}
	OBox(vec3 center, vec3 halfSides[3]) : _center(center), _halfSides{ halfSides[0],halfSides[1],halfSides[2] } {}
	OBox(AABox aabox) : _center(aabox._position) {
		_halfSides[0] = vec3(aabox._size[0], 0, 0);
		_halfSides[1] = vec3(0, aabox._size[1], 0);
		_halfSides[2] = vec3(0, 0, aabox._size[2]);
	}

	void getVertices(vec3 vertices[8]) {
		vertices[0] = _center - _halfSides[0] - _halfSides[1] - _halfSides[2];
		vertices[1] = _center + _halfSides[0] - _halfSides[1] - _halfSides[2];
		vertices[2] = _center + _halfSides[0] + _halfSides[1] - _halfSides[2];
		vertices[3] = _center - _halfSides[0] + _halfSides[1] - _halfSides[2];
		vertices[4] = _center - _halfSides[0] - _halfSides[1] + _halfSides[2];
		vertices[5] = _center + _halfSides[0] - _halfSides[1] + _halfSides[2];
		vertices[6] = _center + _halfSides[0] + _halfSides[1] + _halfSides[2];
		vertices[7] = _center - _halfSides[0] + _halfSides[1] + _halfSides[2];
	}
};