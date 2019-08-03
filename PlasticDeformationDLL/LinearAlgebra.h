#pragma once
#include <Eigen/Dense>

using namespace Eigen;

class LinearAlgebra
{
public:
	LinearAlgebra();
	~LinearAlgebra();

	static Vector3f linePlaneIntersection(Vector3f planePoint, Vector3f planeNormal, Vector3f lineOrigin, Vector3f lineDirection);
};

