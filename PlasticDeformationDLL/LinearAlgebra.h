#pragma once
#include <Eigen/Dense>

using namespace Eigen;

class LinearAlgebra
{
public:
	LinearAlgebra();
	~LinearAlgebra();

	static float tAtLinePlaneIntersection(Vector3f planePoint, Vector3f planeNormal, Vector3f lineOrigin, Vector3f lineDirection);
};

