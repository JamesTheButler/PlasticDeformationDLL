#include "LinearAlgebra.h"

LinearAlgebra::LinearAlgebra(){}
LinearAlgebra::~LinearAlgebra(){}

Vector3f LinearAlgebra::linePlaneIntersection(Vector3f planePoint, Vector3f planeNormal, Vector3f lineOrigin, Vector3f lineDirection){
	float t = (-planeNormal).dot(lineOrigin-planePoint) / planeNormal.dot(lineDirection-lineOrigin);
	return lineOrigin + lineDirection*t;
}
