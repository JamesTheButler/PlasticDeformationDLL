#include "LinearAlgebra.h"

LinearAlgebra::LinearAlgebra(){}
LinearAlgebra::~LinearAlgebra(){}

float LinearAlgebra::tAtLinePlaneIntersection(Vector3f planePoint, Vector3f planeNormal, Vector3f lineOrigin, Vector3f lineDirection){
	return (-planeNormal).dot(lineOrigin - planePoint) / planeNormal.dot(lineDirection - lineOrigin);
}
