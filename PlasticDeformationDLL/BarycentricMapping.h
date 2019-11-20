#include "glm/detail/type_mat4x4.hpp"

// Determinant of a mat4x4 defined by 4 row vectors (vec3)
float determinant4x4(const vec3& v0, const vec3& v1, const vec3& v2, const vec3& v3) {
	return determinant4x4(vec4(v0, 1), vec4(v1, 1), vec4(v2, 1), vec4(v3, 1));
}

// Determinant of a mat4x4 defined by 4 row vectors (vec4)
float determinant4x4(const vec4& v0, const vec4& v1, const vec4& v2, const vec4& v3) {
	return determinant(mat4x4(v0, v1, v2, v3));
}

// Calc barycentric coord of a vertex with respect to 4 verts v0-v3 (of a tetrahedron)
vec4 getBarycentricCoordinate(const vec3& v0, const vec3& v1, const vec3& v2, const vec3& v3, const vec3& vertex) {
	const float determinantTet = determinant4x4(vec4(v0, 1), vec4(v1, 1), vec4(v2, 1), vec4(v3, 1));
	const float determinant_p0 = determinant4x4(vec4(vertex, 1), vec4(v1, 1), vec4(v2, 1), vec4(v3, 1));
	const float determinant_p1 = determinant4x4(vec4(v0, 1), vec4(vertex, 1), vec4(v2, 1), vec4(v3, 1));
	const float determinant_p2 = determinant4x4(vec4(v0, 1), vec4(v1, 1), vec4(vertex, 1), vec4(v3, 1));
	const float determinant_p3 = determinant4x4(vec4(v0, 1), vec4(v1, 1), vec4(v2, 1), vec4(vertex, 1));
	vec4 bCoord;
	bCoord.x = (determinant_p0 / determinantTet);
	bCoord.y = (determinant_p1 / determinantTet);
	bCoord.z = (determinant_p2 / determinantTet);
	bCoord.w = (determinant_p3 / determinantTet);
	return bCoord;
}

// Find out if point p lays inside a tetrahedron (represented by 4 vectors)
bool isVertexInsideTetrahedron(const vec3& v0, const vec3& v1, const vec3& v2, const vec3& v3, const vec3& vertex) {
	const float determinantTet = determinant4x4(vec4(v0, 1), vec4(v1, 1), vec4(v2, 1), vec4(v3, 1));
	const float determinant_p0 = determinant4x4(vec4(vertex, 1), vec4(v1, 1), vec4(v2, 1), vec4(v3, 1));
	const float determinant_p1 = determinant4x4(vec4(v0, 1), vec4(vertex, 1), vec4(v2, 1), vec4(v3, 1));
	const float determinant_p2 = determinant4x4(vec4(v0, 1), vec4(v1, 1), vec4(vertex, 1), vec4(v3, 1));
	const float determinant_p3 = determinant4x4(vec4(v0, 1), vec4(v1, 1), vec4(v2, 1), vec4(vertex, 1));
	// if determinantTet == 0 -> tetrahedron is coplanar
	if (determinantTet != 0) {
		// if all determinants are of the same sign as the tet's determinant, the vertex is inside
		if (determinantTet < 0) {		
			if ((determinant_p0 < 0) && (determinant_p1 < 0) && (determinant_p2 < 0) && (determinant_p3 < 0)) {
				return true;
			}
		}
		if (determinantTet > 0) {
			if ((determinant_p0 > 0) && (determinant_p1 > 0) && (determinant_p2 > 0) && (determinant_p3 > 0)) {
				return true;
			}
		}
	}
	return false;
}

vec3 getCenterOfTetrahedron(const vec3& v0, const vec3& v1, const vec3& v2, const vec3& v3) {
	return (v0 + v1 + v2 + v3) / 4.0f;
}

// Gets the distance between the center of a tetrahedron (represented by its 4 verts) and a vertex.
float getVertexDistanceToTetrahedronCenter(const vec3& v0, const vec3& v1, const vec3& v2, const vec3& v3, const vec3& vertex) {
	return (getCenterOfTetrahedron(v0, v1, v2, v3) - vertex).length();
}

// Finds the barycentric coordinates for a vector of input vertices
void findBaryCentricCoordinatedForVertices(
	const vector<vec3>& inputVertices,
	const vector<vec3>& vertices,
	const vector<vector<int>> &tetVertexIds,
	vector<vec4>& barycentricCoords,
	vector<int>& barycentricTetrahedronIds) {

	barycentricCoords.reserve(inputVertices.size());
	barycentricTetrahedronIds.reserve(inputVertices.size());

	float minDistance;
	unsigned int closestTetrahedronIndex;
	for (auto vertex = inputVertices.begin(); vertex < inputVertices.end(); vertex++) {
		minDistance = FLT_MAX;
		for (int i = 0; i < tetVertexIds.size(); i++) {
			vec3 v0 = vertices[tetVertexIds[i][0]];
			vec3 v1 = vertices[tetVertexIds[i][1]];
			vec3 v2 = vertices[tetVertexIds[i][2]];
			vec3 v3 = vertices[tetVertexIds[i][3]];
			if (isVertexInsideTetrahedron(v0, v1, v2, v3, *vertex)) {
				barycentricCoords.push_back(getBarycentricCoordinate(v0, v1, v2, v3, *vertex));
				barycentricTetrahedronIds.push_back(i);
			} else {
				float distance = getVertexDistanceToTetrahedronCenter(v0, v1, v2, v3, *vertex);
				if (distance < minDistance) {
					minDistance = distance;
					closestTetrahedronIndex = i;
				}
			}
		}
		vec3 v0 = vertices[tetVertexIds[closestTetrahedronIndex][0]];
		vec3 v1 = vertices[tetVertexIds[closestTetrahedronIndex][1]];
		vec3 v2 = vertices[tetVertexIds[closestTetrahedronIndex][2]];
		vec3 v3 = vertices[tetVertexIds[closestTetrahedronIndex][3]];
		barycentricCoords.push_back(getBarycentricCoordinate(v0, v1, v2, v3, *vertex));
		barycentricTetrahedronIds.push_back(closestTetrahedronIndex);
	}
}

// Calculates the position represented by a barycentric coordinate.
vec3 getPositionByBarycentricCoord(const vec3& v0, const vec3& v1, const vec3& v2, const vec3& v3, const vec4& barycentricCoord) {
	return v0 * barycentricCoord.x + v1 * barycentricCoord.y + v2 * barycentricCoord.z + v3 * barycentricCoord.w;
}

void updateSurfaceVertices(
	const vector<vec3>& inputVertices,
	const vector<vec3>& vertices,
	const vector<vector<int>> &tetVertexIds,
	const vector<vec4>& barycentricCoords,
	const vector<int>& barycentricTetIds,
	vector<vec3>& outputVertices) {

	outputVertices.reserve(inputVertices.size());
	for (int inputVertId = 0;  inputVertId<inputVertices.size(); inputVertId++) {
		vec3 v0 = vertices[tetVertexIds[barycentricTetIds[inputVertId]][0]];
		vec3 v1 = vertices[tetVertexIds[barycentricTetIds[inputVertId]][1]];
		vec3 v2 = vertices[tetVertexIds[barycentricTetIds[inputVertId]][2]];
		vec3 v3 = vertices[tetVertexIds[barycentricTetIds[inputVertId]][3]];
		outputVertices.push_back(getPositionByBarycentricCoord(v0,v1,v2,v3, barycentricCoords[inputVertId]));
	}
}