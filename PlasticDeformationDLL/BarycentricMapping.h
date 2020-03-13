#pragma once
#include <vector>
#include <glm/ext/matrix_float4x4.hpp>
#include <tbb/parallel_for.h>
#include "Logger.h"

using namespace tbb;
using namespace std;
using namespace glm;

namespace bcmapping {
	// Determinant of a mat4x4 defined by 4 row vectors (vec3)
	float determinant4x4(const vec3& v0, const vec3& v1, const vec3& v2, const vec3& v3) {
		return determinant(mat4x4(vec4(v0, 1), vec4(v1, 1), vec4(v2, 1), vec4(v3, 1)));
	}

	vec4 getBarycentricCoordinate2(const vec3& v0, const vec3& v1, const vec3& v2, const vec3& v3, const vec3& vertex) {
		mat3x3 t = mat3x3(v0 - v3, v1 - v3, v2 - v3);
		vec3 temp = inverse(t) * (vertex - v3);
		return vec4(temp.x, temp.y, temp.z, 1 - temp.x - temp.y - temp.z);
	}

	// Calc barycentric coord of a vertex with respect to 4 verts v0-v3 (of a tetrahedron)
	vec4 getBarycentricCoordinate(const vec3& v0, const vec3& v1, const vec3& v2, const vec3& v3, const vec3& vertex) {
		const float determinantTet = determinant4x4(v0, v1, v2, v3);
		const float determinant_p0 = determinant4x4(vertex, v1, v2, v3);
		const float determinant_p1 = determinant4x4(v0, vertex, v2, v3);
		const float determinant_p2 = determinant4x4(v0, v1, vertex, v3);
		const float determinant_p3 = determinant4x4(v0, v1, v2, vertex);
		vec4 bCoord;
		bCoord.x = (determinant_p0 / determinantTet);
		bCoord.y = (determinant_p1 / determinantTet);
		bCoord.z = (determinant_p2 / determinantTet);
		bCoord.w = (determinant_p3 / determinantTet);
		return bCoord;
	}

	//TODO: refactor if-statements
	// Find out if point p lays inside a tetrahedron (represented by 4 vectors)
	bool isVertexInsideTetrahedron(const vec3& v0, const vec3& v1, const vec3& v2, const vec3& v3, const vec3& vertex) {
		const float determinantTet = determinant4x4(v0, v1, v2, v3);
		const float determinant_p0 = determinant4x4(vertex, v1, v2, v3);
		const float determinant_p1 = determinant4x4(v0, vertex, v2, v3);
		const float determinant_p2 = determinant4x4(v0, v1, vertex, v3);
		const float determinant_p3 = determinant4x4(v0, v1, v2, vertex);
		// if determinantTet == 0 -> tetrahedron is coplanar
		if (determinantTet != 0) {
			// if all determinants are of the same sign as the tet's determinant, the vertex is inside
			if (determinantTet < 0) {
				if ((determinant_p0 < 0) && (determinant_p1 < 0) && (determinant_p2 < 0) && (determinant_p3 < 0)) {
					return true;
				}
			} else if (determinantTet > 0) {
				if ((determinant_p0 > 0) && (determinant_p1 > 0) && (determinant_p2 > 0) && (determinant_p3 > 0)) {
					return true;
				}
			}
		}
		return false;
	}
	// Gets center point of a tetrahedron (represented by its 4 verts).
	vec3 getCenterOfTetrahedron(const vec3& v0, const vec3& v1, const vec3& v2, const vec3& v3) {
		return (v0 + v1 + v2 + v3) / 4.0f;
	}

	// Gets the distance between the center of a tetrahedron (represented by its 4 verts) and a vertex.
	float getVertexDistanceToTetrahedronCenter(const vec3& v0, const vec3& v1, const vec3& v2, const vec3& v3, const vec3& vertex) {
		return glm::length(getCenterOfTetrahedron(v0, v1, v2, v3) - vertex);
	}
	
	// For each vertex, finds a tetrahedron that contains the vertex or (if that does not exist) the closest tetrahedron
	void findBarycentricTetIds(
		const vector<vec3>& inputVertices,
		const vector<vec3>& tetMeshVertices,
		const vector<ivec4>& tetMeshTetrahedra,
		vector<int>& barycentricTetrahedronIds) {

		barycentricTetrahedronIds.resize(inputVertices.size(), -1);
		
		for (int vertex = 0; vertex < inputVertices.size(); vertex++) {		//for each input vertex
			float minDistance = FLT_MAX;
			// check for surface vertex-tet mesh vertex mapping
			for (int tet = 0; tet < tetMeshTetrahedra.size(); tet++) {// for each tetrahedron
				ivec4 tetMeshVertexId = tetMeshTetrahedra[tet];
				vec3 v0 = tetMeshVertices[tetMeshVertexId.x];
				vec3 v1 = tetMeshVertices[tetMeshVertexId.y];
				vec3 v2 = tetMeshVertices[tetMeshVertexId.z];
				vec3 v3 = tetMeshVertices[tetMeshVertexId.w];
				// check if vert is in tetrahedron, else remember closest tetrahedron
				if (bcmapping::isVertexInsideTetrahedron(v0, v1, v2, v3, inputVertices[vertex])) {
					barycentricTetrahedronIds[vertex] = tet;
					break;
				} else {
					float distance = getVertexDistanceToTetrahedronCenter(v0, v1, v2, v3, inputVertices[vertex]);
					if (distance < minDistance) {
						minDistance = distance;
						barycentricTetrahedronIds[vertex] = tet;
					}
				}
			}
		}
	}

	void findBarycentricTetIds_m(
		const vector<vec3>& inputVertices,
		const vector<vec3>& tetMeshVertices,
		const vector<ivec4>& tetMeshTetrahedra,
		const vector<int>& surfaceToTetMeshVertexMap,
		vector<int>& barycentricTetrahedronIds) {

		barycentricTetrahedronIds.resize(inputVertices.size(), -1);

		for(int vertex=0; vertex < inputVertices.size(); vertex++) {		//for each input vertex
			float minDistance = FLT_MAX;
			// check for surface vertex-tet mesh vertex mapping
			if (surfaceToTetMeshVertexMap[vertex] == -1) {//if surface vertex is NOT mapped directly to tet mesh vetex
				for (int tet = 0; tet < tetMeshTetrahedra.size(); tet++) {// for each tetrahedron
					ivec4 tetMeshVertexId = tetMeshTetrahedra[tet];
					vec3 v0 = tetMeshVertices[tetMeshVertexId.x];
					vec3 v1 = tetMeshVertices[tetMeshVertexId.y];
					vec3 v2 = tetMeshVertices[tetMeshVertexId.z];
					vec3 v3 = tetMeshVertices[tetMeshVertexId.w];
					// check if vert is in tetrahedron, else remember closest tetrahedron
					if (bcmapping::isVertexInsideTetrahedron(v0, v1, v2, v3, inputVertices[vertex])) {
						barycentricTetrahedronIds[vertex] = tet;
						break;
					} else {
						float distance = getVertexDistanceToTetrahedronCenter(v0, v1, v2, v3, inputVertices[vertex]);
						if (distance < minDistance) {
							minDistance = distance;
							barycentricTetrahedronIds[vertex] = tet;
						}
					}
				}
			}
		}
	}

	void generateBarycentricMapping(
		const vector<vec3>& inputVertices,
		const vector<vec3>& tetMeshVertices,
		const vector<ivec4>& tetMeshTetrahedra,
		vector<vec4>& barycentricCoords,
		vector<int>& barycentricTetrahedronIds) {

		logger::log("--findBaryCentricCoordinatedForVerticesWithMapping");
		if (barycentricTetrahedronIds.size() != inputVertices.size())
			logger::logError("\tbarycetric tet ids must be calculated before hand");
		barycentricCoords.resize(inputVertices.size(), vec4());

		parallel_for((size_t)0, inputVertices.size(), (size_t)1, [&](size_t vertex) {		//for each input vertex
			ivec4 tetMeshVertexId = tetMeshTetrahedra[barycentricTetrahedronIds[vertex]];
			vec3 v0 = tetMeshVertices[tetMeshVertexId.x];
			vec3 v1 = tetMeshVertices[tetMeshVertexId.y];
			vec3 v2 = tetMeshVertices[tetMeshVertexId.z];
			vec3 v3 = tetMeshVertices[tetMeshVertexId.w];
			barycentricCoords[vertex] = getBarycentricCoordinate(v0, v1, v2, v3, inputVertices[vertex]);
		});
	}

	// Generates barycentric mapping for a set of input vertices and a tetrahedral mesh.
	// Uses pre-calculated closest tetrahedra for barycentric mapping.
	void generateBarycentricMapping_m(
		const vector<vec3>& inputVertices,
		const vector<vec3>& tetMeshVertices,
		const vector<ivec4>& tetMeshTetrahedra,
		const vector<int>& surfaceToTetMeshVertexMap,
		vector<vec4>& barycentricCoords,
		vector<int>& barycentricTetrahedronIds) {

		logger::log("--findBaryCentricCoordinatedForVerticesWithMapping");
		if (barycentricTetrahedronIds.size() != inputVertices.size())
			logger::logError("\tbarycetric tet ids must be calculated before hand");
		barycentricCoords.resize(inputVertices.size(), vec4());
		//barycentricTetrahedronIds.resize(inputVertices.size(), 0);
		
		parallel_for((size_t)0, inputVertices.size(), (size_t)1, [&](size_t vertex) {		//for each input vertex
			if (surfaceToTetMeshVertexMap[vertex] != -1) { // vertex can be directly mapped to a tetmesh vertex
				barycentricCoords[vertex] = vec4();
				barycentricTetrahedronIds[vertex] = -1;
			} else { // vertex is mapped to a tetmesh tetrahedron
				ivec4 tetMeshVertexId = tetMeshTetrahedra[barycentricTetrahedronIds[vertex]];
				vec3 v0 = tetMeshVertices[tetMeshVertexId.x];
				vec3 v1 = tetMeshVertices[tetMeshVertexId.y];
				vec3 v2 = tetMeshVertices[tetMeshVertexId.z];
				vec3 v3 = tetMeshVertices[tetMeshVertexId.w];
				barycentricCoords[vertex] = getBarycentricCoordinate(v0, v1, v2, v3, inputVertices[vertex]);
			}
		});
	}


	//TODO: change flags to bool (is in map: true, else is inside true else false)
	// Generates barycentric mapping for a set of input vertices and a tetrahedral mesh.
	/*void findBaryCentricCoordinatesForVerticesWithMapping(
		const vector<vec3>& inputVertices,
		const vector<vec3>& tetMeshVertices,
		const vector<ivec4> &tetMeshTetrahedra,
		const vector<int> &surfaceToTetMeshVertexMap,
		vector<vec4>& barycentricCoords,
		vector<int>& barycentricTetrahedronIds,
		vector<int>& insideFlags, 
		vector<float>& minDistances) {

		logger::log("--findBaryCentricCoordinatedForVerticesWithMapping");

		barycentricCoords.resize(inputVertices.size(), vec4());
		barycentricTetrahedronIds.resize(inputVertices.size(), 0);

		float result = 0.0f;
		float minDistance;
		unsigned int closestTetrahedronIndex;
		bool isInside;

		parallel_for((size_t)0, inputVertices.size(), (size_t)1, [&](size_t vertex) {		//for each input vertex
			minDistance = FLT_MAX;
			isInside = false;
			vec3 vert = inputVertices[vertex];
			// check for surface vertex-tet mesh vertex mapping

			if (surfaceToTetMeshVertexMap[vertex] != -1) {
				barycentricCoords[vertex] = vec4();
				barycentricTetrahedronIds[vertex] = -1;
				insideFlags[vertex] = -1;
			}
			else {		//if surface vertex is NOT mapped to tet mesh vetex
				parallel_for((size_t)0, tetMeshTetrahedra.size(), (size_t)1, [&](size_t tet) {// for each tetrahedron	
					if (!isInside) {	// skip further iterations if vertex is inside tetrahedron (for the lack of tbb::parallel_break)
						ivec4 tetMeshVertexId = tetMeshTetrahedra[tet];
						vec3 v0 = tetMeshVertices[tetMeshVertexId.x];
						vec3 v1 = tetMeshVertices[tetMeshVertexId.y];
						vec3 v2 = tetMeshVertices[tetMeshVertexId.z];
						vec3 v3 = tetMeshVertices[tetMeshVertexId.w];
						if (isVertexInsideTetrahedron(v0, v1, v2, v3, vert)) {	// if vertex is inside a tetrahedron of the tet mesh, set barycentric coords of it
							barycentricCoords[vertex] = getBarycentricCoordinate(v0, v1, v2, v3, vert);
							//barycentricCoords[vertex] = getBarycentricCoordinate2(v0, v1, v2, v3, vert);
							barycentricTetrahedronIds[vertex] = tet;
							isInside = true;
							insideFlags[vertex] = 1;
							minDistances[vertex] = -1;
							minDistance = FLT_MAX;
							task::self().cancel_group_execution();
						}
						else {	// if vert is NOT inside a tetraheron, find closest tetrahedron
							//isInside = false;
							insideFlags[vertex] = 0;
							float distance = getVertexDistanceToTetrahedronCenter(v0, v1, v2, v3, vert);
							if (distance < minDistance) {
								minDistance = distance;
								closestTetrahedronIndex = tet;
							}
						}
					}
				});

				if (!isInside) {			// set closest tetrahedron as barycentric tetrahedron
					ivec4 tetMeshVertexId = tetMeshTetrahedra[closestTetrahedronIndex];
					vec3 v0 = tetMeshVertices[tetMeshVertexId.x];
					vec3 v1 = tetMeshVertices[tetMeshVertexId.y];
					vec3 v2 = tetMeshVertices[tetMeshVertexId.z];
					vec3 v3 = tetMeshVertices[tetMeshVertexId.w];
					minDistances[vertex] = minDistance;
					barycentricCoords[vertex] = getBarycentricCoordinate(v0, v1, v2, v3, vert);
					//barycentricCoords[vertex] = getBarycentricCoordinate2(v0, v1, v2, v3, vert);
					barycentricTetrahedronIds[vertex] = closestTetrahedronIndex;
					minDistance = FLT_MAX;
				}
			}
		});
	}*/

	// Generates barycentric mapping for a set of input vertices and a tetrahedral mesh.
	/*void findBaryCentricCoordinatesForVerticesWithMapping(
		const vector<vec3>& inputVertices,
		const vector<vec3>& tetMeshVertices,
		const vector<ivec4> &tetMeshTetrahedra,
		const vector<int> &surfaceToTetMeshVertexMap,
		vector<vec4>& barycentricCoords,
		vector<int>& barycentricTetrahedronIds) {

		logger::log("--findBaryCentricCoordinatedForVerticesWithMapping");

		barycentricCoords.resize(inputVertices.size(), vec4());
		barycentricTetrahedronIds.resize(inputVertices.size(), 0);

		float result = 0.0f;
		float minDistance;
		unsigned int closestTetrahedronIndex;
		bool isInside;

		parallel_for((size_t)0, inputVertices.size(), (size_t)1, [&](size_t vertex) {		//for each input vertex
			minDistance = FLT_MAX;
			isInside = false;
			// check for surface vertex-tet mesh vertex mapping

			if (surfaceToTetMeshVertexMap[vertex] != -1) {
				barycentricCoords[vertex] = vec4();
				barycentricTetrahedronIds[vertex] = -1;
			}
			else {		//if surface vertex is NOT mapped to tet mesh vetex
				parallel_for((size_t)0, tetMeshTetrahedra.size(), (size_t)1, [&](size_t tet) {// for each tetrahedron	
					if (!isInside) {	// skip further iterations if vertex is inside tetrahedron (for the lack of tbb::parallel_break)
						ivec4 tetMeshVertexId = tetMeshTetrahedra[tet];
						vec3 v0 = tetMeshVertices[tetMeshVertexId.x];
						vec3 v1 = tetMeshVertices[tetMeshVertexId.y];
						vec3 v2 = tetMeshVertices[tetMeshVertexId.z];
						vec3 v3 = tetMeshVertices[tetMeshVertexId.w];
						if (isVertexInsideTetrahedron(v0, v1, v2, v3, inputVertices[vertex])) {	// if vertex is inside a tetrahedron of the tet mesh, set barycentric coords of it
							barycentricCoords[vertex] = getBarycentricCoordinate(v0, v1, v2, v3, inputVertices[vertex]);
							//barycentricCoords[vertex] = getBarycentricCoordinate2(v0, v1, v2, v3, inputVertices[vertex]);
							barycentricTetrahedronIds[vertex] = tet;
							isInside = true;
							task::self().cancel_group_execution();
						}
						else {	// if vert is NOT inside a tetraheron, find closest tetrahedron
							float distance = getVertexDistanceToTetrahedronCenter(v0, v1, v2, v3, inputVertices[vertex]);
							if (distance < minDistance) {
								minDistance = distance;
								closestTetrahedronIndex = tet;
							}
						}
					}
				});

				if (!isInside) {			// set closest tetrahedron as barycentric tetrahedron
					ivec4 tetMeshVertexId = tetMeshTetrahedra[closestTetrahedronIndex];
					vec3 v0 = tetMeshVertices[tetMeshVertexId.x];
					vec3 v1 = tetMeshVertices[tetMeshVertexId.y];
					vec3 v2 = tetMeshVertices[tetMeshVertexId.z];
					vec3 v3 = tetMeshVertices[tetMeshVertexId.w];
					barycentricCoords[vertex] = getBarycentricCoordinate(v0, v1, v2, v3, inputVertices[vertex]);
					//barycentricCoords[vertex] = getBarycentricCoordinate2(v0, v1, v2, v3, inputVertices[vertex]);
					barycentricTetrahedronIds[vertex] = closestTetrahedronIndex;
					minDistance = FLT_MAX;
				}
			}
		});
	}*/

	/*void findBaryCentricCoordinatedForVertices(
		const vector<vec3>& inputVertices,
		const vector<vec3>& tetMeshVertices,
		const vector<ivec4> &tetrahedraVertexIds,
		vector<vec4>& barycentricCoords,
		vector<int>& barycentricTetrahedronIds) {

		barycentricCoords.resize(inputVertices.size(), vec4());
		barycentricTetrahedronIds.resize(inputVertices.size(), 0);

		float result = 0.0f;
		float minDistance;
		unsigned int closestTetrahedronIndex;
		bool isInside;
		parallel_for((size_t)0, inputVertices.size(), (size_t)1, [&](size_t vertex) {		//for each input vertex
			minDistance = FLT_MAX;
			isInside = false;
			parallel_for((size_t)0, tetrahedraVertexIds.size(), (size_t)1, [&](size_t tet) {// for each tetrahedron	
				if (!isInside) {	// skip further iterations if vertex is inside tetrahedron (for the lack of tbb::parallel_break)
					ivec4 tetMeshVertexId = tetrahedraVertexIds[tet];
					vec3 v0 = tetMeshVertices[tetMeshVertexId.x];
					vec3 v1 = tetMeshVertices[tetMeshVertexId.y];
					vec3 v2 = tetMeshVertices[tetMeshVertexId.z];
					vec3 v3 = tetMeshVertices[tetMeshVertexId.w];
					if (isVertexInsideTetrahedron(v0, v1, v2, v3, inputVertices[vertex])) {	// if vertex is inside a tetrahedron of the tet mesh, set barycentric coords of it
						barycentricCoords[vertex] = getBarycentricCoordinate(v0, v1, v2, v3, inputVertices[vertex]);
						barycentricTetrahedronIds[vertex] = tet;
						isInside = true;
						task::self().cancel_group_execution();
					}
					else {	// if vert is NOT inside a tetraheron, find closest tetrahedron
						float distance = getVertexDistanceToTetrahedronCenter(v0, v1, v2, v3, inputVertices[vertex]);
						if (distance < minDistance) {
							minDistance = distance;
							closestTetrahedronIndex = tet;
						}
					}
				}
			});

			if (!isInside) {			// set closest tetrahedron as barycentric tetrahedron
				ivec4 tetMeshVertexId = tetrahedraVertexIds[closestTetrahedronIndex];
				vec3 v0 = tetMeshVertices[tetMeshVertexId.x];
				vec3 v1 = tetMeshVertices[tetMeshVertexId.y];
				vec3 v2 = tetMeshVertices[tetMeshVertexId.z];
				vec3 v3 = tetMeshVertices[tetMeshVertexId.w];
				barycentricCoords[vertex] = getBarycentricCoordinate(v0, v1, v2, v3, inputVertices[vertex]);
				barycentricTetrahedronIds[vertex] = closestTetrahedronIndex;
			}
		});
	}*/

	// Calculates the position represented by a barycentric coordinate.
	vec3 getPositionByBarycentricCoord(const vec3& v0, const vec3& v1, const vec3& v2, const vec3& v3, const vec4& barycentricCoord) {
		return v0 * barycentricCoord.x + v1 * barycentricCoord.y + v2 * barycentricCoord.z + v3 * barycentricCoord.w;
	}

	void indexSubsetVertices(const vector<vec3> &subSet, vector<vec3>& superSet, vector<int>& indeces) {
		logger::log("--indexSubsetVertices");
		logger::log("\t-subset count:" + to_string(subSet.size()) + ",\t superSet count: " + to_string(superSet.size()));
		indeces.resize(subSet.size(), -1);
		parallel_for((size_t)0, subSet.size(), [&](size_t &i) {
			indeces[i] = vectorFuncs::findIdOfSimilar(subSet[i], superSet, 0.01f);
		});
	}

	// updates the positions of a set of input vertices according to their respective barycentric coordinates with regards to the tet mesh vertices.
	void updateSurfaceVertices(
		vector<vec3>& surfaceVertices,
		const vector<vec3>& tetMeshVertices,
		const vector<ivec4> &tetMeshTetrahedra,
		const vector<vec4>& barycentricCoords,
		const vector<int>& barycentricTetIds) {

		//logger::log("--updateSurfaceVerticesWithMapping");
		parallel_for((size_t)0, (size_t)surfaceVertices.size(), (size_t)1, [&](size_t surfVertex) {
			// get positions of related verts
			ivec4 tetMeshTetrahedron = tetMeshTetrahedra[barycentricTetIds[surfVertex]];
			vec3 v0 = tetMeshVertices[tetMeshTetrahedron.x];
			vec3 v1 = tetMeshVertices[tetMeshTetrahedron.y];
			vec3 v2 = tetMeshVertices[tetMeshTetrahedron.z];
			vec3 v3 = tetMeshVertices[tetMeshTetrahedron.w];
			surfaceVertices[surfVertex] = getPositionByBarycentricCoord(v0, v1, v2, v3, barycentricCoords[surfVertex]);
		});
	}

	// updates the positions of a set of input vertices according to their respective barycentric coordinates with regards to the tet mesh vertices.
	void updateSurfaceVertices_m(
		vector<vec3>& surfaceVertices,
		const vector<vec3>& tetMeshVertices,
		const vector<ivec4> &tetMeshTetrahedra,
		const vector<int> &surfaceToTetMeshVertexMap,
		const vector<vec4>& barycentricCoords,
		const vector<int>& barycentricTetIds) {

		//logger::log("--updateSurfaceVerticesWithMapping");
		parallel_for((size_t)0, (size_t)surfaceVertices.size(), (size_t)1, [&](size_t surfVertex) {
			if (surfaceToTetMeshVertexMap[surfVertex] != -1) {
				surfaceVertices[surfVertex] = tetMeshVertices[surfaceToTetMeshVertexMap[surfVertex]];
			} else {
				// get positions of related verts
				ivec4 tetMeshTetrahedron = tetMeshTetrahedra[barycentricTetIds[surfVertex]];
				vec3 v0 = tetMeshVertices[tetMeshTetrahedron.x];
				vec3 v1 = tetMeshVertices[tetMeshTetrahedron.y];
				vec3 v2 = tetMeshVertices[tetMeshTetrahedron.z];
				vec3 v3 = tetMeshVertices[tetMeshTetrahedron.w];
				surfaceVertices[surfVertex] = getPositionByBarycentricCoord(v0, v1, v2, v3, barycentricCoords[surfVertex]);
			}
		});
	}
}