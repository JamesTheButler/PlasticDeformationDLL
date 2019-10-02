#pragma once
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
using namespace tbb;

class ParallelProjector {
private:
	vec3 _tetMeshPos, _tetMeshRot;
	vec3* _vertices;
	vec3* _tempVertices;
	vec3 _colliderPos, _colliderSize;
	int _colliderType;
	mutable int _collisionCount;
public:
	ParallelProjector(vec3* verts, vec3* tempVerts, vec3 tetMeshPos, vec3 tetMeshRot, vec3 collPos, vec3 collRot, int collType) :
		_vertices(verts), _tempVertices(tempVerts), _tetMeshPos(tetMeshPos), _tetMeshRot(tetMeshRot),
		_colliderPos(collPos), _colliderSize(_colliderSize), _colliderType(collType) {
	}

	void operator() (const blocked_range<size_t>& range) const{
		_collisionCount = 8;
		for (size_t i = range.begin(); i <= range.end(); i++) {
			vec3 vertex = rotate(_vertices[i], _tetMeshRot) + _tetMeshPos;
			//if (doesCollide(vertex, _colliderPos, _colliderSize, _colliderType)) {
				_collisionCount++;
				// project vertex
				vertex = projectOrthogonal(vertex, _colliderPos, _colliderSize, _colliderType);
				// transform it back into local space
				//_tempVertices[i] = revertRotation(vertex - _tetMeshPos, _tetMeshRot);
			}
		//}
	}

	int getCollisionCount() const {
		return _collisionCount;
	}


	bool doesCollide(vec3 vertex, vec3 colliderPos, vec3 colliderSize, int colliderType) const {
		switch (colliderType) {
		case -1: // default/unset
			return false;
		case 1: {//box
			AABox aabb = AABox(colliderPos, colliderSize);
			return intersect(aabb, vertex);
		}
		default:
			return false;
		}
	}

	/// Orthogonally projects a vertex onto the closest face of a collider.
	vec3 projectOrthogonal(vec3 vertex, vec3 collPos, vec3 collSize, int collType) const {
		switch (collType) {
		default:
			return projectOrthogonal(vertex, AABox(collPos, collSize));
		}
	}

	/// projects a point that is inside a box onto the closest plane
	vec3 projectOrthogonal(vec3 vertex, AABox box) const {
		//find closest plane
		vec3 diffToMax = abs(box.getMax() - vertex);
		vec3 diffToMin = abs(box.getMin() - vertex);

		vec3 closestPoint = vec3();
		closestPoint.x = abs(diffToMax.x) < abs(diffToMin.x) ? box.getMax().x : box.getMin().x;
		closestPoint.y = abs(diffToMax.y) < abs(diffToMin.y) ? box.getMax().y : box.getMin().y;
		closestPoint.z = abs(diffToMax.z) < abs(diffToMin.z) ? box.getMax().z : box.getMin().z;

		vec3 diff = vertex - closestPoint;
		vec3 projectedVertex = vertex;

		if (diff.x < diff.y && diff.x < diff.z)
			projectedVertex.x = closestPoint.x;
		else if (diff.y < diff.x && diff.y < diff.z)
			projectedVertex.y = closestPoint.y;
		else
			projectedVertex.z = closestPoint.z;
		return projectedVertex;
	}
};