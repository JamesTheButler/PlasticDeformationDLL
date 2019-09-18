#pragma once

bool intersect(Sphere sphere, vec3 point) {
	return distance(point, sphere._center) < sphere._radius;
}

bool intersect(Sphere sphere1, Sphere sphere2) {
	return distance(sphere1._center, sphere2._center) < (sphere1._radius + sphere2._radius);
}

/// taken from the gmtl implementation @ http://ggt.sourceforge.net/html/main.html
bool intersect(AABox box, vec3 point) {
	// Look for a separating axis on each box for each axis
	if (box.getMin()[0] > point[0])  return false;
	if (box.getMin()[1] > point[1])  return false;
	if (box.getMin()[2] > point[2])  return false;

	if (point[0] > box.getMax()[0])  return false;
	if (point[1] > box.getMax()[1])  return false;
	if (point[2] > box.getMax()[2])  return false;
	return true;
}


/// taken from the gmtl implementation @ http://ggt.sourceforge.net/html/main.html
bool intersect(AABox box, Ray ray, vec3& inPos, vec3& outPos){
	float tIn = -(std::numeric_limits<float>::max)();
	float tOut = (std::numeric_limits<float>::max)();
	float t0, t1;
	const float epsilon(0.0000001);

	// YZ plane.
	if (abs(ray._direction[0]) < epsilon) {
		// Ray parallel to plane.
		if (ray._origin[0] < box.getMin()[0] || ray._origin[0] > box.getMax()[0]) {
			return false;
		}
	}

	// XZ plane.
	if (abs(ray._direction[1]) < epsilon) {
		// Ray parallel to plane.
		if (ray._origin[1] < box.getMin()[1] || ray._origin[1] > box.getMax()[1]) {
			return false;
		}
	}

	// XY plane.
	if (abs(ray._direction[2]) < epsilon) {
		// Ray parallel to plane.
		if (ray._origin[2] < box.getMin()[2] || ray._origin[2] > box.getMax()[2]) {
			return false;
		}
	}

	// YZ plane.
	t0 = (box.getMin()[0] - ray._origin[0]) / ray._direction[0];
	t1 = (box.getMax()[0] - ray._origin[0]) / ray._direction[0];

	if (t0 > t1) {
		std::swap(t0, t1);
	}

	if (t0 > tIn) {
		tIn = t0;
	}
	if (t1 < tOut) {
		tOut = t1;
	}

	if (tIn > tOut || tOut < 0.0f) {
		return false;
	}

	// XZ plane.
	t0 = (box.getMin()[1] - ray._origin[1]) / ray._direction[1];
	t1 = (box.getMax()[1] - ray._origin[1]) / ray._direction[1];

	if (t0 > t1) {
		std::swap(t0, t1);
	}
	if (t0 > tIn) {
		tIn = t0;
	}
	if (t1 < tOut) {
		tOut = t1;
	}
	if (tIn > tOut || tOut < 0.0f) {
		return false;
	}

	// XY plane.
	t0 = (box.getMin()[2] - ray._origin[2]) / ray._direction[2];
	t1 = (box.getMax()[2] - ray._origin[2]) / ray._direction[2];

	if (t0 > t1) {
		std::swap(t0, t1);
	}
	if (t0 > tIn) {
		tIn = t0;
	}
	if (t1 < tOut) {
		tOut = t1;
	}

	inPos = ray._origin + (ray._direction* tIn);
	outPos = ray._origin + (ray._direction* tOut);

	if (tIn > tOut || tOut < 0.0f) {
		return false;
	}

	return true;
}