#pragma once

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