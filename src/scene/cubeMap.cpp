#include "cubeMap.h"
#include "ray.h"
#include "../ui/TraceUI.h"
#include "../scene/material.h"
#include <iostream>
extern TraceUI* traceUI;

glm::dvec2 getUV(double a, double b, double axis) {
	a /= std::abs(axis);
	a += 1.0;
	a /= 2.0;
	b /= std::abs(axis);
	b += 1.0;
	b /= 2.0;
	return glm::dvec2(a, b);
}

glm::dvec3 CubeMap::getColor(ray r) const
{
	// YOUR CODE HERE
	// FIXME: Implement Cube Map here
	double x = r.getDirection().x;
	double y = r.getDirection().y;
	double z = r.getDirection().z;
	double a = std::max(abs(x), std::max(abs(y), abs(z)));

	if (a == x) { // basically i have no idea if any of these are correct im totally guessing tbh
		return tMap[0]->getMappedValue(getUV(z, y, x));
	}
	else if (a == -x) {
		return tMap[1]->getMappedValue(getUV(-z, y, x));
	}
	else if (a == y) {
		return tMap[2]->getMappedValue(getUV(x, z, y));
	}
	else if (a == -y) {
		return tMap[3]->getMappedValue(getUV(x, -z, y));
	}
	else if (a == z) {
		return tMap[5]->getMappedValue(getUV(-x, y, z));
	}
	else if (a == -z) {
		return tMap[4]->getMappedValue(getUV(x, y, z));
	}
	else {
		std::cout << "error" << std::endl;
	}
	return glm::dvec3();
}

CubeMap::CubeMap()
{
}

CubeMap::~CubeMap()
{
}

void CubeMap::setNthMap(int n, TextureMap* m)
{
	if (m != tMap[n].get())
		tMap[n].reset(m);
}
