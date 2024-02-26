#include <cmath>
#include <iostream>

#include "light.h"
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>


using namespace std;

double DirectionalLight::distanceAttenuation(const glm::dvec3& P) const
{
	// distance to light is infinite, so f(di) goes to 0.  Return 1.
	return 1.0;
}


glm::dvec3 DirectionalLight::shadowAttenuation(const ray& r, const glm::dvec3& p, double thresh) const
{
	// YOUR CODE HERE:
	// You should implement shadow-handling code here.
	if (glm::distance(r.getAtten(), glm::dvec3(0, 0, 0)) < thresh) {
		return glm::dvec3(0.0, 0.0, 0.0);
	}
	isect i;
	ray shadow_r = r;
	if (scene->intersect(shadow_r, i)) {
		const Material& m = i.getMaterial();
		if (m.Trans()) {
			double t = i.getT();
			glm::dvec3 pos = shadow_r.at(t + RAY_EPSILON); // to get through the surface
			glm::dvec3 atten = glm::dvec3(1, 1, 1);
			bool leaving = glm::dot(shadow_r.getDirection(), i.getN()) > 0;
			if (leaving) {
				glm::dvec3 kt = m.kt(i);
				atten *= glm::dvec3(pow(kt.x, t), pow(kt.y, t), pow(kt.z, t));
			}
			ray recur(pos, shadow_r.getDirection(), shadow_r.getAtten(), ray::SHADOW);
			recur.setAtten(atten * recur.getAtten());
			return atten * shadowAttenuation(recur, pos, thresh);
		}
		else {
			return glm::dvec3(0.0, 0.0, 0.0);
		}
	}
	return glm::dvec3(1.0, 1.0, 1.0);
}

glm::dvec3 DirectionalLight::getColor() const
{
	return color;
}

glm::dvec3 DirectionalLight::getDirection(const glm::dvec3& P) const
{
	return -orientation;
}

double PointLight::distanceAttenuation(const glm::dvec3& P) const
{

	// YOUR CODE HERE

	// You'll need to modify this method to attenuate the intensity 
	// of the light based on the distance between the source and the 
	// point P.  For now, we assume no attenuation and just return 1.0
	double d = glm::length(position - P);
	return min(1.0 / (constantTerm + linearTerm * d + quadraticTerm * d * d), 1.0);
}

glm::dvec3 PointLight::getColor() const
{
	return color;
}

glm::dvec3 PointLight::getDirection(const glm::dvec3& P) const
{
	return glm::normalize(position - P);
}


glm::dvec3 PointLight::shadowAttenuation(const ray& r, const glm::dvec3& p, double thresh) const
{
	if (glm::distance(r.getAtten(), glm::dvec3(0, 0, 0)) < thresh) {
		return glm::dvec3(0.0, 0.0, 0.0);
	}
	isect i;
	ray shadow_r = r;
	if (scene->intersect(shadow_r, i)) {
		if (i.getT() >= glm::length(position - shadow_r.getPosition())) {
			return glm::dvec3(1.0, 1.0, 1.0);
		}
		const Material& m = i.getMaterial();
		if (m.Trans()) {
			double t = i.getT();
			glm::dvec3 pos = shadow_r.at(t + RAY_EPSILON); // to get through the surface
			glm::dvec3 atten = glm::dvec3(1, 1, 1);
			bool leaving = glm::dot(r.getDirection(), i.getN()) > 0;
			if (leaving) {
				glm::dvec3 kt = m.kt(i);
				atten *= glm::dvec3(pow(kt.x, t), pow(kt.y, t), pow(kt.z, t));
			}
			ray recur(pos, r.getDirection(), r.getAtten(), ray::SHADOW);
			recur.setAtten(recur.getAtten() * atten);
			return atten * shadowAttenuation(recur, pos, thresh);
		}
		else {
			return glm::dvec3(0.0, 0.0, 0.0);
		}
	}
	return glm::dvec3(1.0, 1.0, 1.0);
}

#define VERBOSE 0

