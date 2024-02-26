#include "material.h"
#include "../ui/TraceUI.h"
#include "light.h"
#include "ray.h"
extern TraceUI* traceUI;

#include <glm/gtx/io.hpp>
#include <iostream>
#include "../fileio/images.h"

using namespace std;
extern bool debugMode;

Material::~Material()
{
}

// Apply the phong model to this point on the surface of the object, returning
// the color of that point.
glm::dvec3 Material::shade(Scene* scene, ray& r, const isect& i, double thresh) const
{
	// REDACTED for CS354R Ray Tracer project. 
}

TextureMap::TextureMap(string filename)
{
	data = readImage(filename.c_str(), width, height);
	if (data.empty()) {
		width = 0;
		height = 0;
		string error("Unable to load texture map '");
		error.append(filename);
		error.append("'.");
		throw TextureMapException(error);
	}
}

glm::dvec3 TextureMap::getMappedValue(const glm::dvec2& coord) const
{

	double x = coord.x;
	double y = coord.y;


	double u = x * (width - 1.0);
	int u1 = (int) std::floor(u);
	int u2 = (int) std::ceil(u);
	double v = y * (height - 1.0);
	int v1 = (int) std::floor(v);
	int v2 = (int) std::ceil(v);

	double localu = u - u1;
	double localv = v - v1;

	glm::dvec3 res = getPixelAt(u1, v1) * (1.0 - localu) * (1.0 - localv) +
		getPixelAt(u2, v1) * localu * (1.0 - localv) +
		getPixelAt(u1, v2) * (1.0 - localu) * localv +
		getPixelAt(u2, v2) * localu * localv;

	return res;
}

glm::dvec3 TextureMap::getPixelAt(int x, int y) const
{

//	glm::dvec3 RayTracer::getPixel(int i, int j)
//	{
//		unsigned char* pixel = buffer.data() + (i + j * buffer_width) * 3;
//		return glm::dvec3((double)pixel[0] / 255.0, (double)pixel[1] / 255.0, (double)pixel[2] / 255.0);
//	}

	//if (x < 0 || x >= width || y < 0 || y >= height) {
	//	cout << "ERROR " << x << " " << y << endl;
	//	return glm::dvec3(0.0, 0.0, 0.0);
	//}

	uint8_t* pixel = (uint8_t*) (data.data() + (x + y * width) * 3);

	return glm::dvec3((double)pixel[0] / 255.0, (double)pixel[1] / 255.0, (double)pixel[2] / 255.0);
}

glm::dvec3 MaterialParameter::value(const isect& is) const
{
	if (0 != _textureMap)
		return _textureMap->getMappedValue(is.getUVCoordinates());
	else
		return _value;
}

double MaterialParameter::intensityValue(const isect& is) const
{
	if (0 != _textureMap) {
		glm::dvec3 value(
		        _textureMap->getMappedValue(is.getUVCoordinates()));
		return (0.299 * value[0]) + (0.587 * value[1]) +
		       (0.114 * value[2]);
	} else
		return (0.299 * _value[0]) + (0.587 * _value[1]) +
		       (0.114 * _value[2]);
}
