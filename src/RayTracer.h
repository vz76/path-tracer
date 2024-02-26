#ifndef __RAYTRACER_H__
#define __RAYTRACER_H__

#define MAX_THREADS 32

// The main ray tracer.

#include <time.h>
#include <glm/vec3.hpp>
#include <queue>
#include <thread>
#include "scene/cubeMap.h"
#include "scene/ray.h"
#include <mutex>

class Scene;
class Pixel {
public:
	Pixel(int i, int j, unsigned char* ptr) : ix(i), jy(j), value(ptr) {}

	int ix;
	int jy;
	unsigned char* value;
};


class RayTracer {
public:
	RayTracer();
	~RayTracer();

	glm::dvec3 tracePixel(int i, int j);
	glm::dvec3 traceRay(ray& r, double thresh, int depth,
	                    double& length);
	glm::dvec3 traceRaySecondary(ray& r, double thresh, int depth,
		double& length);

	glm::dvec3 getPixel(int i, int j);
	void setPixel(int i, int j, glm::dvec3 color);
	void getBuffer(unsigned char*& buf, int& w, int& h);
	double aspectRatio();

	void traceImage(int w, int h);
	glm::dvec3 asRecur(double m1, double n1, double m2, double n2, int depth, bool intensity);
	glm::dvec3 aaTracePixel(int i, int j, int n);
	int aaImage();
	bool checkRender();
	void waitRender();

	void traceSetup(int w, int h);

	bool loadScene(const char* fn);
	bool sceneLoaded() { return scene != 0; }

	void setReady(bool ready) { m_bBufferReady = ready; }
	bool isReady() const { return m_bBufferReady; }

	const Scene& getScene() { return *scene; }

	bool stopTrace;

private:
	glm::dvec3 trace(double x, double y);

	std::vector<unsigned char> buffer;
	std::vector<bool> threadsState;
	std::vector<std::thread> threadsVector;
	int buffer_width, buffer_height;
	int bufferSize;
	unsigned int threads;
	int block_size;
	double thresh;
	double aaThresh;
	int samples;
	enum aaType {avg, aS, jS, nR, mjS};
	aaType type;
	std::unique_ptr<Scene> scene;

	bool m_bBufferReady;

};

#endif // __RAYTRACER_H__
