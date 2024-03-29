#include "trimesh.h"
#include <assert.h>
#include <float.h>
#include <string.h>
#include <algorithm>
#include <cmath>
#include "../ui/TraceUI.h"
extern TraceUI* traceUI;

using namespace std;

Trimesh::~Trimesh()
{
	for (auto m : materials)
		delete m;
	for (auto f : faces)
		delete f;
}

// must add vertices, normals, and materials IN ORDER
void Trimesh::addVertex(const glm::dvec3& v)
{
	vertices.emplace_back(v);
}

void Trimesh::addMaterial(Material* m)
{
	materials.emplace_back(m);
}

void Trimesh::addNormal(const glm::dvec3& n)
{
	normals.emplace_back(n);
}

// Returns false if the vertices a,b,c don't all exist
bool Trimesh::addFace(int a, int b, int c)
{
	int vcnt = vertices.size();

	if (a >= vcnt || b >= vcnt || c >= vcnt)
		return false;

	TrimeshFace* newFace = new TrimeshFace(
	        scene, new Material(*this->material), this, a, b, c);
	newFace->setTransform(this->transform);
	if (!newFace->degen)
		faces.push_back(newFace);
	else
		delete newFace;

	// Don't add faces to the scene's object list so we can cull by bounding
	// box
	return true;
}

// Check to make sure that if we have per-vertex materials or normals
// they are the right number.
const char* Trimesh::doubleCheck()
{
	if (!materials.empty() && materials.size() != vertices.size())
		return "Bad Trimesh: Wrong number of materials.";
	if (!normals.empty() && normals.size() != vertices.size())
		return "Bad Trimesh: Wrong number of normals.";

	return 0;
}

bool Trimesh::intersectLocal(ray& r, isect& i) const
{
	bool have_one = false;
	for (auto face : faces) {
		isect cur;
		if (face->intersectLocal(r, cur)) {
			if (!have_one || (cur.getT() < i.getT())) {
				i = cur;
				have_one = true;
			}
		}
	}
	if (!have_one)
		i.setT(1000.0);
	return have_one;
}

bool TrimeshFace::intersect(ray& r, isect& i) const
{
	return intersectLocal(r, i);
}

// Intersect ray r with the triangle abc.  If it hits returns true,
// and put the parameter in t and the barycentric coordinates of the
// intersection in u (alpha) and v (beta).
bool TrimeshFace::intersectLocal(ray& r, isect& i) const
{
	// YOUR CODE HERE
	//
	// FIXME: Add ray-trimesh intersection

	glm::dvec3 a_coords = parent->vertices[ids[0]];
	glm::dvec3 b_coords = parent->vertices[ids[1]];
	glm::dvec3 c_coords = parent->vertices[ids[2]];
	glm::dvec3 vab = (b_coords - a_coords);
	glm::dvec3 vac = (c_coords - a_coords);
	glm::dvec3 vcb = (b_coords - c_coords);

	double t = glm::dot(a_coords - r.getPosition(), normal) / glm::dot(r.getDirection(), normal);
	if (t < 0) return false;
	i.setT(t);
	glm::dvec3 c = r.at(t);
	glm::dvec3 vacenter = c - a_coords;
	double u, v;
	glm::dmat2x2 mat = glm::inverse(glm::dmat2x2(glm::dot(vab, vab), glm::dot(vac, vab), glm::dot(vab, vac), glm::dot(vac, vac)));
	glm::dvec2 res = mat * glm::dvec2(glm::dot(vacenter, vab), glm::dot(vacenter, vac));
	u = res.x;
	v = res.y;
	if (!(u >= 0 && u <= 1 && v >= 0 && v <= 1 && u + v <= 1)) return false;

	i.setUVCoordinates(glm::dvec2(u, v));
	i.setBary(glm::dvec3(u, v, 1-u-v));

	if (parent->normals.size() != 0) {
		glm::dvec3 a_normal = parent->normals[ids[0]];
		glm::dvec3 b_normal = parent->normals[ids[1]];
		glm::dvec3 c_normal = parent->normals[ids[2]];
		i.setN(a_normal * (1 - u - v) + b_normal * u + c_normal * v);
	}
	else {
		i.setN(normal);
	}

	if (parent->materials.size() != 0) {
		Material* a_m = parent->materials[ids[0]];
		Material* b_m = parent->materials[ids[1]];
		Material* c_m = parent->materials[ids[2]];
		Material res = (1 - u - v) * *a_m;
		res += u * *b_m;
		res += v * *c_m;
		i.setMaterial(res);
	}
	else {
		i.setMaterial(parent->getMaterial());
	}
	return true;
}

// Once all the verts and faces are loaded, per vertex normals can be
// generated by averaging the normals of the neighboring faces.
void Trimesh::generateNormals()
{
	int cnt = vertices.size();
	normals.resize(cnt);
	std::vector<int> numFaces(cnt, 0);

	for (auto face : faces) {
		glm::dvec3 faceNormal = face->getNormal();

		for (int i = 0; i < 3; ++i) {
			normals[(*face)[i]] += faceNormal;
			++numFaces[(*face)[i]];
		}
	}

	for (int i = 0; i < cnt; ++i) {
		if (numFaces[i])
			normals[i] /= numFaces[i];
	}

	vertNorms = true;
}

