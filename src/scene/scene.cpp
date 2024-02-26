#include <cmath>

#include "scene.h"
#include "light.h"
#include "kdTree.h"
#include "../ui/TraceUI.h"
#include <glm/gtx/extended_min_max.hpp>
#include <iostream>
#include <glm/gtx/io.hpp>

using namespace std;

bool Geometry::intersect(ray& r, isect& i) const {
	double tmin, tmax;
	if (hasBoundingBoxCapability() && !(bounds.intersect(r, tmin, tmax))) return false;
	// Transform the ray into the object's local coordinate space
	glm::dvec3 pos = transform->globalToLocalCoords(r.getPosition());
	glm::dvec3 dir = transform->globalToLocalCoords(r.getPosition() + r.getDirection()) - pos;
	double length = glm::length(dir);
	dir = glm::normalize(dir);
	// Backup World pos/dir, and switch to local pos/dir
	glm::dvec3 Wpos = r.getPosition();
	glm::dvec3 Wdir = r.getDirection();
	r.setPosition(pos);
	r.setDirection(dir);
	bool rtrn = false;
	if (intersectLocal(r, i))
	{
		// Transform the intersection point & normal returned back into global space.
		i.setN(transform->localToGlobalCoordsNormal(i.getN()));
		i.setT(i.getT()/length);
		rtrn = true;
	}
	// Restore World pos/dir
	r.setPosition(Wpos);
	r.setDirection(Wdir);
	return rtrn;
}

bool Geometry::hasBoundingBoxCapability() const {
	// by default, primitives do not have to specify a bounding box.
	// If this method returns true for a primitive, then either the ComputeBoundingBox() or
    // the ComputeLocalBoundingBox() method must be implemented.

	// If no bounding box capability is supported for an object, that object will
	// be checked against every single ray drawn.  This should be avoided whenever possible,
	// but this possibility exists so that new primitives will not have to have bounding
	// boxes implemented for them.
	return true; // check if all geometries have bounding box
}

void Geometry::ComputeBoundingBox() {
    // take the object's local bounding box, transform all 8 points on it,
    // and use those to find a new bounding box.

    BoundingBox localBounds = ComputeLocalBoundingBox();
        
    glm::dvec3 min = localBounds.getMin();
    glm::dvec3 max = localBounds.getMax();

    glm::dvec4 v, newMax, newMin;

    v = transform->localToGlobalCoords( glm::dvec4(min[0], min[1], min[2], 1) );
    newMax = v;
    newMin = v;
    v = transform->localToGlobalCoords( glm::dvec4(max[0], min[1], min[2], 1) );
    newMax = glm::max(newMax, v);
    newMin = glm::min(newMin, v);
    v = transform->localToGlobalCoords( glm::dvec4(min[0], max[1], min[2], 1) );
    newMax = glm::max(newMax, v);
    newMin = glm::min(newMin, v);
    v = transform->localToGlobalCoords( glm::dvec4(max[0], max[1], min[2], 1) );
    newMax = glm::max(newMax, v);
    newMin = glm::min(newMin, v);
    v = transform->localToGlobalCoords( glm::dvec4(min[0], min[1], max[2], 1) );
    newMax = glm::max(newMax, v);
    newMin = glm::min(newMin, v);
    v = transform->localToGlobalCoords( glm::dvec4(max[0], min[1], max[2], 1) );
    newMax = glm::max(newMax, v);
    newMin = glm::min(newMin, v);
    v = transform->localToGlobalCoords( glm::dvec4(min[0], max[1], max[2], 1) );
    newMax = glm::max(newMax, v);
    newMin = glm::min(newMin, v);
    v = transform->localToGlobalCoords( glm::dvec4(max[0], max[1], max[2], 1) );
    newMax = glm::max(newMax, v);
    newMin = glm::min(newMin, v);
		
    bounds.setMax(glm::dvec3(newMax));
    bounds.setMin(glm::dvec3(newMin));
}

Scene::Scene()
{
	ambientIntensity = glm::dvec3(0, 0, 0);
}

Scene::~Scene()
{
}

BoundingBox* computeBoundingBox(std::vector<Geometry*>& objects) {
	BoundingBox* res = new BoundingBox();
	for (const auto& obj : objects) {
		obj->ComputeBoundingBox();
		res->merge(obj->getBoundingBox());
	}
	return res;
}

BBoxNode* buildBVHSub(std::vector<Geometry*>& objects) {
	if (objects.size() == 1) { // leaf case
		objects[0]->ComputeBoundingBox();
		BBoxNode* res = new BBoxNode(objects[0]->getBoundingBox());
		/*if (objects[0]->isTrimesh()) {
			auto faces = objects[0]->getFaces();
			for (auto face : faces) {
				face->ComputeBoundingBox();
			}
			res->setL(buildBVHSub(faces));
			BBoxNode* leaf = new BBoxNode(objects[0]->getBoundingBox());
			res->setR(leaf);
			res->setLeaf(true);
			res->setGeo(objects[0]);
		}
		else { */
			res->setLeaf(true);
			res->setGeo(objects[0]);
		// }
		return res;
	}
	BoundingBox* bbox = computeBoundingBox(objects);
	BBoxNode* res = new BBoxNode(bbox);
	double x = bbox->getMax().x - bbox->getMin().x;
	double y = bbox->getMax().y - bbox->getMin().y;
	double z = bbox->getMax().z - bbox->getMin().z;

	if (x > y && x > z) {
		sort(objects.begin(), objects.end(), [](const Geometry* a, const Geometry* b) {
			BoundingBox a_box = a->getBoundingBox();
			BoundingBox b_box = b->getBoundingBox();
			return (a_box.getMin() + a_box.getMax()).x < (b_box.getMin() + b_box.getMax()).x;
			});
	}
	else if (y > x && y > z) {
		sort(objects.begin(), objects.end(), [](const Geometry* a, const Geometry* b) {
			BoundingBox a_box = a->getBoundingBox();
		BoundingBox b_box = b->getBoundingBox();
		return (a_box.getMin() + a_box.getMax()).y < (b_box.getMin() + b_box.getMax()).y;
			});
	}
	else {
		sort(objects.begin(), objects.end(), [](const Geometry* a, const Geometry* b) {
			BoundingBox a_box = a->getBoundingBox();
		BoundingBox b_box = b->getBoundingBox();
		return (a_box.getMin() + a_box.getMax()).z < (b_box.getMin() + b_box.getMax()).z;
			});
	}
	std::vector<Geometry*> one = std::vector<Geometry*>(objects.begin(), objects.end() - objects.size() / 2);
	std::vector<Geometry*> two = std::vector<Geometry*>(objects.end() - objects.size() / 2, objects.end());
	if (one.size() == 0 || two.size() == 0) {
		cout << "why" << endl;
	}
	res->setL(buildBVHSub(one));
	res->setR(buildBVHSub(two));
	return res;
}

void Scene::buildBVH() {
	BoundingBox bbox = sceneBounds;
	root = new BBoxNode(&bbox);
	if (geometries.size() == 0) return;
	root->setL(buildBVHSub(geometries));
	/*if (objects.size() == 0) return;
	if (objects.size() == 1) {
		if (objects[0]->isTrimesh()) {
			auto faces = objects[0]->getFaces();
			for (auto face : faces) {
				face->ComputeBoundingBox();
			}
			root->setL(buildBVHSub(faces));
		}
		else {
			root->setLeaf(true);
			root->setGeo(objects[0].get());
		}
		return;		
	}
	std::vector<Geometry*> one;
	std::vector<Geometry*> two;
	double x = bbox.getMax().x - bbox.getMin().x;
	double y = bbox.getMax().y - bbox.getMin().y;
	double z = bbox.getMax().z - bbox.getMin().z;
	if (x > y && x > z) {
		sort(objects.begin(), objects.end(), [](const unique_ptr<Geometry>& a, const unique_ptr<Geometry>& b) {
			BoundingBox a_box = a->getBoundingBox();
		BoundingBox b_box = b->getBoundingBox();
		return (a_box.getMin() + a_box.getMax()).x < (b_box.getMin() + b_box.getMax()).x;
			});
	}
	else if (y > x && y > z) {
		sort(objects.begin(), objects.end(), [](const unique_ptr<Geometry>& a, const unique_ptr<Geometry>& b) {
			BoundingBox a_box = a->getBoundingBox();
		BoundingBox b_box = b->getBoundingBox();
		return (a_box.getMin() + a_box.getMax()).y < (b_box.getMin() + b_box.getMax()).y;
			});
	}
	else {
		sort(objects.begin(), objects.end(), [](const unique_ptr<Geometry>& a, const unique_ptr<Geometry>& b) {
			BoundingBox a_box = a->getBoundingBox();
		BoundingBox b_box = b->getBoundingBox();
		return (a_box.getMin() + a_box.getMax()).z < (b_box.getMin() + b_box.getMax()).z;
			});
	}
	for (int i = 0; i < objects.size(); i++) {
		if (i < objects.size() / 2) {
			one.push_back(objects[i].get());
		}
		else {
			two.push_back(objects[i].get());
		}
	}
	root->setL(buildBVHSub(one));
	root->setR(buildBVHSub(two));*/
}

void Scene::add(Geometry* obj) {
	obj->ComputeBoundingBox();
	sceneBounds.merge(obj->getBoundingBox());
	objects.emplace_back(obj);
	if (obj->isTrimesh()) {
		auto faces = obj->getFaces();
		for (auto face : faces) {
			face->ComputeLocalBoundingBox();
			geometries.emplace_back(face);
		}
	}
	else {
		geometries.emplace_back(obj);
	}
}

void Scene::add(Light* light)
{
	lights.emplace_back(light);
}


void Scene::addPathLight(Geometry* light, glm::dvec3 light_color)
{
	path_lights.emplace_back(std::pair<Geometry*, glm::dvec3>(light, light_color));
}

std::vector<std::pair<glm::dvec3, glm::dvec3>> Scene::randLightPoints() {
	std::vector<std::pair<Geometry*, glm::dvec3>> pathlights = this->getAllPathLights();
	std::vector<std::pair<glm::dvec3, glm::dvec3>> rands;
	for (int i = 0; i < pathlights.size(); i++) {
		glm::dvec3 min = pathlights.at(i).first->getBoundingBox().getMin();
		glm::dvec3 max = pathlights.at(i).first->getBoundingBox().getMax();
		double r1 = std::rand() * 1.0 / RAND_MAX;
		double r2 = std::rand() * 1.0 / RAND_MAX;
		double r3 = std::rand() * 1.0 / RAND_MAX;

		double x = (max.x - min.x) * r1 + min.x;
		double y = (max.y - min.y) * r2 + min.y;
		double z = (max.z - min.z) * r3 + min.z;

		rands.emplace_back(std::pair<glm::dvec3, glm::dvec3>(glm::dvec3(x, y, z), pathlights.at(i).second));
	}
	return rands;
}

void traverse(BBoxNode* bboxNode, ray& r, isect& i, bool* have_one, double& tmin, double& tmax) {
	if (bboxNode != nullptr) {
		if (bboxNode->isLeaf()) {
			isect cur;
			if (bboxNode->getGeo()->intersect(r, cur)) {
				if (cur.getT() > 0 && (!(*have_one) || cur.getT() < i.getT())) {
					i = cur;
					*have_one = true;
				}
			}
			return;
		}
		BBoxNode* left = bboxNode->getL();
		bool left_hit = left != nullptr && (left->isLeaf() || left->get()->intersect(r, tmin, tmax));
		BBoxNode* right = bboxNode->getR();
		bool right_hit = right != nullptr && (right->isLeaf() || right->get()->intersect(r, tmin, tmax));
		if (left_hit) {
			traverse(left, r, i, have_one, tmin, tmax);
		}
		if (right_hit) {
			traverse(right, r, i, have_one, tmin, tmax);
		}
	}
	return;
}

// Get any intersection with an object.  Return information about the 
// intersection through the reference parameter.
bool Scene::intersect(ray& r, isect& i) const {
	double tmin = 0.0;
	double tmax = 0.0;
	bool have_one = false;
	BBoxNode* cur = root;
	if (sceneBounds.intersect(r, tmin, tmax)) {
		traverse(root, r, i, &have_one, tmin, tmax);
	}
	/*for(const auto& obj : objects) {
		isect cur;
		if( obj->intersect(r, cur) ) {
			if(!have_one || (cur.getT() < i.getT())) {
				i = cur;
				have_one = true;
			}
		}
	}*/
	if(!have_one)
		i.setT(1000.0);
	// if debugging,
	if (TraceUI::m_debug)
	{
		addToIntersectCache(std::make_pair(new ray(r), new isect(i)));
	}
	return have_one;
}

TextureMap* Scene::getTexture(string name) {
	auto itr = textureCache.find(name);
	if (itr == textureCache.end()) {
		textureCache[name].reset(new TextureMap(name));
		return textureCache[name].get();
	}
	return itr->second.get();
}


