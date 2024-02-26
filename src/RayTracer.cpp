// The main ray tracer.

#pragma warning (disable: 4786)

#include "RayTracer.h"
#include "scene/light.h"
#include "scene/material.h"
#include "scene/ray.h"

#include "parser/Tokenizer.h"
#include "parser/Parser.h"

#include "ui/TraceUI.h"
#include <cmath>
#include <algorithm>
#include <random>
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
#include <string.h> // for memset

#include <iostream>
#include <fstream>


#define RAN_DOUBLE(l, h) (((double) std::rand() / RAND_MAX) * ((h)-(l)) + (l))
#define RAN_INT(l, h)    ((int) (RAN_DOUBLE(0, (h)-(l)+l) + (l)))


using namespace std;
extern TraceUI* traceUI;

// Use this variable to decide if you want to print out
// debugging messages.  Gets set in the "trace single ray" mode
// in TraceGLWindow, for example.
bool debugMode = false;

std::default_random_engine generator;
std::uniform_real_distribution<float> distribution(0, 1);

void createCoordinateSystem(const glm::dvec3& N, glm::dvec3& Nt, glm::dvec3& Nb) {
    if (glm::abs(N.x) > glm::abs(N.y)) {
        Nt = glm::normalize(glm::dvec3(N.z, 0, -N.x));
    }
    else {
        Nt = glm::normalize(glm::dvec3(0, -N.z, N.y));
    }
    Nb = glm::cross(N, Nt);
}

glm::dvec3 uniformSampleHemisphere() {
    float r1 = distribution(generator);
    float r2 = distribution(generator);
    // cos(theta) = r1 = y
    // cos^2(theta) + sin^2(theta) = 1 -> sin(theta) = srtf(1 - cos^2(theta))
    float sinTheta = glm::sqrt(1 - r1 * r1);
    float phi = 2 * M_PI * r2;
    float x = sinTheta * glm::cos(phi);
    float z = sinTheta * glm::sin(phi);
    return glm::dvec3(x, r1, z);
}

glm::dvec3 randUnitVector() {
    double x = (double)rand() / RAND_MAX;
    double y = (double)rand() / RAND_MAX;
    double z = (double)rand() / RAND_MAX;
    return glm::normalize(glm::dvec3(x, y, z));
}

glm::dvec3 pathDir(const glm::dvec3& N) {
    glm::dvec3 Nt, Nb;
    createCoordinateSystem(N, Nt, Nb);
    glm::dvec3 local = uniformSampleHemisphere();
    return glm::dvec3(local.x * Nb.x + local.y * N.x + local.z * Nt.x,
        local.x * Nb.y + local.y * N.y + local.z * Nt.y,
        local.x * Nb.z + local.y * N.z + local.z * Nt.z);
}

// Trace a top-level ray through pixel(i,j), i.e. normalized window coordinates (x,y),
// through the projection plane, and out into the scene.  All we do is
// enter the main ray-tracing method, getting things started by plugging
// in an initial ray weight of (0.0,0.0,0.0) and an initial recursion depth of 0.
glm::dvec3 RayTracer::trace(double x, double y)
{
    // Clear out the ray cache in the scene for debugging purposes,
    if (TraceUI::m_debug)
    {
        scene->clearIntersectCache();		
    }

    ray r(glm::dvec3(0,0,0), glm::dvec3(0,0,0), glm::dvec3(1,1,1), ray::VISIBILITY);
    scene->getCamera().rayThrough(x,y,r);
    double dummy;
    glm::dvec3 ret = traceRaySecondary(r, traceUI->getThreshold(), traceUI->getDepth(), dummy);
    ret = glm::clamp(ret, 0.0, 1.0);
    return ret;
}

glm::dvec3 RayTracer::tracePixel(int i, int j)
{
    glm::dvec3 col(0,0,0);

    if( ! sceneLoaded() ) return col;

    unsigned char *pixel = buffer.data() + ( i + j * buffer_width ) * 3;


    double N = 100;
    for (int iter = 0; iter < N; iter++) {
        double x = (double(i) + std::rand() * 1.0 / RAND_MAX) / double(buffer_width);
        double y = (double(j) + std::rand() * 1.0 / RAND_MAX) / double(buffer_height);
        col += trace(x, y);
    }
    if (N != 0) {
        col /= N;
    }

    pixel[0] = (int)( 255.0 * min(max(col[0], 0.0), 1.0));
    pixel[1] = (int)( 255.0 * min(max(col[1], 0.0), 1.0));
    pixel[2] = (int)( 255.0 * min(max(col[2], 0.0), 1.0));
    return col;
}

#define VERBOSE 0

// direct shadow ray, shoot when not on a light
glm::dvec3 emit(isect i, Scene* scene, ray r, double thresh) {

    if (i.getMaterial().Light()) { // we are ending on a light
        return i.getMaterial().ke(i);
    }
    else { // we are not ending on a light, shoot shadow ray
        return i.getMaterial().shade(scene, r, i, thresh);
    }
}

double FresnelReflectAmount(double n1, double n2, glm::dvec3 normal, glm::dvec3 incident, double f0, double f90)
{
    // Schlick aproximation
    double r0 = (n1 - n2) / (n1 + n2);
    r0 *= r0;
    double cosX = -glm::dot(normal, incident);
    if (n1 > n2)
    {
        double n = n1 / n2;
        double sinT2 = n * n * (1.0 - cosX * cosX);
        // Total internal reflection
        if (sinT2 > 1.0)
            return f90;
        cosX = sqrt(1.0 - sinT2);
    }
    double x = 1.0 - cosX;
    double ret = r0 + (1.0 - r0) * x * x * x * x * x;

    // adjust reflect multiplier for object reflectivity
    return glm::mix(f0, f90, ret);
}


glm::dvec3 RayTracer::traceRaySecondary(ray& r, double thresh, int depth, double& t)
{
    isect i;
    //double N = 0;
    glm::dvec3 colorC = glm::dvec3(0.0, 0.0, 0.0);

    if (scene->intersect(r, i)) {
        colorC += emit(i, scene.get(), r, thresh);

        const Material& m = i.getMaterial();
        glm::dvec3 normal = i.getN();
        glm::dvec3 rayDir = r.getDirection();
        glm::dvec3 omega_i = pathDir(normal);
        glm::dvec3 ks = m.ks(i);
        double percentSpecular = ks.y;
        double percentTrans = ks.z;
        int rayType = 0; // 0 if diffuse, 1 if refl, 2 if trans
        glm::dvec3 kr = m.kr(i);
        glm::dvec3 kt = m.kt(i);
        double random = ((double)rand() / (RAND_MAX));
        double epsilonSign = 1;
        double roughness = m.shininess(i);
        double index = m.index(i);

        bool fromInside = (glm::dot(rayDir, normal) > 0);
        if (fromInside) normal = -normal;

        double rayProbability = 1.0;
        if (m.Refl())
        {
            percentSpecular = FresnelReflectAmount(fromInside ? index : 1.0, fromInside ? index : 1.0,
                rayDir, normal, percentSpecular, 1.0);

            percentTrans *= (1.0 - percentSpecular) / max(1.0 - ks.y, 0.001);
        }

        if (m.Refl() > 0.0 && random < percentSpecular)
        {
            glm::dvec3 specularRayDir = glm::reflect(rayDir, normal);
            rayType = 1;
            rayProbability = percentSpecular;
            omega_i = glm::normalize(roughness * omega_i + (1 - roughness) * specularRayDir);
        }
        else if (m.Trans() && random <= percentSpecular + percentTrans + 0.001)
        {
            rayType = 2;
            rayProbability = percentTrans;
            double eta = index;
            if (!fromInside) {
                eta = 1 / eta;
            }
            double cos_i = -glm::dot(normal, rayDir);
            double nsin_i = pow(eta, 2) * (1.0 - cos_i * cos_i);
            if (nsin_i > 1.0) { // total internal reflection TODO: might want to take out
                glm::dvec3 specularRayDir = glm::reflect(rayDir, -normal);
                omega_i = glm::normalize(roughness * omega_i + (1 - roughness) * specularRayDir);
            }
            else { // normal refraction
                double cos_t = sqrt(1.0 - nsin_i);
                glm::dvec3 transRayDir = glm::normalize(eta * rayDir + (eta * cos_i - cos_t) * normal);

                if (glm::length(transRayDir) < 0.001) {
                    cout << "refraction 0 bad!!" << endl;
                }
                omega_i = glm::normalize(roughness * (-normal + randUnitVector()) + (1 - roughness) * 2 * transRayDir);
                epsilonSign = -1;
            }
        }
        else
        {
            rayProbability = 1.0 - (percentSpecular + percentTrans);
        }

        // numerical problems can cause rayProbability to become small enough to cause a divide by zero.
        rayProbability = glm::max(rayProbability, 0.001);
        
        double prob_i = 1.0 / (2.0 * M_PI);
        glm::dvec3 throughput = 0.99 * r.getAtten();
        double rr_prob = glm::max(glm::max(throughput.x, throughput.y), throughput.z);
        double cosTheta = glm::dot(normal, omega_i);
        glm::dvec3 kd = m.kd(i);
        glm::dvec3 k = kd;
        if (rayType == 1) {
            k = kr;
        }
        else if (rayType == 2) {
            k = glm::dvec3(1.0, 1.0, 1.0);
        }
        glm::dvec3 brdf = k / M_PI;
        double maxk = glm::max(glm::max(k.x, k.y), k.z);
        if (maxk == 0 || cosTheta == 0) {
            return colorC;
        }
        if (depth >= 3) {
            throughput *= maxk * cosTheta;
            if (cosTheta == 0) cout << "cosTheta 0" << endl;
            //throughput *= 0.8;
        }

        double time = i.getT();
        ray next_r(r.at(time - RAY_EPSILON * epsilonSign), omega_i, throughput, ray::VISIBILITY);
        isect next_i;
        glm::dvec3 next_color = glm::dvec3(0.0, 0.0, 0.0);
        if ((double)std::rand() / RAND_MAX < rr_prob) {
            next_color = traceRaySecondary(next_r, thresh, depth + 1, t);
        }
        if (fromInside) {
            next_color *= glm::pow(m.kt(i), glm::dvec3(time, time, time));
        }
        colorC += brdf * next_color * cosTheta / (prob_i * rr_prob) / rayProbability;
        if (prob_i == 0) {
            cout << "prob_i /0 bad" << endl;
        }
        if (rr_prob == 0) {
            cout << "rr_prob /0 bad" << endl;
        }
        if (rayProbability == 0) {
            cout << "rayProbability /0 bad" << endl;
        }
    }
    else if(traceUI->cubeMap()) {
        colorC = traceUI->getCubeMap()->getColor(r);
    }
    return colorC;
}

/*
 * RayTracer::traceImage
 *
 *	Trace the image and store the pixel data in RayTracer::buffer.
 *
 *	Arguments:
 *		w:	width of the image buffer
 *		h:	height of the image buffer
 *
 */
void RayTracer::traceImage(int w, int h)
{
    // Always call traceSetup before rendering anything.
    traceSetup(w, h);

    int index = 0;
    int step = 64;
    int totalThreads = w / step + (w % step != 0);
    while (index * step < w) {
        threadsState.push_back(false);
        threadsVector.push_back(std::thread([index, w, h, this, step]() {
            for (int i = index * step; i < glm::min((index + 1) * step, w); i++) {
                for (int j = 0; j < h; j++) {
                    tracePixel(i, j);
                }
            }
            threadsState.at(index) = true;
            }));
        index++;
    }
}

RayTracer::RayTracer()
    : scene(nullptr), buffer(0), threadsState(0), threadsVector(0), thresh(0), buffer_width(0), buffer_height(0), m_bBufferReady(false)
{
}

RayTracer::~RayTracer()
{
}

void RayTracer::getBuffer( unsigned char *&buf, int &w, int &h )
{
    buf = buffer.data();
    w = buffer_width;
    h = buffer_height;
}

double RayTracer::aspectRatio()
{
    return sceneLoaded() ? scene->getCamera().getAspectRatio() : 1;
}

bool RayTracer::loadScene(const char* fn)
{

    ifstream ifs(fn);
    if( !ifs ) {
        string msg( "Error: couldn't read scene file " );
        msg.append( fn );
        traceUI->alert( msg );
        return false;
    }

    // Strip off filename, leaving only the path:
    string path( fn );
    if (path.find_last_of( "\\/" ) == string::npos)
        path = ".";
    else
        path = path.substr(0, path.find_last_of( "\\/" ));

    // Call this with 'true' for debug output from the tokenizer
    Tokenizer tokenizer( ifs, false );
    Parser parser( tokenizer, path );
    try {
        scene.reset(parser.parseScene());
    }
    catch( SyntaxErrorException& pe ) {
        traceUI->alert( pe.formattedMessage() );
        return false;
    } catch( ParserException& pe ) {
        string msg( "Parser: fatal exception " );
        msg.append( pe.message() );
        traceUI->alert( msg );
        return false;
    } catch( TextureMapException e ) {
        string msg( "Texture mapping exception: " );
        msg.append( e.message() );
        traceUI->alert( msg );
        return false;
    }

    if (!sceneLoaded())
        return false;

    scene->buildBVH();

    return true;
}

void RayTracer::traceSetup(int w, int h)
{
    size_t newBufferSize = w * h * 3;
    if (newBufferSize != buffer.size()) {
        bufferSize = (int) newBufferSize;
        buffer.resize(bufferSize);
    }
    buffer_width = w;
    buffer_height = h;
    std::fill(buffer.begin(), buffer.end(), 0);
    m_bBufferReady = true;
    threadsState = std::vector<bool>(0);
    threadsVector = std::vector<std::thread>(0);

    /*
     * Sync with TraceUI
     */

    threads = traceUI->getThreads();
    block_size = traceUI->getBlockSize();
    thresh = traceUI->getThreshold();
    samples = traceUI->getSuperSamples();
    aaThresh = traceUI->getAaThreshold();

    
    // std::cout << "we have path light length " << scene->getAllPathLights().size() << " with bounding box " << scene->getAllPathLights().at(0)->getBoundingBox().getMax() << " and " << scene->getAllPathLights().at(0)->getBoundingBox().getMin() << endl;
    // std::cout << randLightPoints(scene.get()).at(0) << endl;
}



glm::dvec3 RayTracer::asRecur(double m1, double n1, double m2, double n2, int depth, bool intensity) {
    glm::dvec3 col(0, 0, 0);

    if (depth >= 5) { // max recursion depth, at 5 it shoots 5^5 = more than 3000 rays per pixel
        return col;
    }

    // cout << "calling asRecur for " << m1 << " " << n1 << " " << m2 << " " << n2 << " with depth " << depth << endl;

    double x1 = double(m1) / double(buffer_width);
    double y1 = double(n1) / double(buffer_height);
    double x2 = double(m2) / double(buffer_width);
    double y2 = double(n2) / double(buffer_height);

    double xcenter = double((m1 + m2) / 2.0) / double(buffer_width);
    double ycenter = double((n1 + n2) / 2.0) / double(buffer_height);

    glm::dvec3 topleft = trace(x1, y1);
    glm::dvec3 topright = trace(x2, y1);
    glm::dvec3 bottomleft = trace(x1, y2);
    glm::dvec3 bottomright = trace(x2, y2);
    glm::dvec3 center = trace(xcenter, ycenter);

    if (glm::distance(topleft, center) >= aaThresh) {
        col += asRecur(m1, n1, (m1 + m2) / 2.0, (n1 + n2) / 2.0, depth + 1, intensity);
    }
    else {
        col += intensity ? glm::dvec3(0, 0, 0) : std::pow(0.25, depth) * (topleft + center) / 2.0;
    }

    if (glm::distance(topright, center) >= aaThresh) {
        col += asRecur((m1 + m2) / 2.0, n1, m2, (n1 + n2) / 2.0, depth + 1, intensity);
    }
    else {
        col += intensity ? glm::dvec3(0, 0, 0) : std::pow(0.25, depth) * (topright + center) / 2.0;
    }

    if (glm::distance(bottomleft, center) >= aaThresh) {
        col += asRecur(m1, (n1 + n2) / 2.0, (m1 + m2) / 2.0, n2, depth + 1, intensity);
    }
    else {
        col += intensity ? glm::dvec3(0, 0, 0) : std::pow(0.25, depth) * (bottomleft + center) / 2.0;
    }

    if (glm::distance(bottomright, center) >= aaThresh) {
        col += asRecur((m1 + m2) / 2.0, (n1 + n2) / 2.0, m2, n2, depth + 1, intensity);
    }
    else {
        col += intensity ? glm::dvec3(0, 0, 0) : std::pow(0.25, depth) * (bottomright + center) / 2.0;
    }

    return col + (intensity ? glm::dvec3(25.0 / 255.0, 25.0 / 255.0, 25.0 / 255.0) : glm::dvec3(0, 0, 0)); // for each ray shot, we increase the intensity by 5 rgb values out of 256
}

struct point {
    double x;
    double y;
    point(double xx, double yy) {
        x = xx;
        y = yy;
    }
    point() {
        x = 0;
        y = 0;
    }
};

glm::dvec3 RayTracer::aaTracePixel(int i, int j, int n)
{
    glm::dvec3 col(0, 0, 0);
    unsigned char* pixel = buffer.data() + (i + j * buffer_width) * 3;

    bool intensity = false; // do you want to generate an intensity image (for adaptive supersampling) or a normal image?

    if (!sceneLoaded()) return col;

    if (type == aS) { // adaptive supersampling
        double m1 = i;
        double n1 = j;
        double m2 = i + 1.0;
        double n2 = j + 1.0;

        col += asRecur(m1, n1, m2, n2, 1, intensity);
    }
    else {
        if (type == jS || type == avg) {
            for (double a = 0.0; a < n; a++) {
                for (double b = 0.0; b < n; b++) {
                    double a1 = i + a / n + (type == jS ? std::rand() / RAND_MAX * a / n : 0);
                    double j1 = j + b / n + (type == jS ? std::rand() / RAND_MAX * b / n : 0);

                    double x = double(a1) / double(buffer_width);
                    double y = double(j1) / double(buffer_height);

                    col += intensity ? glm::dvec3(0, 0, 0) : trace(x, y);
                }
            }
            if (intensity) {
                col += glm::dvec3((5.0 * std::pow(n, 2)) / 255.0, (5.0 * std::pow(n, 2)) / 255.0, (5.0 * std::pow(n, 2)) / 255.0); // for each ray shot, we increase the intensity by 5 rgb values out of 256
            }
            else {
                col /= std::pow(double(n), 2);
            }
        }
        else if (type == nR) {
            std::vector<double> pointx;
            std::vector<double> pointy;
            for (double a = 0.0; a < std::pow(n, 2); a++) {
                double a1 = i + a / std::pow(n, 2) + std::rand() / RAND_MAX * 1 / std::pow(n, 2);
                double a2 = j + a / std::pow(n, 2) + std::rand() / RAND_MAX * 1 / std::pow(n, 2);
                pointx.push_back(a1);
                pointy.push_back(a2);
            }
            std::random_shuffle(pointx.begin(), pointx.end());
            std::random_shuffle(pointy.begin(), pointy.end());

            for (int a = 0; a < pointx.size(); a++) {
                double x = double(pointx[a]) / double(buffer_width);
                double y = double(pointy[a]) / double(buffer_height);
                col += trace(x, y);
            }
            col /= std::pow(n, 2);
        }
        else if (type == mjS) {
            std::vector<point> points (n * n);
            for (double a = 0.0; a < n; a++) {
                for (double b = 0.0; b < n; b++) {
                    points[(int) a * (int) n + (int) b] = point(a / n + b / std::pow(n, 2) + std::rand() / RAND_MAX * 1 / std::pow(n, 2),
                        b / n + a / std::pow(n, 2) + std::rand() / RAND_MAX * 1 / std::pow(n, 2));
                }
            }

            for (int a = 0; a < n; a++) {
                for (int b = 0; b < n; b++) {
                    double tempx;
                    int randx = std::rand() / RAND_MAX * (n - 1 - b) + b;
                    //if (randx < b || randx > n - 1) {
                    //    std::cout << "ERROR" << endl;
                    //}
                    tempx = points[a * n + b].x;
                    points[a * n + b].x = points[a * n + randx].x;
                    points[a * n + randx].x = tempx;

                    double tempy;
                    int randy = std::rand() / RAND_MAX * (n-1-b) + b;
                    //if (randy < b || randy > n - 1) {
                    //   std::cout << "ERROR" << endl;
                    //}
                    tempy = points[b * n + a].y;
                    points[b * n + a].y = points[b * n + randy].y;
                    points[b * n + randy].y = tempy;
                }
            }

            for (int a = 0; a < points.size(); a++) {
                double x = double(i + points[a].x) / double(buffer_width);
                double y = double(j + points[a].y) / double(buffer_height);
                col += trace(x, y);
            }
            col /= std::pow(n, 2);
        }
    }
    

    pixel[0] = (int)(255.0 * min(max(col[0], 0.0), 1.0));
    pixel[1] = (int)(255.0 * min(max(col[1], 0.0), 1.0));
    pixel[2] = (int)(255.0 * min(max(col[2], 0.0), 1.0));



    return col;
}

int RayTracer::aaImage()
{
    
    for (int i = 0; i < buffer_width; i++) {
        for (int j = 0; j < buffer_height; j++) {
            aaTracePixel(i, j, samples);
        }
    }
    return 0;
}

bool RayTracer::checkRender()
{
    for (auto state : threadsState) {
        if (!state) return false;
    }
    return true;
}

void RayTracer::waitRender()
{
    for (std::thread& one: threadsVector) {
       one.join();
    }
}


glm::dvec3 RayTracer::getPixel(int i, int j)
{
    unsigned char *pixel = buffer.data() + ( i + j * buffer_width ) * 3;
    return glm::dvec3((double)pixel[0]/255.0, (double)pixel[1]/255.0, (double)pixel[2]/255.0);
}

void RayTracer::setPixel(int i, int j, glm::dvec3 color)
{
    unsigned char *pixel = buffer.data() + ( i + j * buffer_width ) * 3;

    pixel[0] = (int)( 255.0 * color[0]);
    pixel[1] = (int)( 255.0 * color[1]);
    pixel[2] = (int)( 255.0 * color[2]);
}
