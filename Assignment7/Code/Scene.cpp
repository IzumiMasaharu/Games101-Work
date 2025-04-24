#include "Scene.hpp"
#include <random>

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::SAH);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) 
        if (objects[k]->hasEmit())
            emit_area_sum += objects[k]->getArea();

    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// TODO MISSION
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    //求交
    Intersection intersection = intersect(ray);

    if (!intersection.happened)
        return backgroundColor;
    if(intersection.m->hasEmission())
        return intersection.m->getEmission();

    intersection.normal = normalize(intersection.normal);

    // 对光源采样
    float pdfLight = 0; // 概率密度
    Intersection lightSamplePos;
    sampleLight(lightSamplePos, pdfLight);
    lightSamplePos.normal = normalize(lightSamplePos.normal);

    Vector3f lightDir = lightSamplePos.coords - intersection.coords;
    Ray lightRay(intersection.coords, normalize(lightDir));
    Intersection lightIntersection = intersect(lightRay);

    // 前置判断遮挡 避免边界噪音
    if (lightIntersection.happened && lightIntersection.m->hasEmission()) 
    {
        float lightDistance = lightDir.norm();
        lightDir = normalize(lightDir);
        float cosIntersectionTheta = dotProduct(intersection.normal, lightDir);
        float cosLightTheta = dotProduct(lightSamplePos.normal, -lightDir);
        if (cosIntersectionTheta > 0 && cosLightTheta > 0) 
        {
            Vector3f brdf = intersection.m->eval(ray.direction,lightDir, intersection.normal);
            // 蒙特卡洛
            intersection.emit = lightSamplePos.emit * brdf * cosIntersectionTheta * cosLightTheta / (lightDistance * lightDistance) / pdfLight;
        }
    }

    // Russian Roulette
    float P_RR = get_random_float();
    if (P_RR < RussianRoulette) {
        // 下一轮间接光照
        Vector3f newDir = intersection.m->sample(ray.direction, intersection.normal).normalized();
        
        Ray newRay(intersection.coords, newDir);
        Intersection newIntersection = intersect(newRay);
        float pdf = intersection.m->pdf(ray.direction, newDir, intersection.normal);
        if (pdf>0.01 && newIntersection.happened && !newIntersection.m->hasEmission()) 
        {
            // 计算新的光照
            Vector3f newBrdf = intersection.m->eval(ray.direction, newDir, intersection.normal);
            float cosIntersectionTheta = dotProduct(intersection.normal, newDir);
            Vector3f indirectLight = castRay(newRay, depth + 1) * newBrdf * cosIntersectionTheta / pdf;
            intersection.emit += indirectLight / RussianRoulette; // 满足数学期望为全局光照
        }
    }

    return intersection.emit;
}
