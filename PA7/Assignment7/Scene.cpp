//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    if (depth >= 20)
    {
        return Vector3f();
    }
    // TO DO Implement Path Tracing Algorithm here
    auto result = intersect(ray);

    // 没射到物体
    if (!result.happened)
    {
        return backgroundColor;
    }
    
    // 射到灯了
    if (result.obj->hasEmit())
    {
        return result.m->getEmission();
    }
    Vector3f light_r;
    float light_pdf = 0;
    Intersection light;
    sampleLight(light, light_pdf);
    if (light_pdf != 0) {  // 有光线
        Vector3f direct_to_light = light.coords - result.coords;
        float light_to_p_distance = direct_to_light.norm();
        direct_to_light = direct_to_light.normalized();
        Ray light_ray(result.coords, direct_to_light);
        auto light_result = intersect(light_ray);
        if (light_result.happened && light_result.obj->hasEmit())
        {
            float cos_theta = dotProduct(result.normal, direct_to_light);
            float cos_theta_x = dotProduct(light_result.normal, -direct_to_light);
            auto L_i = light_result.m->getEmission();
            auto f_r = result.m->eval(ray.direction, direct_to_light, result.normal);
            light_r = L_i * f_r * cos_theta * cos_theta / (light_to_p_distance * light_to_p_distance) / light_pdf;
    
        }
    }

    float p_rr = get_random_float();
    if (p_rr > RussianRoulette)
    {
        return Vector3f();
    }

    Vector3f obj_r;
    Vector3f wo = -ray.direction;
    Vector3f wi = result.m->sample(wo, result.normal);
    auto out_ray = Ray(result.coords, wi);
    auto obj_result = intersect(out_ray);
    if (obj_result.happened && !obj_result.m->hasEmission())
    {
        obj_r = castRay(out_ray, depth+1)
            * result.m->eval(wo, wi, result.normal)
            * dotProduct(wi, result.normal)
            / result.m->pdf(wi, wo, result.normal)
            / RussianRoulette;
    }
    return light_r + obj_r;
}
