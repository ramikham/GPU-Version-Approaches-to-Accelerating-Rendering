//
// Created by ramik on 2/17/2024.
//

#ifndef UNTITLED_SPHERE_H
#define UNTITLED_SPHERE_H

#include "Primitive.h"

class Sphere: public Primitive  {
public:
    __device__ Sphere() {}
    __device__ Sphere(Vec3D sphere_center, float sphere_radius, Material *sphere_material) : center(sphere_center), radius(sphere_radius), mat_ptr(sphere_material)  {};
    __device__ virtual bool intersection(const Ray& r, float tmin, float tmax, Intersection_Information& rec) const;

    __device__ bool bounding_box(float time_0, float time_1, AABB &surrounding_AABB) const override;

    Vec3D center;
    float radius;
    Material *mat_ptr;
};

__device__ bool Sphere::intersection(const Ray& r, float t_min, float t_max, Intersection_Information& rec) const {
    Vec3D oc = r.get_ray_origin() - center;
    float a = dot_product(r.get_ray_direction(), r.get_ray_direction());
    float b = dot_product(oc, r.get_ray_direction());
    float c = dot_product(oc, oc) - radius * radius;
    float discriminant = b*b - a*c;
    if (discriminant > 0) {
        float temp = (-b - sqrt(discriminant))/a;
        if (temp < t_max && temp > t_min) {
            rec.t = temp;
            rec.p = r.at(rec.t);
            rec.normal = (rec.p - center) / radius;
            rec.mat_ptr = mat_ptr;
            return true;
        }
        temp = (-b + sqrt(discriminant)) / a;
        if (temp < t_max && temp > t_min) {
            rec.t = temp;
            rec.p = r.at(rec.t);
            rec.normal = (rec.p - center) / radius;
            rec.mat_ptr = mat_ptr;
            return true;
        }
    }
    return false;
}

__device__ bool Sphere::bounding_box(float time_0, float time_1, AABB &surrounding_AABB) const {
    surrounding_AABB = AABB{
            center - Vec3D{ radius, radius, radius },
            center + Vec3D{ radius, radius, radius }
    };

    return true;
}

#endif //UNTITLED_SPHERE_H
