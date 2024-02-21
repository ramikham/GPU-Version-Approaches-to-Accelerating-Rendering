//
// Created by ramik on 2/17/2024.
//

#ifndef UNTITLED_PRIMITIVE_H
#define UNTITLED_PRIMITIVE_H

#include "Ray.h"
#include "AABB.h"

class Material;

struct Intersection_Information
{
    float t;
    Vec3D p;
    Vec3D normal;
    Material *mat_ptr;
    bool front_face;                        // did the ray intersect the front face of the primitive?

    // Function to make the normal always point out against the ray.
    __device__ inline void set_face_normal(const Ray& r, const Vec3D& outward_normal) {
        if (dot_product(r.get_ray_direction(), outward_normal) > 0.0) {
            normal = -outward_normal;
            front_face = false;
        } else {
            normal = outward_normal;
            front_face = true;
        }
    }
};

class Primitive  {
public:
    __device__ virtual bool intersection(const Ray& r, float t_min, float t_max, Intersection_Information& rec) const = 0;
    __device__ virtual ~Primitive() {
        delete mat_ptr;
    }
    __device__ virtual bool bounding_box(float time_0, float time_1, AABB& surrounding_AABB) const = 0;
public:
    Material* mat_ptr = nullptr;
};


#endif //UNTITLED_PRIMITIVE_H
