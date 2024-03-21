//
// Created by ramik on 2/17/2024.
//

#ifndef UNTITLED_SPHERE_H
#define UNTITLED_SPHERE_H

#include "Primitive.h"

class Sphere: public Primitive  {
public:
    // Constructors
    // -----------------------------------------------------------------------
    __device__ Sphere() {}
    __device__ Sphere(Vec3D sphere_center, float sphere_radius, Material *sphere_material) : center(sphere_center), radius(sphere_radius), mat_ptr(sphere_material)  {};

    // Overloaded Functions
    // -----------------------------------------------------------------------
    __device__ virtual bool intersection(const Ray& r, float tmin, float tmax, Intersection_Information& rec) const;
    __device__ bool bounding_box(float time_0, float time_1, AABB &surrounding_AABB) const override;

    // Data Members
    // -----------------------------------------------------------------------
    Vec3D center;
    float radius;
    Material *mat_ptr;
};

__device__ bool Sphere::intersection(const Ray& r, float t_min, float t_max, Intersection_Information& rec) const {
    // Does the ray intersect the sphere?

    // Get the A, B, C of the quadratic equation
    Vec3D OC = r.get_ray_origin() - center;
    auto A = r.get_ray_direction().length_squared();
    auto half_B = dot_product(OC, r.get_ray_direction());           // half_B is a shortcut
    auto C = OC.length_squared() - radius*radius;

    // Calculate the quadratic equation discriminant.
    auto discriminant = half_B * half_B - A * C;

    // If the discriminant is negative, the ray misses the sphere.
    if (discriminant < 0) return false;

    // Calculate the square root of the discriminant.
    float sqrt_discriminant = sqrt(discriminant);

    // Since t > 0 is part of the ray definition, we examine the two
    // roots. The smaller, positive real root is the one that is closest
    // to the intersection distance on the ray.
    float intersection_t = (-half_B - sqrt_discriminant) / A;       // first root
    if (intersection_t <= t_min || t_max <= intersection_t) {
        // first root not in range [t_0,t_1], so calculate
        // the second root.
        intersection_t = (-half_B + sqrt_discriminant) / A;

        // Check if second root is also not in the range [t_0,t_1].
        if (intersection_t <= t_min || t_max <= intersection_t)
            return false;
    }

    // We know the ray intersects the sphere, so we should update the
    // intersection information
    rec.t = intersection_t;
    rec.p = r.at(intersection_t);
    Vec3D outward_normal = (rec.p - center) / radius;
    rec.set_face_normal(r, outward_normal);
    rec.mat_ptr = mat_ptr;

    return true;
}

__device__ bool Sphere::bounding_box(float time_0, float time_1, AABB &surrounding_AABB) const {
    // Does the sphere have a bounding box?

    surrounding_AABB = AABB{
            center - Vec3D{ radius, radius, radius },
            center + Vec3D{ radius, radius, radius }
    };

    return true;
}

#endif //UNTITLED_SPHERE_H
