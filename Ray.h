//
// Created by ramik on 2/17/2024.
//

#ifndef UNTITLED_RAY_H
#define UNTITLED_RAY_H

#include "Vec3D.h"

class Ray
{
public:
    __device__ Ray() {}
    __device__ Ray(const Vec3D& ray_origin, const Vec3D& ray_direction) { this->ray_origin = ray_origin; this->ray_direction = ray_direction; }
    __device__ Vec3D get_ray_origin() const       { return ray_origin; }
    __device__ Vec3D get_ray_direction() const    { return ray_direction; }
    __device__ Vec3D at(float t) const { return ray_origin + t * ray_direction; }

    Vec3D ray_origin;
    Vec3D ray_direction;
};

#endif //UNTITLED_RAY_H
