//
// Created by ramik on 2/17/2024.
//

#ifndef UNTITLED_RAY_H
#define UNTITLED_RAY_H

#include "Vec3D.h"

class Ray {
public:
    // Constructors
    // -----------------------------------------------------------------------
    __device__ Ray() {}
    __device__ Ray(const Vec3D& ray_origin, const Vec3D& ray_direction, float time=0.0) {
        this->ray_origin = ray_origin;

        this->ray_direction = ray_direction;

        this->time = time;

        this->inv_direction = Vec3D(1.0f/ray_direction.x(), 1.0f/ray_direction.y(), 1.0f/ray_direction.z());

        this->sign[0] = (this->inv_direction.x() < 0);
        this->sign[1] = (this->inv_direction.y() < 0);
        this->sign[2] = (this->inv_direction.z() < 0);
    }

    // Getters
    // -----------------------------------------------------------------------
    __device__ Vec3D get_ray_origin() const       { return ray_origin; }
    __device__ Vec3D get_ray_direction() const    { return ray_direction; }
    __device__ Vec3D at(float t) const { return ray_origin + t * ray_direction; }
    __device__ float get_time() const {return time; };
    __device__ Vec3D get_inverse_direction() const { return {1.0f/ray_direction.x(), 1.0f/ray_direction.y(), 1.0f/ray_direction.z()}; }

    // Data Members
    // -----------------------------------------------------------------------
    float time;
    Vec3D ray_origin;
    Vec3D ray_direction;
    Vec3D inv_direction;
    int sign[3];
};

#endif //UNTITLED_RAY_H
