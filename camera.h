//
// Created by ramik on 2/17/2024.
//

#ifndef UNTITLED_CAMERA_H
#define UNTITLED_CAMERA_H

#include <curand_kernel.h>
#include "Ray.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

__device__ Vec3D random_in_unit_disk(curandState *local_rand_state) {
    Vec3D p;
    do {
        p = 2.0f * Vec3D(curand_uniform(local_rand_state), curand_uniform(local_rand_state), 0) - Vec3D(1, 1, 0);
    } while (dot_product(p, p) >= 1.0f);
    return p;
}

class camera {
public:
    __device__ camera(Vec3D lookfrom, Vec3D lookat, Vec3D vup, float vfov, float aspect, float aperture, float focus_dist) { // vfov is top to bottom in degrees
        lens_radius = aperture / 2.0f;
        float theta = vfov*((float)M_PI)/180.0f;
        float half_height = tan(theta/2.0f);
        float half_width = aspect * half_height;
        origin = lookfrom;
        w = unit_vector(lookfrom - lookat);
        u = unit_vector(cross_product(vup, w));
        v = cross_product(w, u);
        lower_left_corner = origin  - half_width*focus_dist*u -half_height*focus_dist*v - focus_dist*w;
        horizontal = 2.0f*half_width*focus_dist*u;
        vertical = 2.0f*half_height*focus_dist*v;
    }
    __device__ Ray get_ray(float s, float t, curandState *local_rand_state) {
        Vec3D rd = lens_radius*random_in_unit_disk(local_rand_state);
        Vec3D offset = u * rd.x() + v * rd.y();
        return Ray(origin + offset, lower_left_corner + s * horizontal + t * vertical - origin - offset);
    }

    Vec3D origin;
    Vec3D lower_left_corner;
    Vec3D horizontal;
    Vec3D vertical;
    Vec3D u, v, w;
    float lens_radius;
};

#endif //UNTITLED_CAMERA_H
