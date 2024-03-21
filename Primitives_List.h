//
// Created by ramik on 2/17/2024.
//

#ifndef UNTITLED_PRIMITIVES_LIST_H
#define UNTITLED_PRIMITIVES_LIST_H

#include "Primitive.h"

class Primitives_List: public Primitive  {
public:
    __device__ Primitives_List() {}
    __device__ Primitives_List(Primitive **l, int n) { list = l; list_size = n; }
    __device__ virtual bool intersection(const Ray& r, float tmin, float tmax, Intersection_Information& rec) const;

    __device__ float PDF_value(const Vec3D &o, const Vec3D &v) const override;

    __device__ Vec3D random(const Vec3D &o, curandState *local_rand_state) const override;
    // Getters
    __device__ Primitive** get_primitives() const {
        return list;
    }

    __device__ bool bounding_box(float time_0, float time_1, AABB &surrounding_AABB) const override;

    __device__ int get_list_size() const {
        return list_size;
    }

public:
    Primitive **list;
    int list_size;
};

__device__ bool Primitives_List::intersection(const Ray& r, float t_min, float t_max, Intersection_Information& rec) const {
    Intersection_Information temp_rec;
    bool hit_anything = false;
    float closest_so_far = t_max;
    for (int i = 0; i < list_size; i++) {
        if (list[i]->intersection(r, t_min, closest_so_far, temp_rec)) {
            hit_anything = true;
            closest_so_far = temp_rec.t;
            rec = temp_rec;
        }
    }
    return hit_anything;
}

__device__ bool Primitives_List::bounding_box(float time_0, float time_1, AABB &surrounding_AABB) const {
    if (!list_size) {
        return false;
    }

    const auto object = list[0];
    AABB current_box;


    if (!object->bounding_box(time_0, time_1, current_box)) {
        return false;
    }

    surrounding_AABB = current_box;

    for (size_t i = 1; i < list_size; ++i) {
        if (!(list[i])->bounding_box(time_0, time_1, current_box)) {        // no more bounding boxes?
            return false;
        }

        // Keep updating....
        surrounding_AABB = enclosing_box(surrounding_AABB, current_box);
    }

    return true;
}

__device__ float Primitives_List::PDF_value(const Vec3D &o, const Vec3D &v) const {
    // Returns the PDF for the list of primitives by averaging the PDF
    // of evey primitives that constitutes it

    float weight = 1.0f/static_cast<float>(list_size);
    float sum = 0.0;

    for (int i = 0; i < list_size; i++) {
        sum += weight * list[i]->PDF_value(o, v);
    }

    return sum;
}

__device__ Vec3D Primitives_List::random(const Vec3D &o, curandState *local_rand_state) const {
    // Returns a random direction (for importance sampling) by randomly selecting one
    // primitive from the list and then calling its own random()

    auto int_size = static_cast<int>(list_size);

    // Get random in range
    // https://stackoverflow.com/questions/18501081/generating-random-number-within-cuda-kernel-in-a-varying-range
    return list[static_cast<int>(curand_uniform(local_rand_state) * list_size)]->random(o, local_rand_state);
}

#endif //UNTITLED_PRIMITIVES_LIST_H