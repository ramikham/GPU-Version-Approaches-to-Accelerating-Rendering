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
    if (!list_size)
    {
        return false;
    }

    const auto object = list[0];
    AABB current_box;

    // First object needs to be dealt separately
    if (!object->bounding_box(time_0, time_1, current_box))
    {
        return false;
    }
    surrounding_AABB = current_box;

    for (size_t i = 1; i < list_size; ++i)
    {
        // Compute box updates the current_box
        if (!(list[i])->bounding_box(time_0, time_1, current_box))
        {
            return false;
        }

        // We merge the boxes
        surrounding_AABB = enclosing_box(surrounding_AABB, current_box);
    }

    return true;
}

#endif //UNTITLED_PRIMITIVES_LIST_H
