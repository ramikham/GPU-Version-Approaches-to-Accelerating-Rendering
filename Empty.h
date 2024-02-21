//
// Created by ramik on 2/19/2024.
//

#ifndef UNTITLED_EMPTY_H
#define UNTITLED_EMPTY_H

#include "Primitive.h"

class Empty : public Primitive {
public:
    __device__ Empty() {}

    __device__ bool intersection(const Ray &r, float t_min, float t_max, Intersection_Information &rec) const override {
        return false;
    }

    __device__ ~Empty() override {}

    __device__ bool bounding_box(float time_0, float time_1, AABB &surrounding_AABB) const override {
        return false;
    }
};
#endif //UNTITLED_EMPTY_H
