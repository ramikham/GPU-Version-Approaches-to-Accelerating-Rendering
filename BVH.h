//
// Created by ramik on 2/19/2024.
//

#ifndef UNTITLED_BVH_H
#define UNTITLED_BVH_H

#include "Primitive.h"
#include "Primitives_List.h"
#include "AABB.h"
#include "Empty.h"
#include <thrust/sort.h>

enum Axis { X, Y, Z };

__device__ void swap(Primitive*& p1, Primitive*& p2) {
    Primitive* temp = p1;
    *p1 = *p2;
    p2 = temp;
}

template<Axis axis>
__device__ void bubble_sort(Primitive** e, int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            AABB box_left, box_right;
            Primitive *ah = e[j];
            Primitive *bh = e[j+1];

            ah->bounding_box(0, 0, box_left);
            bh->bounding_box(0, 0, box_right);

            if ((axis == X && (box_left.min().x() - box_right.min().x()) < 0.0)
                || (axis == Y && (box_left.min().y() - box_right.min().y()) < 0.0)
                || (axis == Z && (box_left.min().z() - box_right.min().z()) < 0.0)) {
                swap(e[j], e[j+1]);
            }
        }
    }
}
__device__ inline bool compare_AABBs(const Primitive* a, const Primitive* b, int axis) {
    // Compares two primitives based on the minimum coordinates of their bounding boxes along a specified axis.

    AABB box_a;
    AABB box_b;

    if (!a->bounding_box(0,0, box_a) || !b->bounding_box(0,0, box_b)) {
        printf("NO BBOX");
    }

    // Compare by the minimum coordinate of the bounding boxes
    return box_a.min().V[axis] < box_b.min().V[axis];
}

__device__ bool box_x_compare(const Primitive* a, const Primitive* b){
    // Utilizes compare_AABBs() to compare the primitives a and b along the x-axis.

    return compare_AABBs(a, b, 0);
}

__device__ bool box_y_compare(const Primitive* a, const Primitive* b){
    // Utilizes compare_AABBs() to compare the primitives a and b along the y-axis.

    return compare_AABBs(a, b, 1);
}

__device__ bool box_z_compare(const Primitive* a, const Primitive* b){
    // Utilizes compare_AABBs() to compare the primitives a and b along the z-axis.

    return compare_AABBs(a, b, 2);
}
__device__ inline float random_unit(curandState_t* rand_state)
{
    return curand_uniform(rand_state);
}

__device__ inline float uniform_rand(curandState_t* rand_state, const float min, const float max)
{
    return min + (max - min) * random_unit(rand_state);
}

struct BoxCompare {
    __device__ BoxCompare(int m): mode(m) {}
    __device__ bool operator()(Primitive* a, Primitive* b) const {
        // return true;

        AABB box_left, box_right;
        Primitive* ah = a;
        Primitive* bh = b;

        if(!ah->bounding_box(0, 0, box_left) || !bh->bounding_box(0, 0, box_right)) {
            return false;
        }

        float val1, val2;
        if(mode == 1) {
            val1 = box_left.min().x();
            val2 = box_right.min().x();
        }else if(mode == 2) {
            val1 = box_left.min().y();
            val2 = box_right.min().y();
        }else if(mode == 3) {
            val1 = box_left.min().z();
            val2 = box_right.min().z();
        }

        if(val1 - val2 < 0.0){
            return false;
        } else{
            return true;
        }
    }
    // mode: 1, x; 2, y; 3, z
    int mode;
};

class BVH : public Primitive {
public:
    __device__ BVH(Primitive **l, int n, float time0, float time1, curandState* local_rand_state) {
       // printf("Inside BVH, n = %d", n);
        int axis = int(3 * curand_uniform(local_rand_state));
        if (axis == 0){
           // qsort(l, l+n, sizeof(Primitive*), BoxCompare(1));
            thrust::sort(l, l + n, BoxCompare(1));
        }else if (axis == 1){
            thrust::sort(l, l + n, BoxCompare(2));
        }else{
            thrust::sort(l, l + n, BoxCompare(3));
        }

        if (n == 1) {
            left = right = l[0];
        }else if (n == 2) {
            left  = l[0];
            right = l[1];
        }else {
            left  = new BVH(      l,     n/2, time0, time1, local_rand_state);
            right = new BVH(l + n/2, n - n/2, time0, time1, local_rand_state);
        }

        AABB box_left, box_right;
        if (!left->bounding_box(time0, time1, box_left) ||
            !right->bounding_box(time0, time1, box_right)){
            return;
            // std::cerr << "no bounding box in BVHNode constructor \n";
        }
        box = enclosing_box(box_left, box_right);
    }

    __device__ bool intersection(const Ray &r, float t_min, float t_max, Intersection_Information &rec) const override {
        if (!box.intersection(r, t_min, t_max)) {
            return false;
        }

        bool hit_right = right->intersection(r, t_min, t_max, rec);
        bool hit_left = left->intersection(r, t_min, hit_right ? rec.t : t_max, rec);

        return hit_left || hit_right;
    }

    __device__ bool bounding_box(float time_0, float time_1, AABB &surrounding_AABB) const override {
        surrounding_AABB = box;
        return true;
    }

private:
    Primitive *left;
    Primitive *right;
    AABB box;
};

#endif //UNTITLED_BVH_H
