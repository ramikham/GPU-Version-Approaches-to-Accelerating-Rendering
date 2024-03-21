//
// Created by ramik on 2/19/2024.
//

#ifndef UNTITLED_BVH_H
#define UNTITLED_BVH_H

#include "Primitive.h"
#include "Primitives_List.h"
#include "AABB.h"
#include <thrust/sort.h>

__device__ inline bool compare_AABBs(const Primitive* a, const Primitive* b, int axis) {
    // Compares two primitives based on the minimum coordinates of their bounding boxes along a specified axis.

    AABB box_a;
    AABB box_b;

    if (!a->bounding_box(0,0, box_a) || !b->bounding_box(0,0, box_b)) {
        printf("NO BBOX");
    }

    // Compare by the minimum coordinate of the bounding boxes
    return box_a.get_max().V[axis] < box_b.get_max().V[axis];
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

class BVH : public Primitive {
public:
    __device__ BVH(Primitive **l, int n, float time0, float time1, curandState* local_rand_state, int axis_ctr=0) {
        // printf("Inside BVH, n = %d", n);
        int axis = axis_ctr % 3;
        auto comparator = (axis == 0) ? box_x_compare
                                      : (axis == 1) ? box_y_compare
                                             : box_z_compare;

        if (n == 1) {
            left = right = l[0];
        }else if (n == 2) {
            left  = l[0];
            right = l[1];
        }else {
            thrust::sort(l, l + n, comparator);
            auto m = n / 2;

            // Allocate memory for each half
            Primitive **first_half = new Primitive*[m];
            Primitive **second_half = new Primitive*[n - m];

            // Copy the first half
            for (int i = 0; i < m; ++i)
                first_half[i] = l[i];

            // Copy the second half
            for (int i = 0; i < n - m; ++i)
                second_half[i] = l[m + i];

           // Launch recursion
            left  = new BVH(first_half, m, time0, time1, local_rand_state, axis_ctr + 1);
            right = new BVH(second_half, n - m, time0, time1, local_rand_state, axis_ctr + 1);
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
            return false;  // EARLY EXIT !!!!
        }

        bool hit_right = right->intersection(r, t_min, t_max, rec);
        bool hit_left = left->intersection(r, t_min, hit_right ? rec.t : t_max, rec);

        return hit_left || hit_right;
    }

    __device__ bool bounding_box(float time_0, float time_1, AABB &surrounding_AABB) const override {
        surrounding_AABB = box;
        return true;
    }

    __device__ ~BVH() override {
        if (left != right) {
            delete left;
            delete right;
        }
        else
            delete left;
    }
private:
    Primitive *left;
    Primitive *right;
    AABB box;
};

#endif //UNTITLED_BVH_H
