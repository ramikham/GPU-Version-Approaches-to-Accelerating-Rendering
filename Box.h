//
// Created by ramik on 2/18/2024.
//

#ifndef UNTITLED_BOX_H
#define UNTITLED_BOX_H

#include "Primitive.h"
#include "Primitives_List.h"
#include "XZ_Rectangle.h"
#include "XY_Rectangle.h"
#include "YZ_Rectangle.h"

class Box : public Primitive {
public:
    __device__ Box(const Vec3D &min_point, const Vec3D &max_point, Material* mat_ptr) :
            this_min_point(min_point), this_max_point(max_point), this_material(mat_ptr) {
        const auto lst = new Primitive * [6];

        /*
        lst[0] = new XY_Rectangle(Vec3D(min_point.x(), min_point.y(), min_point.z()),
                                  Vec3D(max_point.x(), max_point.y(), min_point.z()),
                                  mat_ptr);

        lst[1] = new XY_Rectangle(Vec3D(min_point.y(), min_point.z(), max_point.x()),
                                  Vec3D(max_point.y(), max_point.z(), max_point.x()),
                                  mat_ptr);

        lst[2] = new YZ_Rectangle(Vec3D(min_point.y(), min_point.z(), min_point.x()),
                                  Vec3D(max_point.y(), max_point.z(), min_point.x()),
                                  mat_ptr);

       lst[3] = new YZ_Rectangle(Vec3D(min_point.y(), min_point.z(), max_point.x()),
                                  Vec3D(max_point.y(), max_point.z(), max_point.x()),
                                  mat_ptr);

        lst[4] = new XZ_Rectangle(Vec3D(min_point.x(), min_point.z(), min_point.y()),
                                  Vec3D(max_point.x(), max_point.z(), min_point.y()),
                                  mat_ptr);

        lst[5] = new XZ_Rectangle(Vec3D(min_point.x(), min_point.z(), max_point.y()),
                                  Vec3D(max_point.x(), max_point.z(), max_point.y()),
                                  mat_ptr);
        */


        lst[0] = new XY_Rectangle(min_point,
                                  Vec3D(max_point.x(), max_point.y(), min_point.z()),
                                  mat_ptr);

        lst[1] = new XY_Rectangle(Vec3D(min_point.x(), min_point.y(), max_point.z()),
                                  Vec3D(max_point.x(), max_point.y(), max_point.z()),
                                  mat_ptr);

        lst[2] = new YZ_Rectangle(min_point, Vec3D(min_point.x(), max_point.y(), max_point.z()), mat_ptr);

        lst[3] = new YZ_Rectangle(Vec3D(max_point.x(), min_point.y(), min_point.z()),
                                  Vec3D(max_point.x(), max_point.y(), max_point.z()),
                                  mat_ptr);

        lst[4] = new XZ_Rectangle(min_point, Vec3D(max_point.x(), min_point.y(), max_point.z()), mat_ptr);

        lst[5] = new XZ_Rectangle(Vec3D(min_point.x(), max_point.y(), min_point.z()),
                                  Vec3D(max_point.x(), max_point.y(), max_point.z()),
                                  mat_ptr);

        box_sides = new Primitives_List(lst, 6);

    }

    __device__ ~Box() override {
        for (int i = 0; i < 6; i++) {
            // Nullify
            box_sides->get_primitives()[i]->mat_ptr = nullptr;
        }
        delete box_sides;
    }

    __device__ bool intersection(const Ray &r, float t_min, float t_max, Intersection_Information &rec) const override {
        return box_sides->intersection(r, t_min, t_max, rec);
    }

    __device__ bool bounding_box(float time_0, float time_1, AABB &surrounding_AABB) const override {
        surrounding_AABB = AABB{ this_min_point, this_max_point };
        return true;
    }

private:
    Vec3D this_min_point;
    Vec3D this_max_point;
    Material* this_material;
    Primitives_List* box_sides;
};
#endif //UNTITLED_BOX_H
