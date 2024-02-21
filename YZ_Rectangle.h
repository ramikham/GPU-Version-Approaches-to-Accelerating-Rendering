//
// Created by ramik on 2/17/2024.
//

#ifndef UNTITLED_YZ_RECTANGLE_H
#define UNTITLED_YZ_RECTANGLE_H

#include "Primitive.h"

/// Reference: Ray Tracing: The Next Week
class YZ_Rectangle : public Primitive {
public:
    // Constructor
    // -----------------------------------------------------------------------
    __device__ YZ_Rectangle(const Vec3D &minPoint, const Vec3D &maxPoint, Material* matPtr)
            : min_point(minPoint), max_point(maxPoint), mat_ptr(matPtr) {
        if (minPoint.x() != maxPoint.x()) {
            printf("YZ_RECTANGLE MUST HAVE ITS X COMPONENTS IN MIN AND MAX EQUAL!\n");
        }
        x_comp = minPoint.x();
    }

    __device__ bool intersection(const Ray &r, float t_min, float t_max, Intersection_Information &rec) const override {
        // Does the ray intersect the YZ_Rectangle?

        float t = (x_comp - r.get_ray_origin().x()) / r.get_ray_direction().x();

        if (t < t_min || t > t_max)
            return false;

        float y_comp = r.get_ray_origin().y() + t * r.get_ray_direction().y();
        float z_comp = r.get_ray_origin().z() + t * r.get_ray_direction().z();

        if (y_comp < min_point.y() || y_comp > max_point.y() || z_comp < min_point.z() || z_comp > max_point.z())
            return false;

        rec.t = t;
        Vec3D outward_normal = unit_vector(cross_product(Vec3D(0,min_point.y() - max_point.y(),0), Vec3D(0,0,min_point.z() - max_point.z())));
        rec.set_face_normal(r, outward_normal);
        rec.mat_ptr = mat_ptr;
        rec.p = r.at(t);

        return true;
    }

    __device__ bool bounding_box(float time_0, float time_1, AABB &surrounding_AABB) const override {
        surrounding_AABB = AABB{ {min_point.x() - 0.0001f, min_point.y(), min_point.z()},
                                 {max_point.x() + 0.0001f, max_point.y(), max_point.z()} };
        return true;
    }

public:
    // Data Members
    // -----------------------------------------------------------------------
    Vec3D min_point;
    Vec3D max_point;
    float x_comp;
    Material *mat_ptr;
};
#endif //UNTITLED_YZ_RECTANGLE_H
