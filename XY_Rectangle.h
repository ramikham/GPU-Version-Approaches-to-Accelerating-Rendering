//
// Created by ramik on 2/17/2024.
//

#ifndef UNTITLED_XY_RECTANGLE_H
#define UNTITLED_XY_RECTANGLE_H

#include "Primitive.h"

class XY_Rectangle : public Primitive {
public:
    __device__ XY_Rectangle(const Vec3D &minPoint, const Vec3D &maxPoint,  Material* matPtr) : min_point(
            minPoint), max_point(maxPoint), mat_ptr(matPtr) {
        if (minPoint.z() != maxPoint.z()) {
            printf("XY_RECTANGLE MUST HAVE ITS Z COMPONENTS IN MIN AND MAX EQUAL!\n");
        }
        z_comp = minPoint.z();
    }

    __device__ bool intersection(const Ray &r, float t_min, float t_max, Intersection_Information &rec) const override {
        // Does the ray intersect the XY_Rectangle?

        float t = (z_comp - r.get_ray_origin().z()) / r.get_ray_direction().z();

        if (t < t_min || t > t_max)
            return false;

        float x_comp = r.get_ray_origin().x() + t * r.get_ray_direction().x();
        float y_comp = r.get_ray_origin().y() + t * r.get_ray_direction().y();

        if (x_comp < min_point.x() || x_comp > max_point.x() || y_comp < min_point.y() || y_comp > max_point.y())
            return false;

        /*
        intersection_info.p = point3D((x_comp - min_point.x())/(max_point.x() - min_point.x()),
                                      (y_comp - min_point.y())/(max_point.y() - min_point.y()),
                                      z_comp);*/
        rec.t = t;
        Vec3D outward_normal = unit_vector(Vec3D(0, 0, max_point.z() > min_point.z() ? 1 : -1));
        rec.set_face_normal(r, outward_normal);
        rec.mat_ptr = mat_ptr;
        rec.p = r.at(t);

        return true;
    }

    __device__ bool bounding_box(float time_0, float time_1, AABB &surrounding_AABB) const override {
        surrounding_AABB = AABB{ {min_point.x(), min_point.y(), min_point.z() - 0.0001f},
                                 {max_point.x(), max_point.y(), max_point.z() + 0.0001f} };
        return true;
    }

public:
    // Data Members
    // -----------------------------------------------------------------------
    Vec3D min_point;
    Vec3D max_point;
    float z_comp;
    Material *mat_ptr;
};
#endif //UNTITLED_XY_RECTANGLE_H
