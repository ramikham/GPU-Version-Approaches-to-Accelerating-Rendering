//
// Created by ramik on 2/17/2024.
//

#ifndef UNTITLED_XY_RECTANGLE_H
#define UNTITLED_XY_RECTANGLE_H

#include "Primitive.h"

/// Reference: Ray Tracing: The Next Week
/// Note: Some changes were made. I changed the way the rectangles are represented and the importance sampling process.
class XY_Rectangle : public Primitive {
public:
    __device__ XY_Rectangle(const Vec3D &minPoint, const Vec3D &maxPoint,  Material* matPtr) : min_point(
            minPoint), max_point(maxPoint), mat_ptr(matPtr) {
        if (minPoint.z() != maxPoint.z()) {
            printf("XY_RECTANGLE MUST HAVE ITS Z COMPONENTS IN MIN AND MAX EQUAL!\n");
        }
        z_comp = minPoint.z();
    }

    // Overloaded Functions
    // -----------------------------------------------------------------------
    __device__ bool intersection(const Ray &r, float t_min, float t_max, Intersection_Information &rec) const override {
        // Does the ray intersect the XY_Rectangle?

        float t = (z_comp - r.get_ray_origin().z()) / r.get_ray_direction().z();

        if (t < t_min || t > t_max)
            return false;

        float x_comp = r.get_ray_origin().x() + t * r.get_ray_direction().x();
        float y_comp = r.get_ray_origin().y() + t * r.get_ray_direction().y();

        if (x_comp < min_point.x() || x_comp > max_point.x() || y_comp < min_point.y() || y_comp > max_point.y())
            return false;

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

    __device__ float PDF_value(const Vec3D &o, const Vec3D &v) const override {
        // Calculate the PDF value, the likelihood of sampling a random direction on the XY_Rectangle

        Intersection_Information intersection_information;
        if (!this->intersection(Ray(o, v), 0.001, INFINITY, intersection_information))
            return 0;

        auto area = (max_point.x()-min_point.x())*(max_point.y()-min_point.y());
        auto distance_squared = intersection_information.t * intersection_information.t * v.length_squared();
        auto cosine = fabs(dot_product(v, intersection_information.normal) / v.length());

        return distance_squared / (cosine * area);
    }

    __device__ Vec3D random(const Vec3D &o, curandState *local_rand_state) const override {
        // Generates a random direction within the XY_Rectangle based on importance sampling

        // Get the random y and z components
        float rand_x_comp = min_point.x() + curand_uniform(local_rand_state) * (max_point.x() - min_point.x());
        float rand_y_comp = min_point.y() + curand_uniform(local_rand_state) * (max_point.y() - min_point.y());

        // Use them!
        Vec3D random_point = Vec3D(rand_x_comp, rand_y_comp, z_comp);

        return random_point - o;
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