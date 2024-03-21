//
// Created by ramik on 2/17/2024.
//

#ifndef UNTITLED_XZ_RECTANGLE_H
#define UNTITLED_XZ_RECTANGLE_H

#include "Primitive.h"

/// Reference: Ray Tracing: The Next Week
/// Note: Some changes were made. I changed the way the rectangles are represented and the importance sampling process.
class XZ_Rectangle : public Primitive {
public:
    // Constructor
    // -----------------------------------------------------------------------
    __device__ XZ_Rectangle(const Vec3D &minPoint, const Vec3D &maxPoint, Material* matPtr)
            : min_point(minPoint), max_point(maxPoint), mat_ptr(matPtr) {
        if (minPoint.y() != maxPoint.y()) {
            printf("XZ_RECTANGLE MUST HAVE ITS Y COMPONENTS IN MIN AND MAX EQUAL!\n");
        }
        y_comp = minPoint.y();
        area = (max_point.x() - min_point.x()) * (max_point.z() - min_point.z());
    }

    // Overloaded Functions
    // -----------------------------------------------------------------------
    __device__ bool intersection(const Ray &r, float t_min, float t_max, Intersection_Information &rec) const override {
        // Does the ray intersect the XZ_Rectangle?

        float t = (y_comp - r.get_ray_origin().y()) / r.get_ray_direction().y();

        if (t < t_min || t > t_max)
            return false;

        float x_comp = r.get_ray_origin().x() + t * r.get_ray_direction().x();
        float z_comp = r.get_ray_origin().z() + t * r.get_ray_direction().z();

        if (x_comp < min_point.x() || x_comp > max_point.x() || z_comp < min_point.z() || z_comp > max_point.z())
            return false;

        rec.t = t;
        Vec3D outward_normal = unit_vector(cross_product(Vec3D(min_point.x() - max_point.x(),0,0), Vec3D(0,0,min_point.z() - max_point.z())));
        rec.set_face_normal(r, outward_normal);
        rec.mat_ptr = mat_ptr;
        rec.p = r.at(t);

        return true;
    }

    __device__ bool bounding_box(float time_0, float time_1, AABB &surrounding_AABB) const override {
        // Does the XZ_Rectangle have a bounding box?

        surrounding_AABB = AABB{ {min_point.x(), min_point.y() - 0.0001f, min_point.z()},
                                 {max_point.x(), max_point.y() + 0.0001f, max_point.z()} };
        return true;
    }

    __device__ float PDF_value(const Vec3D &o, const Vec3D &v) const override {
        // Calculates the PDF value, the likelihood of sampling a random direction on the XZ_Rectangle

        Intersection_Information intersection_info;
        if (!this->intersection(Ray(o, v), 0.001, INFINITY, intersection_info))
            return 0;

        auto distance_squared = intersection_info.t * intersection_info.t * v.length_squared();
        auto cosine = fabs(dot_product(v, intersection_info.normal) / v.length());

        return distance_squared / (cosine * area);
    }

    __device__ Vec3D random(const Vec3D &o, curandState *local_rand_state) const override {
        // Generates a random direction within the XZ_Rectangle based on importance sampling

        // Get the random X and z components
        float rand_x_comp = min_point.x() + curand_uniform(local_rand_state) * (max_point.x() - min_point.x());
        float rand_z_comp = min_point.z() + curand_uniform(local_rand_state) * (max_point.z() - min_point.z());

        // Use them!
        Vec3D random_point = Vec3D(rand_x_comp, y_comp, rand_z_comp);

        return random_point - o;
    }

public:
    // Data Members
    // -----------------------------------------------------------------------
    Vec3D min_point;
    Vec3D max_point;
    float y_comp;
    float area;
    Material *mat_ptr;
};
#endif //UNTITLED_XZ_RECTANGLE_H