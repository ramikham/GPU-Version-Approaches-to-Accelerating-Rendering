//
// Created by ramik on 2/18/2024.
//

#ifndef UNTITLED_TRIANGLE_H
#define UNTITLED_TRIANGLE_H

#include "Primitive.h"

// A Triangle class that includes the following ray/triangle intersection algorithms:
//          1. Möller–Trumbore ray-triangle intersection algorithm
//          2. Snyder & Barr ray-triangle intersection algorithm
class Triangle : public Primitive {
public:
    __device__ Triangle(const Vec3D &a, const Vec3D &b, const Vec3D &c, Material* triangle_material)
            : a(a), b(b), c(c), mat_ptr(triangle_material) {}

    __device__ bool intersection(const Ray &r, float t_min, float t_max, Intersection_Information &rec) const override {
        // Does the ray intersect the triangle?

        /*
        // Snyder & Barr ray-triangle intersection algorithm

        // Get Ray details
        // ----
        Vec3D ending_pos = r.get_ray_origin();
        Vec3D d = r.get_ray_direction();

        // Get Triangle'starting_pos details
        Vec3D B_A = b - a;           // (b - a), an edge
        Vec3D C_A = c - a;           // (c - a), an edge

        // Set variable'starting_pos for Cramer'starting_pos Rule
        float A = a.x() - b.x();
        float D = a.x() - c.x();
        float G = d.x();

        float B = a.y() - b.y();
        float E = a.y() - c.y();
        float H  = d.y();

        float C = a.z() - b.z();
        float F = a.z() - c.z();
        float I = d.z();

        // Compute M
        float EI = E * I;
        float HF = H * F;

        float GF = G * F;
        float DI = D * I;

        float DH = D * H;
        float EG = E * G;

        float M = A * (EI - HF) + B * (GF - DI) + C * (DH - EG);

        // Compute t
        float J = a.x() - ending_pos.x();
        float K = a.y() - ending_pos.y();
        float L = a.z() - ending_pos.z();

        float AK = A * K;
        float JB = J * B;

        float JC = J * C;
        float AL = A * L;

        float BL = B * L;
        float KC = K * C;

        float t = -(F * (AK - JB) + E * (JC - AL) + D * (BL - KC)) / M;

        // Check for visibility in [t_min,t_max]
        if (t < t_min || t > t_max)
            return false;

        // Compute GAMMA
        float GAMMA = (I * (AK - JB) + H * (JC - AL) + G * (BL - KC)) / M;

        // Check GAMMA'starting_pos range
        if (GAMMA < 0 || GAMMA > 1)
            return false;

        // Compute BETA
        float BETA = (J * (EI - HF) + K * (GF - DI) + L * (DH - EG)) / M;

        // Check BETA'starting_pos range
        if (BETA < 0 || BETA > 1 - GAMMA)
            return false;

        // An intersection must happen, so update all intersection information
        rec.t = t;
        rec.p = r.at(t);
        Vec3D n = cross_product(B_A, C_A);

        rec.set_face_normal(r, unit_vector(n));
        rec.mat_ptr = mat_ptr;

        return true;
        */

         // Möller–Trumbore ray-triangle intersection algorithm
        Vec3D edge_1 = b - a;
        Vec3D edge_2 = c - a;
        Vec3D ray_cross_e2 = cross_product(r.get_ray_direction(), edge_2);
        float D = dot_product(edge_1, ray_cross_e2);
        float epsilon = 1e-9f;

        // Front face culling
        if (D > -epsilon && D < epsilon)
            return false;  // parallel ray

        float inv_D = 1.0f / D;
        Vec3D s = r.get_ray_origin() - a;
        float u = inv_D * dot_product(s, ray_cross_e2);

        if (u < 0 || u > 1)
            return false;

        Vec3D s_cross_e1 = cross_product(s, edge_1);
        float v = inv_D * dot_product(r.get_ray_direction(), s_cross_e1);

        if (v < 0 || u + v > 1)
            return false;

        float t = inv_D * dot_product(edge_2, s_cross_e1);

        // NOTE: newly added -> Check for visibility in [t_0,t_1]
        if (t < t_min || t > t_max)
            return false;

        if (t > epsilon) {
            // Ray intersects
            rec.t = t;
            rec.p = r.at(t);
            Vec3D n = cross_product(edge_1, edge_2);

            rec.set_face_normal(r, unit_vector(n));
            rec.mat_ptr = mat_ptr;

            return true;
        } else
            return false;

    }

    __device__ bool bounding_box(float time_0, float time_1, AABB &surrounding_AABB) const override {
        // Does the triangle have a bounding box?

        Vec3D EPS(1e-9, 1e-9, 1e-9);
        surrounding_AABB =  AABB(
                min(a, min(b, c)) - EPS,
                max(a, max(b, c)) + EPS
        );

        return true;
    }

    __device__ float PDF_value(const Vec3D &o, const Vec3D &v) const override {
        // Calculate the PDF value, the likelihood of sampling a random direction on the triangle

        Ray r(o, v);

        Intersection_Information intersection_info;

        if (!this->intersection(Ray(o, v), 0.001, INFINITY, intersection_info))
            return 0;

        float cost_theta_I = dot_product(-v, intersection_info.normal);

        if (cost_theta_I <= 0.0f)
            return 0;

        float dis_sq = (r.at(intersection_info.t) - o).length_squared();

        return dis_sq / (cost_theta_I * area());
    }

    __device__ Vec3D random(const Vec3D &o, curandState *local_rand_state) const override {
        // Generates a random direction within the triangle based on importance sampling

        float u1 = curand_uniform(local_rand_state);
        float u2 = curand_uniform(local_rand_state);

        if (u1 + u2 > 1.0) {
            u1 = 1.0f - u1;
            u2 = 1.0f - u2;
        }

        Vec3D e1 = b - a;
        Vec3D e2 = c - a;

        Vec3D w = u1*e1 + u2*e2;

        return w + a - o;
    }

private:
    // Data Members
    // -----------------------------------------------------------------------
    Vec3D a,b,c;                   // vertices of the triangle
    Material* mat_ptr;            // triangle'starting_pos material

    __device__ float area() const {
        // Calculate the area of the triangle (you may use a more accurate formula)
        Vec3D edge1 = b - a;
        Vec3D edge2 = c - a;
        return 0.5f * cross_product(edge1, edge2).length();
    }
};


#endif //UNTITLED_TRIANGLE_H
