//
// Created by ramik on 2/17/2024.
//

#ifndef UNTITLED_MATERIAL_H
#define UNTITLED_MATERIAL_H

struct Intersection_Information;

#include "Ray.h"
#include "Primitive.h"
#include "PDF.h"


__device__ float schlick(float cosine, float ref_idx) {
    float r0 = (1.0f-ref_idx) / (1.0f+ref_idx);
    r0 = r0*r0;
    return r0 + (1.0f-r0)*pow((1.0f - cosine),5.0f);
}

__device__ bool refract(const Vec3D& v, const Vec3D& n, float ni_over_nt, Vec3D& refracted) {
    Vec3D uv = unit_vector(v);
    float dt = dot_product(uv, n);
    float discriminant = 1.0f - ni_over_nt*ni_over_nt*(1-dt*dt);
    if (discriminant > 0) {
        refracted = ni_over_nt*(uv - n*dt) - n*sqrt(discriminant);
        return true;
    }
    else
        return false;
}

#define RANDVEC3 Vec3D(curand_uniform(local_rand_state),curand_uniform(local_rand_state),curand_uniform(local_rand_state))

__device__ Vec3D random_in_unit_sphere(curandState *local_rand_state) {
    Vec3D p;
    do {
        p = 2.0f*RANDVEC3 - Vec3D(1, 1, 1);
    } while (p.length_squared() >= 1.0f);
    return p;
}

__device__ Vec3D reflect(const Vec3D& v, const Vec3D& n) {
    return v - 2.0f * dot_product(v, n) * n;
}

class Material  {
public:
    __device__ virtual bool evaluate(const Ray& r_in, const Intersection_Information& rec, Vec3D& attenuation, Ray& scattered, curandState *local_rand_state, float& pdf, PDF* surface_pdf_ptr) const = 0;

    __device__ virtual Vec3D emitted(const Vec3D& p, const Intersection_Information& intersection_information) {
        return Vec3D(0,0,0);
    }

    __device__ virtual Vec3D BRDF(const Ray& incident_ray, const Intersection_Information& intersection_information, const Ray& scattered_ray, Vec3D& attenuated_color) const {
        return Vec3D(1,1,1);
    }
};

class lambertian : public Material {
public:
    __host__ __device__ lambertian(const Vec3D& a) : albedo(a) {}
    __device__ virtual bool evaluate(const Ray& r_in, const Intersection_Information& rec, Vec3D& attenuation, Ray& scattered, curandState *local_rand_state, float& pdf, PDF* surface_pdf_ptr) const override {

        /* old
        Vec3D target = rec.p + rec.normal + random_in_unit_sphere(local_rand_state);
        scattered = Ray(rec.p, target - rec.p);
        attenuation = albedo;
        return true;
         */

        /* pdf code */
        Vec3D reflection_direction = rec.normal + random_in_unit_sphere(local_rand_state);
        scattered = Ray(rec.p, reflection_direction);
        attenuation = albedo;
        auto cos_theta = dot_product(rec.normal, unit_vector(scattered.get_ray_direction()));
        pdf = cos_theta < 0 ? 0 : cos_theta/3.141592654f;
        return true;
    }

    __device__ inline float PDF(const Ray &incident_ray, const Intersection_Information &intersection_info, const Ray &scattered_ray) const {
        // DEPRECATED: Only needed if you will use the OLD formulation of diffuse (commented above).
        // The new formulation used the PDF associated with the generated scattered ray direction.

        auto cos_theta = dot_product(intersection_info.normal, unit_vector(scattered_ray.get_ray_direction()));
        return cos_theta < 0 ? 0 : cos_theta/3.141592654f;
    }

    __device__ Vec3D BRDF(const Ray &incident_ray, const Intersection_Information &intersection_information, const Ray &scattered_ray,
         Vec3D &attenuated_color) const override {
        auto cos_theta = dot_product(intersection_information.normal, unit_vector(scattered_ray.get_ray_direction()));
        return cos_theta < 0 ? Vec3D(0,0,0) : attenuated_color * cos_theta/M_PI;
    }

    Vec3D albedo;
};


class metal : public Material {
public:
    __device__ metal(const Vec3D& a, float f) : albedo(a) { if (f < 1) fuzz = f; else fuzz = 1; }
    __device__ virtual bool evaluate(const Ray& r_in, const Intersection_Information& rec, Vec3D& attenuation, Ray& scattered, curandState *local_rand_state, float& pdf, PDF* surface_pdf_ptr) const  {
        Vec3D reflected = reflect(unit_vector(r_in.get_ray_direction()), rec.normal);
        scattered = Ray(rec.p, reflected + fuzz * random_in_unit_sphere(local_rand_state));
        attenuation = albedo;
        return (dot_product(scattered.get_ray_direction(), rec.normal) > 0.0f);
    }
    Vec3D albedo;
    float fuzz;
};

class dielectric : public Material {
public:
    __device__ dielectric(float ri) : ref_idx(ri) {}
    __device__ virtual bool evaluate(const Ray& r_in,
                                    const Intersection_Information& rec,
                                    Vec3D& attenuation,
                                    Ray& scattered,
                                    curandState *local_rand_state, float& pdf, PDF* surface_pdf_ptr) const  {
        Vec3D outward_normal;
        Vec3D reflected = reflect(r_in.get_ray_direction(), rec.normal);
        float ni_over_nt;
        attenuation = Vec3D(1.0, 1.0, 1.0);
        Vec3D refracted;
        float reflect_prob;
        float cosine;
        if (dot_product(r_in.get_ray_direction(), rec.normal) > 0.0f) {
            outward_normal = -rec.normal;
            ni_over_nt = ref_idx;
            cosine = dot_product(r_in.get_ray_direction(), rec.normal) / r_in.get_ray_direction().length();
            cosine = sqrt(1.0f - ref_idx*ref_idx*(1-cosine*cosine));
        }
        else {
            outward_normal = rec.normal;
            ni_over_nt = 1.0f / ref_idx;
            cosine = -dot_product(r_in.get_ray_direction(), rec.normal) / r_in.get_ray_direction().length();
        }
        if (refract(r_in.get_ray_direction(), outward_normal, ni_over_nt, refracted))
            reflect_prob = schlick(cosine, ref_idx);
        else
            reflect_prob = 1.0f;
        if (curand_uniform(local_rand_state) < reflect_prob)
            scattered = Ray(rec.p, reflected);
        else
            scattered = Ray(rec.p, refracted);
        return true;
    }

    float ref_idx;
};

class Diffuse_Light : public Material {
public:
    __device__ Diffuse_Light(Vec3D light_color) : light_color(light_color) {}

    __device__ Diffuse_Light(Vec3D& lightColor, float& area, float& x_min, float& x_max,
                             float& y_min, float& y_max, float& z_min, float& z_max) :
            light_color(light_color), area(area), x_min(x_min), x_max(x_max),
            y_min(y_min), y_max(y_max), z_min(z_min), z_max(z_max) {}

    __device__ Diffuse_Light(Vec3D light_color, float area, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max) {
        // I think I can improve this.

        this->light_color = light_color;
        this->area = area;
        this->x_min = x_min;
        this->x_max = x_max;
        this->y_min = y_min;
        this->y_max = y_max;
        this->z_min = z_min;
        this->z_max = z_max;
    }

    __device__ bool evaluate(const Ray &r_in, const Intersection_Information &rec, Vec3D &attenuation, Ray &scattered,
                             curandState *local_rand_state, float& pdf, PDF* surface_pdf_ptr) const override {
        return false;
    }

    __device__ Vec3D emitted(const Vec3D &p, const Intersection_Information &intersection_information) override {
        if (!intersection_information.front_face)
            return Vec3D(0,0,0);
        return light_color;
    }

private:
    // Supporting Functions
    // -----------------------------------------------------------------------
    Vec3D light_color;                                  // radiance of the light
    float area;                                        // area of light; needed for importance sampling
    float x_min, x_max, y_min, y_max, z_min, z_max;    // coordinates of the primitive representing the light
};
#endif //UNTITLED_MATERIAL_H
