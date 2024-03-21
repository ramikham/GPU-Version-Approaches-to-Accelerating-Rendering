//
// Created by ramik on 3/15/2024.
//

#ifndef UNTITLED_PDF_H
#define UNTITLED_PDF_H

#endif //UNTITLED_PDF_H

class PDF {
public:
    __device__ virtual ~PDF() {}
    __device__ virtual float PDF_value(const Vec3D& direction) const {return 1.0f; };
    __device__ virtual Vec3D generate_a_random_direction_based_on_PDF() const {return {1.0, 1.0, 1.0}; };
    __device__ virtual Vec3D generate_a_random_direction_based_on_PDF(const Vec3D& normal) const {
        return {0,0,0};
    };
};

__device__ inline Vec3D global_to_ONB_local(const Vec3D ONB_axes[3], const Vec3D& v) {
    return v.x() * ONB_axes[0] + v.y() * ONB_axes[1] + v.z() * ONB_axes[2];
}

__device__ inline void build_ONB(const Vec3D& w, Vec3D ONB[3]) {
    // Constructs and returns an orthonormal basis from the given vector w.

    Vec3D unit_w = unit_vector(w);
    Vec3D a = (fabs(unit_w.x()) > 0.9) ? Vec3D(0, 1, 0) : Vec3D(1, 0, 0);
    Vec3D v = unit_vector(cross_product(unit_w, a));
    Vec3D u = cross_product(unit_w, v);

    ONB[0] = u;
    ONB[1] = v;
    ONB[2] = unit_w;

//    return ONB;
}

class Mixture_PDF : public PDF {
public:
    __device__ Mixture_PDF(PDF* p0, PDF* p1, curandState *local_rand_state) {
        p[0] = p0;
        p[1] = p1;
        this->local_rand_state = local_rand_state;
    }

    __device__ float PDF_value(const Vec3D &direction) const override {
        /* DEBUGGING
          std::cout << "Light PDF: " <<p[0]->PDF_value(direction) << ", Surface PDF: " << p[1]->PDF_value(direction) << std::endl;
          std::cout << direction << std::endl;
          if (std::isnan(direction.x())) {
            // Handle the case where pdf0 or pdf1 is NaN
            std::cerr << "PDF_value: NaN detected in pdf0 or pdf1." << std::endl;
            return 0.0; // Or handle it according to your requirements
          }
    */
        return 0.5f * p[0]->PDF_value(direction) + 0.5f * p[1]->PDF_value(direction);
    }

    __device__ Vec3D generate_a_random_direction_based_on_PDF() const override {
        float rand_float = curand_uniform(local_rand_state);
        if (rand_float < 0.5) {
            //    if (std::isnan(p[0]->generate_a_random_direction_based_on_PDF().x())) {
            //         std::cout << p[0]->generate_a_random_direction_based_on_PDF();
            //    }
            //    std::cout << p[0]->generate_a_random_direction_based_on_PDF() << std::endl;
            return p[0]->generate_a_random_direction_based_on_PDF();
        }
        else {
            // std::cout << p[1]->generate_a_random_direction_based_on_PDF();
            return p[1]->generate_a_random_direction_based_on_PDF();
        }
    }

private:
    PDF* p[2];
    curandState *local_rand_state;
};

class Cosine_Weighted_PDF : public PDF {
public:
    __device__ Cosine_Weighted_PDF(const Vec3D& w, curandState *local_rand_state) {
        build_ONB(w, uvw);
        this->local_rand_state = local_rand_state;
    }

    __device__ float PDF_value(const Vec3D& direction) const override {
        auto cosine_theta = dot_product(uvw[2], unit_vector(direction));
        return fmaxf(0.0f, cosine_theta / M_PI);
    }

    __device__ Vec3D generate_a_random_direction_based_on_PDF() const override {
        return global_to_ONB_local(uvw, cosine_weighted_direction(local_rand_state));
    }

private:
    __device__ inline Vec3D cosine_weighted_direction(curandState *local_rand_state) const {
        float r1 = curand_uniform(local_rand_state);
        // printf("r1 inside cons_weighted = %f", r1);
        float r2 = curand_uniform(local_rand_state);
        // printf("r2 inside cons_weighted = %f", r2);

        float phi = 6.28318530718f * r1;
        float sqrt_r2 = sqrt(r2);
        float cos_phi = cos(phi);

        return {cos_phi * sqrt_r2, sin(phi) * sqrt_r2, sqrtf(1.0f - r2)};
    }

    curandState *local_rand_state;
    Vec3D uvw[3];
};

class Primitive_PDF : public PDF {
public:
    __device__ Primitive_PDF(const Primitive& primitives, const Vec3D& origin, curandState *local_rand_state) : primitives(primitives),
                                                                                                                origin(origin), local_rand_state(local_rand_state) {
        if (std::isnan(origin.x()) || std::isnan(origin.y()) || std::isnan(origin.z())) {
            printf("Primitive_PDF: NaN detected in origin during construction.\n");
            // POTENTIAL BUGGG!
        }
    }

    __device__ float PDF_value(const Vec3D& direction) const override {
        return primitives.PDF_value(origin, direction);
    }

    __device__ Vec3D generate_a_random_direction_based_on_PDF() const override {
        //  std::cout << "Group PDF: " << primitives.random(origin) << std::endl;
        //  std::cout << "Origin in primitive_pdf " << origin << std::endl;
        return primitives.random( origin, local_rand_state);
    }

private:
    curandState *local_rand_state;
    const Primitive& primitives;
    Vec3D origin;
};
