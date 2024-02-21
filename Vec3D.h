//
// Created by ramik on 2/17/2024.
//

#ifndef UNTITLED_VEC3D_H
#define UNTITLED_VEC3D_H

#include <math.h>
#include <stdlib.h>
#include <iostream>

class Vec3D  {
public:
    __host__ __device__ Vec3D() {}
    __host__ __device__ Vec3D(float e0, float e1, float e2) { V[0] = e0; V[1] = e1; V[2] = e2; }
    __host__ __device__ inline float x() const { return V[0]; }
    __host__ __device__ inline float y() const { return V[1]; }
    __host__ __device__ inline float z() const { return V[2]; }
    __host__ __device__ inline float r() const { return V[0]; }
    __host__ __device__ inline float g() const { return V[1]; }
    __host__ __device__ inline float b() const { return V[2]; }

    __host__ __device__ inline const Vec3D& operator+() const { return *this; }
    __host__ __device__ inline Vec3D operator-() const { return Vec3D(-V[0], -V[1], -V[2]); }
    __host__ __device__ inline float operator[](int i) const { return V[i]; }
    __host__ __device__ inline float& operator[](int i) { return V[i]; };

    __host__ __device__ inline Vec3D& operator+=(const Vec3D &v2);
    __host__ __device__ inline Vec3D& operator-=(const Vec3D &v2);
    __host__ __device__ inline Vec3D& operator*=(const Vec3D &v2);
    __host__ __device__ inline Vec3D& operator/=(const Vec3D &v2);
    __host__ __device__ inline Vec3D& operator*=(const float t);
    __host__ __device__ inline Vec3D& operator/=(const float t);

    __host__ __device__ inline float length() const { return sqrt(V[0] * V[0] + V[1] * V[1] + V[2] * V[2]); }
    __host__ __device__ inline float length_squared() const { return V[0] * V[0] + V[1] * V[1] + V[2] * V[2]; }
    __host__ __device__ inline void make_unit_vector();


    float V[3];
};

inline std::istream& operator>>(std::istream &is, Vec3D &t) {
    is >> t.V[0] >> t.V[1] >> t.V[2];
    return is;
}

inline std::ostream& operator<<(std::ostream &os, const Vec3D &t) {
    os << t.V[0] << " " << t.V[1] << " " << t.V[2];
    return os;
}

__host__ __device__ inline void Vec3D::make_unit_vector() {
    float k = 1.0 / sqrt(V[0] * V[0] + V[1] * V[1] + V[2] * V[2]);
    V[0] *= k; V[1] *= k; V[2] *= k;
}

__host__ __device__ inline Vec3D operator+(const Vec3D &v1, const Vec3D &v2) {
    return Vec3D(v1.V[0] + v2.V[0], v1.V[1] + v2.V[1], v1.V[2] + v2.V[2]);
}

__host__ __device__ inline Vec3D operator-(const Vec3D &v1, const Vec3D &v2) {
    return Vec3D(v1.V[0] - v2.V[0], v1.V[1] - v2.V[1], v1.V[2] - v2.V[2]);
}

__host__ __device__ inline Vec3D operator*(const Vec3D &v1, const Vec3D &v2) {
    return Vec3D(v1.V[0] * v2.V[0], v1.V[1] * v2.V[1], v1.V[2] * v2.V[2]);
}

__host__ __device__ inline Vec3D operator/(const Vec3D &v1, const Vec3D &v2) {
    return Vec3D(v1.V[0] / v2.V[0], v1.V[1] / v2.V[1], v1.V[2] / v2.V[2]);
}

__host__ __device__ inline Vec3D operator*(float t, const Vec3D &v) {
    return Vec3D(t * v.V[0], t * v.V[1], t * v.V[2]);
}

__host__ __device__ inline Vec3D operator/(Vec3D v, float t) {
    return Vec3D(v.V[0] / t, v.V[1] / t, v.V[2] / t);
}

__host__ __device__ inline Vec3D operator*(const Vec3D &v, float t) {
    return Vec3D(t * v.V[0], t * v.V[1], t * v.V[2]);
}

__host__ __device__ inline float dot_product(const Vec3D &v1, const Vec3D &v2) {
    return v1.V[0] * v2.V[0] + v1.V[1] * v2.V[1] + v1.V[2] * v2.V[2];
}

__host__ __device__ inline Vec3D cross_product(const Vec3D &v1, const Vec3D &v2) {
    return Vec3D((v1.V[1] * v2.V[2] - v1.V[2] * v2.V[1]),
                 (-(v1.V[0] * v2.V[2] - v1.V[2] * v2.V[0])),
                 (v1.V[0] * v2.V[1] - v1.V[1] * v2.V[0]));
}


__host__ __device__ inline Vec3D& Vec3D::operator+=(const Vec3D &v){
    V[0]  += v.V[0];
    V[1]  += v.V[1];
    V[2]  += v.V[2];
    return *this;
}

__host__ __device__ inline Vec3D& Vec3D::operator*=(const Vec3D &v){
    V[0]  *= v.V[0];
    V[1]  *= v.V[1];
    V[2]  *= v.V[2];
    return *this;
}

__host__ __device__ inline Vec3D& Vec3D::operator/=(const Vec3D &v){
    V[0]  /= v.V[0];
    V[1]  /= v.V[1];
    V[2]  /= v.V[2];
    return *this;
}

__host__ __device__ inline Vec3D& Vec3D::operator-=(const Vec3D& v) {
    V[0]  -= v.V[0];
    V[1]  -= v.V[1];
    V[2]  -= v.V[2];
    return *this;
}

__host__ __device__ inline Vec3D& Vec3D::operator*=(const float t) {
    V[0]  *= t;
    V[1]  *= t;
    V[2]  *= t;
    return *this;
}

__host__ __device__ inline Vec3D& Vec3D::operator/=(const float t) {
    float k = 1.0/t;

    V[0]  *= k;
    V[1]  *= k;
    V[2]  *= k;
    return *this;
}

__host__ __device__ inline Vec3D unit_vector(Vec3D v) {
    return v / v.length();
}

__host__ __device__ inline Vec3D min(const Vec3D& a, const Vec3D& b) {
    return { fmin(a.x(), b.x()), fmin(a.y(), b.y()), fmin(a.z(), b.z()) };
}

__host__ __device__ inline Vec3D max(const Vec3D& a, const Vec3D& b) {
    return { fmax(a.x(), b.x()), fmax(a.y(), b.y()), fmax(a.z(), b.z()) };
}


#endif //UNTITLED_VEC3D_H
