#ifndef PARSER_JSON
#define PARSER_JSON

#include <string>
#include <vector>
#include <iostream>
#include <cmath>

namespace parser
{
    struct Vec3f
    {
        float x, y, z;
    };

    struct Vec3i
    {
        int x, y, z;
    };

    struct Vec4f
    {
        float x, y, z, w;
    };

    struct Matrix3f {
        // a b c
        // d e f
        // g h i
        float a, b, c, d, e, f, g, h, i;

        Matrix3f() {
            a = 0; b = 0; c = 0;
            d = 0; e = 0; f = 0;
            g = 0; h = 0; i = 0;
        }

        Matrix3f(float a1, float b1, float c1, float d1, float e1, float f1, float g1, float h1, float i1) {
            a = a1; b = b1; c = c1;
            d = d1; e = e1; f = f1;
            g = g1; h = h1; i = i1;
        }

        Matrix3f(parser::Vec3f c1, parser::Vec3f c2, parser::Vec3f c3) {
            a = c1.x; b = c1.y; c = c1.z;
            d = c2.x; e = c2.y; f = c2.z;
            g = c3.x; h = c3.y; i = c3.z;
        }
    };

    struct Matrix4f {
        // a  b  c  d
        // e  f  g  h
        // i  j  k  l
        // m  n  o  p
        float a, b, c, d;
        float e, f, g, h;
        float i, j, k, l;
        float m, n, o, p;

        Matrix4f() {
            a = 0; b = 0; c = 0; d = 0;
            e = 0; f = 0; g = 0; h = 0;
            i = 0; j = 0; k = 0; l = 0;
            m = 0; n = 0; o = 0; p = 0;
        }

        Matrix4f(float a1, float b1, float c1, float d1,
                float e1, float f1, float g1, float h1,
                float i1, float j1, float k1, float l1,
                float m1, float n1, float o1, float p1)
        {
            a = a1; b = b1; c = c1; d = d1;
            e = e1; f = f1; g = g1; h = h1;
            i = i1; j = j1; k = k1; l = l1;
            m = m1; n = n1; o = o1; p = p1;
        }
    };

    float determinant(Matrix4f m) {
        float det3_1 = m.f * (m.k * m.p - m.o * m.l) - m.g * (m.j * m.p - m.n * m.l) + m.h * (m.j * m.o - m.n * m.k);
        float det3_2 = m.e * (m.k * m.p - m.o * m.l) - m.g * (m.i * m.p - m.m * m.l) + m.h * (m.i * m.o - m.m * m.k);
        float det3_3 = m.e * (m.j * m.p - m.n * m.l) - m.f * (m.i * m.p - m.m * m.l) + m.h * (m.i * m.n - m.m * m.j);
        float det3_4 = m.e * (m.j * m.o - m.n * m.k) - m.f * (m.i * m.o - m.m * m.k) + m.g * (m.i * m.n - m.m * m.j);

        return m.a * det3_1 - m.b * det3_2 + m.c * det3_3 - m.d * det3_4;
    }

    float determinant(Matrix3f m) {
        return m.a*(m.e*m.i - m.h*m.f) + m.b*(m.g*m.f - m.d*m.i) + m.c*(m.d*m.h - m.e*m.g);  
    }

    template<typename T> float magnitude(T vec) { //Can give 0 for small vectors
        return std::sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
    }

    template<typename T> T neg(T vec) {
        return {-vec.x, -vec.y, -vec.z};
    }

    template<typename T> T normalize(T vec) {
        T norm;
        float mag = magnitude(vec);
        norm.x = vec.x / mag;
        norm.y = vec.y / mag;
        norm.z = vec.z / mag;
        return norm;
    }

    template<typename T> T cross(T a, T b) {
        T cross_product;
        cross_product.x = a.y * b.z - a.z * b.y;
        cross_product.y = a.z * b.x - a.x * b.z;
        cross_product.z = a.x * b.y - a.y * b.x;
        return cross_product;
    }

    template<typename T> float dot(T a, T b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    template<typename T> T vector_sub(T a, T b) {
        T result;
        result.x = a.x - b.x;
        result.y = a.y - b.y;
        result.z = a.z - b.z;
        return result;
    }

    template<typename T> T vector_add(T a, T b) {
        T result;
        result.x = a.x + b.x;
        result.y = a.y + b.y;
        result.z = a.z + b.z;
        return result;
    }

    template<typename T> T scalar_mult(T a, float b) {
        T result;
        result.x = a.x * b;
        result.y = a.y * b;
        result.z = a.z * b;
        return result;
    }

    template<typename T> T scalar_div(T a, float b) {
        T result;
        result.x = a.x / b;
        result.y = a.y / b;
        result.z = a.z / b;
        return result;
    }

    template<typename T> T negate(T a) {
        T result;
        result.x = -a.x;
        result.y = -a.y;
        result.z = -a.z;
        return result;
    }

    struct Camera
    {
        bool lookAt;
        Vec3f position;
        Vec3f gaze;
        Vec3f gazePoint;
        Vec3f up;
        Vec3f right;
        Vec3f corner;
        float FovY;
        Vec4f near_plane;
        float near_distance;
        int image_width, image_height;
        std::string image_name;
        std::vector<std::string> transformations;
    };

    struct PointLight
    {
        Vec3f position;
        Vec3f intensity;
    };

    struct Material
    {
        bool is_mirror = false;
        bool is_conductor = false;
        bool is_dielectric = false;
        Vec3f ambient;
        Vec3f diffuse;
        Vec3f specular;
        Vec3f mirror;
        float refract_ind;
        float absorb_ind;
        Vec3f absorb_coeff;
        float phong_exponent;
    };

    struct Face
    {
        int v0, v1, v2;
        bool triangle;
    };

    struct Mesh
    {
        bool smooth_shading;
        int material_id;
        std::string ply_file;
        std::vector<Face> faces;
        std::vector<int> translations;
        std::vector<int> rotations;
        std::vector<int> scalings;
    };

    struct Triangle
    {
        int material_id;
        int v0, v1, v2;
        Vec3f normal;
        std::vector<int> translations;
        std::vector<int> rotations;
        std::vector<int> scalings;
    };

    struct Sphere
    {
        int material_id;
        int center_vertex_id;
        float radius;
        std::vector<int> translations;
        std::vector<int> rotations;
        std::vector<int> scalings;
    };

    struct Plane
    {
        int material_id;
        Vec3f normal;
        int point_id;
        std::vector<int> translations;
        std::vector<int> rotations;
        std::vector<int> scalings;
    };


    struct Scene
    {
        //Data
        Vec3i background_color;
        float shadow_ray_epsilon;
        float intersection_test_epsilon;
        int max_recursion_depth;
        std::vector<Camera> cameras;
        Vec3f ambient_light;
        std::vector<PointLight> point_lights;
        std::vector<Material> materials;
        std::vector<Vec3f> vertex_data;
        std::vector<Mesh> meshes;
        std::vector<Triangle> triangles;
        std::vector<Sphere> spheres;
        std::vector<Plane> planes;

        std::vector<Matrix4f> translations;
        std::vector<Matrix4f> scalings;
        std::vector<Matrix4f> rotations;
        std::vector<Matrix4f> composites;

        //Functions
        void loadFromJSON(std::string filepath);
    };
}

#endif