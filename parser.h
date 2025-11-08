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
    };

    struct Triangle
    {
        int material_id;
        int v0, v1, v2;
        Vec3f normal;
    };

    struct Sphere
    {
        int material_id;
        int center_vertex_id;
        float radius;
    };

    struct Plane
    {
        int material_id;
        Vec3f normal;
        int point_id;
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

        //Functions
        void loadFromJSON(std::string filepath);
    };
}

#endif