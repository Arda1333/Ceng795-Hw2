#include <iostream>
#include "parser.h"
#include "raytracer.h"
#include <cstdint>
#include <chrono>
#include <algorithm>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"




int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <scene_file>" << std::endl;
        return 1;
    }    

    const char* scene_file = argv[1];
    
    std::string s(argv[1]);

    if (s.size() < 5 || s.substr(s.size()-4, 4) == ".ply") {
        std::cerr << "Can only read JSON files unfortunately." << std::endl;
        return 1;
    }

    std::cout << "Loading scene from " << scene_file << "..." << std::endl;

    parser::Scene scene;
    scene.loadFromJSON(scene_file);

    std::cout << "Rendering..." << std::endl;

    std::cout << "Gaze: " << scene.cameras[0].gaze.x << ", " << scene.cameras[0].gaze.y << ", " << scene.cameras[0].gaze.z << std::endl;
    std::cout << "Up: " << scene.cameras[0].up.x << ", " << scene.cameras[0].up.y << ", " << scene.cameras[0].up.z << std::endl;
    std::cout << "Position: " << scene.cameras[0].position.x << ", " << scene.cameras[0].position.y << ", " << scene.cameras[0].position.z << std::endl;
    std::cout << "Corner: " << scene.cameras[0].corner.x << ", " << scene.cameras[0].corner.y << ", " << scene.cameras[0].corner.z << std::endl;
    std::cout << "Total pixels: " << scene.cameras[0].image_height * scene.cameras[0].image_width << std::endl;

    // Starting the timer
    auto start = std::chrono::high_resolution_clock::now();


    // Apply the transformations
    for (auto& triangle : scene.triangles){
        triangle.v0 = parser::matrixMult(triangle.transformation, parser::Vec4f(triangle.v0));
        triangle.v1 = parser::matrixMult(triangle.transformation, parser::Vec4f(triangle.v1));
        triangle.v2 = parser::matrixMult(triangle.transformation, parser::Vec4f(triangle.v2));
        triangle.normal = parser::matrixMult(triangle.normal_transform, parser::Vec4f(triangle.normal));
    }

    for (auto& sphere : scene.spheres){
        sphere.center_vertex = parser::matrixMult(sphere.transformation, parser::Vec4f(sphere.center_vertex));
    }

    for (auto& plane : scene.planes){
        plane.point = parser::matrixMult(plane.transformation, parser::Vec4f(plane.point));
        plane.normal = parser::matrixMult(plane.normal_transform, parser::Vec4f(plane.normal));
    }

    for (auto& mesh : scene.meshes){
        for (auto& face : mesh.faces){
            face.v0 = parser::matrixMult(mesh.transformation, parser::Vec4f(face.v0));
            face.v1 = parser::matrixMult(mesh.transformation, parser::Vec4f(face.v1));
            face.v2 = parser::matrixMult(mesh.transformation, parser::Vec4f(face.v2));
            face.normal = parser::matrixMult(mesh.normal_transform, parser::Vec4f(face.normal));
        }
    }



    long no_hits = 0;
    long black_pixels = 0;
    for (auto camera : scene.cameras) {
        //unsigned char image[camera.image_height * camera.image_width * 3] = {0};

        std::vector<unsigned char> image(camera.image_height * camera.image_width * 3, 0);
        unsigned char* img_ptr = image.data();

        // Ray tracing loop (inside camera loop)
        float horiz_step = (camera.near_plane.y - camera.near_plane.x) / camera.image_width;
        float vert_step = (camera.near_plane.w - camera.near_plane.z) / camera.image_height;
        for (int i = 0; i < camera.image_height; ++i) {
            for (int j = 0; j < camera.image_width; ++j) {
                // Compute pixel center in the view plane
                float s_u = (j + 0.5f) * horiz_step; // j = horizontal column
                float s_v = (i + 0.5f) * vert_step;  // i = vertical row

                parser::Vec3f pointOnPlane = parser::vector_add(
                    camera.corner,
                    parser::vector_sub(
                        parser::scalar_mult(camera.right, s_u),
                        parser::scalar_mult(camera.up, s_v)
                    )
                );

                parser::Vec3f rayDirection = parser::normalize(parser::vector_sub(pointOnPlane, camera.position));
                rayDirection = parser::normalize(rayDirection);
                // Trace the ray recursively
                parser::Vec3i color = trace(camera.position, rayDirection, 0, scene);
                // Write to image
                size_t idx = 3 * (i * camera.image_width + j);
                image[idx]   = color.x;
                image[idx+1] = color.y;
                image[idx+2] = color.z;
            }
        }


        // End the timer
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        std::cout << "Execution time: " << duration.count() << " seconds\n";
        
        // Save image using stb_image_write
        stbi_write_png(camera.image_name.c_str(), camera.image_width, camera.image_height, 3, img_ptr, camera.image_width * 3);
        std::cout << "Image saved to " << camera.image_name << std::endl;
        std::cout << "No hits: " << no_hits << std::endl;
        std::cout << "Black pixels despite a hit: " << black_pixels << std::endl;

    }
    
    return 0;
}





















////////////////////////// Ray-Object Intersection Functions //////////////////////////

float intersectsPlane(parser::Vec3f& rayOrigin, parser::Vec3f& rayDirection, parser::Plane& plane, float t_min, parser::Scene& scene) {

    float t = parser::dot(parser::vector_sub(plane.point, rayOrigin), plane.normal) / parser::dot(rayDirection, plane.normal);

    if (t > t_min) {
        //std::cout << "Plane intersection at t: " << t << std::endl;
        return t;
    } else {
        return -1;
    }
}


float intersectsSphere(parser::Vec3f& rayOrigin, parser::Vec3f& rayDirection, parser::Sphere& sphere, float t_min, parser::Scene& scene) {
    
    float a = parser::dot(rayDirection, rayDirection);
    float b = 2.0f * parser::dot(rayDirection, parser::vector_sub(rayOrigin, sphere.center_vertex));
    float c = parser::dot(parser::vector_sub(rayOrigin, sphere.center_vertex), parser::vector_sub(rayOrigin, sphere.center_vertex)) - sphere.radius * sphere.radius;

    float discriminant = b*b - 4*a*c;
    
    if (discriminant < 0) {
        return -1;
    } 

    float t1 = (-b - std::sqrt(discriminant)) / (2.0 * a);
    float t2 = (-b + std::sqrt(discriminant)) / (2.0 * a);

    if (t1 > t_min) {
        // std::cout << "Sphere intersection at t: " << t1 << std::endl;
        return t1;
    } else if (t2 > t_min) {
        // std::cout << "Sphere intersection at t: " << t2 << std::endl;
        return t2;
    } else {
        return -1;
    }
}


float intersectsTriangle(parser::Vec3f& rayOrigin, parser::Vec3f& rayDirection, parser::Triangle& triangle, float t_min, parser::Scene& scene) {
    parser::Vec3f a = triangle.v0;
    parser::Vec3f b = triangle.v1;
    parser::Vec3f c = triangle.v2;

    parser::Matrix3f ma(parser::vector_sub(a,b), parser::vector_sub(a,c), rayDirection);
    parser::Matrix3f mb(parser::vector_sub(a, rayOrigin), parser::vector_sub(a,c), rayDirection);
    parser::Matrix3f my(parser::vector_sub(a,b), parser::vector_sub(a, rayOrigin), rayDirection);
    parser::Matrix3f mt(parser::vector_sub(a,b), parser::vector_sub(a,c), parser::vector_sub(a, rayOrigin));

    float det_ma = parser::determinant(ma);
    float det_mb = parser::determinant(mb);
    float det_my = parser::determinant(my);
    float det_mt = parser::determinant(mt);

    // Divisioın by zero check
    if (std::fabs(det_ma) < 1e-6f) return -1.0f;

    float beta = det_mb / det_ma;
    float gama = det_my / det_ma;
    float t = det_mt / det_ma;


    if (beta + gama > 1 || beta < 0 || gama < 0 || t < t_min) return -1.0;
    return t;
}


float intersectsObject(parser::Vec3f& rayOrigin, parser::Vec3f& rayDirection, float t_min, int& material_id, parser::Vec3f& surfaceNormal, parser::Vec3f& intersectionPoint, parser::Scene& scene) {
    float closest_t = std::numeric_limits<float>::max();
    
    for (auto sphere : scene.spheres) {
        float t = intersectsSphere(rayOrigin, rayDirection, sphere, t_min, scene);
        if (t > 0 && t < closest_t) {
            closest_t = t;
            material_id = sphere.material_id;
            surfaceNormal = parser::normalize(parser::vector_sub(parser::vector_add(rayOrigin, parser::scalar_mult(rayDirection, t)), sphere.center_vertex));
        }
    }

    for (auto triangle : scene.triangles) {
        float t = intersectsTriangle(rayOrigin, rayDirection, triangle, t_min, scene);
        if (t > 0 && t < closest_t) {
            closest_t = t;
            material_id = triangle.material_id;
            surfaceNormal = triangle.normal;
        }
    }

    for (auto plane : scene.planes) {
        float t = intersectsPlane(rayOrigin, rayDirection, plane, t_min, scene);
        if (t > 0 && t < closest_t) {
            closest_t = t;
            material_id = plane.material_id;
            surfaceNormal = plane.normal;
        }
    }

    for (auto mesh : scene.meshes) {
        for (auto face : mesh.faces) {
            if (face.triangle) {
                parser::Triangle triangle;
                triangle.v0 = face.v0;
                triangle.v1 = face.v1;
                triangle.v2 = face.v2;
                triangle.normal = face.normal;

                float t = intersectsTriangle(rayOrigin, rayDirection, triangle, t_min, scene);
                if (t > 0 && t < closest_t) {
                    closest_t = t;
                    material_id = mesh.material_id;
                    surfaceNormal = triangle.normal;
                }
            }
        }
    }

    if (closest_t < std::numeric_limits<float>::max()) {
        intersectionPoint = parser::vector_add(rayOrigin, parser::scalar_mult(rayDirection, closest_t));
        return closest_t;
    }
    else return -1;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////




////////////////////////// Shadow, Color, and Mirror Calculation Function //////////////////////////

bool calculateShadow(parser::Vec3f& interectionPoint, parser::Vec3f& lightDirection, float t_min, float t_light, parser::Scene& scene){

    bool inShadow = false;

    // Check intersections with all objects
    for (auto sphere : scene.spheres) {
        float t = intersectsSphere(interectionPoint, lightDirection, sphere, t_min, scene);
        if (t > 0 && t < t_light) {
            inShadow = true;
            break;
        }
    }

    if (!inShadow) {
        for (auto triangle : scene.triangles) {
            float t = intersectsTriangle(interectionPoint, lightDirection, triangle, t_min, scene);
            if (t > 0 && t < t_light) {
                inShadow = true;
                break;
            }
        }
    }

    if (!inShadow) {
        for (auto plane : scene.planes) {
            float t = intersectsPlane(interectionPoint, lightDirection, plane, t_min, scene);
            if (t > 0 && t < t_light) {
                inShadow = true;
                break;
            }
        }
    }

    if (!inShadow) {
        for (auto mesh : scene.meshes) {
            if (inShadow) break;

            for (auto face : mesh.faces) {
                if (face.triangle) {
                    parser::Triangle triangle;
                    triangle.v0 = face.v0;
                    triangle.v1 = face.v1;
                    triangle.v2 = face.v2;

                    // Calculate Normal from VertexData and Indices
                    parser::Vec3f vec1, vec2;
                    vec1 = parser::vector_sub(triangle.v1, triangle.v0);
                    vec2 = parser::vector_sub(triangle.v2, triangle.v0);
                    triangle.normal = parser::normalize(parser::cross(vec1, vec2));

                    float t = intersectsTriangle(interectionPoint, lightDirection, triangle, t_min, scene);
                    if (t > 0 && t < t_light) {
                        inShadow = true;
                        break;
                    }
                }
            }
        }
    }
    
    
    return inShadow;
}

parser::Vec3i calculateColor(parser::Vec3f& rayOrigin, parser::Vec3f& rayDirection, parser::Vec3f& intersectionPoint, parser::Vec3f& surfaceNormal, float& closest_t, float& t_min, parser::Material& material, parser::Scene& scene) {
    parser::Vec3i color = {0, 0, 0};
    
    if (closest_t == std::numeric_limits<float>::max()) {
        // Assign background color to pixel
        color.x = scene.background_color.x;
        color.y = scene.background_color.y;
        color.z = scene.background_color.z;
    }
    else{
        // Ambient Shading
        color.x += (int)(material.ambient.x * scene.ambient_light.x);
        color.y += (int)(material.ambient.y * scene.ambient_light.y);
        color.z += (int)(material.ambient.z * scene.ambient_light.z);


        // Diffuse and Specular Shading
        for (auto light : scene.point_lights) {

            // Check for shadow
            parser::Vec3f tempIntersectionPoint = parser::vector_add(intersectionPoint, parser::scalar_mult(surfaceNormal, scene.shadow_ray_epsilon));
            parser::Vec3f lightDirection = parser::vector_sub(light.position, intersectionPoint);
            float t_light = parser::magnitude(lightDirection);
            lightDirection = parser::normalize(lightDirection);

            bool inShadow = calculateShadow(tempIntersectionPoint, lightDirection, t_min, t_light, scene);

            if (!inShadow) {
                // Diffuse
                float light_distance = parser::magnitude(parser::vector_sub(light.position, intersectionPoint));
                color.x += (int)(material.diffuse.x * (light.intensity.x / (light_distance * light_distance)) * std::max(0.0f, parser::dot(surfaceNormal, parser::normalize(parser::vector_sub(light.position, intersectionPoint)))));
                color.y += (int)(material.diffuse.y * (light.intensity.y / (light_distance * light_distance)) * std::max(0.0f, parser::dot(surfaceNormal, parser::normalize(parser::vector_sub(light.position, intersectionPoint)))));
                color.z += (int)(material.diffuse.z * (light.intensity.z / (light_distance * light_distance)) * std::max(0.0f, parser::dot(surfaceNormal, parser::normalize(parser::vector_sub(light.position, intersectionPoint)))));

                // Specular
                parser::Vec3f halfVector = parser::normalize(parser::vector_add(parser::normalize(parser::vector_sub(rayOrigin, intersectionPoint)), parser::normalize(parser::vector_sub(light.position, intersectionPoint))));
                color.x += (int)(material.specular.x * (light.intensity.x / (light_distance * light_distance)) * std::pow(std::max(0.0f, parser::dot(surfaceNormal, halfVector)), material.phong_exponent));
                color.y += (int)(material.specular.y * (light.intensity.y / (light_distance * light_distance)) * std::pow(std::max(0.0f, parser::dot(surfaceNormal, halfVector)), material.phong_exponent));
                color.z += (int)(material.specular.z * (light.intensity.z / (light_distance * light_distance)) * std::pow(std::max(0.0f, parser::dot(surfaceNormal, halfVector)), material.phong_exponent));
            
            }

            // Clamp color values to [0, 255]
            color.x = std::min(255, color.x);
            color.y = std::min(255, color.y);
            color.z = std::min(255, color.z);
            
        }
    }

    return color;
}


parser::Vec3f calculateMirror(parser::Vec3f& currOrigin, parser::Vec3f& currNormal, parser::Vec3f& currDir, parser::Vec3f intersectionPoint, float t_min, parser::Vec3f accumulated, parser::Vec3f reflectWeight, float eps, int maxDepth, parser::Scene scene) {
    for (int depth = 0; depth < maxDepth; depth++) {
        currDir = parser::normalize(currDir);

        // Calculate reflect direction
        parser::Vec3f originDir = parser::normalize(parser::vector_sub(currOrigin, intersectionPoint));
        float dn = parser::dot(originDir, currNormal);
        parser::Vec3f reflectDir = parser::vector_sub(parser::scalar_mult(currNormal, 2.0f * dn), originDir);
        reflectDir = parser::normalize(reflectDir);

        // Offset origin along the reflected direction
        currOrigin = intersectionPoint;
        currOrigin = parser::vector_add(currOrigin, parser::scalar_mult(reflectDir, eps));

        // Trace the reflected ray
        int r_material_id = -1;
        parser::Vec3f rNormal;
        float r_t = intersectsObject(currOrigin, reflectDir, t_min, r_material_id, rNormal, intersectionPoint, scene);

        // No hit
        if (r_material_id == -1 || r_t <= t_min || r_t == std::numeric_limits<float>::max()) {
            accumulated.x += reflectWeight.x * scene.background_color.x;
            accumulated.y += reflectWeight.y * scene.background_color.y;
            accumulated.z += reflectWeight.z * scene.background_color.z;
            break;
        }

        // Compute Color
        parser::Material rmat = scene.materials[r_material_id - 1];
        parser::Vec3i rcolor = calculateColor(currOrigin, reflectDir, intersectionPoint, rNormal, r_t, t_min, rmat, scene);

        accumulated.x += reflectWeight.x * (float)rcolor.x;
        accumulated.y += reflectWeight.y * (float)rcolor.y;
        accumulated.z += reflectWeight.z * (float)rcolor.z;

        // Break if new material is not a mirror
        if (!rmat.is_mirror && !rmat.is_conductor && !rmat.is_dielectric) break;

        // Update reflection weights
        reflectWeight = rmat.mirror;

        // update current origin, direction and normal for next iteration
        currOrigin = intersectionPoint;
        currNormal = rNormal;
        currDir = reflectDir; // next incident direction is the reflected direction we just traced
    }

    return accumulated;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////




// Recursive ray tracing function: returns RGB color for a ray
parser::Vec3i trace(const parser::Vec3f &orig, const parser::Vec3f &dir, int depth, const parser::Scene &scene) {
    // Base case: exceed recursion depth
    if (depth > scene.max_recursion_depth) {
        return scene.background_color;
    }

    // Choose t_min: use a tiny epsilon for primary rays (depth==0),
    // and use the scene.shadow_ray_epsilon for secondary rays.
    float t_min = 1e-6f;

    // Intersect scene
    int mat_id = -1;
    parser::Vec3f normal, hitPoint;
    float t = intersectsObject((parser::Vec3f&)orig, (parser::Vec3f&)dir, t_min,
                               mat_id, normal, hitPoint, const_cast<parser::Scene&>(scene));

    // intersectsObject returns -1 for no hit
    if (t < 0.0f || mat_id < 0) {
        return scene.background_color;
    }

    const parser::Material &mat = scene.materials[mat_id - 1];

    // Ensure normal faces the incoming ray (so N·L and N·H are meaningful)
    if (parser::dot(dir, normal) > 0.0f) {
        normal = parser::neg(normal);
    }

    // Accumulate in float
    parser::Vec3f color_f(0.0f, 0.0f, 0.0f);

    // Ambient term
    color_f.x += mat.ambient.x * scene.ambient_light.x;
    color_f.y += mat.ambient.y * scene.ambient_light.y;
    color_f.z += mat.ambient.z * scene.ambient_light.z;

    // Local lighting
    for (auto &light : scene.point_lights) {
        parser::Vec3f L = parser::vector_sub(light.position, hitPoint);
        float distToLight = parser::magnitude(L);
        if (distToLight <= 0.0f) continue;
        L = parser::normalize(L);

        // Shadow ray origin
        parser::Vec3f shadowOrig = parser::vector_add(hitPoint, parser::scalar_mult(normal, scene.shadow_ray_epsilon));
        bool inShadow = calculateShadow(shadowOrig, L, scene.shadow_ray_epsilon, distToLight, const_cast<parser::Scene&>(scene));
        if (inShadow) continue;

        float NdotL = std::max(0.0f, parser::dot(normal, L));
        color_f.x += mat.diffuse.x  * (light.intensity.x / (distToLight*distToLight)) * NdotL;
        color_f.y += mat.diffuse.y  * (light.intensity.y / (distToLight*distToLight)) * NdotL;
        color_f.z += mat.diffuse.z  * (light.intensity.z / (distToLight*distToLight)) * NdotL;

        // Specular (Phong with half-vector)
        parser::Vec3f V = parser::vector_sub(orig, hitPoint);
        parser::Vec3f H = parser::normalize(parser::vector_add(parser::normalize(V), L));
        float NdotH = std::max(0.0f, parser::dot(normal, H));
        float specFactor = std::pow(NdotH, mat.phong_exponent);
        color_f.x += mat.specular.x * (light.intensity.x / (distToLight*distToLight)) * specFactor;
        color_f.y += mat.specular.y * (light.intensity.y / (distToLight*distToLight)) * specFactor;
        color_f.z += mat.specular.z * (light.intensity.z / (distToLight*distToLight)) * specFactor;
    }

    // Reflection (mirror or conductor)
    if (mat.is_mirror || mat.is_conductor) {
        parser::Vec3f reflectDir = parser::vector_sub(dir, parser::scalar_mult(normal, 2.0f * parser::dot(dir, normal)));
        reflectDir = parser::normalize(reflectDir);
        parser::Vec3f reflectOrig = parser::vector_add(hitPoint, parser::scalar_mult(reflectDir, scene.shadow_ray_epsilon));
        parser::Vec3i reflColor = trace(reflectOrig, reflectDir, depth + 1, scene);
        parser::Vec3f reflF((float)reflColor.x, (float)reflColor.y, (float)reflColor.z);
        // Apply mirror color
        reflF.x *= mat.mirror.x; reflF.y *= mat.mirror.y; reflF.z *= mat.mirror.z;

        if (mat.is_conductor) {
            float cosTh = std::clamp(parser::dot(parser::neg(dir), normal), 0.0f, 1.0f);
            float a2 = mat.absorb_ind * mat.absorb_ind + mat.refract_ind * mat.refract_ind;
            float two = 2.0f * mat.refract_ind * cosTh;
            float Rs = (a2 - two + cosTh*cosTh) / (a2 + two + cosTh*cosTh);
            float Rp = (a2 * (cosTh*cosTh) - two + 1.0f) / (a2 * (cosTh*cosTh) + two + 1.0f);
            float Fr = std::clamp((Rs + Rp) * 0.5f, 0.0f, 1.0f);
            reflF = parser::scalar_mult(reflF, Fr);
        }
        color_f.x += reflF.x;
        color_f.y += reflF.y;
        color_f.z += reflF.z;
    }

    // Refraction (dielectric)
    if (mat.is_dielectric) {
        float eta = mat.refract_ind;
        float cosTh = std::clamp(parser::dot(parser::neg(dir), normal), 0.0f, 1.0f);
        float k = 1.0f - (1.0f/(eta*eta)) * (1 - cosTh*cosTh);
        if (k >= 0.0f) {
            // refracted dir
            parser::Vec3f refrDir = parser::vector_sub(parser::scalar_mult(dir, 1.0f/eta),
                                                      parser::scalar_mult(normal, (cosTh/eta - std::sqrt(k))));
            refrDir = parser::normalize(refrDir);

            // Fresnel
            float Rpar = (eta*cosTh - std::sqrt(k)) / (eta*cosTh + std::sqrt(k));
            float Rperp = (cosTh - eta*std::sqrt(k)) / (cosTh + eta*std::sqrt(k));
            float Fr = std::clamp(0.5f*(Rpar*Rpar + Rperp*Rperp), 0.0f, 1.0f);
            float Ft = 1.0f - Fr;

            // Reflection part weighted by Fr
            parser::Vec3f reflectDir = parser::vector_sub(dir, parser::scalar_mult(normal, 2.0f * parser::dot(dir, normal)));
            reflectDir = parser::normalize(reflectDir);
            parser::Vec3f reflectOrig = parser::vector_add(hitPoint, parser::scalar_mult(reflectDir, scene.shadow_ray_epsilon));
            parser::Vec3i reflColor = trace(reflectOrig, reflectDir, depth + 1, scene);
            parser::Vec3f reflF((float)reflColor.x, (float)reflColor.y, (float)reflColor.z);
            reflF = parser::scalar_mult(reflF, Fr);
            color_f.x += reflF.x; color_f.y += reflF.y; color_f.z += reflF.z;

            // Refraction part weighted by Ft and attenuated by Beer-Lambert
            parser::Vec3f refrOrig = parser::vector_add(hitPoint, parser::scalar_mult(refrDir, scene.shadow_ray_epsilon));
            parser::Vec3i refrColor = trace(refrOrig, refrDir, depth + 1, scene);
            parser::Vec3f refrF((float)refrColor.x, (float)refrColor.y, (float)refrColor.z);

            // Find distance inside medium (exit intersection)
            int exitMat = -1; parser::Vec3f exitNorm, exitPt;
            float distInside = intersectsObject((parser::Vec3f&)refrOrig, (parser::Vec3f&)refrDir, scene.shadow_ray_epsilon,
                                                exitMat, exitNorm, exitPt, const_cast<parser::Scene&>(scene));
            // If no exit found or negative distance assume no attenuation (robust fallback)
            parser::Vec3f atten(1.0f, 1.0f, 1.0f);
            if (distInside > 0.0f) {
                atten.x = std::exp(-mat.absorb_coeff.x * distInside);
                atten.y = std::exp(-mat.absorb_coeff.y * distInside);
                atten.z = std::exp(-mat.absorb_coeff.z * distInside);
            }

            refrF.x *= Ft * atten.x; refrF.y *= Ft * atten.y; refrF.z *= Ft * atten.z;
            color_f.x += refrF.x; color_f.y += refrF.y; color_f.z += refrF.z;
        }
    }

    // Clamp and convert to integer color
    parser::Vec3i finalColor;
    finalColor.x = (int)std::min(255.0f, std::max(0.0f, color_f.x));
    finalColor.y = (int)std::min(255.0f, std::max(0.0f, color_f.y));
    finalColor.z = (int)std::min(255.0f, std::max(0.0f, color_f.z));
    return finalColor;
}
