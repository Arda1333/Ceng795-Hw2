#include <iostream>
#include "parser.h"
#include "raytracer.h"
#include <cstdint>
#include <chrono>

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

    std::cout << "Translations loaded: " << scene.translations.size() << std::endl;
    std::cout << "Rotations loaded: " << scene.rotations.size() << std::endl;
    std::cout << "Scalings loaded: " << scene.scalings.size() << std::endl;

    if (scene.spheres.size() > 0){
        std::cout << "Transformation count of first sphere: "
        << scene.spheres[0].rotations.size() + scene.spheres[0].translations.size() + scene.spheres[0].scalings.size()
        << std::endl;
    }

    return 0;


    std::cout << "Rendering..." << std::endl;

    // std::cout << "Gaze: " << scene.cameras[0].gaze.x << ", " << scene.cameras[0].gaze.y << ", " << scene.cameras[0].gaze.z << std::endl;
    // std::cout << "Up: " << scene.cameras[0].up.x << ", " << scene.cameras[0].up.y << ", " << scene.cameras[0].up.z << std::endl;
    // std::cout << "Position: " << scene.cameras[0].position.x << ", " << scene.cameras[0].position.y << ", " << scene.cameras[0].position.z << std::endl;
    // std::cout << "Corner: " << scene.cameras[0].corner.x << ", " << scene.cameras[0].corner.y << ", " << scene.cameras[0].corner.z << std::endl;
    // std::cout << "Total pixels: " << scene.cameras[0].image_height * scene.cameras[0].image_width << std::endl;

    // Starting the timer
    auto start = std::chrono::high_resolution_clock::now();

    long no_hits = 0;
    long black_pixels = 0;
    for (auto camera : scene.cameras) {
        //unsigned char image[camera.image_height * camera.image_width * 3] = {0};

        std::vector<unsigned char> image(camera.image_height * camera.image_width * 3, 0);
        unsigned char* img_ptr = image.data();

        // Ray tracing loop
        float horiz_step = (camera.near_plane.y - camera.near_plane.x) / camera.image_width;
        float vert_step = (camera.near_plane.w - camera.near_plane.z) / camera.image_height;
        for (int i = 0; i < camera.image_height; ++i) {
            for (int j = 0; j < camera.image_width; ++j) {

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
                
                float t_min = 1e-6f;
                rayDirection = parser::normalize(rayDirection);
                

                int material_id = -1;
                parser::Vec3f surfaceNormal, intersectionPoint;

                ////////////////////////// Check intersections with all objects //////////////////////////
                float closest_t = intersectsObject(camera.position, rayDirection, t_min, material_id, surfaceNormal, intersectionPoint, scene);
                ////////////////////////////////////////////////////////////////////////////////////////



                ////////////////////////// Shadow, Mirror, and Color Calculation //////////////////////////

                size_t idx = 3 * (i * camera.image_width + j);

                parser::Vec3i color;
                parser::Material mat;

                // Ambient, Diffuse, Specular, and Shadow calculation
                if (material_id != -1) {
                    mat = scene.materials[material_id-1];
                    color = calculateColor(camera.position, rayDirection, intersectionPoint, surfaceNormal, closest_t, t_min, mat, scene);
                }

                else color = scene.background_color;

                image[idx] = color.x;
                image[idx+1] = color.y;
                image[idx+2] = color.z;


                // Mirror, Conductor, and Dielectric calculation
                if (material_id != -1 && (mat.is_mirror || mat.is_conductor || mat.is_dielectric)) {
                    parser::Vec3f currOrigin = camera.position;
                    parser::Vec3f currNormal = surfaceNormal;
                    parser::Vec3f currDir = rayDirection;
                    parser::Vec3f mIntersectionPoint = intersectionPoint;
                    parser::Vec3f reflectWeight = scene.materials[material_id-1].mirror;
                    float eps = scene.shadow_ray_epsilon;

                    parser::Vec3f accumulated = calculateMirror(currOrigin, currNormal, currDir, mIntersectionPoint, t_min, {0,0,0}, reflectWeight, eps, scene.max_recursion_depth, scene);

                    if (mat.is_conductor){ // Apply Fresnel reflection if conductor
                        float cosTh = parser::dot(parser::neg(rayDirection), surfaceNormal);
                        cosTh = std::max(0.0f, std::min(1.0f, cosTh));

                        float Rs = mat.absorb_ind*mat.absorb_ind + mat.refract_ind*mat.refract_ind - (2 * mat.refract_ind * cosTh) + cosTh*cosTh;
                        Rs /= mat.absorb_ind*mat.absorb_ind + mat.refract_ind*mat.refract_ind + (2 * mat.refract_ind * cosTh) + cosTh*cosTh;

                        float Rp = (mat.absorb_ind*mat.absorb_ind + mat.refract_ind*mat.refract_ind) * (cosTh*cosTh) - (2 * mat.refract_ind * cosTh) + 1.0;
                        Rp /= (mat.absorb_ind*mat.absorb_ind + mat.refract_ind*mat.refract_ind) * (cosTh*cosTh) + (2 * mat.refract_ind * cosTh) + 1.0;

                        float Fr = (Rs + Rp) / 2;
                        Fr = std::max(0.0f, std::min(1.0f, Fr));

                        accumulated = parser::scalar_mult(accumulated, Fr);
                    }

                    if (mat.is_dielectric){ // Calculate refraction and Fresnel reflection if dielectric
                        float cosTh = parser::dot(parser::neg(rayDirection), surfaceNormal);
                        cosTh = std::max(0.0f, std::min(1.0f, cosTh));

                        float cosPhi = 1 - (1/mat.refract_ind)*(1/mat.refract_ind)*(1 - cosTh*cosTh);


                        if (cosPhi >= 0) { // Refraction occurs
                            cosPhi = std::sqrt(cosPhi);

                            float Rparal = mat.refract_ind*cosTh - cosPhi;
                            Rparal /= mat.refract_ind*cosTh + cosPhi;

                            float Rperp = cosTh - mat.refract_ind*cosPhi;
                            Rperp /= cosTh + mat.refract_ind*cosPhi;

                            float Fr = (Rparal*Rparal + Rperp*Rperp) / 2;
                            Fr = std::max(0.0f, std::min(1.0f, Fr));
                            float Ft = 1 - Fr;

                            // Apply the reflection ratio
                            accumulated = parser::scalar_mult(accumulated, Fr);

                            parser::Vec3f transmitDir = parser::scalar_div(parser::vector_add(rayDirection, parser::scalar_mult(surfaceNormal, cosTh)), mat.refract_ind);
                            transmitDir = parser::vector_sub(transmitDir, parser::scalar_mult(surfaceNormal, cosPhi));

                            int refMatId = -1;
                            parser::Vec3f refSurfaceNormal, refIntersectionPoint;
                            float t_attenuation = intersectsObject(intersectionPoint, transmitDir, t_min, refMatId, refSurfaceNormal, refIntersectionPoint, scene);

                            float atten_rate_x = mat.refract_ind * std::exp(-mat.absorb_coeff.x*t_attenuation);
                            float atten_rate_y = mat.refract_ind * std::exp(-mat.absorb_coeff.y*t_attenuation);
                            float atten_rate_z = mat.refract_ind * std::exp(-mat.absorb_coeff.z*t_attenuation);

                            // To find the point where the light exits the material
                            float ref_closest_t = intersectsObject(intersectionPoint, transmitDir, t_attenuation, refMatId, refSurfaceNormal, refIntersectionPoint, scene);

                            parser::Material refMat;
                            parser::Vec3i refColor;

                            if (refMatId != -1) {
                                refMat = scene.materials[refMatId-1];
                                parser::Vec3f epsIntersection = parser::vector_add(intersectionPoint, parser::scalar_mult(parser::neg(surfaceNormal), ref_closest_t+eps));
                                refColor = calculateColor(epsIntersection, transmitDir, refIntersectionPoint, refSurfaceNormal, ref_closest_t, t_attenuation, refMat, scene);
                            }
                            else refColor = scene.background_color;

                            refColor.x *= atten_rate_x * Ft;
                            refColor.y *= atten_rate_y * Ft;
                            refColor.z *= atten_rate_z * Ft;

                            accumulated.x += std::min(255.0f, (float)refColor.x);
                            accumulated.y += std::min(255.0f, (float)refColor.y);
                            accumulated.z += std::min(255.0f, (float)refColor.z);
                        }
                    }

                    // Add the accumulated color value
                    image[idx] = std::min(255, image[idx] + (int)accumulated.x);
                    image[idx+1] = std::min(255, image[idx+1] + (int)accumulated.y);
                    image[idx+2] = std::min(255, image[idx+2] + (int)accumulated.z);
                }

                
                ///////////////////////////////////////////////////////////////////////////////////

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
    parser::Vec3f p = scene.vertex_data[plane.point_id-1];

    float t = parser::dot(parser::vector_sub(p, rayOrigin), plane.normal) / parser::dot(rayDirection, plane.normal);

    if (t > t_min) {
        //std::cout << "Plane intersection at t: " << t << std::endl;
        return t;
    } else {
        return -1;
    }
}


float intersectsSphere(parser::Vec3f& rayOrigin, parser::Vec3f& rayDirection, parser::Sphere& sphere, float t_min, parser::Scene& scene) {
    parser::Vec3f center = scene.vertex_data[sphere.center_vertex_id-1];


    
    float a = parser::dot(rayDirection, rayDirection);
    float b = 2.0f * parser::dot(rayDirection, parser::vector_sub(rayOrigin, center));
    float c = parser::dot(parser::vector_sub(rayOrigin, center), parser::vector_sub(rayOrigin, center)) - sphere.radius * sphere.radius;

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
    parser::Vec3f a = scene.vertex_data[triangle.v0-1];
    parser::Vec3f b = scene.vertex_data[triangle.v1-1];
    parser::Vec3f c = scene.vertex_data[triangle.v2-1];

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
            surfaceNormal = parser::normalize(parser::vector_sub(parser::vector_add(rayOrigin, parser::scalar_mult(rayDirection, t)), scene.vertex_data[sphere.center_vertex_id-1]));
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

                // Calculate Normal from VertexData and Indices
                parser::Vec3f vec1, vec2;
                vec1 = parser::vector_sub(scene.vertex_data[triangle.v1-1], scene.vertex_data[triangle.v0-1]);
                vec2 = parser::vector_sub(scene.vertex_data[triangle.v2-1], scene.vertex_data[triangle.v0-1]);
                triangle.normal = parser::normalize(parser::cross(vec1, vec2));

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
                    vec1 = parser::vector_sub(scene.vertex_data[triangle.v1-1], scene.vertex_data[triangle.v0-1]);
                    vec2 = parser::vector_sub(scene.vertex_data[triangle.v2-1], scene.vertex_data[triangle.v0-1]);
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