#include "parser.h"
#include "nlohmann/json.hpp"
using json = nlohmann::json;
#include <sstream>
#include <fstream>
#include <cmath>

void parser::Scene::loadFromJSON(std::string filepath){
    std::stringstream stream;

    std::ifstream f(filepath);
    json data = json::parse(f);
    

    //////////////////////////////// Get MaxRecursionDepth ////////////////////////////////
    auto element = data["Scene"]["MaxRecursionDepth"];
    if (element.is_null()){
        stream << "0" << std::endl;
    } else {
        std::string depth = element.get<std::string>();
        stream << depth << std::endl;
        //std::cout << depth << std::endl;
    }
    stream >> max_recursion_depth;
    //std::cout << "MaxRecursionDepth: " << max_recursion_depth << std::endl;

    ////////////////////////////////////////////////////////////////////////////////////////////////

    // std::cout << "Got recursion depth\n";


    //////////////////////////////// Get BackgroundColor ////////////////////////////////
    element = data["Scene"]["BackgroundColor"];
    if (element.is_null()){
        stream << "0 0 0" << std::endl;
    } else {
        std::string color_str = element.get<std::string>();
        stream << color_str << std::endl;
        //std::cout << color_str << std::endl;
    }
    stream >> background_color.x >> background_color.y >> background_color.z;
    //std::cout << "BackgroundColor: " << background_color.x << " " << background_color.y << " " << background_color.z << std::endl;

    ////////////////////////////////////////////////////////////////////////////////////////////////

    // std::cout << "Got background color\n";


    //////////////////////////////// Get ShadowRayEpsilon ////////////////////////////////
    element = data["Scene"]["ShadowRayEpsilon"];
    if (element.is_null()){
        stream << "0.001" << std::endl;
    } else {
        std::string epsilon = element.get<std::string>();
        stream << epsilon << std::endl;
        //std::cout << epsilon << std::endl;
    }
    stream >> shadow_ray_epsilon;
    //std::cout << "ShadowRayEpsilon: " << shadow_ray_epsilon << std::endl;

    ////////////////////////////////////////////////////////////////////////////////////////////////

    // std::cout << "Got shadow ray epsilon\n";

    
    //////////////////////////////// Get IntersectionTestEpsilon ////////////////////////////////
    element = data["Scene"]["IntersectionTestEpsilon"];
    if (element.is_null()){
        stream << "1e-6" << std::endl;
    } else {
        std::string epsilon = element.get<std::string>();
        stream << epsilon << std::endl;
        //std::cout << epsilon << std::endl;
    }
    stream >> intersection_test_epsilon;
    //std::cout << "IntersectionTestEpsilon: " << intersection_test_epsilon << std::endl;

    ////////////////////////////////////////////////////////////////////////////////////////////////

    // std::cout << "Got Intersection epsilon\n";


    //////////////////////////////// Get Cameras ////////////////////////////////

    element = data["Scene"]["Cameras"];
    element = element["Camera"];

    // Single Camera
    if (element.type() != json::value_t::array && !element.is_null()) {
        Camera camera;
        
        auto child = element["Position"];
        stream << child.get<std::string>() << std::endl;
        stream >> camera.position.x >> camera.position.y >> camera.position.z;

        child = element["NearDistance"];
        stream << child.get<std::string>() << std::endl;
        stream >> camera.near_distance;

        child = element["ImageResolution"];
        stream << child.get<std::string>() << std::endl;
        stream >> camera.image_width >> camera.image_height;

        child = element["_type"];
        if (!child.is_null()){
            camera.lookAt = true;

            child = element["GazePoint"];
            stream << child.get<std::string>() << std::endl;
            stream >> camera.gazePoint.x >> camera.gazePoint.y >> camera.gazePoint.z;
            
            // Calculate Gaze from Position and GazePoint
            Vec3f temp_gaze;
            temp_gaze.x = camera.gazePoint.x - camera.position.x;
            temp_gaze.y = camera.gazePoint.y - camera.position.y;
            temp_gaze.z = camera.gazePoint.z - camera.position.z;

            camera.gaze = normalize(temp_gaze);

            child = element["FovY"];
            stream << child.get<std::string>() << std::endl;
            stream >> camera.FovY;

            // Calculate NearPlane from FovY and NearDistance
            camera.near_plane.w = tan((camera.FovY * M_PI / 180) / 2) * camera.near_distance;
            camera.near_plane.z = -camera.near_plane.w;
            float aspect_ratio = (float)camera.image_width / (float)camera.image_height;
            camera.near_plane.x = -camera.near_plane.w * aspect_ratio;
            camera.near_plane.y = -camera.near_plane.x;

        } else {
            camera.lookAt = false;

            child = element["Gaze"];
            stream << child.get<std::string>() << std::endl;
            stream >> camera.gaze.x >> camera.gaze.y >> camera.gaze.z;

            child = element["NearPlane"];
            stream << child.get<std::string>() << std::endl;
            stream >> camera.near_plane.x >> camera.near_plane.y >> camera.near_plane.z >> camera.near_plane.w;
        }
        stream.str("");
        stream.clear();


        child = element["Up"];
        stream << child.get<std::string>() << std::endl;
        stream >> camera.up.x >> camera.up.y >> camera.up.z;

        // Adjust Up vector to be orthogonal to Gaze vector
        if (dot( camera.gaze, camera.up) != 0) {
            Vec3f w = scalar_div(camera.gaze, magnitude(camera.gaze));
            w = negate(w);
            Vec3f v = normalize(camera.up);
            Vec3f u = cross( v, w );    // u is already normalized since v and w are orthogonal and normalized
            camera.up = cross( w, u );  // up is already normalized since w and u are orthogonal and normalized
        }

        camera.right = cross( camera.gaze, camera.up );

        Vec3f center = vector_add(camera.position, scalar_mult(camera.gaze, camera.near_distance));
        camera.corner = vector_add(vector_add(center, scalar_mult(camera.up, camera.near_plane.w)), scalar_mult(camera.right, camera.near_plane.x));

        child = element["ImageName"];
        stream << child.get<std::string>() << std::endl;
        stream >> camera.image_name;

        cameras.push_back(camera);
    }
    else {  // Multiple Cameras
        for (auto& item : element.items()) {
            Camera camera;

            auto child = item.value()["Position"];
            stream << child.get<std::string>() << std::endl;
            stream >> camera.position.x >> camera.position.y >> camera.position.z;

            child = item.value()["NearDistance"];
            stream << child.get<std::string>() << std::endl;
            stream >> camera.near_distance;

            child = item.value()["ImageResolution"];
            stream << child.get<std::string>() << std::endl;
            stream >> camera.image_width >> camera.image_height;

            child = item.value()["_type"];
            if (!child.is_null()){
                camera.lookAt = true;

                child = item.value()["GazePoint"];
                stream << child.get<std::string>() << std::endl;
                stream >> camera.gazePoint.x >> camera.gazePoint.y >> camera.gazePoint.z;
                
                // Calculate Gaze from Position and GazePoint
                Vec3f temp_gaze;
                temp_gaze.x = camera.gazePoint.x - camera.position.x;
                temp_gaze.y = camera.gazePoint.y - camera.position.y;
                temp_gaze.z = camera.gazePoint.z - camera.position.z;

                camera.gaze = normalize(temp_gaze);

                child = item.value()["FovY"];
                stream << child.get<std::string>() << std::endl;
                stream >> camera.FovY;

                // Calculate NearPlane from FovY and NearDistance
                camera.near_plane.w = tan((camera.FovY * M_PI / 180) / 2) * camera.near_distance;
                camera.near_plane.z = -camera.near_plane.w;
                float aspect_ratio = (float)camera.image_width / (float)camera.image_height;
                camera.near_plane.x = -camera.near_plane.w * aspect_ratio;
                camera.near_plane.y = -camera.near_plane.x;

            } else {
                camera.lookAt = false;

                child = item.value()["Gaze"];
                stream << child.get<std::string>() << std::endl;
                stream >> camera.gaze.x >> camera.gaze.y >> camera.gaze.z;

                child = item.value()["NearPlane"];
                stream << child.get<std::string>() << std::endl;
                stream >> camera.near_plane.x >> camera.near_plane.y >> camera.near_plane.z >> camera.near_plane.w;
                stream.str("");
                stream.clear();
            }

            child = item.value()["Up"];
            stream << child.get<std::string>() << std::endl;
            stream >> camera.up.x >> camera.up.y >> camera.up.z;

            // Adjust Up vector to be orthogonal to Gaze vector
            if (dot( camera.gaze, camera.up) != 0) {
                Vec3f w = scalar_div(camera.gaze, magnitude(camera.gaze));
                w = negate(w);
                Vec3f v = normalize(camera.up);
                Vec3f u = normalize(cross( v, w ));
                camera.up = normalize(cross( w, u ));
            }

            camera.right = cross( camera.gaze, camera.up );

            Vec3f center = vector_add(camera.position, scalar_mult(camera.gaze, camera.near_distance));
            camera.corner = vector_add(vector_add(center, scalar_mult(camera.up, camera.near_plane.w)), scalar_mult(camera.right, camera.near_plane.x));

            child = item.value()["ImageName"];
            stream << child.get<std::string>() << std::endl;
            stream >> camera.image_name;

            cameras.push_back(camera);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////


    // std::cout << "Got cameras\n";

    //////////////////////////////// Get Lights ////////////////////////////////
    element = data["Scene"]["Lights"];
    stream << element["AmbientLight"].get<std::string>() << std::endl;
    stream >> ambient_light.x >> ambient_light.y >> ambient_light.z;
    element = element["PointLight"];
    
    // Single PointLight
    if (element.type() != json::value_t::array && !element.is_null()){
        PointLight point_light;

        auto child = element["Position"];
        stream << child.get<std::string>() << std::endl;
        stream >> point_light.position.x >> point_light.position.y >> point_light.position.z;

        child = element["Intensity"];
        stream << child.get<std::string>() << std::endl;
        stream >> point_light.intensity.x >> point_light.intensity.y >> point_light.intensity.z;

        point_lights.push_back(point_light);
    }
    else { // Multiple PointLights
        for (auto& item : element.items()) {
            PointLight point_light;

            auto child = item.value()["Position"];
            stream << child.get<std::string>() << std::endl;
            stream >> point_light.position.x >> point_light.position.y >> point_light.position.z;

            child = item.value()["Intensity"];
            stream << child.get<std::string>() << std::endl;
            stream >> point_light.intensity.x >> point_light.intensity.y >> point_light.intensity.z;

            point_lights.push_back(point_light);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    
    // std::cout << "Got lights\n";

    //////////////////////////////// Get Materials ////////////////////////////////
    element = data["Scene"]["Materials"]["Material"];
    
    // Single Material
    if (element.type() != json::value_t::array && !element.is_null()){
        Material material;

        auto child = element["AmbientReflectance"];
        stream << child.get<std::string>() << std::endl;
        stream >> material.ambient.x >> material.ambient.y >> material.ambient.z;

        child = element["DiffuseReflectance"];
        stream << child.get<std::string>() << std::endl;
        stream >> material.diffuse.x >> material.diffuse.y >> material.diffuse.z;

        child = element["SpecularReflectance"];
        stream << child.get<std::string>() << std::endl;    
        stream >> material.specular.x >> material.specular.y >> material.specular.z;

        child = element["PhongExponent"];
        stream << child.get<std::string>() << std::endl;
        stream >> material.phong_exponent;

        child = element["MirrorReflectance"];
        if (child.is_null()){
            material.mirror.x = material.mirror.y = material.mirror.z = 0;
        } else {
            stream << child.get<std::string>() << std::endl;
            stream >> material.mirror.x >> material.mirror.y >> material.mirror.z;
        }

        child = element["RefractionIndex"];
        if (child.is_null()){
            material.refract_ind = 0.0f;
        }
        else {
            stream << child.get<std::string>() << std::endl;
            stream >> material.refract_ind;
        }

        child = element["AbsorptionIndex"];
        if (child.is_null()){
            material.absorb_ind = 0.0f;
        }
        else {
            stream << child.get<std::string>() << std::endl;
            stream >> material.absorb_ind;
        }

        child = element["AbsorptionCoefficient"];
        if (child.is_null()){
            material.absorb_coeff = {0, 0, 0};
        }
        else {
            stream << child.get<std::string>() << std::endl;
            stream >> material.absorb_coeff.x >> material.absorb_coeff.y >> material.absorb_coeff.z;
        }

        child = element["_type"];
        if (!child.is_null()){
            std::string mat_type = child.get<std::string>();

            if (mat_type == "mirror") material.is_mirror = true;

            else if (mat_type == "conductor") material.is_conductor = true;

            else if (mat_type == "dielectric") material.is_dielectric = true;
        }

        materials.push_back(material);
    }
    else { // Multiple Materials
        for (auto& item : element.items()) {
            Material material;

            auto child = item.value()["AmbientReflectance"];
            stream << child.get<std::string>() << std::endl;
            stream >> material.ambient.x >> material.ambient.y >> material.ambient.z;

            child = item.value()["DiffuseReflectance"];
            stream << child.get<std::string>() << std::endl;
            stream >> material.diffuse.x >> material.diffuse.y >> material.diffuse.z;

            child = item.value()["SpecularReflectance"];
            stream << child.get<std::string>() << std::endl;
            stream >> material.specular.x >> material.specular.y >> material.specular.z;

            child = item.value()["PhongExponent"];
            if (child.is_null()){
                material.phong_exponent = 0;
            }
            else {
                stream << child.get<std::string>() << std::endl;
                stream >> material.phong_exponent;
            }

            child = item.value()["MirrorReflectance"];
            if (child.is_null()){
                material.mirror.x = material.mirror.y = material.mirror.z = 0;
            } else {
                stream << child.get<std::string>() << std::endl;
                stream >> material.mirror.x >> material.mirror.y >> material.mirror.z;
            }

            child = item.value()["RefractionIndex"];
            if (child.is_null()){
                material.refract_ind = 0.0f;
            }
            else {
                stream << child.get<std::string>() << std::endl;
                stream >> material.refract_ind;
            }

            child = item.value()["AbsorptionIndex"];
            if (child.is_null()){
                material.absorb_ind = 0.0f;
            }
            else {
                stream << child.get<std::string>() << std::endl;
                stream >> material.absorb_ind;
            }

            child = item.value()["AbsorptionCoefficient"];
            if (child.is_null()){
                material.absorb_coeff = {0, 0, 0};
            }
            else {
                stream << child.get<std::string>() << std::endl;
                stream >> material.absorb_coeff.x >> material.absorb_coeff.y >> material.absorb_coeff.z;
            }
            
            child = item.value()["_type"];
            if (!child.is_null()){
                std::string mat_type = child.get<std::string>();

                if (mat_type == "mirror") material.is_mirror = true;

                if (mat_type == "conductor") material.is_conductor = true;

                if (mat_type == "dielectric") material.is_dielectric = true;
            }

            materials.push_back(material);
        }
    }

    // std::cout << "Materials loaded: " << materials.size() << std::endl;
    // std::cout << "Materials[0].is_mirror: " << materials[0].is_mirror << std::endl;
    // std::cout << "Materials[0].mirror: " << materials[0].mirror.x << " " << materials[0].mirror.y << " " << materials[0].mirror.z << std::endl;
    // std::cout << "Materials[1].is_mirror: " << materials[1].is_mirror << std::endl;
    // std::cout << "Materials[1].mirror: " << materials[1].mirror.x << " " << materials[1].mirror.y << " " << materials[1].mirror.z << std::endl;

    ////////////////////////////////////////////////////////////////////////////////////////////////

    // std::cout << "Got materials\n";


    //////////////////////////////// Get Transformations ////////////////////////////////
    // Scalings
    element = data["Scene"]["Transformations"]["Scaling"];

    // Single Scaling
    if (!element.is_null() && element.type() != json::value_t::array) {
        auto child = element["_data"];
        stream << child.get<std::string>() << std::endl;
        Matrix4f matrix;
        matrix.p = 1.0;

        stream >> matrix.a >> matrix.f >> matrix.k;
        scalings.push_back(matrix);
    }
    else if (!element.is_null()) { // Multiple scalings
        Matrix4f matrix;
        matrix.p = 1.0;
        for (auto& item : element.items()){
            auto child = item.value()["_data"];
            stream << child.get<std::string>() << std::endl;
            stream >> matrix.a >> matrix.f >> matrix.k;
            scalings.push_back(matrix);
        }
    }

    // Rotations
    element = data["Scene"]["Transformations"]["Rotation"];


    // Single rotation
    if (!element.is_null() && element.type() != json::value_t::array) {
        auto child = element["_data"];
        stream << child.get<std::string>() << std::endl;
        float thetaDegrees;
        Vec3f axis;

        stream >> thetaDegrees;
        stream >> axis.x >> axis.y >> axis.z;

        axis = normalize(axis);
        // Convert degrees to radians
        float theta = thetaDegrees * static_cast<float>(M_PI) / 180.0f;

        float c = std::cos(theta);
        float s = std::sin(theta);
        float t = 1.0f - c;

        Matrix4f matrix(
            t*axis.x*axis.x + c,     t*axis.x*axis.y - s*axis.z,  t*axis.x*axis.z + s*axis.y,  0.0f,
            t*axis.x*axis.y + s*axis.z,   t*axis.y*axis.y + c,    t*axis.y*axis.z - s*axis.x,  0.0f,
            t*axis.x*axis.z - s*axis.y,   t*axis.y*axis.z + s*axis.x,  t*axis.z*axis.z + c,    0.0f,
            0.0f,          0.0f,         0.0f,         1.0f
        );

        rotations.push_back(matrix);
    }
    else if (!element.is_null()) { // Multiple rotations
        float thetaDegrees;
        Vec3f axis;
        for (auto& item : element.items()){
            auto child = item.value()["_data"];
            stream << child.get<std::string>() << std::endl;

            stream >> thetaDegrees;
            stream >> axis.x >> axis.y >> axis.z;

            axis = normalize(axis);
            // Convert degrees to radians
            float theta = thetaDegrees * static_cast<float>(M_PI) / 180.0f;

            float c = std::cos(theta);
            float s = std::sin(theta);
            float t = 1.0f - c;

            Matrix4f matrix(
                t*axis.x*axis.x + c,     t*axis.x*axis.y - s*axis.z,  t*axis.x*axis.z + s*axis.y,  0.0f,
                t*axis.x*axis.y + s*axis.z,   t*axis.y*axis.y + c,    t*axis.y*axis.z - s*axis.x,  0.0f,
                t*axis.x*axis.z - s*axis.y,   t*axis.y*axis.z + s*axis.x,  t*axis.z*axis.z + c,    0.0f,
                0.0f,          0.0f,         0.0f,         1.0f
            );

            rotations.push_back(matrix);
        }
    }


    // Translations
    element = data["Scene"]["Transformations"]["Translation"];


    // Single translation
    if (!element.is_null() && element.type() != json::value_t::array) {
        auto child = element["_data"];
        stream << child.get<std::string>() << std::endl;
        Matrix4f matrix;
        matrix.a = 1; matrix.f = 1; matrix.k = 1; matrix.p = 1;

        stream >> matrix.d >> matrix.h >> matrix.l;

        translations.push_back(matrix);
    }
    else if (!element.is_null()) { // Multiple translations
        for (auto& item : element.items()){
            auto child = item.value()["_data"];
            stream << child.get<std::string>() << std::endl;
            Matrix4f matrix;
            matrix.a = 1; matrix.f = 1; matrix.k = 1; matrix.p = 1;

            stream >> matrix.d >> matrix.h >> matrix.l;

            translations.push_back(matrix);
        }
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    //////////////////////////////// Get VertexData ////////////////////////////////
    element = data["Scene"]["VertexData"];
    
    if (element.type() != json::value_t::string) element = element["_data"];

    stream << element.get<std::string>() << std::endl;
    Vec3f vertex;
    while (stream >> vertex.x >> vertex.y >> vertex.z) {
        vertex_data.push_back(vertex);
    }
    stream.str("");
    stream.clear();

    // std::cout << "VertexData loaded: " << vertex_data.size() << std::endl;
    // std::cout << "VertexData[0]: " << vertex_data[0].x << " " << vertex_data[0].y << " " << vertex_data[0].z << std::endl;

    ////////////////////////////////////////////////////////////////////////////////////////////////

    // std::cout << "Got vertex data\n";
    

    //////////////////////////////// Get Meshes ////////////////////////////////
    element = data["Scene"]["Objects"]["Mesh"];

    // Single Mesh
    if (element.type() != json::value_t::array && !element.is_null()){
        Mesh mesh;

        auto child = element["SmoothShading"];
        if (!child.is_null()){
            mesh.smooth_shading = true;
        } else {
            mesh.smooth_shading = false;
        }

        child = element["Material"];
        stream << child.get<std::string>() << std::endl;
        stream >> mesh.material_id;

        child = element["Faces"]["_plyFile"];
        if (!child.is_null()){
            stream << child.get<std::string>() << std::endl;
            stream >> mesh.ply_file;
        } else {
            mesh.ply_file = "";

            child = element["Faces"]["_data"];
            stream << child.get<std::string>() << std::endl;
            
            Face face;
            Vec3i indices;

            while (stream >> indices.x >> indices.y >> indices.z) {
                // Change when non-triangle faces are given
                face.triangle = true;
                face.v0 = vertex_data[indices.x-1];
                face.v1 = vertex_data[indices.y-1];
                face.v2 = vertex_data[indices.z-1];

                parser::Vec3f vec1, vec2;
                vec1 = parser::vector_sub(face.v1, face.v0);
                vec2 = parser::vector_sub(face.v2, face.v0);
                face.normal = parser::normalize(parser::cross(vec1, vec2));

                mesh.faces.push_back(face);
            }
            stream.str("");
            stream.clear();
        }


        child = element["Transformations"];
        if (!child.is_null()){
            stream << child.get<std::string>() << std::endl;
            std::string transform_id;
            parser::Matrix4f transform;
            transform.a = 1; transform.f = 1; transform.k = 1; transform.p = 1;

            while(stream >> transform_id){
                if (transform_id[0] == 't') transform = parser::matrixMult(transform, translations[(int)(transform_id[1]-'0')]);

                else if (transform_id[0] == 'r') transform = parser::matrixMult(transform, rotations[(int)(transform_id[1]-'0')]);

                else transform = parser::matrixMult(transform, scalings[(int)(transform_id[1]-'0')]);
            }

            mesh.transformation = transform;
            mesh.normal_transform = parser::matrixInverse(parser::matrixTranspose(transform));

            stream.str("");
            stream.clear();
        }
        
        meshes.push_back(mesh);
    }
    else { // Multiple Meshes
        for (auto& item : element.items()) {
            Mesh mesh;
            auto child = item.value()["SmoothShading"];
            if (!child.is_null()){
                mesh.smooth_shading = true;
            } else {
                mesh.smooth_shading = false;
            }

            child = item.value()["Material"];
            stream << child.get<std::string>() << std::endl;
            stream >> mesh.material_id;

            child = item.value()["Faces"]["_plyFile"];
            if (!child.is_null()){
                stream << child.get<std::string>() << std::endl;
                stream >> mesh.ply_file;
            } else {
                mesh.ply_file = "";

                child = item.value()["Faces"]["_data"];
                stream << child.get<std::string>() << std::endl;
                
                Face face;
                Vec3i indices;
                while (stream >> indices.x >> indices.y >> indices.z) {
                    // Change when non-triangle faces are given
                    face.triangle = true;
                    face.v0 = vertex_data[indices.x-1];
                    face.v1 = vertex_data[indices.y-1];
                    face.v2 = vertex_data[indices.z-1];

                    parser::Vec3f vec1, vec2;
                    vec1 = parser::vector_sub(face.v1, face.v0);
                    vec2 = parser::vector_sub(face.v2, face.v0);
                    face.normal = parser::normalize(parser::cross(vec1, vec2));
                    
                    mesh.faces.push_back(face);
                }
                stream.str("");
                stream.clear();
            }


            child = item.value()["Transformations"];
            if (!child.is_null()){
                stream << child.get<std::string>() << std::endl;
                std::string transform_id;
                parser::Matrix4f transform;
                transform.a = 1; transform.f = 1; transform.k = 1; transform.p = 1;

                while(stream >> transform_id){
                    if (transform_id[0] == 't') transform = parser::matrixMult(transform, translations[(int)(transform_id[1]-'0')]);

                    else if (transform_id[0] == 'r') transform = parser::matrixMult(transform, rotations[(int)(transform_id[1]-'0')]);

                    else transform = parser::matrixMult(transform, scalings[(int)(transform_id[1]-'0')]);
                }
                
                mesh.transformation = transform;
                mesh.normal_transform = parser::matrixInverse(parser::matrixTranspose(transform));

                stream.str("");
                stream.clear();
            }


            meshes.push_back(mesh);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////


    // std::cout << "Got meshes\n";

    //////////////////////////////// Get Triangles ////////////////////////////////
    element = data["Scene"]["Objects"]["Triangle"];

    if (element.type() != json::value_t::array && !element.is_null()) { // Single Triangle
        Triangle triangle;
        auto child = element["Indices"];
        Vec3i indices;
        stream << child.get<std::string>() << std::endl;
        stream >> indices.x >> indices.y >> indices.z;
        triangle.v0 = vertex_data[indices.x-1];
        triangle.v1 = vertex_data[indices.y-1];
        triangle.v2 = vertex_data[indices.z-1];
        
        child = element["Material"];
        stream << child.get<std::string>() << std::endl;
        stream >> triangle.material_id;

        // Calculate Normal from VertexData and Indices
        Vec3f vec1, vec2;
        vec1 = vector_sub(triangle.v1, triangle.v0);
        vec2 = vector_sub(triangle.v2, triangle.v0);
        triangle.normal = normalize(cross(vec1, vec2));

        child = element["Transformations"];
        if (!child.is_null()){
            stream << child.get<std::string>() << std::endl;
            std::string transform_id;
            parser::Matrix4f transform;
            transform.a = 1; transform.f = 1; transform.k = 1; transform.p = 1;

            while(stream >> transform_id){
                if (transform_id[0] == 't') transform = parser::matrixMult(transform, translations[(int)(transform_id[1]-'0')]);

                else if (transform_id[0] == 'r') transform = parser::matrixMult(transform, rotations[(int)(transform_id[1]-'0')]);

                else transform = parser::matrixMult(transform, scalings[(int)(transform_id[1]-'0')]);
            }

            triangle.transformation = transform;
            triangle.normal_transform = parser::matrixInverse(parser::matrixTranspose(transform));

            stream.str("");
            stream.clear();
        }

        triangles.push_back(triangle);
    }
    else { // Multiple Triangles
        for (auto& item : element.items()) {
            Triangle triangle;
            auto child = item.value()["Indices"];
            Vec3i indices;
            stream << child.get<std::string>() << std::endl;
            stream >> indices.x >> indices.y >> indices.z;
            triangle.v0 = vertex_data[indices.x-1];
            triangle.v1 = vertex_data[indices.y-1];
            triangle.v2 = vertex_data[indices.z-1];
            
            child = item.value()["Material"];
            stream << child.get<std::string>() << std::endl;
            stream >> triangle.material_id;

            // Calculate Normal from VertexData and Indices
            Vec3f vec1, vec2;
            vec1 = vector_sub(triangle.v1, triangle.v0);
            vec2 = vector_sub(triangle.v2, triangle.v0);
            triangle.normal = normalize(cross(vec1, vec2));

            child = item.value()["Transformations"];
            if (!child.is_null()){
                stream << child.get<std::string>() << std::endl;
                std::string transform_id;
                parser::Matrix4f transform;
                transform.a = 1; transform.f = 1; transform.k = 1; transform.p = 1;

                while(stream >> transform_id){
                    if (transform_id[0] == 't') transform = parser::matrixMult(transform, translations[(int)(transform_id[1]-'0')]);

                    else if (transform_id[0] == 'r') transform = parser::matrixMult(transform, rotations[(int)(transform_id[1]-'0')]);

                    else transform = parser::matrixMult(transform, scalings[(int)(transform_id[1]-'0')]);
                }

                triangle.transformation = transform;
                triangle.normal_transform = parser::matrixInverse(parser::matrixTranspose(transform));

                stream.str("");
                stream.clear();
            }

            triangles.push_back(triangle);
        }
    }

    // std::cout << "Triangles loaded: " << triangles.size() << std::endl;
    // std::cout << "Triangles[0].material_id: " << triangles[0].material_id << std::endl;
    // std::cout << "Triangles[0].indices: " << triangles[0].indices.v0_id << " " << triangles[0].indices.v1_id << " " << triangles[0].indices.v2_id << std::endl;
    ////////////////////////////////////////////////////////////////////////////////////////////////

    // std::cout << "Got triangles\n";
    

    //////////////////////////////// Get Spheres ////////////////////////////////
    element = data["Scene"]["Objects"]["Sphere"];

    if (element.type() != json::value_t::array && !element.is_null()) { // Single Sphere
        Sphere sphere;
        auto child = element["Center"];
        int ind;
        stream << child.get<std::string>() << std::endl;
        stream >> ind;
        sphere.center_vertex = vertex_data[ind-1];

        child = element["Radius"];
        stream << child.get<std::string>() << std::endl;
        stream >> sphere.radius;

        child = element["Material"];
        stream << child.get<std::string>() << std::endl;
        stream >> sphere.material_id;

        child = element["Transformations"];
        if (!child.is_null()){
            stream << child.get<std::string>() << std::endl;
            std::string transform_id;
            parser::Matrix4f transform;
            transform.a = 1; transform.f = 1; transform.k = 1; transform.p = 1;

            while(stream >> transform_id){
                if (transform_id[0] == 't') transform = parser::matrixMult(transform, translations[(int)(transform_id[1]-'0')]);

                else if (transform_id[0] == 'r') transform = parser::matrixMult(transform, rotations[(int)(transform_id[1]-'0')]);

                else transform = parser::matrixMult(transform, scalings[(int)(transform_id[1]-'0')]);
            }

            sphere.transformation = transform;

            stream.str("");
            stream.clear();
        }

        spheres.push_back(sphere);
    }
    else { // Multiple Spheres
        for (auto& item : element.items()) {
            Sphere sphere;
            auto child = item.value()["Center"];
            int ind;
            stream << child.get<std::string>() << std::endl;
            stream >> ind;
            sphere.center_vertex = vertex_data[ind-1];

            child = item.value()["Radius"];
            stream << child.get<std::string>() << std::endl;
            stream >> sphere.radius;

            child = item.value()["Material"];
            stream << child.get<std::string>() << std::endl;
            stream >> sphere.material_id;

            child = item.value()["Transformations"];
            if (!child.is_null()){
                stream << child.get<std::string>() << std::endl;
                std::string transform_id;
                parser::Matrix4f transform;
                transform.a = 1; transform.f = 1; transform.k = 1; transform.p = 1;

                while(stream >> transform_id){
                    if (transform_id[0] == 't') transform = parser::matrixMult(transform, translations[(int)(transform_id[1]-'0')]);

                    else if (transform_id[0] == 'r') transform = parser::matrixMult(transform, rotations[(int)(transform_id[1]-'0')]);

                    else transform = parser::matrixMult(transform, scalings[(int)(transform_id[1]-'0')]);
                }

                sphere.transformation = transform;

                stream.str("");
                stream.clear();
            }

            spheres.push_back(sphere);
        }
    }

    // std::cout << "Spheres loaded: " << spheres.size() << std::endl;
    // std::cout << "Spheres[0].center_vertex_id: " << spheres[0].center_vertex_id << std::endl;
    // std::cout << "Spheres[0].radius: " << spheres[0].radius << std::endl;
    // std::cout << "Spheres[0].material_id: " << spheres[0].material_id << std::endl;
    //////////////////////////////////////////////////////////////////////////////////////////////// 
    

    // std::cout << "Got spheres\n";

    //////////////////////////////// Get Planes ////////////////////////////////
    element = data["Scene"]["Objects"]["Plane"];

    if (element.type() != json::value_t::array && !element.is_null()) { // Single Plane
        Plane plane;
        auto child = element["Normal"];
        stream << child.get<std::string>() << std::endl;
        stream >> plane.normal.x >> plane.normal.y >> plane.normal.z;

        child = element["Point"];
        int ind;
        stream << child.get<std::string>() << std::endl;
        stream >> ind;
        plane.point = vertex_data[ind-1];

        child = element["Material"];
        stream << child.get<std::string>() << std::endl;
        stream >> plane.material_id;

        child = element["Transformations"];
        if (!child.is_null()){
            stream << child.get<std::string>() << std::endl;
            std::string transform_id;
            parser::Matrix4f transform;
            transform.a = 1; transform.f = 1; transform.k = 1; transform.p = 1;

            while(stream >> transform_id){
                if (transform_id[0] == 't') transform = parser::matrixMult(transform, translations[(int)(transform_id[1]-'0')]);

                else if (transform_id[0] == 'r') transform = parser::matrixMult(transform, rotations[(int)(transform_id[1]-'0')]);

                else transform = parser::matrixMult(transform, scalings[(int)(transform_id[1]-'0')]);
            }

            plane.transformation = transform;
            plane.normal_transform = parser::matrixInverse(parser::matrixTranspose(transform));

            stream.str("");
            stream.clear();
        }

        planes.push_back(plane);
    }
    else { // Multiple Planes
        for (auto& item : element.items()) {
            Plane plane;
            auto child = item.value()["Normal"];
            stream << child.get<std::string>() << std::endl;
            stream >> plane.normal.x >> plane.normal.y >> plane.normal.z;

            child = item.value()["Point"];
            int ind;
            stream << child.get<std::string>() << std::endl;
            stream >> ind;
            plane.point = vertex_data[ind-1];

            child = item.value()["Material"];
            stream << child.get<std::string>() << std::endl;
            stream >> plane.material_id;

            child = item.value()["Transformations"];
            if (!child.is_null()){
                stream << child.get<std::string>() << std::endl;
                std::string transform_id;
                parser::Matrix4f transform;
                transform.a = 1; transform.f = 1; transform.k = 1; transform.p = 1;

                while(stream >> transform_id){
                    if (transform_id[0] == 't') transform = parser::matrixMult(transform, translations[(int)(transform_id[1]-'0')]);

                    else if (transform_id[0] == 'r') transform = parser::matrixMult(transform, rotations[(int)(transform_id[1]-'0')]);

                    else transform = parser::matrixMult(transform, scalings[(int)(transform_id[1]-'0')]);
                }

                plane.transformation = transform;
                plane.normal_transform = parser::matrixInverse(parser::matrixTranspose(transform));

                stream.str("");
                stream.clear();
            }

            planes.push_back(plane);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    // std::cout << "Got planes\n";
}

float parser::determinant(parser::Matrix4f m) {
    float det3_1 = m.f * (m.k * m.p - m.o * m.l) - m.g * (m.j * m.p - m.n * m.l) + m.h * (m.j * m.o - m.n * m.k);
    float det3_2 = m.e * (m.k * m.p - m.o * m.l) - m.g * (m.i * m.p - m.m * m.l) + m.h * (m.i * m.o - m.m * m.k);
    float det3_3 = m.e * (m.j * m.p - m.n * m.l) - m.f * (m.i * m.p - m.m * m.l) + m.h * (m.i * m.n - m.m * m.j);
    float det3_4 = m.e * (m.j * m.o - m.n * m.k) - m.f * (m.i * m.o - m.m * m.k) + m.g * (m.i * m.n - m.m * m.j);

    return m.a * det3_1 - m.b * det3_2 + m.c * det3_3 - m.d * det3_4;
}

float parser::determinant(parser::Matrix3f m) {
    return m.a*(m.e*m.i - m.h*m.f) + m.b*(m.g*m.f - m.d*m.i) + m.c*(m.d*m.h - m.e*m.g);  
}

parser::Vec3f parser::matrixMult(parser::Matrix4f m, parser::Vec4f vec){
    parser::Vec3f result;

    result.x = m.a*vec.x + m.b*vec.y + m.c*vec.z + m.d*vec.w;
    result.y = m.e*vec.x + m.f*vec.y + m.g*vec.z + m.h*vec.w;
    result.z = m.i*vec.x + m.j*vec.y + m.k*vec.z + m.l*vec.w;

    return result;
}

parser::Matrix4f parser::matrixMult(parser::Matrix4f m1, parser::Matrix4f m2) {
    parser::Matrix4f result;

    result.a = m1.a * m2.a + m1.b * m2.e + m1.c * m2.i + m1.d * m2.m;
    result.b = m1.a * m2.b + m1.b * m2.f + m1.c * m2.j + m1.d * m2.n;
    result.c = m1.a * m2.c + m1.b * m2.g + m1.c * m2.k + m1.d * m2.o;
    result.d = m1.a * m2.d + m1.b * m2.h + m1.c * m2.l + m1.d * m2.p;

    result.e = m1.e * m2.a + m1.f * m2.e + m1.g * m2.i + m1.h * m2.m;
    result.f = m1.e * m2.b + m1.f * m2.f + m1.g * m2.j + m1.h * m2.n;
    result.g = m1.e * m2.c + m1.f * m2.g + m1.g * m2.k + m1.h * m2.o;
    result.h = m1.e * m2.d + m1.f * m2.h + m1.g * m2.l + m1.h * m2.p;

    result.i = m1.i * m2.a + m1.j * m2.e + m1.k * m2.i + m1.l * m2.m;
    result.j = m1.i * m2.b + m1.j * m2.f + m1.k * m2.j + m1.l * m2.n;
    result.k = m1.i * m2.c + m1.j * m2.g + m1.k * m2.k + m1.l * m2.o;
    result.l = m1.i * m2.d + m1.j * m2.h + m1.k * m2.l + m1.l * m2.p;

    result.m = m1.m * m2.a + m1.n * m2.e + m1.o * m2.i + m1.p * m2.m;
    result.n = m1.m * m2.b + m1.n * m2.f + m1.o * m2.j + m1.p * m2.n;
    result.o = m1.m * m2.c + m1.n * m2.g + m1.o * m2.k + m1.p * m2.o;
    result.p = m1.m * m2.d + m1.n * m2.h + m1.o * m2.l + m1.p * m2.p;

    return result;
}

parser::Matrix4f parser::matrixTranspose(parser::Matrix4f m) {
    return parser::Matrix4f(
        m.a, m.e, m.i, m.m,
        m.b, m.f, m.j, m.n,
        m.c, m.g, m.k, m.o,
        m.d, m.h, m.l, m.p
    );
}

parser::Matrix4f parser::matrixInverse(parser::Matrix4f m) {
    parser::Matrix4f inv;

    inv.a =  m.f * m.k * m.p - m.f * m.l * m.o - m.j * m.g * m.p + m.j * m.h * m.o + m.n * m.g * m.l - m.n * m.h * m.k;
    inv.b = -m.b * m.k * m.p + m.b * m.l * m.o + m.j * m.c * m.p - m.j * m.d * m.o - m.n * m.c * m.l + m.n * m.d * m.k;
    inv.c =  m.b * m.g * m.p - m.b * m.h * m.o - m.f * m.c * m.p + m.f * m.d * m.o + m.n * m.c * m.h - m.n * m.d * m.g;
    inv.d = -m.b * m.g * m.l + m.b * m.h * m.k + m.f * m.c * m.l - m.f * m.d * m.k - m.j * m.c * m.h + m.j * m.d * m.g;

    inv.e = -m.e * m.k * m.p + m.e * m.l * m.o + m.i * m.g * m.p - m.i * m.h * m.o - m.m * m.g * m.l + m.m * m.h * m.k;
    inv.f =  m.a * m.k * m.p - m.a * m.l * m.o - m.i * m.c * m.p + m.i * m.d * m.o + m.m * m.c * m.l - m.m * m.d * m.k;
    inv.g = -m.a * m.g * m.p + m.a * m.h * m.o + m.e * m.c * m.p - m.e * m.d * m.o - m.m * m.c * m.h + m.m * m.d * m.g;
    inv.h =  m.a * m.g * m.l - m.a * m.h * m.k - m.e * m.c * m.l + m.e * m.d * m.k + m.i * m.c * m.h - m.i * m.d * m.g;

    inv.i =  m.e * m.j * m.p - m.e * m.l * m.n - m.i * m.f * m.p + m.i * m.h * m.n + m.m * m.f * m.l - m.m * m.h * m.j;
    inv.j = -m.a * m.j * m.p + m.a * m.l * m.n + m.i * m.b * m.p - m.i * m.d * m.n - m.m * m.b * m.l + m.m * m.d * m.j;
    inv.k =  m.a * m.f * m.p - m.a * m.h * m.n - m.e * m.b * m.p + m.e * m.d * m.n + m.m * m.b * m.h - m.m * m.d * m.f;
    inv.l = -m.a * m.f * m.l + m.a * m.h * m.j + m.e * m.b * m.l - m.e * m.d * m.j - m.i * m.b * m.h + m.i * m.d * m.f;

    inv.m = -m.e * m.j * m.o + m.e * m.k * m.n + m.i * m.f * m.o - m.i * m.g * m.n - m.m * m.f * m.k + m.m * m.g * m.j;
    inv.n =  m.a * m.j * m.o - m.a * m.k * m.n - m.i * m.b * m.o + m.i * m.c * m.n + m.m * m.b * m.k - m.m * m.c * m.j;
    inv.o = -m.a * m.f * m.o + m.a * m.g * m.n + m.e * m.b * m.o - m.e * m.c * m.n - m.m * m.b * m.g + m.m * m.c * m.f;
    inv.p =  m.a * m.f * m.k - m.a * m.g * m.j - m.e * m.b * m.k + m.e * m.c * m.j + m.i * m.b * m.g - m.i * m.c * m.f;

    float det = m.a * inv.a + m.b * inv.e + m.c * inv.i + m.d * inv.m;

    if (det == 0) {
        std::cerr << "Matrix inversion failed: determinant is zero." << std::endl;
        return parser::Matrix4f(); // returns zero matrix
    }

    float invDet = 1.0f / det;

    inv.a *= invDet; inv.b *= invDet; inv.c *= invDet; inv.d *= invDet;
    inv.e *= invDet; inv.f *= invDet; inv.g *= invDet; inv.h *= invDet;
    inv.i *= invDet; inv.j *= invDet; inv.k *= invDet; inv.l *= invDet;
    inv.m *= invDet; inv.n *= invDet; inv.o *= invDet; inv.p *= invDet;

    return inv;
}
