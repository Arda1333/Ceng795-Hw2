#include <iostream>
#include "parser.h"


float intersectsSphere(parser::Vec3f& rayOrigin, parser::Vec3f& rayDirection, parser::Sphere& sphere, float t_min, parser::Scene& scene);

float intersectsTriangle(parser::Vec3f& rayOrigin, parser::Vec3f& rayDirection, parser::Triangle& triangle, float t_min, parser::Scene& scene);

float intersectsPlane(parser::Vec3f& rayOrigin, parser::Vec3f& rayDirection, parser::Plane& plane, float t_min, parser::Scene& scene);

float intersectsObject(parser::Vec3f& rayOrigin, parser::Vec3f& rayDirection, float t_min, int& material_id, parser::Vec3f& surfaceNormal, parser::Vec3f& intersectionPoint, parser::Scene& scene);

bool calculateShadow( parser::Vec3f& intersectionPoint, parser::Vec3f& lightDirection, float t_min, float t_light, parser::Scene& scene);

parser::Vec3i calculateColor(parser::Vec3f& rayOrigin, parser::Vec3f& rayDirection, parser::Vec3f& intersectionPoint, parser::Vec3f& surfaceNormal, float& closest_t, float& t_min, parser::Material& material, parser::Scene& scene);

parser::Vec3f calculateMirror(parser::Vec3f& currOrigin, parser::Vec3f& currNormal, parser::Vec3f& currDir, parser::Vec3f intersectionPoint, float t_min, parser::Vec3f accumulated, parser::Vec3f reflectWeight, float eps, int maxDepth, parser::Scene scene);