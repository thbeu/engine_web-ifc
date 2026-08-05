#include <vector>
#include <algorithm>
#include <unordered_map>
#include <glm/glm.hpp>
#include "aabb.h"
#include "face.h"
#include "plane.h"

#pragma once

namespace bimGeometry {
    
    struct Geometry
    {
        bool hasPlanes = false;
        uint32_t numPoints = 0;
        uint32_t numFaces = 0;
        std::vector<float> fvertexData;
        std::vector<double> vertexData;
        std::vector<uint32_t> indexData;
        std::vector<uint32_t> planeData;
        std::vector<Plane> planes;

        AABB GetAABB() const;
        Vec GetPoint(size_t index) const;
        void SetPoint(double x, double y, double z, size_t index);
        Face GetFace(size_t index) const;
        void buildPlanes();
        size_t AddPlane(const glm::dvec3 &normal, double d);
        void AddFace(glm::dvec3 a, glm::dvec3 b, glm::dvec3 c, uint32_t pId = UINT32_MAX);
        void AddFace(uint32_t a, uint32_t b, uint32_t c, uint32_t pId = UINT32_MAX);
        void AddPoint(glm::dvec4& pt, glm::dvec3& n);
        void AddPoint(const glm::dvec3& pt, const glm::dvec3& n);
        void AddGeometry(Geometry geom);

        private:
            // Spatial hash used by AddPlane() to deduplicate planes in (near) O(1)
            // instead of scanning every plane built so far (quadratic in buildPlanes()).
            std::unordered_map<uint64_t, std::vector<uint32_t>> _planeBuckets;
            size_t _planeBucketsUpTo = 0;
            static uint64_t PlaneKey(long long kx, long long ky, long long kz, long long kd);
            void RebuildPlaneBuckets();
    };
}