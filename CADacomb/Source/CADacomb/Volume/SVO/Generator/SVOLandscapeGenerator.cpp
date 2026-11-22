#include "SVOLandscapeGenerator.h"

#include <algorithm>
#include <cmath>

namespace CADacomb
{
    SVOLandscapeGenerator::SVOLandscapeGenerator() {}

    SVOLandscapeGenerator::~SVOLandscapeGenerator() {}

    int SVOLandscapeGenerator::fastFloor(double x)
    {
        int i = static_cast<int>(x);
        return (x < static_cast<double>(i)) ? (i - 1) : i;
    }

    uint32_t SVOLandscapeGenerator::hash3D(int x, int y, int z, uint32_t seed)
    {
        uint32_t h = seed ^ (static_cast<uint32_t>(x) * 0x73856093u) ^ (static_cast<uint32_t>(y) * 0x19379363u) ^ (static_cast<uint32_t>(z) * 0x834923f9u);
        h ^= h >> 16;
        h *= 0x85ebca6bu;
        h ^= h >> 13;
        h *= 0xc2b2ae35u;
        h ^= h >> 16;
        return h;
    }

    double SVOLandscapeGenerator::grad(uint32_t hash, double x, double y, double z)
    {
        switch (hash % 12)
        {
            case 0:
                return x + y;
            case 1:
                return -x + y;
            case 2:
                return x - y;
            case 3:
                return -x - y;
            case 4:
                return x + z;
            case 5:
                return -x + z;
            case 6:
                return x - z;
            case 7:
                return -x - z;
            case 8:
                return y + z;
            case 9:
                return -y + z;
            case 10:
                return y - z;
            case 11:
                return -y - z;
            default:
                return 0.0;
        }
    }

    double SVOLandscapeGenerator::fade(double t)
    {
        return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
    }

    double SVOLandscapeGenerator::lerp(double a, double b, double t)
    {
        return a + t * (b - a);
    }

    double SVOLandscapeGenerator::perlinNoise3D(const glm::dvec3& pos, uint32_t seed)
    {
        int x0 = fastFloor(pos.x);
        int y0 = fastFloor(pos.y);
        int z0 = fastFloor(pos.z);

        int x1 = x0 + 1;
        int y1 = y0 + 1;
        int z1 = z0 + 1;

        double dx = pos.x - static_cast<double>(x0);
        double dy = pos.y - static_cast<double>(y0);
        double dz = pos.z - static_cast<double>(z0);

        double u = fade(dx);
        double v = fade(dy);
        double w = fade(dz);

        double g000 = grad(hash3D(x0, y0, z0, seed), dx, dy, dz);
        double g100 = grad(hash3D(x1, y0, z0, seed), dx - 1.0, dy, dz);
        double g010 = grad(hash3D(x0, y1, z0, seed), dx, dy - 1.0, dz);
        double g110 = grad(hash3D(x1, y1, z0, seed), dx - 1.0, dy - 1.0, dz);

        double g001 = grad(hash3D(x0, y0, z1, seed), dx, dy, dz - 1.0);
        double g101 = grad(hash3D(x1, y0, z1, seed), dx - 1.0, dy, dz - 1.0);
        double g011 = grad(hash3D(x0, y1, z1, seed), dx, dy - 1.0, dz - 1.0);
        double g111 = grad(hash3D(x1, y1, z1, seed), dx - 1.0, dy - 1.0, dz - 1.0);

        double x00 = lerp(g000, g100, u);
        double x10 = lerp(g010, g110, u);
        double x01 = lerp(g001, g101, u);
        double x11 = lerp(g011, g111, u);

        double y0Val = lerp(x00, x10, v);
        double y1Val = lerp(x01, x11, v);

        return lerp(y0Val, y1Val, w);
    }

    double SVOLandscapeGenerator::fbm3D(const glm::dvec3& pos, int octaves, double frequency, double persistence, double lacunarity, uint32_t seed)
    {
        double sum    = 0.0;
        double amp    = 1.0;
        double maxAmp = 0.0;
        double freq   = frequency;

        for (int i = 0; i < octaves; i++)
        {
            sum += amp * perlinNoise3D(pos * freq, seed + static_cast<uint32_t>(i * 31));
            maxAmp += amp;
            freq *= lacunarity;
            amp *= persistence;
        }

        return (maxAmp > 0.0) ? (sum / maxAmp) : 0.0;
    }

    double SVOLandscapeGenerator::evaluateDensity(const glm::dvec3& localPos, double volumeHeight, const LandscapeConfig& config)
    {
        if (config.customDensity)
        {
            return config.customDensity(localPos);
        }

        double h          = (volumeHeight > 0.0) ? (localPos.y / volumeHeight) : 0.0;
        double heightGrad = (config.baseHeightRatio - h) * 2.0 * config.heightScale;

        double terrainNoise = fbm3D(localPos, config.octaves, config.frequency, config.persistence, config.lacunarity, config.seed);

        double density = heightGrad + terrainNoise;

        if (config.enableCaves && h < 0.9)
        {
            double caveNoise = fbm3D(localPos, 2, config.caveFrequency, 0.5, 2.0, config.seed + 9999);

            if (std::abs(caveNoise) < config.caveThreshold)
            {
                density = -1.0;
            }
        }

        return density;
    }

    void SVOLandscapeGenerator::addPerlinOctaveBlock(const glm::ivec3& blockMin,
                                                     uint32_t          blockSize,
                                                     const glm::dvec3& worldOffset,
                                                     double            freq,
                                                     double            weight,
                                                     uint32_t          seed,
                                                     double*           outValues)
    {
        static constexpr double gradX[12] = { 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0 };
        static constexpr double gradY[12] = { 1.0, 1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 1.0, -1.0 };
        static constexpr double gradZ[12] = { 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, -1.0 };

        const double baseX = (static_cast<double>(blockMin.x) + worldOffset.x) * freq;
        const double baseY = (static_cast<double>(blockMin.y) + worldOffset.y) * freq;
        const double baseZ = (static_cast<double>(blockMin.z) + worldOffset.z) * freq;

        const double maxX = (static_cast<double>(blockMin.x + static_cast<int>(blockSize) - 1) + worldOffset.x) * freq;
        const double maxY = (static_cast<double>(blockMin.y + static_cast<int>(blockSize) - 1) + worldOffset.y) * freq;
        const double maxZ = (static_cast<double>(blockMin.z + static_cast<int>(blockSize) - 1) + worldOffset.z) * freq;

        const int x0Min = fastFloor(baseX);
        const int y0Min = fastFloor(baseY);
        const int z0Min = fastFloor(baseZ);

        const int x0Max = fastFloor(maxX);
        const int y0Max = fastFloor(maxY);
        const int z0Max = fastFloor(maxZ);

        if (x0Min == x0Max && y0Min == y0Max && z0Min == z0Max && blockSize <= 16)
        {
            const int x1 = x0Min + 1;
            const int y1 = y0Min + 1;
            const int z1 = z0Min + 1;

            const uint32_t h000 = hash3D(x0Min, y0Min, z0Min, seed) % 12u;
            const uint32_t h100 = hash3D(x1, y0Min, z0Min, seed) % 12u;
            const uint32_t h010 = hash3D(x0Min, y1, z0Min, seed) % 12u;
            const uint32_t h110 = hash3D(x1, y1, z0Min, seed) % 12u;
            const uint32_t h001 = hash3D(x0Min, y0Min, z1, seed) % 12u;
            const uint32_t h101 = hash3D(x1, y0Min, z1, seed) % 12u;
            const uint32_t h011 = hash3D(x0Min, y1, z1, seed) % 12u;
            const uint32_t h111 = hash3D(x1, y1, z1, seed) % 12u;

            double dxArr[16];
            double uArr[16];
            double dyArr[16];
            double vArr[16];
            double dzArr[16];
            double wArr[16];

            for (uint32_t i = 0; i < blockSize; i++)
            {
                double di = static_cast<double>(i);
                dxArr[i]  = (static_cast<double>(blockMin.x) + di + worldOffset.x) * freq - static_cast<double>(x0Min);
                uArr[i]   = fade(dxArr[i]);
                dyArr[i]  = (static_cast<double>(blockMin.y) + di + worldOffset.y) * freq - static_cast<double>(y0Min);
                vArr[i]   = fade(dyArr[i]);
                dzArr[i]  = (static_cast<double>(blockMin.z) + di + worldOffset.z) * freq - static_cast<double>(z0Min);
                wArr[i]   = fade(dzArr[i]);
            }

            const double g000x = gradX[h000], g000y = gradY[h000], g000z = gradZ[h000];
            const double g100x = gradX[h100], g100y = gradY[h100], g100z = gradZ[h100];
            const double g010x = gradX[h010], g010y = gradY[h010], g010z = gradZ[h010];
            const double g110x = gradX[h110], g110y = gradY[h110], g110z = gradZ[h110];
            const double g001x = gradX[h001], g001y = gradY[h001], g001z = gradZ[h001];
            const double g101x = gradX[h101], g101y = gradY[h101], g101z = gradZ[h101];
            const double g011x = gradX[h011], g011y = gradY[h011], g011z = gradZ[h011];
            const double g111x = gradX[h111], g111y = gradY[h111], g111z = gradZ[h111];

            size_t outIdx = 0;
            for (uint32_t lz = 0; lz < blockSize; lz++)
            {
                const double dz  = dzArr[lz];
                const double dz1 = dz - 1.0;
                const double w   = wArr[lz];

                for (uint32_t ly = 0; ly < blockSize; ly++)
                {
                    const double dy  = dyArr[ly];
                    const double dy1 = dy - 1.0;
                    const double v   = vArr[ly];

                    const double c000 = g000y * dy + g000z * dz;
                    const double c100 = g100y * dy + g100z * dz - g100x;
                    const double c010 = g010y * dy1 + g010z * dz;
                    const double c110 = g110y * dy1 + g110z * dz - g110x;
                    const double c001 = g001y * dy + g001z * dz1;
                    const double c101 = g101y * dy + g101z * dz1 - g101x;
                    const double c011 = g011y * dy1 + g011z * dz1;
                    const double c111 = g111y * dy1 + g111z * dz1 - g111x;

                    for (uint32_t lx = 0; lx < blockSize; lx++)
                    {
                        const double dx = dxArr[lx];
                        const double u  = uArr[lx];

                        const double n000 = g000x * dx + c000;
                        const double n100 = g100x * dx + c100;
                        const double n010 = g010x * dx + c010;
                        const double n110 = g110x * dx + c110;
                        const double n001 = g001x * dx + c001;
                        const double n101 = g101x * dx + c101;
                        const double n011 = g011x * dx + c011;
                        const double n111 = g111x * dx + c111;

                        const double x00 = n000 + u * (n100 - n000);
                        const double x10 = n010 + u * (n110 - n010);
                        const double x01 = n001 + u * (n101 - n001);
                        const double x11 = n011 + u * (n111 - n011);

                        const double y0Val = x00 + v * (x10 - x00);
                        const double y1Val = x01 + v * (x11 - x01);

                        outValues[outIdx] += weight * (y0Val + w * (y1Val - y0Val));
                        outIdx++;
                    }
                }
            }
            return;
        }

        size_t outIdx = 0;
        for (uint32_t lz = 0; lz < blockSize; lz++)
        {
            const double pz = (static_cast<double>(blockMin.z + static_cast<int>(lz)) + worldOffset.z) * freq;
            const int    z0 = fastFloor(pz);
            const int    z1 = z0 + 1;
            const double dz = pz - static_cast<double>(z0);
            const double w  = fade(dz);

            for (uint32_t ly = 0; ly < blockSize; ly++)
            {
                const double py = (static_cast<double>(blockMin.y + static_cast<int>(ly)) + worldOffset.y) * freq;
                const int    y0 = fastFloor(py);
                const int    y1 = y0 + 1;
                const double dy = py - static_cast<double>(y0);
                const double v  = fade(dy);

                for (uint32_t lx = 0; lx < blockSize; lx++)
                {
                    const double px = (static_cast<double>(blockMin.x + static_cast<int>(lx)) + worldOffset.x) * freq;
                    const int    x0 = fastFloor(px);
                    const int    x1 = x0 + 1;
                    const double dx = px - static_cast<double>(x0);
                    const double u  = fade(dx);

                    const uint32_t h000 = hash3D(x0, y0, z0, seed) % 12u;
                    const uint32_t h100 = hash3D(x1, y0, z0, seed) % 12u;
                    const uint32_t h010 = hash3D(x0, y1, z0, seed) % 12u;
                    const uint32_t h110 = hash3D(x1, y1, z0, seed) % 12u;
                    const uint32_t h001 = hash3D(x0, y0, z1, seed) % 12u;
                    const uint32_t h101 = hash3D(x1, y0, z1, seed) % 12u;
                    const uint32_t h011 = hash3D(x0, y1, z1, seed) % 12u;
                    const uint32_t h111 = hash3D(x1, y1, z1, seed) % 12u;

                    const double dx1 = dx - 1.0;
                    const double dy1 = dy - 1.0;
                    const double dz1 = dz - 1.0;

                    const double n000 = gradX[h000] * dx + gradY[h000] * dy + gradZ[h000] * dz;
                    const double n100 = gradX[h100] * dx1 + gradY[h100] * dy + gradZ[h100] * dz;
                    const double n010 = gradX[h010] * dx + gradY[h010] * dy1 + gradZ[h010] * dz;
                    const double n110 = gradX[h110] * dx1 + gradY[h110] * dy1 + gradZ[h110] * dz;
                    const double n001 = gradX[h001] * dx + gradY[h001] * dy + gradZ[h001] * dz1;
                    const double n101 = gradX[h101] * dx1 + gradY[h101] * dy + gradZ[h101] * dz1;
                    const double n011 = gradX[h011] * dx + gradY[h011] * dy1 + gradZ[h011] * dz1;
                    const double n111 = gradX[h111] * dx1 + gradY[h111] * dy1 + gradZ[h111] * dz1;

                    const double x00   = n000 + u * (n100 - n000);
                    const double x10   = n010 + u * (n110 - n010);
                    const double x01   = n001 + u * (n101 - n001);
                    const double x11   = n011 + u * (n111 - n011);
                    const double y0Val = x00 + v * (x10 - x00);
                    const double y1Val = x01 + v * (x11 - x01);

                    outValues[outIdx] += weight * (y0Val + w * (y1Val - y0Val));
                    outIdx++;
                }
            }
        }
    }

    void SVOLandscapeGenerator::evaluateDensityBlock(const glm::ivec3&      blockMin,
                                                     uint32_t               blockSize,
                                                     double                 volumeHeight,
                                                     const LandscapeConfig& config,
                                                     const glm::dvec3&      worldOffset,
                                                     double*                outDensities)
    {
        if (config.customDensity)
        {
            size_t outIdx = 0;
            for (uint32_t lz = 0; lz < blockSize; lz++)
            {
                double pz = static_cast<double>(blockMin.z + static_cast<int>(lz)) + worldOffset.z;
                for (uint32_t ly = 0; ly < blockSize; ly++)
                {
                    double py = static_cast<double>(blockMin.y + static_cast<int>(ly)) + worldOffset.y;
                    for (uint32_t lx = 0; lx < blockSize; lx++)
                    {
                        double px            = static_cast<double>(blockMin.x + static_cast<int>(lx)) + worldOffset.x;
                        outDensities[outIdx] = config.customDensity(glm::dvec3(px, py, pz));
                        outIdx++;
                    }
                }
            }
            return;
        }

        const double invHeight = (volumeHeight > 0.0) ? (1.0 / volumeHeight) : 0.0;
        size_t       outIdx    = 0;
        for (uint32_t lz = 0; lz < blockSize; lz++)
        {
            for (uint32_t ly = 0; ly < blockSize; ly++)
            {
                double h          = (static_cast<double>(blockMin.y + static_cast<int>(ly)) + worldOffset.y) * invHeight;
                double heightGrad = (config.baseHeightRatio - h) * 2.0 * config.heightScale;
                for (uint32_t lx = 0; lx < blockSize; lx++)
                {
                    outDensities[outIdx] = heightGrad;
                    outIdx++;
                }
            }
        }

        double maxAmp = 0.0;
        double amp    = 1.0;
        for (int i = 0; i < config.octaves; i++)
        {
            maxAmp += amp;
            amp *= config.persistence;
        }

        if (maxAmp > 0.0)
        {
            const double invMaxAmp = 1.0 / maxAmp;
            double       octAmp    = 1.0;
            double       octFreq   = config.frequency;
            for (int i = 0; i < config.octaves; i++)
            {
                addPerlinOctaveBlock(blockMin, blockSize, worldOffset, octFreq, octAmp * invMaxAmp, config.seed + static_cast<uint32_t>(i * 31), outDensities);
                octFreq *= config.lacunarity;
                octAmp *= config.persistence;
            }
        }

        if (config.enableCaves)
        {
            double yMin = static_cast<double>(blockMin.y) + worldOffset.y;
            if (yMin * invHeight < 0.9)
            {
                const size_t count = static_cast<size_t>(blockSize) * blockSize * blockSize;
                bool         anySolidCandidate = false;
                for (size_t i = 0; i < count; i++)
                {
                    if (outDensities[i] > config.solidThreshold)
                    {
                        anySolidCandidate = true;
                        break;
                    }
                }

                if (anySolidCandidate)
                {
                    double caveBuf[64];
                    std::vector<double> dynamicCaveBuf;
                    double*             caveValues = caveBuf;
                    if (count > 64)
                    {
                        dynamicCaveBuf.assign(count, 0.0);
                        caveValues = dynamicCaveBuf.data();
                    }
                    else
                    {
                        std::fill_n(caveValues, count, 0.0);
                    }

                    constexpr double invCaveAmp = 1.0 / 1.5;
                    addPerlinOctaveBlock(blockMin, blockSize, worldOffset, config.caveFrequency, 1.0 * invCaveAmp, config.seed + 9999u, caveValues);
                    addPerlinOctaveBlock(blockMin, blockSize, worldOffset, config.caveFrequency * 2.0, 0.5 * invCaveAmp, config.seed + 9999u + 31u, caveValues);

                    size_t idx = 0;
                    for (uint32_t lz = 0; lz < blockSize; lz++)
                    {
                        for (uint32_t ly = 0; ly < blockSize; ly++)
                        {
                            double h = (static_cast<double>(blockMin.y + static_cast<int>(ly)) + worldOffset.y) * invHeight;
                            for (uint32_t lx = 0; lx < blockSize; lx++)
                            {
                                if (h < 0.9 && std::abs(caveValues[idx]) < config.caveThreshold)
                                {
                                    outDensities[idx] = -1.0;
                                }
                                idx++;
                            }
                        }
                    }
                }
            }
        }
    }

    std::pair<double, double> SVOLandscapeGenerator::estimateDensityBounds(const glm::ivec3&      boxMin,
                                                                           const glm::ivec3&      boxSize,
                                                                           double                 volumeHeight,
                                                                           const LandscapeConfig& config,
                                                                           const glm::dvec3&      worldOffset)
    {
        if (config.customDensity)
        {
            return { -1e9, 1e9 };
        }

        if (volumeHeight <= 0.0 || config.heightScale <= 0.0)
        {
            return { -1.0, 1.0 };
        }

        double yMin = static_cast<double>(boxMin.y) + worldOffset.y;
        double yMax = static_cast<double>(boxMin.y + std::max(1, boxSize.y) - 1) + worldOffset.y;

        double hMin = yMin / volumeHeight;
        double hMax = yMax / volumeHeight;

        double minHeightGrad = (config.baseHeightRatio - hMax) * 2.0 * config.heightScale;
        double maxHeightGrad = (config.baseHeightRatio - hMin) * 2.0 * config.heightScale;

        glm::dvec3 pMin   = glm::dvec3(boxMin) + worldOffset;
        glm::dvec3 pMax   = glm::dvec3(boxMin + glm::max(glm::ivec3(1), boxSize) - 1) + worldOffset;
        glm::dvec3 center = 0.5 * (pMin + pMax);
        double     radius = 0.5 * glm::length(pMax - pMin);

        constexpr double perlinLipschitz = 2.6;

        double sumAmp     = 0.0;
        double currentAmp = 1.0;
        for (int i = 0; i < config.octaves; i++)
        {
            sumAmp += currentAmp;
            currentAmp *= config.persistence;
        }

        double minTerrainNoise = -1.0;
        double maxTerrainNoise = 1.0;

        if (sumAmp > 0.0)
        {
            double boundMaxSum = 0.0;
            double boundMinSum = 0.0;
            double octaveAmp   = 1.0;
            double octaveFreq  = config.frequency;

            for (int i = 0; i < config.octaves; i++)
            {
                double delta = octaveFreq * perlinLipschitz * radius;
                if (delta < 2.0)
                {
                    double n = perlinNoise3D(center * octaveFreq, config.seed + static_cast<uint32_t>(i * 31));
                    boundMaxSum += octaveAmp * std::min(1.0, n + delta);
                    boundMinSum += octaveAmp * std::max(-1.0, n - delta);
                }
                else
                {
                    boundMaxSum += octaveAmp * 1.0;
                    boundMinSum += octaveAmp * (-1.0);
                }
                octaveFreq *= config.lacunarity;
                octaveAmp *= config.persistence;
            }

            maxTerrainNoise = boundMaxSum / sumAmp;
            minTerrainNoise = boundMinSum / sumAmp;
        }

        return { minHeightGrad + minTerrainNoise, maxHeightGrad + maxTerrainNoise };
    }

    bool SVOLandscapeGenerator::canCarveCaves(const glm::ivec3& boxMin, const glm::ivec3& boxSize, double volumeHeight, const LandscapeConfig& config, const glm::dvec3& worldOffset)
    {
        if (!config.enableCaves)
        {
            return false;
        }

        double yMin = static_cast<double>(boxMin.y) + worldOffset.y;
        double hMin = (volumeHeight > 0.0) ? (yMin / volumeHeight) : 0.0;
        if (hMin >= 0.9)
        {
            return false;
        }

        glm::dvec3 pMin   = glm::dvec3(boxMin) + worldOffset;
        glm::dvec3 pMax   = glm::dvec3(boxMin + glm::max(glm::ivec3(1), boxSize) - 1) + worldOffset;
        glm::dvec3 center = 0.5 * (pMin + pMax);
        double     radius = 0.5 * glm::length(pMax - pMin);

        constexpr double perlinLipschitz = 2.6;

        double sumAmp     = 1.0 + 0.5;
        double octaveAmp  = 1.0;
        double octaveFreq = config.caveFrequency;

        double boundMaxSum = 0.0;
        double boundMinSum = 0.0;
        for (int i = 0; i < 2; i++)
        {
            double delta = octaveFreq * perlinLipschitz * radius;
            if (delta < 2.0)
            {
                double n = perlinNoise3D(center * octaveFreq, config.seed + 9999 + static_cast<uint32_t>(i * 31));
                boundMaxSum += octaveAmp * std::min(1.0, n + delta);
                boundMinSum += octaveAmp * std::max(-1.0, n - delta);
            }
            else
            {
                boundMaxSum += octaveAmp * 1.0;
                boundMinSum += octaveAmp * (-1.0);
            }
            octaveFreq *= 2.0;
            octaveAmp *= 0.5;
        }

        double minCaveNoise = boundMinSum / sumAmp;
        double maxCaveNoise = boundMaxSum / sumAmp;

        if (minCaveNoise > config.caveThreshold || maxCaveNoise < -config.caveThreshold)
        {
            return false;
        }

        return true;
    }

    CADacomb::BinaryVolume SVOLandscapeGenerator::generateBinaryVolume(const glm::u64vec3& dimension, const LandscapeConfig& config, const glm::dvec3& worldOffset, const glm::dvec3& worldSize)
    {
        CADacomb::BinaryVolume volume;
        volume.dimension = dimension;
        volume.origin    = worldOffset;
        volume.size      = worldSize;

        size_t totalVoxels = dimension.x * dimension.y * dimension.z;
        if (totalVoxels == 0)
        {
            return volume;
        }

        std::vector<uint8_t> temp(totalVoxels, 0);
        double               volumeHeight = static_cast<double>(dimension.y);

        const int64_t dimZ = static_cast<int64_t>(dimension.z);
        const int64_t dimY = static_cast<int64_t>(dimension.y);
        const int64_t dimX = static_cast<int64_t>(dimension.x);

        int64_t maxY   = dimY;
        int64_t solidY = -1;

        if (!config.customDensity && volumeHeight > 0.0 && config.heightScale > 0.0)
        {
            double minSolidRatio = config.baseHeightRatio - (config.solidThreshold + 1.0) / (2.0 * config.heightScale);
            if (!config.enableCaves && minSolidRatio > 0.0)
            {
                double solidWorldY = minSolidRatio * volumeHeight - worldOffset.y;
                if (solidWorldY > 0.0)
                {
                    solidY = std::min(dimY, static_cast<int64_t>(std::floor(solidWorldY)));
                }
            }

            double maxRatio  = config.baseHeightRatio + (1.0 - config.solidThreshold) / (2.0 * config.heightScale);
            double maxWorldY = maxRatio * volumeHeight - worldOffset.y;
            if (maxWorldY >= 0.0)
            {
                maxY = std::min(dimY, static_cast<int64_t>(std::ceil(maxWorldY)) + 1);
            }
            else
            {
                maxY = 0;
            }
        }

        ProgressBar bar(static_cast<size_t>(dimZ), "Generating binary landscape");

#pragma omp parallel for schedule(dynamic) shared(bar)
        for (int64_t z = 0; z < dimZ; z++)
        {
            if (bar.isCanceled())
            {
                continue;
            }

            for (int64_t y = 0; y < maxY; y++)
            {
                if (y < solidY)
                {
                    size_t rowStart = static_cast<size_t>(y * dimX + z * dimX * dimY);
                    std::fill_n(&temp[rowStart], dimX, uint8_t(1));
                    continue;
                }

                for (int64_t x = 0; x < dimX; x++)
                {
                    glm::dvec3 pos     = glm::dvec3(x, y, z) + worldOffset;
                    double     density = evaluateDensity(pos, volumeHeight, config);
                    if (density > config.solidThreshold)
                    {
                        size_t idx = static_cast<size_t>(x + y * dimX + z * dimX * dimY);
                        temp[idx]  = 1;
                    }
                }
            }

            bar.addProgress(1);
        }

        if (bar.isCanceled())
        {
            return volume;
        }

        volume.data.assign(temp.begin(), temp.end());
        return volume;
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/Volume/SVO/Blocky/SVOBlockyTriangulation.h"
#include <catch2/catch_test_macros.hpp>
#include <chrono>

namespace CADacomb
{
    TEST_CASE("SVOLandscape/PerlinNoiseBoundsAndDeterminism", "[SVO][Landscape][Noise]")
    {
        for (int i = 0; i < 100; i++)
        {
            glm::dvec3 pos(i * 0.37, (i % 7) * 1.83, (i % 11) * 0.91);
            double     val = SVOLandscapeGenerator::perlinNoise3D(pos, 42);
            CHECK(val >= -1.0);
            CHECK(val <= 1.0);
        }

        glm::dvec3 testPos(12.345, 67.891, 23.456);
        double     val1 = SVOLandscapeGenerator::perlinNoise3D(testPos, 1337);
        double     val2 = SVOLandscapeGenerator::perlinNoise3D(testPos, 1337);
        CHECK(val1 == val2);

        double valDiffSeed = SVOLandscapeGenerator::perlinNoise3D(testPos, 9999);
        CHECK(val1 != valDiffSeed);

        double valNear = SVOLandscapeGenerator::perlinNoise3D(testPos + glm::dvec3(0.001, 0.0, 0.0), 1337);
        CHECK(std::abs(val1 - valNear) < 0.02);
    }

    TEST_CASE("SVOLandscape/FBMNoiseEvaluation", "[SVO][Landscape][Noise]")
    {
        glm::dvec3 pos(5.0, 10.0, 15.0);
        double     fbmVal = SVOLandscapeGenerator::fbm3D(pos, 4, 0.05, 0.5, 2.0, 123);
        CHECK(fbmVal >= -1.0);
        CHECK(fbmVal <= 1.0);
    }

    TEST_CASE("SVOLandscape/DensityHeightGradient", "[SVO][Landscape]")
    {
        LandscapeConfig config;
        config.baseHeightRatio = 0.5;
        config.heightScale     = 1.0;
        config.seed            = 12345;

        double volumeHeight = 64.0;

        double deepDensity = SVOLandscapeGenerator::evaluateDensity(glm::dvec3(32.0, 4.0, 32.0), volumeHeight, config);
        CHECK(deepDensity > 0.0);

        double skyDensity = SVOLandscapeGenerator::evaluateDensity(glm::dvec3(32.0, 60.0, 32.0), volumeHeight, config);
        CHECK(skyDensity < 0.0);
    }

    TEST_CASE("SVOLandscape/CaveCarving", "[SVO][Landscape]")
    {
        LandscapeConfig configNoCaves;
        configNoCaves.enableCaves     = false;
        configNoCaves.baseHeightRatio = 0.5;
        configNoCaves.seed            = 777;

        LandscapeConfig configWithCaves = configNoCaves;
        configWithCaves.enableCaves     = true;
        configWithCaves.caveThreshold   = 0.99;

        double     volumeHeight = 64.0;
        glm::dvec3 midPos(32.0, 16.0, 32.0);

        double densityWithCaves = SVOLandscapeGenerator::evaluateDensity(midPos, volumeHeight, configWithCaves);
        CHECK(densityWithCaves == -1.0);
    }

    TEST_CASE("SVOLandscape/BinaryVolumeGeneration", "[SVO][Landscape]")
    {
        LandscapeConfig config;
        config.seed            = 42;
        config.baseHeightRatio = 0.5;

        glm::u64vec3 dim(16, 16, 16);
        BinaryVolume vol = SVOLandscapeGenerator::generateBinaryVolume(dim, config);

        REQUIRE(vol.dimension == dim);
        REQUIRE(vol.data.size() == 16 * 16 * 16);

        size_t solidCount = 0;
        for (bool b : vol.data)
        {
            if (b)
            {
                solidCount++;
            }
        }

        CHECK(solidCount > 0);
        CHECK(solidCount < vol.data.size());
    }

    TEST_CASE("SVOLandscape/SVOGenerationAndTriangulation", "[SVO][Landscape]")
    {
        LandscapeConfig config;
        config.seed            = 99;
        config.octaves         = 1;
        config.baseHeightRatio = 0.5;

        auto svo = SVOLandscapeGenerator::generateSVO<bool>(32, config);
        REQUIRE(svo != nullptr);
        CHECK(svo->getDimension() == 32);

        SVOBlockyOptions options;
        options.targetDepth = 5;
        options.origin      = glm::dvec3(0.0);
        options.size        = glm::dvec3(32.0);

        auto mesh = SVOBlockyTriangulation::triangulate(*svo, options);
        REQUIRE(mesh != nullptr);
        CHECK(!mesh->vertices.empty());
        CHECK(!mesh->indices.empty());
        CHECK(mesh->indices.size() % 3 == 0);

        auto [minB, sz] = mesh->getAABB();
        CHECK(minB.x >= 0.0);
        CHECK(minB.y >= 0.0);
        CHECK(minB.z >= 0.0);
        CHECK(minB.x + sz.x <= 32.0 + 1e-4);
        CHECK(minB.y + sz.y <= 32.0 + 1e-4);
        CHECK(minB.z + sz.z <= 32.0 + 1e-4);
    }

    TEST_CASE("SVOLandscape/FillSVOChunk", "[SVO][Landscape]")
    {
        SVO<bool> svo(64, false, MostPrevalentExceptMergeStrategy<bool, false>::instance());

        LandscapeConfig config;
        config.seed            = 101;
        config.octaves         = 1;
        config.baseHeightRatio = 0.5;

        SVOLandscapeGenerator::fillSVOChunk(svo, glm::ivec3(0, 0, 0), glm::ivec3(32, 32, 32), 64.0, config);

        bool hasSolidInA = false;
        for (int z = 0; z < 32 && !hasSolidInA; z++)
        {
            for (int y = 0; y < 16 && !hasSolidInA; y++)
            {
                for (int x = 0; x < 32 && !hasSolidInA; x++)
                {
                    if (svo.getVoxel(glm::ivec3(x, y, z)))
                    {
                        hasSolidInA = true;
                    }
                }
            }
        }
        CHECK(hasSolidInA);

        bool hasSolidInUnfilled = false;
        for (int z = 32; z < 48 && !hasSolidInUnfilled; z++)
        {
            for (int y = 48; y < 64 && !hasSolidInUnfilled; y++)
            {
                for (int x = 32; x < 48 && !hasSolidInUnfilled; x++)
                {
                    if (svo.getVoxel(glm::ivec3(x, y, z)))
                    {
                        hasSolidInUnfilled = true;
                    }
                }
            }
        }
        CHECK_FALSE(hasSolidInUnfilled);
    }

    TEST_CASE("SVOLandscape/EstimateDensityBounds", "[SVO][Landscape]")
    {
        LandscapeConfig config;
        config.seed            = 42;
        config.octaves         = 4;
        config.baseHeightRatio = 0.5;
        config.heightScale     = 1.2;

        double volumeHeight = 128.0;

        glm::ivec3 skyBoxMin(0, 120, 0);
        glm::ivec3 skyBoxSize(8, 8, 8);
        auto [skyMin, skyMax] = SVOLandscapeGenerator::estimateDensityBounds(skyBoxMin, skyBoxSize, volumeHeight, config);
        CHECK(skyMax < 0.0);

        glm::ivec3 deepBoxMin(0, 0, 0);
        glm::ivec3 deepBoxSize(8, 8, 8);
        auto [deepMin, deepMax] = SVOLandscapeGenerator::estimateDensityBounds(deepBoxMin, deepBoxSize, volumeHeight, config);
        CHECK(deepMin > 0.0);
    }

    TEST_CASE("SVOLandscape/EvaluateDensityBlockMatchesScalar", "[SVO][Landscape]")
    {
        LandscapeConfig config;
        config.seed            = 42;
        config.octaves         = 4;
        config.frequency       = 0.05;
        config.baseHeightRatio = 0.5;
        config.heightScale     = 1.2;
        config.enableCaves     = true;
        config.caveThreshold   = 0.12;

        double     volumeHeight = 64.0;
        glm::ivec3 blockMin(12, 24, 28);
        glm::dvec3 worldOffset(0.5, -0.25, 1.0);

        double blockDensities[64];
        SVOLandscapeGenerator::evaluateDensityBlock(blockMin, 4u, volumeHeight, config, worldOffset, blockDensities);

        for (int lz = 0; lz < 4; lz++)
        {
            for (int ly = 0; ly < 4; ly++)
            {
                for (int lx = 0; lx < 4; lx++)
                {
                    glm::dvec3 pos      = glm::dvec3(blockMin.x + lx, blockMin.y + ly, blockMin.z + lz) + worldOffset;
                    double     expected = SVOLandscapeGenerator::evaluateDensity(pos, volumeHeight, config);
                    double     actual   = blockDensities[lx + ly * 4 + lz * 16];
                    CHECK(std::abs(actual - expected) < 1e-12);
                }
            }
        }
    }

    TEST_CASE("SVOLandscape/PerformanceBenchmark", "[SVO][Landscape][.benchmark]")
    {
        LandscapeConfig config;
        config.octaves = 4;
        config.seed    = 1337;

        for (uint32_t res : { 64u, 128u, 256u, 512u, 1024u })
        {
            auto start   = std::chrono::high_resolution_clock::now();
            auto svo     = SVOLandscapeGenerator::generateSVO<bool>(res, config);
            auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start).count();
            REQUIRE(svo != nullptr);
            REQUIRE(svo->getRaw().size() > 0);
            WARN("Generated " << res << "^3 landscape in " << elapsed << " ms (nodes: " << svo->getRaw().size() << ")");
        }
    }
}
#endif
