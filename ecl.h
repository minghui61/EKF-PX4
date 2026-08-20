#pragma once
#include <stdint.h>
#include <cmath>
#include <stdio.h>

#ifndef ECL_STANDALONE
#define ECL_STANDALONE 1
#endif

// Mock PX4 Logging Macros to use standard printf
#define ECL_INFO(X, ...) printf("[INFO] " X "\n", ##__VA_ARGS__)
#define ECL_WARN(X, ...) printf("[WARN] " X "\n", ##__VA_ARGS__)
#define ECL_ERR(X, ...)  printf("[ERR] " X "\n", ##__VA_ARGS__)

#define ISFINITE(x) std::isfinite(x)
#define PX4_ISFINITE(x) std::isfinite(x)

inline uint64_t hrt_absolute_time() { return 0; }

struct map_projection_reference_s {
    uint64_t timestamp;
    double lat_rad;
    double lon_rad;
    double sin_lat;
    double cos_lat;
    bool init_done;
};

// Stub out the map projection functions so the linker is happy
inline bool map_projection_initialized(const struct map_projection_reference_s *ref) { return ref->init_done; }
inline int map_projection_init_timestamped(struct map_projection_reference_s *ref, double lat_0, double lon_0, uint64_t timestamp) { ref->init_done = true; return 0; }
inline int map_projection_project(const struct map_projection_reference_s *ref, double lat, double lon, float *x, float *y) { *x = 0.0f; *y = 0.0f; return 0; }
inline int map_projection_reproject(const struct map_projection_reference_s *ref, float x, float y, double *lat, double *lon) { *lat = 0.0; *lon = 0.0; return 0; }

namespace ecl {
    constexpr float atmosphere_pressure_sea_level_pa = 101325.0f;
    constexpr float gas_constant = 287.058f;
    constexpr float standard_gravity = 9.80665f;
}

// Define missing float math constants
#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

#ifndef M_PI_2_F
#define M_PI_2_F 1.57079632679489661923f
#endif
