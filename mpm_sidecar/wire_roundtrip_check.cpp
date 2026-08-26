// Write every block of the MPM wire format with distinctive values, for protocol.py to read back.
//
// ⚠ MATCHING STRUCT SIZES ARE NOT ENOUGH, which is why this exists. Two fields of the same type
// swapped between the C++ header and the Python mirror keeps every size identical and silently
// reinterprets the wire — a collider's linear velocity read as its angular velocity would deform
// sand plausibly and wrongly. Distinctive per-field values are what catch that.
#include "mpm/MpmSidecarProtocol.hpp"

#include <cstdio>
#include <cstring>
#include <string>

using namespace msr::airlib::mpm;

namespace {

template <typename Block>
bool writeSegment(const std::string& dir, const char* name, const Block& block)
{
    const std::string path = dir + "/" + name;
    std::FILE* file = std::fopen(path.c_str(), "wb");
    if (file == nullptr) {
        std::fprintf(stderr, "cannot open %s\n", path.c_str());
        return false;
    }
    const size_t written = std::fwrite(&block, 1, sizeof(Block), file);
    std::fclose(file);
    return written == sizeof(Block);
}

} // namespace

int main(int argc, char** argv)
{
    const std::string dir = argc > 1 ? argv[1] : "/tmp";

    MpmRegistryBlock registry;
    registry.sequence = 2;
    registry.collider_count = 2;
    registry.stamp.world_id = 11;
    registry.stamp.world_revision = 22;
    registry.stamp.manifest_revision = 33;
    registry.stamp.reset_epoch = 44;
    registry.sim_fixed_dt = 0.003;

    std::snprintf(registry.colliders[0].stable_id, kMaxColliderIdChars, "Rover1/wheel_lf");
    registry.colliders[0].shape_count = 1;
    registry.colliders[0].role = static_cast<uint32_t>(WireCouplingRole::KinematicOneWay);
    registry.colliders[0].mass = 1.25;
    registry.colliders[0].com_local = WireVec3{ 0.1, 0.2, 0.3 };
    for (int i = 0; i < 9; ++i)
        registry.colliders[0].inertia_local[i] = 100.0 + i;   // 100..108, order-sensitive
    registry.colliders[0].inertia_is_articulated_effective = 0;
    registry.colliders[0].friction = 0.71;
    registry.colliders[0].restitution = 0.13;
    registry.colliders[0].material_reported = 1;
    registry.colliders[0].shapes[0].kind = static_cast<uint32_t>(WireShapeKind::ConvexHull);
    registry.colliders[0].shapes[0].vertex_count = 3;
    registry.colliders[0].shapes[0].position = WireVec3{ 1.5, 2.5, 3.5 };
    registry.colliders[0].shapes[0].orientation = WireQuat{ 0.1, 0.2, 0.3, 0.927 };
    registry.colliders[0].shapes[0].radius = 0.42;
    registry.colliders[0].shapes[0].half_length = 0.84;
    registry.colliders[0].shapes[0].half_extents = WireVec3{ 4.5, 5.5, 6.5 };
    for (int v = 0; v < 3; ++v)
        registry.colliders[0].shapes[0].vertices[v] =
            WireVec3{ 10.0 + v, 20.0 + v, 30.0 + v };

    std::snprintf(registry.colliders[1].stable_id, kMaxColliderIdChars, "Rover2/wheel_rr");
    registry.colliders[1].shape_count = 1;
    registry.colliders[1].role = static_cast<uint32_t>(WireCouplingRole::Static);
    registry.colliders[1].mass = 2.5;
    registry.colliders[1].shapes[0].kind = static_cast<uint32_t>(WireShapeKind::Sphere);
    registry.colliders[1].shapes[0].radius = 0.05;

    MpmStateBlock state;
    state.sequence = 4;
    state.collider_count = 2;
    state.stamp = registry.stamp;
    state.step = 987654321ull;
    state.simulation_time = 12.75;
    // ⚠ Every one of the four vectors gets a distinct value: this is the swap the size check
    // cannot see.
    state.colliders[0].position = WireVec3{ 1.0, 2.0, 3.0 };
    state.colliders[0].orientation = WireQuat{ 0.5, 0.5, 0.5, 0.5 };
    state.colliders[0].linear_velocity = WireVec3{ 4.0, 5.0, 6.0 };
    state.colliders[0].angular_velocity = WireVec3{ 7.0, 8.0, 9.0 };
    state.colliders[1].position = WireVec3{ -1.0, -2.0, -3.0 };

    MpmStatusBlock status;
    status.sequence = 6;
    status.fault = 0;
    status.stamp = registry.stamp;
    status.acknowledged_step = 987654300ull;
    status.sidecar_step = 176;
    status.sidecar_time = 2.9333;
    status.last_solve_seconds = 0.0122;
    status.particle_count = 250000;
    std::snprintf(status.message, sizeof(status.message), "healthy");

    if (!writeSegment(dir, kRegistrySegment, registry)) return 1;
    if (!writeSegment(dir, kStateSegment, state)) return 1;
    if (!writeSegment(dir, kStatusSegment, status)) return 1;

    std::printf("wrote three segments to %s\n", dir.c_str());
    return 0;
}
