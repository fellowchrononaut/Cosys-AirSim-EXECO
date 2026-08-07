// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Standalone raymap dumper — the entry point step 1 of the camera work would otherwise
// not have. Nothing in this step renders and the UE plugin is out of scope, so a
// header-only camera model would never execute and there would be nothing to validate.
// This builds the raymap through exactly the code the simulator will use: the camera
// model parameters go through AirSimSettings' real settings parsing, and the raymap
// comes out of cameras::buildRaymap.
//
// Builds with one command and no build-system change (run from the repository root):
//
//   g++ -std=c++17 -I AirLib/include -I AirLib/deps/eigen3 tools/raymap_dump.cpp \
//       AirLib/src/common/common_utils/FileSystem.cpp -o /tmp/raymap_dump
//
// FileSystem.cpp is not optional: AirSimSettings::load reaches Settings::getExecutableFullPath
// and FileSystem::ensureFolder, so without it the link fails on undefined references rather
// than the compile failing on a missing header.
//
// Usage:
//   raymap_dump --out FILE [--f64]
//               --settings settings.json [--vehicle NAME] --camera NAME
//   raymap_dump --out FILE [--f64]
//               --type DoubleSphere --width 1328 --height 1328
//               --xi .. --alpha .. --fx .. --fy .. --cx .. --cy ..
//
// The direct-parameter form does not bypass the settings parser: it writes the same
// settings.json a user would write (at 17 significant digits, so doubles round-trip
// exactly) and feeds it through AirSimSettings::load.

#include "cameras/CameraModel.hpp"
#include "cameras/Raymap.hpp"
#include "common/AirSimSettings.hpp"

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

using namespace msr::airlib;

namespace
{

    std::string readFile(const std::string& path, bool& ok)
    {
        std::ifstream f(path);
        if (!f) {
            ok = false;
            return "";
        }
        std::stringstream ss;
        ss << f.rdbuf();
        ok = true;
        return ss.str();
    }

    //17 significant digits round-trips an IEEE double exactly through decimal text
    std::string jsonNumber(const std::string& value)
    {
        std::ostringstream ss;
        ss << std::setprecision(17) << std::stod(value);
        return ss.str();
    }

    std::string synthesiseSettings(const std::map<std::string, std::string>& args)
    {
        static const char* kNumericKeys[] = { "fx", "fy", "cx", "cy", "k1", "k2", "k3", "k4",
                                              "xi", "alpha" };
        std::ostringstream js;
        js << "{\n"
           << "  \"SettingsVersion\": 2.0,\n"
           << "  \"SimMode\": \"Multirotor\",\n"
           << "  \"Vehicles\": { \"Drone\": { \"VehicleType\": \"SimpleFlight\",\n"
           << "    \"Cameras\": { \"cam\": { \"CameraModel\": {\n";
        js << "      \"Type\": \"" << args.at("type") << "\"";
        if (args.count("width"))
            js << ",\n      \"Width\": " << args.at("width");
        if (args.count("height"))
            js << ",\n      \"Height\": " << args.at("height");
        for (const char* key : kNumericKeys) {
            auto it = args.find(key);
            if (it != args.end())
                js << ",\n      \"" << key << "\": " << jsonNumber(it->second);
        }
        if (args.count("fov-degrees"))
            js << ",\n      \"FOV_Degrees\": " << jsonNumber(args.at("fov-degrees"));
        if (args.count("path"))
            js << ",\n      \"Path\": \"" << args.at("path") << "\"";
        js << "\n    } } }\n  } }\n}\n";
        return js.str();
    }

    const AirSimSettings::CameraSetting* findCamera(const std::string& vehicle,
                                                    const std::string& camera,
                                                    std::string& where)
    {
        const AirSimSettings& settings = AirSimSettings::singleton();
        for (const auto& v : settings.vehicles) {
            if (!vehicle.empty() && v.first != vehicle)
                continue;
            auto it = v.second->cameras.find(camera);
            if (it != v.second->cameras.end()) {
                where = v.first;
                return &it->second;
            }
        }
        return nullptr;
    }

    void printResolved(const cameras::CameraModelParams& p)
    {
        std::cout << std::setprecision(17);
        std::cout << "  type   : " << cameras::toString(p.type) << "\n"
                  << "  size   : " << p.width << " x " << p.height << "\n"
                  << "  fx fy  : " << p.fx << " " << p.fy << "\n"
                  << "  cx cy  : " << p.cx << " " << p.cy << "\n";
        if (p.type == cameras::CameraModelType::KannalaBrandt)
            std::cout << "  k1..k4 : " << p.k1 << " " << p.k2 << " " << p.k3 << " " << p.k4 << "\n"
                      << "  invertible up to theta = " << p.kb_max_theta << " rad ("
                      << p.kb_max_theta * 180.0 / cameras::kPi << " deg)\n";
        if (p.type == cameras::CameraModelType::DoubleSphere)
            std::cout << "  xi al  : " << p.xi << " " << p.alpha << "\n";
        std::cout << std::setprecision(6);
    }

    //Measures what the KB root find actually costs, so the iteration cap is a reported
    //number rather than an assertion.
    void reportKbIterations(const cameras::CameraModelParams& p)
    {
        int worst = 0;
        double worst_residual = 0.0;
        size_t failed = 0;
        for (unsigned int y = 0; y < p.height; ++y) {
            for (unsigned int x = 0; x < p.width; ++x) {
                const double mx = (static_cast<double>(x) - p.cx) / p.fx;
                const double my = (static_cast<double>(y) - p.cy) / p.fy;
                const double theta_d = std::sqrt(mx * mx + my * my);
                double theta = 0.0;
                int iterations = 0;
                if (!cameras::detail::kbSolveTheta(p, theta_d, theta, iterations)) {
                    ++failed;
                    continue;
                }
                if (iterations > worst)
                    worst = iterations;
                const double residual = std::abs(cameras::detail::kbForward(p, theta) - theta_d);
                if (residual > worst_residual)
                    worst_residual = residual;
            }
        }
        std::cout << "  KB Newton: max iterations " << worst
                  << ", max |residual| " << std::scientific << worst_residual
                  << std::defaultfloat << " rad, unsolved texels " << failed << "\n";
    }

} //namespace

int main(int argc, char** argv)
{
    std::map<std::string, std::string> args;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a.rfind("--", 0) != 0) {
            std::cerr << "unexpected argument " << a << "\n";
            return 2;
        }
        a = a.substr(2);
        if (a == "f64") {
            args[a] = "1";
            continue;
        }
        if (i + 1 >= argc) {
            std::cerr << "missing value for --" << a << "\n";
            return 2;
        }
        args[a] = argv[++i];
    }

    if (!args.count("out")) {
        std::cerr << "need --out FILE\n";
        return 2;
    }

    std::string settings_text;
    std::string camera_name;
    if (args.count("settings")) {
        bool ok = false;
        settings_text = readFile(args.at("settings"), ok);
        if (!ok) {
            std::cerr << "cannot read " << args.at("settings") << "\n";
            return 2;
        }
        if (!args.count("camera")) {
            std::cerr << "need --camera NAME with --settings\n";
            return 2;
        }
        camera_name = args.at("camera");
    }
    else if (args.count("type")) {
        settings_text = synthesiseSettings(args);
        camera_name = "cam";
    }
    else {
        std::cerr << "need either --settings FILE --camera NAME, or --type TYPE ...\n";
        return 2;
    }

    try {
        AirSimSettings::initializeSettings(settings_text);
        AirSimSettings::singleton().load([]() { return std::string("Multirotor"); });
    }
    catch (const std::exception& e) {
        std::cerr << "settings error: " << e.what() << "\n";
        return 1;
    }
    for (const auto& msg : AirSimSettings::singleton().error_messages)
        std::cerr << "settings error: " << msg << "\n";

    std::string vehicle;
    const AirSimSettings::CameraSetting* camera =
        findCamera(args.count("vehicle") ? args.at("vehicle") : std::string(), camera_name, vehicle);
    if (camera == nullptr) {
        std::cerr << "no camera named " << camera_name << " in the settings\n";
        return 1;
    }
    if (!camera->camera_model.enabled) {
        std::cerr << "camera " << camera_name << " has no CameraModel block\n";
        return 1;
    }
    std::cout << "camera '" << camera_name << "' on vehicle '" << vehicle << "'\n";
    printResolved(camera->camera_model.model);

    cameras::Raymap raymap;
    cameras::RaymapStats stats;
    std::string error;
    if (!cameras::buildRaymap(camera->camera_model.model, raymap, stats, error)) {
        std::cerr << "raymap build failed: " << error << "\n";
        return 1;
    }

    if (camera->camera_model.model.type == cameras::CameraModelType::KannalaBrandt)
        reportKbIterations(camera->camera_model.model);

    const cameras::CubeSamplingRecommendation cube_sampling = cameras::recommendCubeSampling(raymap);

    const bool as_float64 = args.count("f64") != 0;
    if (!cameras::writeRaymap(raymap, args.at("out"), as_float64, error)) {
        std::cerr << "raymap write failed: " << error << "\n";
        return 1;
    }

    std::cout << "  texels : " << stats.texels << " (" << stats.invalid
              << " outside the model's valid domain)\n"
              << "  cube   : " << cube_sampling.face_count << " faces at "
              << cube_sampling.face_resolution << "x" << cube_sampling.face_resolution
              << " (horizontal FOV " << cube_sampling.horizontal_fov_radians * 180.0 / cameras::kPi
              << " deg, vertical FOV " << cube_sampling.vertical_fov_radians * 180.0 / cameras::kPi
              << " deg; axis step "
              << cube_sampling.horizontal_axis_step_radians * 180.0 / cameras::kPi << "/"
              << cube_sampling.vertical_axis_step_radians * 180.0 / cameras::kPi << " deg)\n"
              << "  wrote  : " << args.at("out") << " as "
              << (as_float64 ? "float64" : "float32") << ", "
              << cameras::Raymap::kChannels << " channels per texel\n";
    return 0;
}
