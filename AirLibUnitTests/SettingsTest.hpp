#ifndef msr_AirLibUnitTests_SettingsTest_hpp
#define msr_AirLibUnitTests_SettingsTest_hpp

#include "TestBase.hpp"
#include "common/AirSimSettings.hpp"
#include "common/Settings.hpp"

namespace msr
{
namespace airlib
{

    class SettingsTest : public TestBase
    {
    public:
        virtual void run() override
        {
            testSettingsNodeTypePredicates();
            testLegacyDefaultAndSettleCompatibility();
            testCoordinatedDefaults();
            testCoordinatedFixedStepClock();
            testStaticWorldMirrorPolicy();
            testCoordinatedGroundPlanePolicy();
            testGroundHeightFieldPolicy();
            testCoordinatorSchema();
            testSingletonReloadReset();
            testStartupCVarSettings();
            testNonObjectSchemaPositions();
            testStrictValidation();
        }

    private:
        void loadSettings(const std::string& json)
        {
            AirSimSettings::initializeSettings(json);
            AirSimSettings::singleton().load([]() { return std::string("ComputerVision"); });
        }

        void expectInvalid(const std::string& json, const std::string& expected_text)
        {
            bool threw = false;
            try {
                loadSettings(json);
            }
            catch (const std::invalid_argument& ex) {
                threw = true;
                testAssert(std::string(ex.what()).find(expected_text) != std::string::npos,
                           "Settings rejection did not contain expected text '" + expected_text +
                               "': " + ex.what());
            }
            testAssert(threw, "Expected invalid settings to be rejected: " + expected_text);
        }

        void testSettingsNodeTypePredicates()
        {
            Settings& root = Settings::loadJSonString(R"json({
                "Object": { "Value": 1 },
                "Array": [{ "Value": 1 }],
                "EmptyObject": {},
                "EmptyArray": [],
                "String": "value",
                "Bool": true,
                "Integer": 4,
                "Float": 4.0
            })json");
            testAssert(root.isObject(), "Settings root object was not identified as an object");
            testAssert(!root.isArray(), "Settings root object was incorrectly identified as an array");

            Settings child;
            testAssert(root.getChild("Object", child) && child.isObject() && !child.isArray(),
                       "Non-empty Settings object predicate failed");
            testAssert(root.getChild("Array", child) && child.isArray() && !child.isObject(),
                       "Non-empty Settings array predicate failed");
            testAssert(root.getChild("EmptyObject", child) && child.isObject() && !child.isArray(),
                       "Empty Settings object predicate failed");
            testAssert(root.getChild("EmptyArray", child) && child.isArray() && !child.isObject(),
                       "Empty Settings array predicate failed");
            testAssert(root.isString("String") && !root.isString("Bool"),
                       "Settings string key predicate failed");
            testAssert(root.isBool("Bool") && !root.isBool("Integer"),
                       "Settings boolean key predicate failed");
            testAssert(root.isInteger("Integer") && !root.isInteger("Float"),
                       "Settings integer key predicate coerced a JSON float");
            testAssert(root.isNumber("Integer") && root.isNumber("Float") &&
                           !root.isNumber("Bool"),
                       "Settings numeric key predicate failed");
        }

        void testLegacyDefaultAndSettleCompatibility()
        {
            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision"
            })json");
            const AirSimSettings& defaults = AirSimSettings::singleton();
            testAssert(defaults.physics_coordinator.mode ==
                           AirSimSettings::PhysicsCoordinatorMode::Legacy,
                       "Absent PhysicsCoordinator must select Legacy");
            testAssert(defaults.physics_objects.empty(),
                       "Absent PhysicsObjects must leave the registry empty");
            testAssert(defaults.deformable_terrains.empty(),
                       "Absent DeformableTerrains must leave the registry empty");

            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "MultiAgent",
                "PhysicsCoordinator": { "Mode": "Legacy" },
                "Vehicles": {
                    "Bot": {
                        "VehicleType": "urdfbot",
                        "UrdfFile": "robot.urdf",
                        "UrdfSettleSeconds": 0.125
                    }
                }
            })json");
            const AirSimSettings::VehicleSetting* bot =
                AirSimSettings::singleton().getVehicleSetting("Bot");
            testAssert(bot->urdf_settle_seconds, 0.125,
                       "Legacy mode must preserve explicit per-robot UrdfSettleSeconds");
        }

        void testCoordinatedDefaults()
        {
            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": { "Mode": "CoordinatedSingleBackend" }
            })json");
            const AirSimSettings::PhysicsCoordinatorSetting& coordinator =
                AirSimSettings::singleton().physics_coordinator;
            testAssert(coordinator.mode ==
                           AirSimSettings::PhysicsCoordinatorMode::CoordinatedSingleBackend,
                       "CoordinatedSingleBackend mode was not parsed");
            testAssert(coordinator.presettle,
                       "Coordinated mode must default Presettle to true");
            testAssert(coordinator.presettle_seconds, 0.5,
                       "Coordinated mode must default PresettleSeconds to 0.5");

            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "MixedBackendExperimental",
                    "Presettle": false,
                    "PresettleSeconds": 0.0
                }
            })json");
            const AirSimSettings::PhysicsCoordinatorSetting& mixed =
                AirSimSettings::singleton().physics_coordinator;
            testAssert(mixed.mode ==
                           AirSimSettings::PhysicsCoordinatorMode::MixedBackendExperimental,
                       "MixedBackendExperimental mode was not parsed");
            testAssert(!mixed.presettle, "Explicit Presettle=false was not parsed");

            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "PresettleSeconds": 1
                }
            })json");
            testAssert(AirSimSettings::singleton().physics_coordinator.presettle_seconds, 1.0,
                       "Strict numeric parsing must accept a JSON integer for a double field");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "MultiAgent",
                "PhysicsCoordinator": { "Mode": "CoordinatedSingleBackend" },
                "Vehicles": {
                    "Bot": {
                        "VehicleType": "urdfbot",
                        "UrdfFile": "robot.urdf",
                        "UrdfSettleSeconds": 0.25
                    }
                }
            })json",
                          "PhysicsCoordinator.PresettleSeconds");
        }

        void testCoordinatedFixedStepClock()
        {
            // Coordinated physics propagates one fixed dt. A wall-derived clock cannot supply one,
            // so the default is promoted rather than left to whatever the sim mode preferred.
            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": { "Mode": "CoordinatedSingleBackend" }
            })json");
            testAssert(AirSimSettings::singleton().clock_type == "SteppableClock",
                       "Coordinated mode must select the fixed-step clock");

            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "ClockType": "SteppableClock",
                "PhysicsCoordinator": { "Mode": "MixedBackendExperimental" }
            })json");
            testAssert(AirSimSettings::singleton().clock_type == "SteppableClock",
                       "An authored SteppableClock must be accepted in coordinated mode");

            // An authored wall clock is a configuration error, not a silent downgrade to a
            // non-reproducible run.
            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "ClockType": "ScalableClock",
                "PhysicsCoordinator": { "Mode": "CoordinatedSingleBackend" }
            })json",
                          "ClockType 'ScalableClock' cannot be used");

            // Legacy behaviour is untouched: the authored wall clock survives.
            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "ClockType": "ScalableClock"
            })json");
            testAssert(AirSimSettings::singleton().clock_type == "ScalableClock",
                       "Legacy mode must keep an authored ScalableClock");
        }

        void testStaticWorldMirrorPolicy()
        {
            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "StaticWorldMirror": {
                        "CollisionSource": "Complex",
                        "IncludeMovable": true,
                        "MaxInstances": 1200,
                        "RequiredTags": ["Solid"],
                        "DefaultFriction": 0.9
                    }
                }
            })json");
            const AirSimSettings::StaticWorldMirrorSetting& mirror =
                AirSimSettings::singleton().physics_coordinator.static_world_mirror;
            testAssert(mirror.collision_source == AirSimSettings::StaticWorldCollisionSource::Complex,
                       "StaticWorldMirror.CollisionSource was not parsed");
            testAssert(mirror.include_movable, "StaticWorldMirror.IncludeMovable was not parsed");
            testAssert(mirror.max_instances == 1200, "StaticWorldMirror.MaxInstances was not parsed");
            testAssert(mirror.required_tags.size() == 1 && mirror.required_tags[0] == "Solid",
                       "StaticWorldMirror.RequiredTags was not parsed");
            testAssert(mirror.default_friction, 0.9,
                       "StaticWorldMirror.DefaultFriction was not parsed");
            testAssert(mirror.include_landscape && mirror.include_instanced_meshes && mirror.enabled,
                       "Unset StaticWorldMirror keys must keep their documented defaults");

            // The section describes a shared scene that Legacy mode does not have.
            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "Legacy",
                    "StaticWorldMirror": { "IncludeMovable": true }
                }
            })json",
                          "requires a coordinated PhysicsCoordinator.Mode");

            // A kinematic copy of a native body in the same shared scene is a second ghost of it.
            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "StaticWorldMirror": { "IncludeOtherVehicles": true }
                }
            })json",
                          "cross-authority edge");

            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "MixedBackendExperimental",
                    "StaticWorldMirror": { "IncludeOtherVehicles": true }
                }
            })json");
            testAssert(AirSimSettings::singleton()
                           .physics_coordinator.static_world_mirror.include_other_vehicles,
                       "A declared mixed-backend proxy mirror must be accepted");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "StaticWorldMirror": { "MaxInstances": 0 }
                }
            })json",
                          "MaxInstances");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "StaticWorldMirror": { "CollisionSource": "TriMesh" }
                }
            })json",
                          "CollisionSource");

            // Per-robot mirror keys are rejected with a message naming their world-level home.
            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "MultiAgent",
                "PhysicsCoordinator": { "Mode": "CoordinatedSingleBackend" },
                "Vehicles": {
                    "Bot": {
                        "VehicleType": "urdfbot",
                        "UrdfFile": "robot.urdf",
                        "UrdfMirrorLandscape": false
                    }
                }
            })json",
                          "PhysicsCoordinator.StaticWorldMirror.IncludeLandscape");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "MultiAgent",
                "PhysicsCoordinator": { "Mode": "CoordinatedSingleBackend" },
                "Vehicles": {
                    "Bot": {
                        "VehicleType": "urdfbot",
                        "UrdfFile": "robot.urdf",
                        "UrdfWorldGeometryTags": ["Solid"]
                    }
                }
            })json",
                          "PhysicsCoordinator.StaticWorldMirror.RequiredTags");

            // Legacy mode keeps every per-robot mirror key working exactly as before.
            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "MultiAgent",
                "Vehicles": {
                    "Bot": {
                        "VehicleType": "urdfbot",
                        "UrdfFile": "robot.urdf",
                        "UrdfMirrorLandscape": false,
                        "UrdfMirrorMaxInstances": 7
                    }
                }
            })json");
            const auto& legacy_vehicle = *AirSimSettings::singleton().vehicles.at("Bot");
            testAssert(!legacy_vehicle.urdf_mirror_landscape && legacy_vehicle.urdf_mirror_max_instances == 7,
                       "Legacy mode must keep the per-vehicle mirror keys");
        }

        void testCoordinatedGroundPlanePolicy()
        {
            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "GroundPlane": { "Mode": "Explicit", "Z": 0.36, "HalfExtent": 50 }
                }
            })json");
            const AirSimSettings::CoordinatedGroundPlaneSetting& ground =
                AirSimSettings::singleton().physics_coordinator.ground_plane;
            testAssert(ground.mode == AirSimSettings::CoordinatedGroundPlaneMode::Explicit,
                       "GroundPlane.Mode=Explicit was not parsed");
            testAssert(ground.z, 0.36, "GroundPlane.Z was not parsed");
            testAssert(ground.half_extent, 50.0, "GroundPlane.HalfExtent was not parsed");

            testAssert(AirSimSettings::singleton().physics_coordinator.static_world_mirror.enabled,
                       "An unrelated section must keep its defaults");

            // A probe describes one robot's spawn, not a shared world's floor.
            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "GroundPlane": { "Mode": "Probe" }
                }
            })json",
                          "does not support 'Probe'");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "GroundPlane": { "Mode": "Explicit" }
                }
            })json",
                          "GroundPlane.Z' is required");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "GroundPlane": { "Mode": "None", "Z": 1.0 }
                }
            })json",
                          "has no meaning when Mode is 'None'");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "MultiAgent",
                "PhysicsCoordinator": { "Mode": "CoordinatedSingleBackend" },
                "Vehicles": {
                    "Bot": {
                        "VehicleType": "urdfbot",
                        "UrdfFile": "robot.urdf",
                        "UrdfGroundPlaneZ": 0.5
                    }
                }
            })json",
                          "PhysicsCoordinator.GroundPlane.Z");
        }

        void testGroundHeightFieldPolicy()
        {
            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "GroundHeightField": {
                        "Mode": "Authored",
                        "Center": { "X": 10.0, "Y": -5.0 },
                        "HalfExtent": 25.0,
                        "CellSize": 0.5
                    }
                }
            })json");
            const AirSimSettings::GroundHeightFieldSetting& field =
                AirSimSettings::singleton().physics_coordinator.ground_height_field;
            testAssert(field.mode == AirSimSettings::GroundHeightFieldMode::Authored,
                       "GroundHeightField.Mode=Authored was not parsed");
            testAssert(field.center_x, 10.0, "GroundHeightField.Center.X was not parsed");
            testAssert(field.center_y, -5.0, "GroundHeightField.Center.Y was not parsed");
            testAssert(field.half_extent, 25.0, "GroundHeightField.HalfExtent was not parsed");
            testAssert(field.cell_size, 0.5, "GroundHeightField.CellSize was not parsed");
            // 50 m across at 0.5 m cells is a 101 x 101 grid.
            testAssert(field.samplesPerSide(field.half_extent) == 101,
                       "the grid side must follow from extent and cell size");

            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "GroundHeightField": { "Mode": "AutoFromSpawns", "Margin": 15.0 }
                }
            })json");
            testAssert(AirSimSettings::singleton()
                               .physics_coordinator.ground_height_field.mode ==
                           AirSimSettings::GroundHeightFieldMode::AutoFromSpawns,
                       "AutoFromSpawns was not parsed");
            testAssert(AirSimSettings::singleton().physics_coordinator.ground_height_field.margin,
                       15.0, "GroundHeightField.Margin was not parsed");

            // An authored region needs both halves of its definition.
            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "GroundHeightField": { "Mode": "Authored", "HalfExtent": 10.0 }
                }
            })json",
                          "requires both Center and HalfExtent");

            // A derived region must not also carry an authored one.
            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "GroundHeightField": {
                        "Mode": "AutoFromSpawns",
                        "Center": { "X": 0.0, "Y": 0.0 }
                    }
                }
            })json",
                          "only apply when Mode=Authored");

            // ⚠ The cell budget is enforced at LOAD, not discovered as a freeze: one downward
            // trace per cell runs on the game thread. 2 km across at 0.1 m is 400 million cells.
            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "GroundHeightField": {
                        "Mode": "Authored",
                        "Center": { "X": 0.0, "Y": 0.0 },
                        "HalfExtent": 1000.0,
                        "CellSize": 0.1
                    }
                }
            })json",
                          "above MaxCells");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "GroundHeightField": { "Mode": "Authored", "Center": { "X": 0.0, "Y": 0.0 },
                                            "HalfExtent": 10.0, "CellSize": 0.0 }
                }
            })json",
                          "CellSize");

            // Legacy has no shared world for this to describe.
            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "Legacy",
                    "GroundHeightField": { "Mode": "AutoFromSpawns" }
                }
            })json",
                          "requires a coordinated PhysicsCoordinator.Mode");
        }

        void testCoordinatorSchema()
        {
            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "MultiAgent",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "CrossEngineContact": "ReciprocalKinematicProxy",
                    "TopologyPolicy": "Fixed",
                    "ResetSimulationTime": true,
                    "Presettle": true,
                    "PresettleSeconds": 0.5,
                    "ChaosMPMRegistration": {
                        "Mode": "Explicit",
                        "IncludeTags": [],
                        "ExcludeTags": []
                    }
                },
                "PhysicsObjects": {
                    "falling_box": {
                        "Source": {
                            "Type": "LevelComponent",
                            "ActorPath": "/Game/Maps/Test.Test:PersistentLevel.Box_0",
                            "ComponentPath": "BoxMesh"
                        },
                        "PhysicsAuthority": "Box3D",
                        "MotionType": "Dynamic",
                        "InteractWithMPM": true,
                        "MPMPatches": ["bed"]
                    }
                },
                "DeformableTerrains": {
                    "bed": {
                        "Enabled": true,
                        "Backend": "NewtonMPM",
                        "Seed": 12345,
                        "Region": {
                            "Center": { "X": 0.0, "Y": 0.0, "Z": 0.0 },
                            "Size": { "X": 3.0, "Y": 3.0, "Z": 0.3 }
                        },
                        "Coupling": {
                            "Mode": "KinematicOneWay",
                            "RigidTicksPerMPMStep": 4
                        },
                        "RigidSurfacePolicy": "AuthoredReplacementPatch"
                    }
                },
                "Vehicles": {
                    "Bot": {
                        "VehicleType": "urdfbot",
                        "UrdfFile": "robot.urdf",
                        "UrdfLinkPhysics": {
                            "foot": {
                                "InteractWithMPM": true,
                                "MPMPatches": ["bed"]
                            }
                        }
                    }
                }
            })json");

            const AirSimSettings& settings = AirSimSettings::singleton();
            testAssert(settings.physics_objects.size(), 1.0,
                       "PhysicsObjects placeholder did not parse");
            const AirSimSettings::PhysicsObjectSetting& object =
                settings.physics_objects.at("falling_box");
            testAssert(object.has_physics_authority &&
                           object.physics_authority == AirSimSettings::PhysicsObjectAuthority::Box3D,
                       "PhysicsObject authority did not parse");
            testAssert(object.has_motion_type &&
                           object.motion_type == AirSimSettings::PhysicsObjectMotionType::Dynamic,
                       "PhysicsObject motion type did not parse");
            testAssert(object.mpm_patches.size(), 1.0,
                       "PhysicsObject MPMPatches did not parse");

            testAssert(settings.deformable_terrains.size(), 1.0,
                       "DeformableTerrains placeholder did not parse");
            const AirSimSettings::DeformableTerrainSetting& terrain =
                settings.deformable_terrains.at("bed");
            testAssert(terrain.seed, 12345.0, "Deformable terrain seed did not parse");
            testAssert(terrain.region.size.z, 0.3,
                       "Deformable terrain region did not parse");
            testAssert(terrain.coupling.rigid_ticks_per_mpm_step, 4.0,
                       "Deformable terrain coupling cadence did not parse");

            const AirSimSettings::VehicleSetting* bot = settings.getVehicleSetting("Bot");
            testAssert(bot->urdf_settle_seconds, 0.0,
                       "Coordinated mode must disable the legacy per-robot settle path");
            testAssert(bot->urdf_link_physics.at("foot").interact_with_mpm,
                       "UrdfLinkPhysics InteractWithMPM did not parse");
        }

        void testSingletonReloadReset()
        {
            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "MultiAgent",
                "PhysicsCoordinator": {
                    "Mode": "MixedBackendExperimental",
                    "Presettle": false,
                    "PresettleSeconds": 0.0,
                    "ChaosMPMRegistration": {
                        "Mode": "Explicit",
                        "IncludeTags": ["old-tag"]
                    }
                },
                "PhysicsObjects": {
                    "old-object": {
                        "Source": {
                            "Type": "LevelComponent",
                            "ActorPath": "/Game/Maps/Test.Test:PersistentLevel.OldObject_0",
                            "ComponentPath": "Mesh"
                        }
                    }
                },
                "DeformableTerrains": {
                    "old-terrain": { "Enabled": false }
                },
                "Vehicles": {
                    "OldBot": {
                        "VehicleType": "urdfbot",
                        "UrdfFile": "robot.urdf",
                        "UrdfLinkPhysics": {
                            "foot": { "InteractWithMPM": false }
                        }
                    }
                }
            })json");
            const AirSimSettings& populated = AirSimSettings::singleton();
            testAssert(populated.physics_objects.size(), 1.0,
                       "Reload setup did not populate PhysicsObjects");
            testAssert(populated.deformable_terrains.size(), 1.0,
                       "Reload setup did not populate DeformableTerrains");
            testAssert(populated.vehicles.find("OldBot") != populated.vehicles.end(),
                       "Reload setup did not populate the URDF vehicle");
            testAssert(populated.physics_coordinator.chaos_mpm_registration.include_tags.size(), 1.0,
                       "Reload setup did not populate registration tags");

            // An early coordinator parse failure must not leave registries or vehicle-owned link
            // settings from the previous singleton load visible under a reset/default mode.
            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": { "Mode": "InvalidMode" }
            })json",
                          "PhysicsCoordinator.Mode");
            const AirSimSettings& failed = AirSimSettings::singleton();
            testAssert(failed.physics_coordinator.mode ==
                           AirSimSettings::PhysicsCoordinatorMode::Legacy,
                       "Failed reload did not reset coordinator mode");
            testAssert(failed.physics_objects.empty() && failed.deformable_terrains.empty(),
                       "Failed reload retained a previous coordinator registry");
            testAssert(failed.vehicles.empty(),
                       "Failed reload retained previous vehicle/UrdfLinkPhysics settings");

            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision"
            })json");
            const AirSimSettings& reloaded = AirSimSettings::singleton();
            testAssert(reloaded.physics_coordinator.mode ==
                           AirSimSettings::PhysicsCoordinatorMode::Legacy,
                       "Minimal reload did not restore Legacy mode");
            testAssert(reloaded.physics_coordinator.presettle,
                       "Minimal reload retained an old Presettle override");
            testAssert(reloaded.physics_coordinator.presettle_seconds, 0.5,
                       "Minimal reload retained an old PresettleSeconds override");
            testAssert(reloaded.physics_coordinator.chaos_mpm_registration.include_tags.empty(),
                       "Minimal reload retained old Chaos MPM tags");
            testAssert(reloaded.physics_objects.empty() && reloaded.deformable_terrains.empty(),
                       "Minimal reload retained old coordinator registries");
            testAssert(reloaded.vehicles.size(), 1.0,
                       "Minimal ComputerVision reload did not rebuild exactly one default vehicle");
            testAssert(reloaded.vehicles.find("ComputerVision") != reloaded.vehicles.end() &&
                           reloaded.vehicles.find("OldBot") == reloaded.vehicles.end(),
                       "Minimal reload retained the previous URDF vehicle");
        }

        void testStartupCVarSettings()
        {
            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "StartupCVars": {
                    "airsim.StreamCaptureInFlight": 2,
                    "airsim.CaptureSchedulerHz": 10.5,
                    "airsim.StreamDir": "/tmp/captures"
                }
            })json");

            const auto& cvars = AirSimSettings::singleton().startup_cvars;
            testAssert(cvars.size(), 3.0, "StartupCVars did not load all scalar overrides");
            const auto& integer = cvars.at("airsim.StreamCaptureInFlight");
            testAssert(integer.value_type == AirSimSettings::StartupCVarSetting::ValueType::Integer &&
                           integer.integer_value == 2,
                       "StartupCVars integer value was not preserved");
            const auto& number = cvars.at("airsim.CaptureSchedulerHz");
            testAssert(number.value_type == AirSimSettings::StartupCVarSetting::ValueType::Number,
                       "StartupCVars number type was not preserved");
            testAssert(number.number_value, 10.5,
                       "StartupCVars number value was not preserved");
            const auto& string = cvars.at("airsim.StreamDir");
            testAssert(string.value_type == AirSimSettings::StartupCVarSetting::ValueType::String &&
                           string.string_value == "/tmp/captures",
                       "StartupCVars string value was not preserved");

            loadSettings(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision"
            })json");
            testAssert(AirSimSettings::singleton().startup_cvars.empty(),
                       "Minimal reload retained StartupCVars from the previous settings document");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "StartupCVars": []
            })json",
                          "StartupCVars' must be a JSON object");
            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "StartupCVars": { "airsim.Enabled": true }
            })json",
                          "must be an integer, number, or string");
            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "StartupCVars": { "airsim.Test;quit": 1 }
            })json",
                          "unsafe CVar name");
        }

        void testNonObjectSchemaPositions()
        {
            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": [{ "Mode": "Legacy" }]
            })json",
                          "Settings key 'PhysicsCoordinator' must be a JSON object");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "ChaosMPMRegistration": [{ "Mode": "Explicit" }]
                }
            })json",
                          "Settings key 'PhysicsCoordinator.ChaosMPMRegistration' must be a JSON object");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsObjects": [{ "box": {} }]
            })json",
                          "Settings key 'PhysicsObjects' must be a JSON object");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsObjects": { "box": [{ "Source": {} }] }
            })json",
                          "Settings key 'PhysicsObjects.box' must be a JSON object");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsObjects": {
                    "box": { "Source": [{ "Type": "LevelComponent" }] }
                }
            })json",
                          "Settings key 'PhysicsObjects.box.Source' must be a JSON object");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "DeformableTerrains": [{ "bed": {} }]
            })json",
                          "Settings key 'DeformableTerrains' must be a JSON object");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "DeformableTerrains": { "bed": [{ "Enabled": false }] }
            })json",
                          "Settings key 'DeformableTerrains.bed' must be a JSON object");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "DeformableTerrains": {
                    "bed": {
                        "Enabled": true,
                        "Backend": "NewtonMPM",
                        "Seed": 1,
                        "Region": [{ "Center": {}, "Size": {} }]
                    }
                }
            })json",
                          "Settings key 'DeformableTerrains.bed.Region' must be a JSON object");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "DeformableTerrains": {
                    "bed": {
                        "Enabled": true,
                        "Backend": "NewtonMPM",
                        "Seed": 1,
                        "Region": {
                            "Center": [{ "X": 0, "Y": 0, "Z": 0 }],
                            "Size": { "X": 1, "Y": 1, "Z": 1 }
                        }
                    }
                }
            })json",
                          "Settings key 'DeformableTerrains.bed.Region.Center' must be a JSON object");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "DeformableTerrains": {
                    "bed": {
                        "Enabled": true,
                        "Backend": "NewtonMPM",
                        "Seed": 1,
                        "Region": {
                            "Center": { "X": 0, "Y": 0, "Z": 0 },
                            "Size": [{ "X": 1, "Y": 1, "Z": 1 }]
                        }
                    }
                }
            })json",
                          "Settings key 'DeformableTerrains.bed.Region.Size' must be a JSON object");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "DeformableTerrains": {
                    "bed": {
                        "Enabled": true,
                        "Backend": "NewtonMPM",
                        "Seed": 1,
                        "Region": {
                            "Center": { "X": 0, "Y": 0, "Z": 0 },
                            "Size": { "X": 1, "Y": 1, "Z": 1 }
                        },
                        "Coupling": [{ "Mode": "KinematicOneWay" }]
                    }
                }
            })json",
                          "Settings key 'DeformableTerrains.bed.Coupling' must be a JSON object");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "MultiAgent",
                "Vehicles": {
                    "Bot": {
                        "VehicleType": "urdfbot",
                        "UrdfFile": "robot.urdf",
                        "UrdfLinkPhysics": [{ "foot": {} }]
                    }
                }
            })json",
                          "Settings key 'Vehicles.Bot.UrdfLinkPhysics' must be a JSON object");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "MultiAgent",
                "Vehicles": {
                    "Bot": {
                        "VehicleType": "urdfbot",
                        "UrdfFile": "robot.urdf",
                        "UrdfLinkPhysics": {
                            "foot": [{ "InteractWithMPM": true }]
                        }
                    }
                }
            })json",
                          "Settings key 'Vehicles.Bot.UrdfLinkPhysics.foot' must be a JSON object");
        }

        void testStrictValidation()
        {
            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": { "Mode": true }
            })json",
                          "PhysicsCoordinator.Mode' must be a string");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "Presettle": 1
                }
            })json",
                          "PhysicsCoordinator.Presettle' must be a boolean");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "DeformableTerrains": {
                    "bed": {
                        "Enabled": true,
                        "Backend": "NewtonMPM",
                        "Seed": 4.0
                    }
                }
            })json",
                          "DeformableTerrains.bed.Seed' must be an integer");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": { "Mode": "Coordinated" }
            })json",
                          "PhysicsCoordinator.Mode");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "PreSettle": true
                }
            })json",
                          "PhysicsCoordinator.PreSettle");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsCoordinator": {
                    "Mode": "CoordinatedSingleBackend",
                    "ChaosMPMRegistration": { "Mode": "AllSupported" }
                }
            })json",
                          "reserved and not implemented in v1");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsObjects": []
            })json",
                          "PhysicsObjects");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "PhysicsObjects": {
                    "box": {
                        "Source": {
                            "Type": "LevelComponent",
                            "ActorPath": "/Game/Map.Box",
                            "ComponentPath": "Mesh"
                        },
                        "InteractWithMPM": "true"
                    }
                }
            })json",
                          "InteractWithMPM");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "ComputerVision",
                "DeformableTerrains": {
                    "bed": { "Enabled": true, "Backend": "NewtonMPM" }
                }
            })json",
                          "requires Backend, Seed, Region, Coupling, and RigidSurfacePolicy");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "MultiAgent",
                "Vehicles": {
                    "Bot": {
                        "VehicleType": "urdfbot",
                        "UrdfFile": "robot.urdf",
                        "UrdfLinkPhysics": {
                            "foot": { "InteractWithMMP": true }
                        }
                    }
                }
            })json",
                          "InteractWithMMP");

            expectInvalid(R"json({
                "SettingsVersion": 2.0,
                "SimMode": "MultiAgent",
                "Vehicles": {
                    "Bot": {
                        "VehicleType": "urdfbot",
                        "UrdfFile": "robot.urdf",
                        "UrdfLinkPhysics": {
                            "foot": {
                                "InteractWithMPM": true,
                                "MPMPatches": ["missing"]
                            }
                        }
                    }
                }
            })json",
                          "unknown deformable terrain 'missing'");
        }
    };
}
}
#endif
