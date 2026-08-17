// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

using UnrealBuildTool;
using System;
using System.IO;

public class AirSim : ModuleRules
{
    private string ModulePath
    {
        get { return ModuleDirectory; }
    }

    private string AirLibPath
    {
        get { return Path.Combine(ModulePath, "AirLib"); }
    }
    private string AirSimPluginPath
    {
        get { return Directory.GetParent(ModulePath).FullName; }
    }
    private string ProjectBinariesPath
    {
        get
        {
            return Path.Combine(
                Directory.GetParent(AirSimPluginPath).Parent.FullName, "Binaries");
        }
    }
    private string AirSimPluginDependencyPath
    {
        get { return Path.Combine(AirSimPluginPath, "Dependencies"); }
    }

    private enum CompileMode
    {
        HeaderOnlyNoRpc,
        HeaderOnlyWithRpc,
        CppCompileNoRpc,
        CppCompileWithRpc
    }

    private void SetupCompileMode(CompileMode mode, ReadOnlyTargetRules Target)
    {
        LoadAirSimDependency(Target, "MavLinkCom", "MavLinkCom");

        switch (mode)
        {
            case CompileMode.HeaderOnlyNoRpc:
                PublicDefinitions.Add("AIRLIB_HEADER_ONLY=1");
                PublicDefinitions.Add("AIRLIB_NO_RPC=1");
                AddLibDependency("AirLib", Path.Combine(AirLibPath, "lib"), "AirLib", Target, false);
                break;

            case CompileMode.HeaderOnlyWithRpc:
                PublicDefinitions.Add("AIRLIB_HEADER_ONLY=1");
                AddLibDependency("AirLib", Path.Combine(AirLibPath, "lib"), "AirLib", Target, false);
                LoadAirSimDependency(Target, "rpclib", "rpc");
                LoadOptionalAirSimDependency(Target, "box3d", "box3d");
                break;

            case CompileMode.CppCompileNoRpc:
                LoadAirSimDependency(Target, "MavLinkCom", "MavLinkCom");
                PublicDefinitions.Add("AIRLIB_NO_RPC=1");
                break;

            case CompileMode.CppCompileWithRpc:
                LoadAirSimDependency(Target, "rpclib", "rpc");
                LoadOptionalAirSimDependency(Target, "box3d", "box3d");
                break;

            default:
                throw new System.Exception("CompileMode specified in plugin's Build.cs file is not recognized");
        }

    }

    public AirSim(ReadOnlyTargetRules Target) : base(Target)
    {
        //bEnforceIWYU = true; //to support 4.16
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        bEnableExceptions = true;

        // "Projects" is needed by AirSim.cpp's IPluginManager lookup, which registers the
        // /Plugin/AirSim shader directory for the Phase 3b camera-model resample shader.
        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "ImageWrapper", "RenderCore", "RHI", "AssetRegistry", "PhysicsCore", "ChaosVehicles", "Landscape", "CinematicCamera", "Projects", "ProceduralMeshComponent" });
        PrivateDependencyModuleNames.AddRange(new string[] { "UMG", "Slate", "SlateCore", "RenderCore", "ChaosVehicles", "DesktopPlatform" });

        //suppress VC++ proprietary warnings
        PublicDefinitions.Add("_SCL_SECURE_NO_WARNINGS=1");
        PublicDefinitions.Add("_CRT_SECURE_NO_WARNINGS=1");
        PublicDefinitions.Add("HMD_MODULE_INCLUDED=0");

        PublicIncludePaths.Add(Path.Combine(AirLibPath, "include"));
        PublicIncludePaths.Add(Path.Combine(AirLibPath, "deps", "eigen3"));
        AddOSLibDependencies(Target);
        PrivateIncludePaths.AddRange(
            new string[] {
            Path.Combine(GetModuleDirectory("Renderer"), "Private"),
            Path.Combine(GetModuleDirectory("Renderer"), "Internal"),
            }
        );
        

        SetupCompileMode(CompileMode.HeaderOnlyWithRpc, Target);
    }

    private void AddOSLibDependencies(ReadOnlyTargetRules Target)
    {
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // for SHGetFolderPath.
            PublicAdditionalLibraries.Add("Shell32.lib");

            //for joystick support
            PublicAdditionalLibraries.Add("dinput8.lib");
            PublicAdditionalLibraries.Add("dxguid.lib");
        }

        if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            // needed when packaging
            PublicAdditionalLibraries.Add("stdc++");
            PublicAdditionalLibraries.Add("supc++");
        }
    }

    static void CopyFileIfNewer(string srcFilePath, string destFolder)
    {
        FileInfo srcFile = new FileInfo(srcFilePath);
        FileInfo destFile = new FileInfo(Path.Combine(destFolder, srcFile.Name));
        if (!destFile.Exists || srcFile.LastWriteTime > destFile.LastWriteTime)
        {
            srcFile.CopyTo(destFile.FullName, true);
        }
        //else skip
    }

    private bool LoadAirSimDependency(ReadOnlyTargetRules Target, string LibName, string LibFileName)
    {
        string LibrariesPath = Path.Combine(AirLibPath, "deps", LibName, "lib");
        return AddLibDependency(LibName, LibrariesPath, LibFileName, Target, true);
    }

    // As LoadAirSimDependency, but for a dependency that may legitimately be absent.
    //
    // AddLibDependency decides isLibrarySupported from the *platform*, not from whether the file
    // exists, so calling it for a missing library would still add the .a to the link line and
    // still emit WITH_<LIB>_BINDING=1 - producing a link error rather than a clean opt-out. This
    // checks first, and emits WITH_<LIB>_BINDING=0 when the library was never built.
    //
    // That define is what keeps the guarantee in
    // SIMVAL/PhysicsEngineDiscussion/PHYSICS_ENGINE_ANALYSIS.md section 7: a build without the
    // dependency is byte-identical to one from before the dependency existed.
    private bool LoadOptionalAirSimDependency(ReadOnlyTargetRules Target, string LibName, string LibFileName)
    {
        string LibrariesPath = Path.Combine(AirLibPath, "deps", LibName, "lib");
        string ExpectedLib = (Target.Platform == UnrealTargetPlatform.Win64)
            ? Path.Combine(LibrariesPath, "x64", "Release", LibFileName + ".lib")
            : Path.Combine(LibrariesPath, "lib" + LibFileName + ".a");

        if (!File.Exists(ExpectedLib))
        {
            PublicDefinitions.Add(string.Format("WITH_" + LibName.ToUpper() + "_BINDING=0"));
            System.Console.WriteLine("AirSim: optional dependency '" + LibName + "' not found at "
                + ExpectedLib + " - compiling without it.");
            return false;
        }
        return AddLibDependency(LibName, LibrariesPath, LibFileName, Target, true);
    }

    private bool AddLibDependency(string LibName, string LibPath, string LibFileName, ReadOnlyTargetRules Target, bool IsAddLibInclude)
    {
        string PlatformString = (Target.Platform == UnrealTargetPlatform.Win64 || Target.Platform == UnrealTargetPlatform.Mac) ? "x64" : "x86";
        string ConfigurationString = (Target.Configuration == UnrealTargetConfiguration.Debug) ? "Debug" : "Release";
        bool isLibrarySupported = false;


        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            isLibrarySupported = true;

            PublicAdditionalLibraries.Add(Path.Combine(LibPath, PlatformString, ConfigurationString, LibFileName + ".lib"));
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux || Target.Platform == UnrealTargetPlatform.Mac)
        {
            isLibrarySupported = true;
            PublicAdditionalLibraries.Add(Path.Combine(LibPath, "lib" + LibFileName + ".a"));
        }

        if (isLibrarySupported && IsAddLibInclude)
        {
            // Include path
            PublicIncludePaths.Add(Path.Combine(AirLibPath, "deps", LibName, "include"));
        }
        PublicDefinitions.Add(string.Format("WITH_" + LibName.ToUpper() + "_BINDING={0}", isLibrarySupported ? 1 : 0));

        return isLibrarySupported;
    }
}