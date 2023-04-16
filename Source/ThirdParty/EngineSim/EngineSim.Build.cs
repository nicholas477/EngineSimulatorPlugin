// Fill out your copyright notice in the Description page of Project Settings.

using System;
using System.IO;
using System.Diagnostics;
using System.Text;
using UnrealBuildTool;
using System.Linq;

public class EngineSim : ModuleRules
{
    // Set up engine sim path here
    private string GetEngineSimPath(ReadOnlyTargetRules Target)
    {
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            return Path.Combine(ModulePath, "engine-sim");
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            return "C:/local/engine-sim-linux";
        }

        throw new BuildException("Unsupported platform");
    }

    private string GetBoostVersion(ReadOnlyTargetRules Target)
    {
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // If on windows, manually specify the version here
            return "1_70_0";
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            String boostVersionHeader = Path.Combine(GetBoostIncludePath(Target), "boost/version.hpp");
            if (File.Exists(boostVersionHeader))
            {
                string[] lines = System.IO.File.ReadAllLines(boostVersionHeader);
                foreach (var line in lines)
                {
                    if (line.StartsWith("#define BOOST_VERSION "))
                    {
                        int VersionNumber = int.Parse(line.Split(" ").Last());
                        int PatchLevel = VersionNumber % 100;
                        int MinorVersion = (VersionNumber / 100) % 1000;
                        int MajorVersion = VersionNumber / 100000;
                        return String.Concat(MajorVersion, "_", MinorVersion, "_", PatchLevel);
                    }
                }
            }

            throw new BuildException("Unable to find boost");
        }

        throw new BuildException("Unsupported platform");
    }

    private string GetBoostIncludePath(ReadOnlyTargetRules Target)
    {
        return Path.Combine(GetEngineSimPath(Target), "include/");
    }

    private string GetBoostLibPath(ReadOnlyTargetRules Target)
    {
        return Path.Combine(GetEngineSimPath(Target), "libs/");
    }

    private string ModulePath
    {
        get { return ModuleDirectory; }
    }

    public EngineSim(ReadOnlyTargetRules Target) : base(Target)
    {
        Type = ModuleType.External;

        var libPath = Path.Combine(GetEngineSimPath(Target), "libs/");

        String EngineSimLibPath = Path.Combine(libPath, "engine-sim.lib");
        String EngineSimScriptInterpreterLibPath = Path.Combine(libPath, "engine-sim-script-interpreter.lib");
        String EngineSimAppLibPath = Path.Combine(libPath, "engine-sim-app-lib.lib");
        String ConstraintResolverLibPath = Path.Combine(libPath, "simple-2d-constraint-solver.lib");
        String PiranhaLibPath = Path.Combine(libPath, "piranha.lib");
        String DeltaStudioLibPath = Path.Combine(libPath, "delta-core.lib");

        PublicDefinitions.Add("ATG_ENGINE_SIM_PIRANHA_ENABLED=1");

        if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            PublicDefinitions.Add("USE_CXX17_FILESYSTEM=1");
        }
        bEnableExceptions = true;

        PublicIncludePaths.Add(Path.Combine(GetEngineSimPath(Target), "include"));
        PublicIncludePaths.Add(Path.Combine(GetEngineSimPath(Target), "scripting/include"));
        PublicIncludePaths.Add(Path.Combine(GetEngineSimPath(Target), "dependencies/submodules"));
        PublicAdditionalLibraries.Add(EngineSimLibPath);
        PublicAdditionalLibraries.Add(EngineSimScriptInterpreterLibPath);
        PublicAdditionalLibraries.Add(EngineSimAppLibPath);
        PublicAdditionalLibraries.Add(ConstraintResolverLibPath);
        PublicAdditionalLibraries.Add(PiranhaLibPath);
        PublicAdditionalLibraries.Add(DeltaStudioLibPath);

        LinkBoost(Target);
    }

    private void LinkBoost(ReadOnlyTargetRules target)
    {
        string[] BoostLibraries = { "filesystem" };

        string BoostVersion = GetBoostVersion(target);
        string BoostVersionDir = "boost-" + BoostVersion;

        if (target.Platform == UnrealTargetPlatform.Win64)
        {
            string BoostToolsetVersion = "vc142";

            string BoostLibPath = GetBoostLibPath(target);
            string BoostVersionShort = BoostVersion.Substring(BoostVersion.Length - 2) == "_0" ? BoostVersion.Substring(0, BoostVersion.Length - 2) : BoostVersion;

            foreach (string BoostLib in BoostLibraries)
            {
                string BoostLibName = "libboost_" + BoostLib + "-" + BoostToolsetVersion + "-mt-x64" + "-" + BoostVersionShort;
                PublicAdditionalLibraries.Add(Path.Combine(BoostLibPath, BoostLibName + ".lib"));
            }
        }
        else if (target.Platform == UnrealTargetPlatform.Linux)
        {
            string BoostLibPath = GetBoostLibPath(target);
            foreach (string BoostLib in BoostLibraries)
            {
                string BoostLibName = "libboost_" + BoostLib;
                PublicAdditionalLibraries.Add(Path.Combine(BoostLibPath, BoostLibName + ".a"));
            }
        }
    }
}
