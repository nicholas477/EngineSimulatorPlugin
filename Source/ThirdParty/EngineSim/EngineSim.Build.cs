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

    // Set this to false if you've already built engine sim with cmake.
    // This runs cmake
    private bool BuildEngineSim
    {
        get { return false; }
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
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // If on windows, manually specify the path here
            return "C:/local/boost_1_70_0";
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            return "/usr/include";
        }

        throw new BuildException("Unsupported platform");
    }

    private string GetBoostLibPath(ReadOnlyTargetRules Target)
    {
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // If on windows, manually specify the path here
            return "C:/local/boost_1_70_0/lib64-msvc-14.2";
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            return "/usr/lib/x86_64-linux-gnu";

            // This might also be in:
            // /usr/lib
            // /usr/lib64
        }

        throw new BuildException("Unsupported platform");
    }

    private string GetSDL2IncludePath(ReadOnlyTargetRules Target)
    {
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // If on windows, manually specify the path here
            return "C:/local/SDL2/include";
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            return "/usr/include/SD2";
        }

        throw new BuildException("Unsupported platform");
    }

    private string GetSDL2LibPath(ReadOnlyTargetRules Target)
    {
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // If on windows, manually specify the path here
            return "C:/local/SDL2/lib/x64";
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            return "/usr/lib/x86_64-linux-gnu";

            // This might also be in:
            // /usr/lib
            // /usr/lib64
        }

        throw new BuildException("Unsupported platform");
    }

    private string GetSDL2ImageIncludePath(ReadOnlyTargetRules Target)
    {
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // If on windows, manually specify the path here
            return "C:/local/SDL2_image/include";
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            return GetSDL2IncludePath(Target);
        }

        throw new BuildException("Unsupported platform");
    }

    private string GetSDL2ImageLibPath(ReadOnlyTargetRules Target)
    {
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // If on windows, manually specify the path here
            return "C:/local/SDL2_image/lib/x64";
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            return "/usr/lib/x86_64-linux-gnu";

            // This might also be in:
            // /usr/lib
            // /usr/lib64
        }

        throw new BuildException("Unsupported platform");
    }

    // You do not need to set these if you are not building engine-sim with cmake
    private string FlexPath
    {
        get { return "C:/local/WinFlexBison"; }
    }

    private string BisonPath
    {
        get { return "C:/local/WinFlexBison"; }
    }

    private string ModulePath
	{
		get { return ModuleDirectory; }
	}

	public EngineSim(ReadOnlyTargetRules Target) : base(Target)
	{
        Type = ModuleType.External;

        const string buildType = "RelWithDebInfo";
        var buildDirectory = "build/" + buildType;
        var buildPath = Path.Combine(GetEngineSimPath(Target), buildDirectory);


        String EngineSimLibPath = Path.Combine(buildPath, buildType, "engine-sim.lib");
        String EngineSimScriptInterpreterLibPath = Path.Combine(buildPath, buildType, "engine-sim-script-interpreter.lib");
        String EngineSimAppLibPath = Path.Combine(buildPath, buildType, "engine-sim-app-lib.lib");
        String ConstraintResolverLibPath = Path.Combine(buildPath, "dependencies/submodules/simple-2d-constraint-solver", buildType, "simple-2d-constraint-solver.lib");
        String PiranhaLibPath = Path.Combine(buildPath, "dependencies/submodules/piranha", buildType, "piranha.lib");
        String DeltaStudioLibPath = Path.Combine(buildPath, "dependencies/submodules/delta-studio", buildType, "delta-core.lib");

        if (BuildEngineSim)
        {
            var configureCommand = CreateCMakeBuildCommand(Target, buildPath, buildType);
            var configureCode = ExecuteCommandSync(configureCommand);
            if (configureCode != 0)
            {
                String error = "Cannot configure CMake project. Exited with code: "
                    + configureCode;
                System.Console.WriteLine(error);
                throw new BuildException(error);
            }

            var installCommand = CreateCMakeInstallCommand(buildPath, buildType);
            var buildCode = ExecuteCommandSync(installCommand);
            if (buildCode != 0)
            {
                String error = "Cannot build project. Exited with code: " + buildCode;
                System.Console.WriteLine(error);
                throw new BuildException(error);
            }
        }

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
        LinkSDL2(Target);
    }
	
	private string GetGeneratorName(WindowsCompiler compiler)
    {
        string generatorName="";

        switch(compiler)
        {
        case WindowsCompiler.Default:
        break;
        case WindowsCompiler.Clang:
            generatorName="NMake Makefiles";
        break;
        case WindowsCompiler.Intel:
            generatorName="NMake Makefiles";
        break;
        case WindowsCompiler.VisualStudio2019:
            generatorName="Visual Studio 16 2019";
        break;
        case WindowsCompiler.VisualStudio2022:
            generatorName="Visual Studio 17 2022";
        break;
        }

        return generatorName;
    }
	
	private string CreateCMakeBuildCommand(ReadOnlyTargetRules target, string buildDirectory, string buildType)
	{
		const string program = "cmake.exe";
		var rootDirectory = ModulePath;
		var installPath = Path.Combine(rootDirectory, "../../../Intermediate/Build/Win64/", "engine-sim");
        var sourceDir = GetEngineSimPath(target);

        string generator = GetGeneratorName(target.WindowsPlatform.Compiler);

		var arguments = " -G \"" + generator + "\"" +
						" -S \"" + sourceDir + "\"" +
						" -B \"" + buildDirectory + "\"" +
                        " -A x64 " +
						" -T host=x64" +
                        " -Wno-dev " +
                        AddFlexDeps() +
                        AddBisonDeps() +
                        AddBoost(target) +
                        AddSDL(target) +
                        " -DDTV=OFF" +
                        " -DDISCORD_ENABLED=OFF" +
                        " -DPIRANHA_ENABLED=ON" +
                        " -DBUILD_APP_LIB=ON" +
                        " -DBUILD_APP=OFF" +
                        " -DCMAKE_BUILD_TYPE=" + buildType +
						" -DCMAKE_INSTALL_PREFIX=\"" + installPath + "\"";

		return program + arguments;
	}

    private string AddFlexDeps()
    {
        return " -DFLEX_EXECUTABLE=\"" + Path.Combine(FlexPath, "win_flex.exe") + "\"";
    }

    private string AddBisonDeps()
    {
        return " -DBISON_EXECUTABLE=\"" + Path.Combine(BisonPath, "win_bison.exe") + "\"";
    }

    private string AddBoost(ReadOnlyTargetRules target)
    {
        String outString = "";

        outString += " -DBoost_INCLUDE_DIR=\"" + GetBoostIncludePath(Target) + "\"";

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            string BoostToolsetVersion = "vc142";

            string BoostLibPath = GetBoostLibPath(target);

            outString += " -DBOOST_LIB_TOOLSET=\"" + BoostToolsetVersion + "\"";
            outString += " -DBOOST_ALL_NO_LIB=1";
            outString += " -DBOOST_LIBRARYDIR=\"" + BoostLibPath + "\"";
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            string BoostLibPath = GetBoostLibPath(target);

            outString += " -DBOOST_ALL_NO_LIB=1";
            outString += " -DBOOST_LIBRARYDIR=\"" + BoostLibPath + "\"";
        }

        return outString;
    }

    private string AddSDL(ReadOnlyTargetRules target)
    {
        String outString = "";

        outString += " -DSDL2_INCLUDE_DIR=\"" + GetSDL2IncludePath(target) + "\"";
        outString += " -DSDL2_IMAGE_INCLUDE_DIR=\"" + GetSDL2ImageIncludePath(target) + "\"";

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            string SDL2LibPath = Path.Combine(GetSDL2LibPath(target), "SDL2.lib");
            string SDL2MainLibPath = Path.Combine(GetSDL2LibPath(target), "SDL2main.lib");
            outString += " -DSDL2_LIBRARY=\"" + SDL2LibPath + "\"";
            outString += " -DSDL2MAIN_LIBRARY=\"" + SDL2MainLibPath + "\"";

            string SDL2ImageLibPath = Path.Combine(GetSDL2ImageLibPath(target), "SDL2_image.lib");
            outString += " -DSDL2_IMAGE_LIBRARY=\"" + SDL2ImageLibPath + "\"";

            return outString;
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            string SDL2LibPath = Path.Combine(GetSDL2LibPath(target), "libSDL2.a");
            string SDL2MainLibPath = Path.Combine(GetSDL2LibPath(target), "libSDL2main.a");
            outString += " -DSDL2_LIBRARY=\"" + SDL2LibPath + "\"";
            outString += " -DSDL2MAIN_LIBRARY=\"" + SDL2MainLibPath + "\"";

            string SDL2ImageLibPath = Path.Combine(GetSDL2ImageLibPath(target), "libSDL2_image.a");
            outString += " -DSDL2_IMAGE_LIBRARY=\"" + SDL2ImageLibPath + "\"";

            return outString;
        }

        throw new BuildException("Unsupported platform");
    }

    private void LinkBoost(ReadOnlyTargetRules target)
    {
        string[] BoostLibraries = { "filesystem" };

        string BoostVersion = GetBoostVersion(target);
        string BoostVersionDir = "boost-" + BoostVersion;

        string BoostIncludePath = GetBoostIncludePath(target);

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

    private void LinkSDL2(ReadOnlyTargetRules target)
    {
        if (target.Platform == UnrealTargetPlatform.Win64)
        {
            string SDL2LibPath = Path.Combine(GetSDL2LibPath(target), "SDL2.lib");
            string SDL2DLLPath = Path.Combine(GetSDL2LibPath(target), "SDL2main.dll");
            string SDL2MainLibPath = Path.Combine(GetSDL2LibPath(target), "SDL2main.lib");
            PublicAdditionalLibraries.Add(SDL2LibPath);
            PublicAdditionalLibraries.Add(SDL2MainLibPath);

            //PublicDelayLoadDLLs.Add(SDL2DLLPath);
            //RuntimeDependencies.Add(SDL2DLLPath);


            string SDL2ImageLibPath = Path.Combine(GetSDL2ImageLibPath(target), "SDL2_image.lib");
            string SDL2ImageDLLPath = Path.Combine(GetSDL2ImageLibPath(target), "SDL2_image.dll");
            PublicAdditionalLibraries.Add(SDL2ImageLibPath);
            //RuntimeDependencies.Add(SDL2ImageDLLPath);
        }
        else if (target.Platform == UnrealTargetPlatform.Linux)
        {
            string SDL2LibPath = Path.Combine(GetSDL2LibPath(target), "libSDL2.a");
            string SDL2DLLPath = Path.Combine(GetSDL2LibPath(target), "libSDL2main.so");
            string SDL2MainLibPath = Path.Combine(GetSDL2LibPath(target), "libSDL2main.a");
            PublicAdditionalLibraries.Add(SDL2LibPath);
            PublicAdditionalLibraries.Add(SDL2MainLibPath);

            //PublicDelayLoadDLLs.Add(SDL2DLLPath);
            //RuntimeDependencies.Add(SDL2DLLPath);


            string SDL2ImageLibPath = Path.Combine(GetSDL2ImageLibPath(target), "libSDL2_image.a");
            string SDL2ImageDLLPath = Path.Combine(GetSDL2ImageLibPath(target), "libSDL2_image.so");
            PublicAdditionalLibraries.Add(SDL2ImageLibPath);
            //RuntimeDependencies.Add(SDL2ImageDLLPath);
        }
    }

    private string CreateCMakeInstallCommand(string buildDirectory, string buildType)
    {
        return "cmake.exe --build \"" + buildDirectory + "\"" +
               " --target install --config " + buildType;
    }
	
	private int ExecuteCommandSync(string command)
    {
        System.Console.WriteLine("Running: "+command);
        var processInfo = new ProcessStartInfo("cmd.exe", "/c "+command)
        {
            CreateNoWindow=true,
            UseShellExecute=false,
            RedirectStandardError=true,
            RedirectStandardOutput=true,
            WorkingDirectory=ModulePath
		};

        StringBuilder outputString = new StringBuilder();
        Process p = Process.Start(processInfo);

        p.OutputDataReceived+=(sender, args) => {outputString.Append(args.Data); System.Console.WriteLine(args.Data);};
        p.ErrorDataReceived+=(sender, args) => {outputString.Append(args.Data); System.Console.WriteLine(args.Data);};
        p.BeginOutputReadLine();
        p.BeginErrorReadLine();
        p.WaitForExit();

        if(p.ExitCode != 0)
        {
             Console.WriteLine(outputString);
        }
        return p.ExitCode;
    }
}
