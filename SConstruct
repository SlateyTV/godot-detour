#!/usr/bin/env python
import os, subprocess
import sys

env = SConscript("godot-cpp/SConstruct")

# For the reference:
# - CCFLAGS are compilation flags shared between C and C++
# - CFLAGS are for C-specific compilation flags
# - CXXFLAGS are for C++-specific compilation flags
# - CPPFLAGS are for pre-processor flags
# - CPPDEFINES are for pre-processor defines
# - LINKFLAGS are for linking flags

# tweak this if you want to use different folders, or more folders, to store your source code in.
env.Append(CPPPATH=['src/', 'src/util', 'recastnavigation/DebugUtils/Include/', 'recastnavigation/Detour/Include/', 'recastnavigation/DetourCrowd/Include/', 'recastnavigation/DetourTileCache/Include/', 'recastnavigation/Recast/Include/'])
sources = Glob('src/*.cpp')
sources += Glob('src/util/*.cpp')
sources += Glob('src/util/*.c')
sources += Glob('recastnavigation/DebugUtils/Source/*.cpp')
sources += Glob('recastnavigation/Detour/Source/*.cpp')
sources += Glob('recastnavigation/DetourCrowd/Source/*.cpp')
sources += Glob('recastnavigation/DetourTileCache/Source/*.cpp')
sources += Glob('recastnavigation/Recast/Source/*.cpp')


if env["platform"] == "macos":
    library = env.SharedLibrary(
        "demo/bin/godot-detour.{}.{}.framework/slatey-terrain.{}.{}".format(
            env["platform"], env["target"], env["platform"], env["target"]
        ),
        source=sources,
    )
else:
    library = env.SharedLibrary(
        "demo/bin/godot-detour{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
        source=sources,
    )

Default(library)
