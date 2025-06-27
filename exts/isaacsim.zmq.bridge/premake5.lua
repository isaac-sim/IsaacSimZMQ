-- Setup the basic extension information.
local ext = get_current_extension_info()
project_ext(ext)

-- --------------------------------------------------------------------------------------------------------------
-- Helper variable containing standard configuration information for projects containing OGN files.
local ogn = get_ogn_project_information(ext, "isaacsim/zmq/bridge")

-- --------------------------------------------------------------------------------------------------------------
-- Link folders that should be packaged with the extension.
repo_build.prebuild_link {
    { "data", ext.target_dir.."/data" },
    { "docs", ext.target_dir.."/docs" },
}

-- --------------------------------------------------------------------------------------------------------------
-- Copy the __init__.py to allow building of a non-linked ogn/ import directory.
-- In a mixed extension this would be part of a separate Python-based project but since here it is just the one
-- file it can be copied directly with no build dependencies.
repo_build.prebuild_copy {
    { "isaacsim/zmq/bridge/__init__.py", ogn.python_target_path }
}

-- --------------------------------------------------------------------------------------------------------------
-- Breaking this out as a separate project ensures the .ogn files are processed before their results are needed.
project_ext_ogn( ext, ogn )

-- --------------------------------------------------------------------------------------------------------------
-- Build the C++ plugin that will be loaded by the extension.
project_ext_plugin(ext, ogn.plugin_project)
    -- It is important that you add all subdirectories containing C++ code to this project
    add_files("source", "plugins/"..ogn.module)
    add_files("nodes", "plugins/nodes")

    -- same as BUILD_SHARED_LIBS=OFF in CMake, for static linking
    staticruntime "On"

    includedirs { "%{target_deps}/python/include/python3.10",
                  "%{target_deps}/usd/release/include",
                  "%{target_deps}/protobuf/include",
                  "%{target_deps}/zmq/include",
                  "%{target_deps}/zmq/cppzmq",
                  "%{target_deps}/abseil/include",
                }
    libdirs { "%{target_deps}/usd/release/lib",
              "%{target_deps}/protobuf/lib",
              "%{target_deps}/zmq/lib",
              "%{target_deps}/abseil/lib",
             }
    buildoptions { "-fexceptions" }

    -- Dynamic linking
    links {"zmq"}

    -- Static linking
    linkoptions {
        "-Wl,--allow-multiple-definition",
        "-Wl,--whole-archive",
        -- Abseil
        "%{target_deps}/abseil/lib/libabsl_*.a",
        -- Protobuf
        "%{target_deps}/protobuf/lib/lib*.a",
        "-Wl,--no-whole-archive"
    }

    -- Begin OpenUSD
    add_usd("arch", "gf", "sdf", "tf", "usd", "usdGeom", "usdUtils")
    -- End OpenUSD

    -- Add the standard dependencies all OGN projects have; includes, libraries to link, and required compiler flags
    add_ogn_dependencies(ogn)

    -- -- RPATH linking
    -- linkoptions {
    --     "-Wl,-rpath,\\$$ORIGIN/lib"
    -- }
    cppdialect "C++17"

    -- Additional build options for static linking of abseil
    defines { "ABSL_BUILD_DLL=0" }

    filter "action:gmake*"
        linkoptions { "-static-libgcc", "-static-libstdc++" }
    filter {}
