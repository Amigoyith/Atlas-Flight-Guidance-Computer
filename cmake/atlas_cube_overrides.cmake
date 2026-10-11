# Atlas replacements for CubeMX-generated files that have no USER CODE hooks.
#
# Every file in App/CubeOverrides/ mirrors the repository path of the generated
# file it replaces (for example App/CubeOverrides/FATFS/Target/sd_diskio.c
# replaces FATFS/Target/sd_diskio.c). CubeMX may regenerate the originals at any
# time; the build always compiles the Atlas copy instead. See docs/CUBEMX.md.
#
# Include after add_subdirectory(cmake/stm32cubemx).

set(ATLAS_CUBE_OVERRIDES_DIR ${CMAKE_SOURCE_DIR}/App/CubeOverrides)
file(GLOB_RECURSE atlas_override_sources CONFIGURE_DEPENDS ${ATLAS_CUBE_OVERRIDES_DIR}/*.c)
file(GLOB_RECURSE atlas_override_headers CONFIGURE_DEPENDS ${ATLAS_CUBE_OVERRIDES_DIR}/*.h)
set(atlas_unused_overrides ${atlas_override_sources})

# Swap generated sources for their overrides in every CubeMX-built target.
foreach(atlas_target ${CMAKE_PROJECT_NAME} USB_Device_Library FatFs)
    get_target_property(atlas_sources ${atlas_target} SOURCES)
    get_target_property(atlas_source_dir ${atlas_target} SOURCE_DIR)
    set(atlas_replaced_sources)
    foreach(atlas_source IN LISTS atlas_sources)
        cmake_path(ABSOLUTE_PATH atlas_source BASE_DIRECTORY ${atlas_source_dir}
            NORMALIZE OUTPUT_VARIABLE atlas_absolute)
        cmake_path(RELATIVE_PATH atlas_absolute BASE_DIRECTORY ${CMAKE_SOURCE_DIR}
            OUTPUT_VARIABLE atlas_relative)
        set(atlas_override ${ATLAS_CUBE_OVERRIDES_DIR}/${atlas_relative})
        if(EXISTS ${atlas_override})
            list(APPEND atlas_replaced_sources ${atlas_override})
            list(REMOVE_ITEM atlas_unused_overrides ${atlas_override})
        else()
            list(APPEND atlas_replaced_sources ${atlas_source})
        endif()
    endforeach()
    set_property(TARGET ${atlas_target} PROPERTY SOURCES ${atlas_replaced_sources})
endforeach()

# An override that replaces nothing means CubeMX renamed or dropped the file.
if(atlas_unused_overrides)
    message(FATAL_ERROR "App/CubeOverrides sources match no CubeMX-generated source:\n"
        "  ${atlas_unused_overrides}\nUpdate or remove them (docs/CUBEMX.md).")
endif()

# Override headers must be found before the generated ones of the same name.
# A quoted #include searches the including file's own folder first, so a
# generated header would still pick up its stock neighbour. Overriding any
# header therefore requires mirroring every header of that generated folder.
set(atlas_override_include_dirs)
foreach(atlas_header IN LISTS atlas_override_headers)
    cmake_path(RELATIVE_PATH atlas_header BASE_DIRECTORY ${ATLAS_CUBE_OVERRIDES_DIR}
        OUTPUT_VARIABLE atlas_relative)
    if(NOT EXISTS ${CMAKE_SOURCE_DIR}/${atlas_relative})
        message(FATAL_ERROR "App/CubeOverrides/${atlas_relative} matches no CubeMX-generated "
            "header. Update or remove it (docs/CUBEMX.md).")
    endif()
    cmake_path(GET atlas_header PARENT_PATH atlas_header_dir)
    list(APPEND atlas_override_include_dirs ${atlas_header_dir})
endforeach()
list(REMOVE_DUPLICATES atlas_override_include_dirs)
foreach(atlas_header_dir IN LISTS atlas_override_include_dirs)
    cmake_path(RELATIVE_PATH atlas_header_dir BASE_DIRECTORY ${ATLAS_CUBE_OVERRIDES_DIR}
        OUTPUT_VARIABLE atlas_relative)
    file(GLOB atlas_generated_headers RELATIVE ${CMAKE_SOURCE_DIR}/${atlas_relative}
        ${CMAKE_SOURCE_DIR}/${atlas_relative}/*.h)
    foreach(atlas_generated_header IN LISTS atlas_generated_headers)
        if(NOT EXISTS ${atlas_header_dir}/${atlas_generated_header})
            message(FATAL_ERROR "App/CubeOverrides/${atlas_relative} must mirror every header "
                "of ${atlas_relative}; missing ${atlas_generated_header} (docs/CUBEMX.md).")
        endif()
    endforeach()
endforeach()
target_include_directories(stm32cubemx BEFORE INTERFACE ${atlas_override_include_dirs})
