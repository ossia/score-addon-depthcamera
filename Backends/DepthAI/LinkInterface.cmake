### Injected into depthai-core's own configure (CMAKE_PROJECT_depthai_INCLUDE)
### to write down what linking its static library takes.
###
### depthai-core is built static so that it ends up inside the backend module:
### as a DLL it has no export macros, and CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS on
### a library that size overflows the PE export table (LNK1189). A static
### archive, though, brings its whole dependency closure along -- spdlog, fmt,
### yaml-cpp, libarchive, lz4, XLink, the CMakeRC firmware library, a dozen
### more out of vcpkg -- and the package config depthai installs does not
### find those again (it only looks for nlohmann_json, libnop and XLink).
### Nor could the outer project use it: the ExternalProject runs at build
### time, after the outer configure.
###
### So the closure is resolved here, inside the one configure that has every
### one of those targets defined, and written out as a linker response file.
### Deferred to the end of the top-level directory, by which point
### depthai-core's link interface is complete.
###
### Outputs, in SCORE_DAI_LINK_DIR:
###   depthai-link.rsp     ELF: arguments for ld (passed as -Wl,@file, after
###                        the objects). Mach-O: arguments for the compiler
###                        driver. MSVC: arguments for link.exe.
###   depthai-link.cpp     MSVC only: the same libraries as #pragma comment(lib),
###                        compiled into the module. A response file passed to
###                        link.exe would have to survive Ninja's own one.

if(NOT SCORE_DAI_LINK_DIR)
  message(FATAL_ERROR "LinkInterface.cmake: SCORE_DAI_LINK_DIR is not set")
endif()

function(_score_dai_strip_genex item out)
  # Wrappers that only say *when* an item applies to a consumer. A static
  # library's private dependencies come out as $<LINK_ONLY:...>.
  set(_s "${item}")
  foreach(_w LINK_ONLY BUILD_INTERFACE BUILD_LOCAL_INTERFACE)
    if(_s MATCHES "^\\$<${_w}:(.*)>$")
      set(_s "${CMAKE_MATCH_1}")
    endif()
  endforeach()
  if(_s MATCHES "^\\$<INSTALL_INTERFACE:")
    set(_s "")
  endif()
  set(${out} "${_s}" PARENT_SCOPE)
endfunction()

function(_score_dai_walk item)
  _score_dai_strip_genex("${item}" item)
  if(item STREQUAL "")
    return()
  endif()

  if(TARGET "${item}")
    get_target_property(_aliased "${item}" ALIASED_TARGET)
    if(_aliased)
      set(item "${_aliased}")
    endif()

    get_property(_seen GLOBAL PROPERTY _SCORE_DAI_SEEN)
    if("${item}" IN_LIST _seen)
      return()
    endif()
    set_property(GLOBAL APPEND PROPERTY _SCORE_DAI_SEEN "${item}")

    # Threads::Threads is -pthread or nothing; the backend links it itself.
    if(item STREQUAL "Threads::Threads")
      return()
    endif()

    get_target_property(_type "${item}" TYPE)
    if(_type STREQUAL "STATIC_LIBRARY" OR _type STREQUAL "UNKNOWN_LIBRARY")
      set_property(GLOBAL APPEND PROPERTY _SCORE_DAI_FILES "$<TARGET_FILE:${item}>")
    elseif(_type STREQUAL "SHARED_LIBRARY")
      set_property(GLOBAL APPEND PROPERTY _SCORE_DAI_FILES "$<TARGET_LINKER_FILE:${item}>")
    elseif(_type STREQUAL "OBJECT_LIBRARY")
      set_property(GLOBAL APPEND PROPERTY _SCORE_DAI_FILES "$<TARGET_OBJECTS:${item}>")
    endif()

    set(_deps "")
    get_target_property(_ill "${item}" INTERFACE_LINK_LIBRARIES)
    if(_ill)
      list(APPEND _deps ${_ill})
    endif()
    get_target_property(_imported "${item}" IMPORTED)
    if(_imported)
      # Old-style imported targets (some Find modules) list theirs per config.
      string(TOUPPER "${CMAKE_BUILD_TYPE}" _cfg)
      foreach(_p IMPORTED_LINK_INTERFACE_LIBRARIES
                 IMPORTED_LINK_INTERFACE_LIBRARIES_${_cfg}
                 IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE)
        get_target_property(_v "${item}" ${_p})
        if(_v)
          list(APPEND _deps ${_v})
          break()
        endif()
      endforeach()
    endif()
    foreach(_d IN LISTS _deps)
      _score_dai_walk("${_d}")
    endforeach()
    return()
  endif()

  # Not a target: a path, a flag or a bare system library name.
  if(item MATCHES "^\\$<")
    # Config-conditional paths, as vcpkg's wrappers write them
    # ($<$<CONFIG:DEBUG>:...>). Left to file(GENERATE) to evaluate; see
    # _score_dai_quote for the ones that evaluate to nothing.
    set_property(GLOBAL APPEND PROPERTY _SCORE_DAI_FILES "${item}")
  elseif(IS_ABSOLUTE "${item}")
    set_property(GLOBAL APPEND PROPERTY _SCORE_DAI_FILES "${item}")
  elseif(item MATCHES "^-")
    set_property(GLOBAL APPEND PROPERTY _SCORE_DAI_FLAGS "${item}")
  else()
    set_property(GLOBAL APPEND PROPERTY _SCORE_DAI_SYSLIBS "${item}")
  endif()
endfunction()

# One line of output for a path, or nothing at all when the path is a
# generator expression that comes out empty for this configuration -- an
# empty quoted argument is a file named "" to a linker.
function(_score_dai_line path format out)
  string(REPLACE "@" "${path}" _line "${format}")
  if(path MATCHES "^\\$<")
    set(_line "$<$<BOOL:${path}>:${_line}>")
  endif()
  set(${out} "${_line}\n" PARENT_SCOPE)
endfunction()

function(_score_dai_write_link_interface)
  if(NOT TARGET depthai-core)
    message(FATAL_ERROR "LinkInterface.cmake: no depthai-core target")
  endif()
  get_target_property(_type depthai-core TYPE)
  if(NOT _type STREQUAL "STATIC_LIBRARY")
    message(FATAL_ERROR "LinkInterface.cmake: depthai-core is a ${_type}, expected a static library")
  endif()

  set_property(GLOBAL PROPERTY _SCORE_DAI_SEEN "")
  set_property(GLOBAL PROPERTY _SCORE_DAI_FILES "")
  set_property(GLOBAL PROPERTY _SCORE_DAI_FLAGS "")
  set_property(GLOBAL PROPERTY _SCORE_DAI_SYSLIBS "")
  _score_dai_walk(depthai-core)
  get_property(_files GLOBAL PROPERTY _SCORE_DAI_FILES)
  get_property(_flags GLOBAL PROPERTY _SCORE_DAI_FLAGS)
  get_property(_syslibs GLOBAL PROPERTY _SCORE_DAI_SYSLIBS)
  list(REMOVE_DUPLICATES _flags)
  list(REMOVE_DUPLICATES _syslibs)

  set(_rsp "")
  set(_cpp "")
  if(MSVC)
    foreach(_f IN LISTS _files)
      _score_dai_line("${_f}" "\"@\"" _l1)
      _score_dai_line("${_f}" "#pragma comment(lib, \"@\")" _l2)
      string(APPEND _rsp "${_l1}")
      string(APPEND _cpp "${_l2}")
    endforeach()
    foreach(_l IN LISTS _syslibs)
      if(NOT _l MATCHES "\\.lib$")
        set(_l "${_l}.lib")
      endif()
      string(APPEND _rsp "${_l}\n")
      string(APPEND _cpp "#pragma comment(lib, \"${_l}\")\n")
    endforeach()
    foreach(_f IN LISTS _flags)
      message(WARNING "LinkInterface.cmake: dropping link flag ${_f} (MSVC)")
    endforeach()
  elseif(APPLE)
    # ld64 resolves against every archive whatever the order; a driver-level
    # response file, so -framework and -l mean what they say.
    foreach(_f IN LISTS _files)
      if(_f MATCHES "/([^/]+)\\.framework$")
        string(APPEND _rsp "-framework ${CMAKE_MATCH_1}\n")
      else()
        _score_dai_line("${_f}" "\"@\"" _l1)
        string(APPEND _rsp "${_l1}")
      endif()
    endforeach()
    foreach(_f IN LISTS _flags)
      string(APPEND _rsp "${_f}\n")
    endforeach()
    foreach(_l IN LISTS _syslibs)
      string(APPEND _rsp "-l${_l}\n")
    endforeach()
  else()
    # ld-level (the backend passes -Wl,@file), and grouped: the walk is
    # depth-first with duplicates removed, so an archive two others need can
    # come before the second of them.
    string(APPEND _rsp "--start-group\n")
    foreach(_f IN LISTS _files)
      _score_dai_line("${_f}" "\"@\"" _l1)
      string(APPEND _rsp "${_l1}")
    endforeach()
    string(APPEND _rsp "--end-group\n")
    foreach(_f IN LISTS _flags)
      if(_f STREQUAL "-pthread")
        continue()
      elseif(_f MATCHES "^-Wl,(.*)")
        string(REPLACE "," ";" _parts "${CMAKE_MATCH_1}")
        foreach(_p IN LISTS _parts)
          string(APPEND _rsp "${_p}\n")
        endforeach()
      else()
        string(APPEND _rsp "${_f}\n")
      endif()
    endforeach()
    foreach(_l IN LISTS _syslibs)
      string(APPEND _rsp "-l${_l}\n")
    endforeach()
  endif()

  file(GENERATE OUTPUT "${SCORE_DAI_LINK_DIR}/depthai-link.rsp" CONTENT "${_rsp}")
  file(GENERATE OUTPUT "${SCORE_DAI_LINK_DIR}/depthai-link.cpp" CONTENT
    "// Generated by Backends/DepthAI/LinkInterface.cmake.\n${_cpp}")
  message(STATUS "score-addon-orbbec: depthai link interface -> ${SCORE_DAI_LINK_DIR}/depthai-link.rsp")
endfunction()

cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}" CALL _score_dai_write_link_interface)
