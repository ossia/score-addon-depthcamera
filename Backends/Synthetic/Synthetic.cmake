### The synthetic camera: no SDK, so it builds wherever the plug-in does.
###
### Three targets from one implementation:
###   score_depthcam_synthetic_impl   static, linked straight into unit tests
###   score_depthcam_synthetic        the backend, at this tree's ABI
###   score_depthcam_synthetic_abi3   the same camera presenting an ABI 3 table,
###                                   standing in for a package built before 4
###
### The modules land in SCORE_DEPTHCAM_TEST_BACKEND_DIR, never in the package
### directory: a camera that does not exist has no place in what users
### download. Tests point SCORE_DEPTHCAM_BACKEND_PATH at it.
include_guard(GLOBAL)

set(_syn_dir "${CMAKE_CURRENT_LIST_DIR}")
get_filename_component(_syn_root "${_syn_dir}/../.." ABSOLUTE)

if(NOT SCORE_DEPTHCAM_TEST_BACKEND_DIR)
  set(SCORE_DEPTHCAM_TEST_BACKEND_DIR "${CMAKE_BINARY_DIR}/depthcam-test-backends"
      CACHE INTERNAL "")
endif()

function(_score_depthcam_synthetic_common tgt)
  target_include_directories(${tgt} PRIVATE "${_syn_root}" "${_syn_dir}")
  target_compile_features(${tgt} PRIVATE cxx_std_20)
  set_target_properties(${tgt} PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
    UNITY_BUILD OFF)
  find_package(Threads)
  if(TARGET Threads::Threads)
    target_link_libraries(${tgt} PRIVATE Threads::Threads)
  endif()
endfunction()

add_library(score_depthcam_synthetic_impl STATIC
  "${_syn_dir}/SyntheticBackend.cpp"
  "${_syn_dir}/SyntheticBackend.hpp"
  "${_syn_dir}/SyntheticScene.hpp")
_score_depthcam_synthetic_common(score_depthcam_synthetic_impl)
target_include_directories(score_depthcam_synthetic_impl PUBLIC "${_syn_root}" "${_syn_dir}")

foreach(_abi IN ITEMS current 3)
  if(_abi STREQUAL "current")
    set(_tgt score_depthcam_synthetic)
  else()
    set(_tgt score_depthcam_synthetic_abi${_abi})
  endif()
  add_library(${_tgt} MODULE
    "${_syn_dir}/SyntheticBackend.cpp"
    "${_syn_dir}/SyntheticEntry.cpp")
  _score_depthcam_synthetic_common(${_tgt})
  if(NOT _abi STREQUAL "current")
    target_compile_definitions(${_tgt} PRIVATE SYNTHETIC_ABI_VERSION=${_abi})
  endif()
  # The registry looks for score_depthcam_*.dylib on macOS, where a MODULE
  # otherwise ends in .so.
  if(APPLE)
    set_target_properties(${_tgt} PROPERTIES SUFFIX ".dylib")
  endif()
  set_target_properties(${_tgt} PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${SCORE_DEPTHCAM_TEST_BACKEND_DIR}"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${SCORE_DEPTHCAM_TEST_BACKEND_DIR}"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${SCORE_DEPTHCAM_TEST_BACKEND_DIR}")
  # One exported symbol, like every backend.
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(${_tgt} PRIVATE
      "LINKER:--version-script=${_syn_root}/Backends/backend.version")
  elseif(APPLE)
    target_link_options(${_tgt} PRIVATE
      "LINKER:-exported_symbols_list,${_syn_root}/Backends/backend.symbols")
  endif()
endforeach()
