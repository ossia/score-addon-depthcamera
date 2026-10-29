### Tests of the plug-in against the synthetic camera. No camera, no SDK.
###
### The unit test links the synthetic implementation in-process; the ABI
### loading test also dlopens the synthetic modules from
### SCORE_DEPTHCAM_TEST_BACKEND_DIR, including the one presenting an ABI 3
### table. The gfx test renders through score's own offscreen harness and is
### labelled "gpu" like every test that touches a GPU.
set(_dc_root "${CMAKE_CURRENT_SOURCE_DIR}/..")

# The plug-in's classes are hidden in a dynamic-plugin build; the test compiles
# the ones it exercises itself (see score_plugin_hidden_sources).
score_plugin_hidden_sources(_dc_hidden
  "${_dc_root}/DepthCamera/Backend/BackendRegistry.cpp"
  "${_dc_root}/DepthCamera/DepthCameraCalibration.cpp"
  "${_dc_root}/DepthCamera/DepthCameraControls.cpp"
  "${_dc_root}/DepthCamera/DepthCameraGpu.cpp"
  "${_dc_root}/DepthCamera/DepthCameraInfo.cpp"
  "${_dc_root}/DepthCamera/DepthCameraSettings.cpp"
  "${_dc_root}/DepthCamera/DepthCameraStream.cpp")

score_add_test(score_addon_orbbec_unit_test
  SOURCES unit/DepthCameraUnit.cpp ${_dc_hidden}
  PLUGINS score_addon_orbbec
  LIBS score_depthcam_synthetic_impl)
target_include_directories(score_addon_orbbec_unit_test PRIVATE
  "${_dc_root}" "${_dc_root}/DepthCamera/Backend" "${_dc_root}/Backends/Synthetic"
  $<TARGET_PROPERTY:score_addon_orbbec,INCLUDE_DIRECTORIES>)
target_compile_definitions(score_addon_orbbec_unit_test PRIVATE
  SCORE_DEPTHCAM_TEST_BACKEND_DIR="${SCORE_DEPTHCAM_TEST_BACKEND_DIR}")
add_dependencies(score_addon_orbbec_unit_test
  score_depthcam_synthetic score_depthcam_synthetic_abi3)
if(FFMPEG_INCLUDE_DIRS)
  target_include_directories(score_addon_orbbec_unit_test SYSTEM PRIVATE ${FFMPEG_INCLUDE_DIRS})
endif()
set_tests_properties(score_addon_orbbec_unit_test PROPERTIES LABELS "depthcam")

score_plugin_hidden_sources(_dc_settings_hidden
  "${_dc_root}/DepthCamera/DepthCameraSettings.cpp")
score_add_test(score_addon_orbbec_settings_test
  SOURCES unit/DepthCameraSettingsTest.cpp ${_dc_settings_hidden}
  APP
  PLUGINS score_addon_orbbec)
target_include_directories(score_addon_orbbec_settings_test PRIVATE "${_dc_root}"
  "${SCORE_ROOT_SOURCE_DIR}/tests/fixtures")
set_tests_properties(score_addon_orbbec_settings_test PROPERTIES LABELS "depthcam")

# Renders through score's offscreen harness on every backend the machine has.
score_add_test(score_addon_orbbec_gfx_test
  SOURCES gfx/DepthCameraGfx.cpp ${_dc_hidden}
  GUI
  PLUGINS score_addon_orbbec score_plugin_gfx score_plugin_scenario score_lib_process
  LIBS score_depthcam_synthetic_impl test_gfx_engine_glue)
target_include_directories(score_addon_orbbec_gfx_test PRIVATE
  "${_dc_root}" "${_dc_root}/DepthCamera/Backend" "${_dc_root}/Backends/Synthetic"
  "${SCORE_ROOT_SOURCE_DIR}/tests/fixtures"
  $<TARGET_PROPERTY:score_addon_orbbec,INCLUDE_DIRECTORIES>)
if(FFMPEG_INCLUDE_DIRS)
  target_include_directories(score_addon_orbbec_gfx_test SYSTEM PRIVATE ${FFMPEG_INCLUDE_DIRS})
endif()
set_property(TEST score_addon_orbbec_gfx_test APPEND PROPERTY LABELS depthcam gpu)
