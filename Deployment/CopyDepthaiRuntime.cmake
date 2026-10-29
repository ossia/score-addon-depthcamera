### Copies the shared libraries depthai-core installs next to the backend.
###
### depthai-core itself is static and inside the backend; what is left is what
### its vcpkg triplets keep dynamic -- libusb today. Globbed rather than named:
### that depends on how depthai was configured, and naming them would silently
### ship a package missing one. Run from POST_BUILD, since the install tree does not
### exist at configure time.
###
###   cmake -DIN=<depthai install lib dirs> -DOUT=<package dir> -P CopyDepthaiRuntime.cmake

if(NOT IN OR NOT OUT)
  message(FATAL_ERROR "CopyDepthaiRuntime: IN and OUT are both required")
endif()

### IN is a list: on Windows depthai installs RUNTIME to bin/ and LIBRARY to
### lib/, so both are swept and a directory absent on this platform is skipped.
set(_libs "")
foreach(dir ${IN})
  if(NOT IS_DIRECTORY "${dir}")
    continue()
  endif()
  file(GLOB _found
    "${dir}/*.so" "${dir}/*.so.*"
    "${dir}/*.dylib"
    "${dir}/*.dll")
  # A libdepthai-core here is left over from a shared build in the same tree;
  # nothing loads it any more.
  list(FILTER _found EXCLUDE REGEX "depthai-core[^/]*$")
  list(APPEND _libs ${_found})
endforeach()

if(NOT _libs)
  message(FATAL_ERROR
    "CopyDepthaiRuntime: no shared library found in ${IN}. The depthai build "
    "produced nothing to ship, which would make the backend unloadable.")
endif()

file(MAKE_DIRECTORY "${OUT}")
foreach(lib ${_libs})
  get_filename_component(_name "${lib}" NAME)
  # Symlinks are copied as symlinks so a versioned soname chain survives.
  file(COPY "${lib}" DESTINATION "${OUT}")
  message(STATUS "depthai runtime: ${_name}")
endforeach()
