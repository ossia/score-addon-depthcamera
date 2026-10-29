# depthai's own x64-windows overlay triplet, under a name of ours and for both
# Windows architectures: static libraries against the dynamic CRT, so they all
# end up inside score_depthcam_depthai.dll. depthai ships no arm64-windows
# overlay, and vcpkg's stock arm64-windows builds every dependency as a DLL.
#
# libusb stays dynamic, as in all of depthai's triplets: it is LGPL-2.1, and a
# separate DLL keeps it replaceable.
set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)

if(PORT MATCHES "libusb|ffmpeg")
    set(VCPKG_LIBRARY_LINKAGE dynamic)
endif()
