# Answers libfreenect2's FIND_PACKAGE(TurboJPEG) with the copy the OrbbecSDK
# vendors -- libjpeg-turbo, turbojpeg.h and all, already built here as
# turbojpeg_static.
#
# Same reasoning as FindLibUSB.cmake: off Linux there is no system libjpeg-turbo
# to link, and Homebrew's leaves an absolute path in the shipped binary. On
# Windows libfreenect2 requires it outright -- it has no VideoToolbox to fall
# back on -- so without this the Kinect v2 backend cannot be built there at all.

set(TurboJPEG_INCLUDE_DIRS "${SCORE_DEPTHCAM_TURBOJPEG_INCLUDE}")
set(TurboJPEG_LIBRARIES turbojpeg_static)
set(TurboJPEG_FOUND TRUE)
