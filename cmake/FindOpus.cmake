# - Find opus
# Find the native opus includes and libraries
#
# This is just FindOgg find and replaced with opus
#
#  OPUS_INCLUDE_DIRS - where to find opus.h, etc.
#  OPUS_LIBRARIES    - List of libraries when using opus.
#  OPUS_FOUND        - True if opus found.

if(OPUS_INCLUDE_DIR)
    # Already in cache, be silent
    set(OPUS_FIND_QUIETLY TRUE)
endif(OPUS_INCLUDE_DIR)

find_package (PkgConfig QUIET)
pkg_check_modules(PC_OPUS QUIET opus)

find_path(OPUS_INCLUDE_DIR opus.h HINTS ${PC_OPUS_INCLUDEDIR} ${PC_OPUS_INCLUDE_DIRS} ${OPUS_ROOT} PATH_SUFFIXES include)
# MSVC built opus may be named opus_static.
# The provided project files name the library with the lib prefix.
find_library(OPUS_LIBRARY NAMES opus opus_static libopus libopus_static HINTS ${PC_OPUS_LIBDIR} ${PC_OPUS_LIBRARY_DIRS} ${OPUS_ROOT} PATH_SUFFIXES lib)
# Handle the QUIETLY and REQUIRED arguments and set OPUS_FOUND
# to TRUE if all listed variables are TRUE.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Opus DEFAULT_MSG OPUS_INCLUDE_DIR OPUS_LIBRARY)

if (OPUS_FOUND)
	set (OPUS_LIBRARIES ${OPUS_LIBRARY})
	set (OPUS_INCLUDE_DIRS ${OPUS_INCLUDE_DIR})
endif (OPUS_FOUND)

mark_as_advanced(OPUS_INCLUDE_DIR OPUS_LIBRARY)
