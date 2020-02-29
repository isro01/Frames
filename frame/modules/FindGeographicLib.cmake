find_path (GeographicLib_INCLUDE_DIRS NAMES GeographicLib/Config.h)

find_library (GeographicLib_LIBRARIES NAMES Geographic)

include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (GeographicLib DEFAULT_MSG
    GeographicLib_LIBRARIES GeographicLib_INCLUDE_DIRS)

mark_as_advanced (GeographicLib_LIBRARIES GeographicLib_INCLUDE_DIRS)