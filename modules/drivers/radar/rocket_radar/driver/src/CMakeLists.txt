if(MSVC)
    add_definitions(/wd4996)
    add_definitions(/wd4267)
    add_definitions(/wd4244)
endif(MSVC)
if(WIN32)
    set(WIN32FILES win32.cpp)
endif(WIN32)

add_library(srapi STATIC ${WIN32FILES}
    threading.cpp         threading.h
    connection_impl.cpp   connection_impl.h
    logcontrol_impl.cpp   logcontrol_impl.h
    version.cpp
    vehiclecan.cpp
    scan-info-json.cpp
    file_xfer.cpp         file_xfer.h
    scanobject_impl.cpp   scanobject_impl.h
    scanning_impl.cpp     scanning_impl.h
    detections_impl.cpp   detections_impl.h
    staticslice_impl.cpp  staticslice_impl.h
    clutterimage_impl.cpp clutterimage_impl.h
    zerodoppler_impl.cpp  zerodoppler_impl.h
    activations_impl.cpp  activations_impl.h
    pointcloud_impl.cpp   pointcloud_impl.h
    musicdata_impl.cpp    musicdata_impl.h
    polar2cartesian_impl.cpp polar2cartesian_impl.h
    adccapturedata_impl.cpp adccapturedata_impl.h
    radardatacube1_impl.cpp radardatacube1_impl.h)

add_custom_target(revdesc
    python detect-revision.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    VERBATIM)
# update revdesc.h each time you make, if revision info has changed
add_dependencies(srapi revdesc)

# vim: syntax=cmake tw=130
