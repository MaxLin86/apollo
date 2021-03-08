if(MSVC)
    add_definitions(/wd4996)
    add_definitions(/wd4267)
    add_definitions(/wd4244)
endif(MSVC)
if(WIN32)
    set(WIN32FILES win32.cpp)
endif(WIN32)

add_library(srapi STATIC ${WIN32FILES}
    threading.cc         threading.h
    connection_impl.cc   connection_impl.h
    logcontrol_impl.cc   logcontrol_impl.h
    version.cc
    vehiclecan.cc
    scan-info-json.cc
    file_xfer.cc         file_xfer.h
    scanobject_impl.cc   scanobject_impl.h
    scanning_impl.cc     scanning_impl.h
    detections_impl.cc   detections_impl.h
    staticslice_impl.cc  staticslice_impl.h
    clutterimage_impl.cc clutterimage_impl.h
    zerodoppler_impl.cc  zerodoppler_impl.h
    activations_impl.cc  activations_impl.h
    pointcloud_impl.cc   pointcloud_impl.h
    musicdata_impl.cc    musicdata_impl.h
    polar2cartesian_impl.cc polar2cartesian_impl.h
    radardatacube1_impl.cc radardatacube1_impl.h)

add_custom_target(revdesc
    python detect-revision.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    VERBATIM)
# update revdesc.h each time you make, if revision info has changed
add_dependencies(srapi revdesc)

# vim: syntax=cmake tw=130
