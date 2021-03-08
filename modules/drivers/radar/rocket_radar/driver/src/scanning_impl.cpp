#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanning_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/connection_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/include/serializer.h"

Scanning_Impl::Scanning_Impl(Connection_Impl* c)
    : mycon(c)
    , cur_sequence_number(0)
    , last_err(SCANNING_NO_ERROR)
    , session_serializer(NULL)
{
}


Scanning_Impl::Scanning_Impl(const char* session_path, uint32_t first_seq)
    : mycon(NULL)
    , cur_sequence_number(first_seq)
    , last_err(SCANNING_NO_ERROR)
{
    session_serializer = new SessionFolderScanSerializer(session_path);
}


Scanning_Impl::~Scanning_Impl()
{
    delete session_serializer;
}


ScanObject* Scanning_Impl::wait_for_scan()
{

    if (mycon)
    {
        ScanObject_Impl* scan = mycon->scan_wait();
        return scan;
    }
    else if (session_serializer)
    {
        ScanObject* scan = ScanObject::deserialize(*session_serializer, cur_sequence_number);
        // Note we increment cur_sequence_number even if no scan was returned,
        // in order to skip over gaps, if there are any
        cur_sequence_number++;
        return scan;
    }
    else
    {
        return NULL;
    }
}

bool        Scanning_Impl::scan_available()
{
    if (mycon)
    {
        return mycon->scan_available();
    }
    else
    {
        // scan_wait() will never block when replaying a session
        return true;
    }
}

void        Scanning_Impl::release()
{
    if (mycon)
    {
        // was created by Connection object
        mycon->release_scanning(*this);
    }
    else
    {
        // was created by Scanning::replay_session()
        delete this;
    }
}


Scanning* Scanning::replay_session(const char* spath, uint32_t first_seq)
{
    return new Scanning_Impl(spath, first_seq);
}
