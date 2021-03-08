#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/src/threading.h"

Thread::Thread()
{
    handle = 0;
}

#if _MSC_VER

static DWORD WINAPI thread_main_wrapper(Thread *instance)
{
    // defer processing to the virtual function implemented in the derived class
    instance->thread_main();

    return 0;
}

bool Thread::start()
{
    DWORD threadId;

    handle = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)thread_main_wrapper, this, 0, &threadId);

    return threadId > 0;
}

void Thread::stop()
{
    if (handle)
    {
        WaitForSingleObject(handle, INFINITE);
    }
}

Thread::~Thread()
{
    if (handle)
    {
        CloseHandle(handle);
        handle = 0;
    }
}

#else /* POSIX / pthreads */

static void *thread_main_wrapper(void *opaque)
{
    // defer processing to the virtual function implemented in the derived class
    Thread *instance = reinterpret_cast<Thread *>(opaque);

    instance->thread_main();

    return NULL;
}

bool Thread::start()
{
    if (pthread_create(&handle, NULL, thread_main_wrapper, this))
    {
        handle = 0;
        return false;
    }

    return true;
}

void Thread::stop()
{
    if (handle)
    {
        pthread_join(handle, NULL);
        handle = 0;
    }
}

Thread::~Thread() {}

#endif // if _MSC_VER
