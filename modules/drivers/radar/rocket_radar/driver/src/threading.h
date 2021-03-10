// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

#ifdef _MSC_VER
#include <windows.h>
#else
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>
#include <fcntl.h>
#endif

#if MACOS
#include <sys/param.h>
#include <sys/sysctl.h>
#endif

#if __GNUC__               /* GCCs builtin atomics */
#include <sys/time.h>
#endif // ifdef __GNUC__

#ifdef _MSC_VER

typedef HANDLE ThreadHandle;

class Mutex
{
public:

    Mutex()
    {
        InitializeCriticalSection(&handle);
    }

    ~Mutex()
    {
        DeleteCriticalSection(&handle);
    }

    void acquire()
    {
        EnterCriticalSection(&handle);
    }

    bool try_acquire()
    {
        return !!TryEnterCriticalSection(&handle);
    }

    void release()
    {
        LeaveCriticalSection(&handle);
    }

    CRITICAL_SECTION handle;
};


/* Safely increment counters between CPU cores */
class ConditionVar
{
public:

    ConditionVar(Mutex& lock)
        : critical_section(lock.handle)
        , value(0)
    {
        InitializeConditionVariable(&condition_variable);
    }

    ~ConditionVar() {}

    void poke(void)
    {
        /* awaken all waiting threads, but make no change */
        WakeAllConditionVariable(&condition_variable);
    }

    uint32_t waitForChange(uint32_t prev)
    {
        EnterCriticalSection(&critical_section);
        if (value == prev)
            SleepConditionVariableCS(&condition_variable, &critical_section, INFINITE);
        LeaveCriticalSection(&critical_section);
        return value;
    }

    uint32_t get()
    {
        EnterCriticalSection(&critical_section);
        uint32_t ret = value;
        LeaveCriticalSection(&critical_section);
        return ret;
    }

    void set(uint32_t newval)
    {
        EnterCriticalSection(&critical_section);
        value = newval;
        WakeAllConditionVariable(&condition_variable);
        LeaveCriticalSection(&critical_section);
    }

protected:

    CRITICAL_SECTION&  critical_section;
    CONDITION_VARIABLE condition_variable;
    uint32_t           value;
};

class Event
{
public:

    Event()
    {
        handle = CreateEvent(NULL, FALSE, FALSE, NULL);
    }

    ~Event()
    {
        CloseHandle(handle);
    }

    void wait()
    {
        WaitForSingleObject(handle, INFINITE);
    }

    void trigger()
    {
        SetEvent(handle);
    }

protected:

    HANDLE handle;
};

#else /* POSIX / pthreads */

typedef pthread_t ThreadHandle;

class Mutex
{
public:

    Mutex()
    {
        pthread_mutex_init(&handle, NULL);
    }

    ~Mutex()
    {
        pthread_mutex_destroy(&handle);
    }

    void acquire()
    {
        pthread_mutex_lock(&handle);
    }

    bool try_acquire()
    {
        return EBUSY != pthread_mutex_trylock(&handle);
    }

    void release()
    {
        pthread_mutex_unlock(&handle);
    }

    pthread_mutex_t handle;
};


/* Safely increment counters between CPU cores */
class ConditionVar
{
public:

    ConditionVar(Mutex& lock)
        : mutex(lock.handle)
        , value(0)
    {
        if (pthread_cond_init(&condition, NULL))
        {
            printf("fatal: unable to initialize conditional variable\n");
        }
    }

    ~ConditionVar()
    {
        pthread_cond_destroy(&condition);
    }

    void poke(void)
    {
        /* awaken all waiting threads, but make no change, presumes mutex is
         * aquired */
        pthread_cond_broadcast(&condition);
    }

    uint32_t waitForChange(uint32_t prev)
    {
        pthread_mutex_lock(&mutex);
        if (value == prev)
        {
            pthread_cond_wait(&condition, &mutex);
        }
        pthread_mutex_unlock(&mutex);
        return value;
    }

    uint32_t get()
    {
        pthread_mutex_lock(&mutex);
        uint32_t ret = value;
        pthread_mutex_unlock(&mutex);
        return ret;
    }

    void set(uint32_t newval)
    {
        pthread_mutex_lock(&mutex);
        value = newval;
        pthread_cond_broadcast(&condition);
        pthread_mutex_unlock(&mutex);
    }

protected:

    pthread_mutex_t& mutex;
    pthread_cond_t   condition;
    uint32_t         value;
};

class Event
{
public:

    Event()
    {
        counter = 0;
        if (pthread_mutex_init(&mutex, NULL) ||
            pthread_cond_init(&condition, NULL))
        {
            printf("fatal: unable to initialize conditional variable\n");
        }
    }

    ~Event()
    {
        pthread_cond_destroy(&condition);
        pthread_mutex_destroy(&mutex);
    }

    void wait()
    {
        pthread_mutex_lock(&mutex);

        /* blocking wait on conditional variable, mutex is atomically released
         * while blocked. When condition is signaled, mutex is re-acquired */
        while (!counter)
            pthread_cond_wait(&condition, &mutex);

        counter--;
        pthread_mutex_unlock(&mutex);
    }

    void trigger()
    {
        pthread_mutex_lock(&mutex);
        counter++;
        /* Signal a single blocking thread */
        pthread_cond_signal(&condition);
        pthread_mutex_unlock(&mutex);
    }

protected:

    pthread_mutex_t mutex;
    pthread_cond_t  condition;
    uint32_t        counter;
};

#endif // ifdef _WIN32

// Simplistic portable thread class.  Shutdown signalling left to derived class
class Thread
{
private:

    ThreadHandle handle;

public:

    Thread();

    virtual ~Thread();

    //< Derived class must implement ThreadMain.
    virtual void thread_main() = 0;

    //< Returns true if thread was successfully created
    bool start();

    void stop();
};
