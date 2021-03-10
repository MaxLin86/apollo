// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/include/rra.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhunistd.h"

#ifdef _WIN32
#include "Wincon.h"
#else
#include <unistd.h>
#endif

// this example UserLogAgent drops all log messages on the floor (/dev/null)
class NullLogger : public UserLogAgent
{
    virtual void radar_print_message(ICCQTargetEnum cpu, const char* message) { }
    virtual void radar_log_message(ICCQTargetEnum cpu, LogLevelEnum level, const char* message) {}
};


// this example UserLogAgent writes log messages to stdout
class ConsoleLogger : public UserLogAgent
{
public:

#ifdef _WIN32
    HANDLE hConsole;
    enum { WHITE_FOREGROUND = FOREGROUND_BLUE | FOREGROUND_GREEN | FOREGROUND_RED };

    ConsoleLogger()
    {
        hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    }
#endif

    virtual void radar_print_message(ICCQTargetEnum cpu, const char* message)
    {
        printf("%s-UHP: %s\n", cpu_names[cpu], message);
    }

    virtual void radar_log_message(ICCQTargetEnum cpu, LogLevelEnum level, const char* message)
    {
        switch (level)
        {
        case LL_ALWAYS: // green
#ifdef _WIN32
            SetConsoleTextAttribute(hConsole, WHITE_FOREGROUND | BACKGROUND_GREEN | BACKGROUND_INTENSITY);
            printf("%s-LOG: %s\n", cpu_names[cpu], message);
            SetConsoleTextAttribute(hConsole, WHITE_FOREGROUND);
#else
            if (isatty(fileno(stdout)))
            {
                printf("\033[1;42;37m%s-LOG: %s\033[0m\n", cpu_names[cpu], message);
            }
            else
            {
                printf("%s-LOG: %s\n", cpu_names[cpu], message);
            }
#endif
            break;
        case LL_ERROR: // red
#ifdef _WIN32
            SetConsoleTextAttribute(hConsole, WHITE_FOREGROUND | BACKGROUND_RED | BACKGROUND_INTENSITY);
            printf("%s-LOG: %s\n", cpu_names[cpu], message);
            SetConsoleTextAttribute(hConsole, WHITE_FOREGROUND);
#else
            if (isatty(fileno(stdout)))
            {
                printf("\033[1;41;37m%s-LOG: %s\033[0m\n", cpu_names[cpu], message);
            }
            else
            {
                printf("%s-LOG: %s\n", cpu_names[cpu], message);
            }
#endif
            break;
        case LL_WARN: // yellow
#ifdef _WIN32
            SetConsoleTextAttribute(hConsole, WHITE_FOREGROUND | BACKGROUND_RED | BACKGROUND_GREEN | BACKGROUND_INTENSITY);
            printf("%s-LOG: %s\n", cpu_names[cpu], message);
            SetConsoleTextAttribute(hConsole, WHITE_FOREGROUND);
#else
            if (isatty(fileno(stdout)))
            {
                printf("\033[1;43;37m%s-LOG: %s\033[0m\n", cpu_names[cpu], message);
            }
            else
            {
                printf("%s-LOG: %s\n", cpu_names[cpu], message);
            }
#endif
            break;
        case LL_INFO:
            printf("%s-LOG: %s\n", cpu_names[cpu], message);
            break;

        default:
            break;;
        }
    }
};

// this example UserLogAgent writes log messages to stdout
class FileLogger : public UserLogAgent
{
public:

    FileLogger(const char* session_path)
    {
#ifdef _WIN32
        CreateDirectory(session_path, NULL);
#else
        mkdir(session_path, 0770);
#endif
        char fname[1024];
        sprintf(fname, "%s/radarlogs.txt", session_path);
        fp = fopen(fname, "w");
    }

    bool is_ok() const { return !!fp; }

    virtual ~FileLogger()
    {
        fclose(fp);
    }

    virtual void radar_print_message(ICCQTargetEnum cpu, const char* message)
    {
        fprintf(fp, "%s-UHP: %s\n", cpu_names[cpu], message);
    }

    virtual void radar_log_message(ICCQTargetEnum cpu, LogLevelEnum level, const char* message)
    {
        if (level >= LL_INFO)
        {
            fprintf(fp, "%s-LOG: %s\n", cpu_names[cpu], message);
        }
    }

    FILE* fp;
};

