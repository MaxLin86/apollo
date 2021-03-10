// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef COREDEFS_ENVIRONMENT_DEFS
#define COREDEFS_ENVIRONMENT_DEFS 1

/* this file should only contain C macro #define statements. It is included by
 * linker scripts and other things that cannot parse anything else */


#ifndef WITH_HSM_MPU
#define WITH_HSM_MPU   1
#endif

#ifndef WITH_NOC_FIREWALL
#define WITH_NOC_FIREWALL 1
#endif

#ifndef WITH_HSM_DIAGS
#define WITH_HSM_DIAGS 1
#endif

#ifndef WITH_CCP_MPU
#define WITH_CCP_MPU 1
#endif

#ifndef WITH_CCP_DIAGS
#define WITH_CCP_DIAGS 1
#endif
#endif // ifndef COREDEFS_ENVIRONMENT_DEFS
