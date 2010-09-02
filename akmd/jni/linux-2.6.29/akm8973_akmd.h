/*
 * Definitions for akm8973 compass chip.
 */
#ifndef AKM8973_H
#define AKM8973_H

#include <linux/ioctl.h>

struct akm8973_platform_data {
        int (*init)(void);
        void (*exit)(void);
        int (*power_on)(void);
        int (*power_off)(void);
};

/* Compass device dependent definition */
#define AKECS_MODE_MEASURE      0x00    /* Starts measurement. */
#define AKECS_MODE_E2P_READ     0x02    /* E2P access mode (read). */
#define AKECS_MODE_POWERDOWN    0x03    /* Power down mode */

#define RBUFF_SIZE              4       /* Rx buffer size */

/* AK8973 register address */
#define AKECS_REG_ST                    0xC0
#define AKECS_REG_TMPS                  0xC1
#define AKECS_REG_H1X                   0xC2
#define AKECS_REG_H1Y                   0xC3
#define AKECS_REG_H1Z                   0xC4

#define AKECS_REG_MS1                   0xE0
#define AKECS_REG_HXDA                  0xE1
#define AKECS_REG_HYDA                  0xE2
#define AKECS_REG_HZDA                  0xE3
#define AKECS_REG_HXGA                  0xE4
#define AKECS_REG_HYGA                  0xE5
#define AKECS_REG_HZGA                  0xE6

#endif

#define AKMIO                           0xA1

/* IOCTLs for AKM library */
#define ECS_IOCTL_INIT                  _IO(AKMIO, 0x01)
#define ECS_IOCTL_WRITE                 _IOW(AKMIO, 0x02, char[5])
#define ECS_IOCTL_READ                  _IOWR(AKMIO, 0x03, char[5])
#define ECS_IOCTL_RESET                 _IO(AKMIO, 0x04)
#define ECS_IOCTL_SET_MODE              _IOW(AKMIO, 0x07, short)
#define ECS_IOCTL_GETDATA               _IOR(AKMIO, 0x08, char[RBUFF_SIZE+1])
#define ECS_IOCTL_GET_NUMFRQ            _IOR(AKMIO, 0x09, char[2])
#define ECS_IOCTL_SET_YPR               _IOW(AKMIO, 0x0C, short[12])
#define ECS_IOCTL_GET_OPEN_STATUS       _IOR(AKMIO, 0x0D, int)
#define ECS_IOCTL_GET_CLOSE_STATUS      _IOR(AKMIO, 0x0E, int)
#define ECS_IOCTL_GET_DELAY             _IOR(AKMIO, 0x30, short)

/* IOCTLs for APPs */
#define ECS_IOCTL_APP_SET_MFLAG         _IOW(AKMIO, 0x11, short)
#define ECS_IOCTL_APP_GET_MFLAG         _IOW(AKMIO, 0x12, short)
#define ECS_IOCTL_APP_SET_AFLAG         _IOW(AKMIO, 0x13, short)
#define ECS_IOCTL_APP_GET_AFLAG         _IOR(AKMIO, 0x14, short)
#define ECS_IOCTL_APP_SET_TFLAG         _IOR(AKMIO, 0x15, short)
#define ECS_IOCTL_APP_GET_TFLAG         _IOR(AKMIO, 0x16, short)
#define ECS_IOCTL_APP_SET_DELAY         _IOW(AKMIO, 0x18, short)
#define ECS_IOCTL_APP_GET_DELAY         ECS_IOCTL_GET_DELAY
#define ECS_IOCTL_APP_SET_MVFLAG        _IOW(AKMIO, 0x19, short) /* Set raw magnetic vector flag */
#define ECS_IOCTL_APP_GET_MVFLAG        _IOR(AKMIO, 0x1A, short) /* Get raw magnetic vector flag */
