#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "ArduCopter V21.0730.403-CNDN"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 21,730,403,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 21
#define FW_MINOR 730
#define FW_PATCH 403
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV
