#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "AUTO-10 V22.508.403-CNDN"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 22,508,403,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 22
#define FW_MINOR 508
#define FW_PATCH 403
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV
