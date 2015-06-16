/*! ----------------------------------------------------------------------------
 * @file	deca_version.h
 * @brief	Defines the version info for the DW1000 device driver including its API
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#ifndef _DECA_VERSION_H_
#define _DECA_VERSION_H_

//
// The DW1000 device driver is separately version numbered to any version the application using it may have
//
// Two symbols are defined here one a hexidecimal value and string that incluides the hex bytes.
// Both should be updated together in a consistent way when the software is being modified.
//
// The format of the hex version is 0xAABBCC and the string ends with AA.BB.CC, where...
//
// Quantity CC is updated for minor changes/bug fixes that should need user code changes
// Quantity BB is updated for changes/bug fixes that may need user code changes
// Quantity AA is updated for major changes that will need user code changes
//

#define DW1000_DRIVER_VERSION               0x021300
#define DW1000_DEVICE_DRIVER_VER_STRING     "DW1000 Device Driver Version 02.13.00"

#endif
