/**
 * @file Hardware/SPI.h
 * @version 1.0
 *
 * @section License
 * Copyright (C) 2017, Mikael Patel
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 */

#ifndef HARDWARE_SPI_H
#define HARDWARE_SPI_H
#if defined(AVR)
#include "Hardware/AVR/SPI.h"
#elif defined(SAM)
#include "Hardware/SAM/SPI.h"
#endif
#endif
