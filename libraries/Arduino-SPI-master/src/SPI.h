/**
 * @file SPI.h
 * @version 1.5
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

#ifndef SPI_H
#define SPI_H
//#define SPI_HAS_TRANSACTION

#include "GPIO.h"

/**
 * Serial Perpheral Interface (SPI) abstract class.
 */
class SPI {
public:
  /** Maximum clock frequency. */
  static const uint32_t MAX_FREQ = F_CPU / 2;

  /**
   * Construct and initiate bus manager.
   */
  SPI() : m_busy(false) {}

  /**
   * @override{SPI}
   * Calculate clock prescale from device frequency.
   * @param[in] frequency device access.
   * @return prescale
   */
  virtual uint8_t prescale(uint32_t frequency)
  {
    return (F_CPU / frequency);
  }

  /**
   * @override{SPI}
   * Acquire bus access with given mode.
   * @param[in] mode of access.
   * @param[in] bitorder of serial data.
   * @param[in] prescale for device.
   */
  virtual void acquire(uint8_t mode, uint8_t bitorder, uint8_t prescale)
  {
    (void) mode;
    (void) bitorder;
    (void) prescale;
    lock();
  }

  /**
   * @override{SPI}
   * Release bus access.
   */
  virtual void release()
  {
    unlock();
  }

  /**
   * @override{SPI}
   * Send given byte to device. Returns byte received from device.
   * @param[in] value to send to device.
   * @return received value.
   */
  virtual uint8_t transfer(uint8_t value) = 0;

  /**
   * Read value from device.
   * @param[out] value read from device.
   */
  SPI& operator>>(uint8_t& value)
    __attribute__((always_inline))
  {
    value = transfer(0);
    return (*this);
  }

  /**
   * Write value to device. Shorthand for write().
   * @param[in] value to write to device.
   */
  SPI& operator<<(uint8_t value)
    __attribute__((always_inline))
  {
    transfer(value);
    return (*this);
  }

  /**
   * @override{SPI}
   * Transfer given number of bytes from source buffer to
   * device. Store received bytes in destination buffer.
   * @param[in] dest destination buffer.
   * @param[in] src source buffer.
   * @param[in] count number of bytes to transfer.
   */
  virtual void transfer(void* dest, const void* src, size_t count)
  {
    if (count == 0 || dest == NULL || src == NULL) return;
    const uint8_t* sp = (const uint8_t*) src;
    uint8_t* dp = (uint8_t*) dest;
    do *dp++ = transfer(*sp++); while (--count);
  }

  /**
   * @override{SPI}
   * Read given number of bytes from device and store in buffer.
   * @param[in] buf buffer pointer.
   * @param[in] count number of bytes.
   */
  virtual void read(void* buf, size_t count)
  {
    if (count == 0 || buf == NULL) return;
    uint8_t* bp = (uint8_t*) buf;
    do *bp++ = transfer(0); while (--count);
  }

  /**
   * @override{SPI}
   * Write given number of bytes from buffer to device.
   * @param[in] buf buffer pointer.
   * @param[in] count number of bytes.
   */
  virtual void write(const void* buf, size_t count)
  {
    if (count == 0 || buf == NULL) return;
    uint8_t* bp = (uint8_t*) buf;
    do transfer(*bp++); while (--count);
  }

  /**
   * Serial Perpheral Interface (SPI) device driver class.
   * @param[in] MODE clock mode (polarity and phase).
   * @param[in] BITORDER LSB or MSB first.
   * @param[in] FREQ max clock frequency.
   * @param[in] SS_PIN board pin for device slave select.
   */
  template<uint8_t MODE, uint8_t BITORDER, uint32_t FREQ, BOARD::pin_t SS_PIN>
  class Device {
  public:
    /**
     * Construct Serial Perpheral Interface (SPI) device driver
     * instance and initiate slave select pin.
     * @param[in] spi bus manager.
     * @param[in] ss initial slave select pin state (default HIGH).
     */
    Device(SPI& spi, bool ss = HIGH) :
      m_spi(spi),
      PRESCALE(spi.prescale(FREQ))
    {
      m_ss.output();
      m_ss = ss;
    }

    /**
     * Acquire bus manager and slave device.
     */
    void acquire()
      __attribute__((always_inline))
    {
      m_spi.acquire(MODE, BITORDER, PRESCALE);
      m_ss.toggle();
    }

    /**
     * Release bus manager and slave device.
     */
    void release()
      __attribute__((always_inline))
    {
      m_ss.toggle();
      m_spi.release();
    }

    /**
     * Send given byte to device. Returns byte received from device.
     * @param[in] value to send to device.
     * @return received value.
     */
    uint8_t transfer(uint8_t value)
      __attribute__((always_inline))
    {
      return (m_spi.transfer(value));
    }

    /**
     * Transfer given number of bytes from source buffer to
     * device. Store received bytes in destination buffer.
     * @param[in] dest destination buffer.
     * @param[in] src source buffer.
     * @param[in] count number of bytes to transfer.
     */
    void transfer(void* dest, const void* src, size_t count)
      __attribute__((always_inline))
    {
      m_spi.transfer(dest, src, count);
    }

    /**
     * Read given number of bytes from device and store in buffer.
     * @param[in] buf buffer pointer.
     * @param[in] count number of bytes.
     */
    void read(void* buf, size_t count)
      __attribute__((always_inline))
    {
      m_spi.read(buf, count);
    }

    /**
     * Write given number of bytes from buffer to device.
     * @param[in] buf buffer pointer.
     * @param[in] count number of bytes.
     */
    void write(const void* buf, size_t count)
      __attribute__((always_inline))
    {
      m_spi.write(buf, count);
    }

  protected:
    /** Bus manager. */
    SPI& m_spi;

    /** Slave select pin. */
    GPIO<SS_PIN> m_ss;

    /** Bus clock prescale. */
    const uint8_t PRESCALE;
  };

protected:
  /** Bus manager semaphore. */
  volatile bool m_busy;

  /**
   * Lock bus manager.
   */
  void lock()
    __attribute__((always_inline))
  {
    while (m_busy) yield();
    m_busy = true;
  }

  /**
   * Unlock bus manager.
   */
  void unlock()
    __attribute__((always_inline))
  {
    m_busy = false;
  }
};
#endif
