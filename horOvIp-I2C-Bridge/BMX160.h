/*
 * BMX160.h
 *
 * Created: 13.10.2019 17:03:37
 *  Author: kai_horstmann
 */ 


#ifndef BMX160_H_
#define BMX160_H_

#include <stdbool.h>

#include "config.h"
#include "BMX160net.h"


/** \brief Activates and configures the sensor
 *
 * This function runs synchronous
 *
 */
void BMX160Init();

/** \brief Start timer driven cyclic data capturing from the sensor
 *
 */
void BMX160StartDataCapturing();

/** \brief Stop timer driven cyclic data capturing from the sensor
 *
 */
void BMX160StopDataCapturing();

/** Start reading sensor data.
 *
 * This call is synchronous.
 * You can access the data with \ref BMX160GetData()
 */
void BMX160ReadoutSensors();

/** \brief Read the magnetometer factory trim registers
 *
 * This call is synchonous. You can retrieve the data with \ref BMX160GetData() immediately
 *
 * Copied with some modification from the Bosch BMM150 driver
 *
 */
void BMX160ReadTrimRegisters();

/** \brief Retrieve the sensor measurements or magnetometer trim data
 *
 * The function will return the pointer to the data in any case. Use \ref BMX160IsDataValid() to check if data is valid.
 * When call this function repeatedly fast it will return the same data again and again
 * until a new read cycle is initiated by the timer.
 * You can save sensorTime0-3 in the \ref BMX160Data structure from the last read, and compare it with the current one.
 * When they match no update took place.
 *
 * @return Pointer to valid sensor or magnetometer trim data.
 */
struct BMX160Data* BMX160GetData();

#endif /* BMX160_H_ */
