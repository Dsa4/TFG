/**************************************************************************//**
 *  @file
 *
 *  @brief
 *    Driver for reading Decagon Devices sensors and send it back to PANDA
 *
 *  @author
 *    GIE - adevice - RGG - CieNTi - CBR
 *
 *  @todo 
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "em_gpio.h"
#include "em_adc.h"
#include "em_int.h"
#include "em_timer.h"
#include "gpiointerrupt.h"
#include "swserial.h"
#include "rtc.h"
#include "core_init.h"
#include "API_PANDA.h"


/* Decagon ECT */
/**
 * @addtogroup decagonect_driver
 * @brief  Driver for Decagon ECT device.
 * @details
 *  Driver to read from the Decagon ECT sensor through analog interfaces.
 * @{
 */

#define REF_V   (2.5f)
#define SCALE_F (10.0f)    /* Scale factor (multiplication by this value) */

static uint16_t get_average_decagonect(void);
static float volt_to_temperature_ect(float voltage);
static inline float get_average_voltage(void);

/** @} (end addtogroup decagonect_driver) */


/* pow() for float base and integer exponent */
static inline float powfi(float x, int y)
{
  unsigned int n = y;
  if (y < 0)
    n = 0 - n;

  for (float z = 1; ; x *= x)
    {
      if ((n & 1) != 0)
        z *= x;
      if ((n >>= 1) == 0)
        return (y < 0 ? (z == 0.0f ? (float)HUGE_VAL : (float)(1) / z) : z);
    }
}


void ADB_periodic_read_decagonect(void)
{
  unsigned channels = datos_conf.num_reg;  /* number of channels to read */
  float volts;

  /* API_PANDA */
  N_data_error = 0;
  N_data_pending = channels;

  /* Activate ADB supply */
  ADB_supply_enable (true);

  
    /* Get PCB 4-20mA_0 (volts) */
    setupADC(ADC_1V_0, adcRef2V5);
    estructura_datos[0].uuid_serv = estructura_modbus[0].uuid_serv;
    estructura_datos[0].long_data = estructura_modbus[0].long_data;
    estructura_datos[0].date_read = get_RTC();
    estructura_datos[0].code_error = 0x00;

    /* Capture 32 values, calculate average and set final variable value */
    volts = get_average_voltage();
    estructura_datos[0].data = (uint16_t) volt_to_temperature_ect(volts);

  
  /* Math */
  indice = N_data_pending;
  indice_err = N_data_pending;
  N_data_pending -= N_data_error;

  /* Deactivate ADB supply */
  ADB_supply_enable (false);
}


/* Decagon ECT */
/**
 * @addtogroup decagonect_driver
 * @{
 */

/**
 *  @brief
 *    Get averaged value
 *
 *  @details
 *    Capture 32 ADC values on selected channel and return averaged data
 */
static uint16_t get_average_decagonect (void)
{
  uint8_t sample_count;
  uint32_t adc_averaged;

  adc_averaged = 0;
  for (sample_count = 0; sample_count < 32; sample_count++)
    {
      /**
       *  ADC minimal expression. CAUTION: Blocking -> Implement via DMA
       **/
      ADC_Start (ADC0, adcStartSingle);

      /* Wait while conversion is active */
      while (ADC0->STATUS & ADC_STATUS_SINGLEACT) ;

      /* Get ADC result */
      adc_averaged += ADC_DataSingleGet(ADC0);
    }

  /* Divide by 32 */
  adc_averaged >>= 5;

  /* Return it! */
  return (uint16_t)adc_averaged;
}


/**
 * @brief
 *    Get averaged voltage value
 *
 */
static inline float get_average_voltage(void)
{
  uint16_t raw_value;

  raw_value = get_average_decagonect();
  return (float)raw_value * REF_V / 4096.0f;
}


/**
 * @brief
 *    Get the temperature from the voltage measured.
 *
 * @note
 *    Valid only for Decagon ECT sensor.
 *
 * @param voltage
 *    Measured voltage in volts
 *
 * @return
 *    Temperature corresponding to the voltage.
 */
static float volt_to_temperature_ect(float voltage)
{
  float chi;
  float temperature;
  
  float power = 3.5;
  //float power = adcSingleInputVDDDiv3;

  chi=log((power/voltage)-1);

  temperature = -0.08372f * powfi(chi, 3) + 1.532f * powfi(chi, 2)
    - 22.84f * chi + 25.02; 

/* Factor for 1 more decimal */
  return temperature * SCALE_F;
}

/** @} (end addtogroup decagonect_driver) */


/* EOF */