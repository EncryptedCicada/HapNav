/******************************************************************************
Fifo_Example.ino

https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO
https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO_Arduino_Library

Description:
This example enables the use of the FIFO for data collection. The FIFO is set
to collect 500 bytes of temperature and gyroscopic data before it stops
collecting data (FIFO_MODE). Upon reading the data (or if you continuously read the FIFO)
the FIFO will begine to refill. 

There are other available modes (see datasheet found in github repository for
more information) but the entire list of fifo functionality predefines can be found below. 

myIMU.setFifoMode( -arguments below- )

FIFO_MODE_DISABLED       
FIFO_MODE_STOP_WHEN_FULL 
FIFO_MODE_CONT_TO_FIFO   
FIFO_MODE_BYPASS_TO_CONT 
FIFO_MODE_CONTINUOUS     
FIFO_MODE_BYPASS_TO_FIFO 
FIFO_MODE_MASK           

Development environment tested:
Arduino IDE 1.8.2

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfugn.com.
Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFunLSM6DSO.h"
#include "Wire.h"
//#include "SPI.h"

LSM6DSO myIMU;
fifoData myFifo; //This will hold our FIFO data
int availableBytes = 0;
static stmdev_ctx_t mag_ctx;

void setup()
{

  Serial.begin(115200);
  delay(500); 
  
  Wire.begin();
  delay(10);
  if( myIMU.begin() )
    Serial.println("Ready.");
  else { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if( myIMU.initialize(FIFO_SETTINGS) )
    Serial.println("Loaded Settings.");

	
  
}


void loop()
{



  /* Initialize lis2mdl driver interface */
  mag_ctx.read_reg = lsm6dso_read_lis2mdl_cx;
  mag_ctx.write_reg = lsm6dso_write_lis2mdl_cx;
  mag_ctx.handle = &SENSOR_BUS;


  availableBytes = myIMU.getFifoStatus();  //Check for data in FIFO

  if( availableBytes > 0 ){
    Serial.print("Number of bytes in FIFO: ");
    Serial.println(availableBytes);

    myFifo = myIMU.fifoRead(); // Get the data

    if( myFifo.fifoTag == ACCELEROMETER_DATA ){
      Serial.println("Accelerometer:");
      Serial.print(" X = ");
      Serial.println(myFifo.xAccel, 3);
      Serial.print(" Y = ");
      Serial.println(myFifo.yAccel, 3);
      Serial.print(" Z = ");
      Serial.println(myFifo.zAccel, 3);
    }

    if( myFifo.fifoTag == GYROSCOPE_DATA ){
      Serial.println("Gyroscope: ");
      Serial.print(" X = ");
      Serial.println(myFifo.xGyro, 3);
      Serial.print(" Y = ");
      Serial.println(myFifo.yGyro, 3);
      Serial.print(" Z = ");
      Serial.println(myFifo.zGyro, 3);
    }
  }

  delay(500);
 
}



/*
 * @brief  Write lsm2mdl device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t lsm6dso_write_lis2mdl_cx(void *ctx, uint8_t reg,
                                        const uint8_t *data, uint16_t len)
{
  axis3bit16_t data_raw_acceleration;
  int32_t ret;
  uint8_t drdy;
  lsm6dso_status_master_t master_status;
  lsm6dso_sh_cfg_write_t sh_cfg_write;
  /* Configure Sensor Hub to read LIS2MDL. */
  sh_cfg_write.slv0_add = LIS2MDL_I2C_ADD; /* 8bit I2C address */
  sh_cfg_write.slv0_subadd = reg,
  sh_cfg_write.slv0_data = *data,
  ret = lsm6dso_sh_cfg_write(&ag_ctx, &sh_cfg_write);
  /* Disable accelerometer. */
  lsm6dso_xl_data_rate_set(&ag_ctx, LSM6DSO_XL_ODR_OFF);
  /* Enable I2C Master. */
  lsm6dso_sh_master_set(&ag_ctx, PROPERTY_ENABLE);
  /* Enable accelerometer to trigger Sensor Hub operation. */
  lsm6dso_xl_data_rate_set(&ag_ctx, LSM6DSO_XL_ODR_104Hz);
  /* Wait Sensor Hub operation flag set. */
  lsm6dso_acceleration_raw_get(&ag_ctx, data_raw_acceleration.i16bit);

  do {
    HAL_Delay(20);
    lsm6dso_xl_flag_data_ready_get(&ag_ctx, &drdy);
  } while (!drdy);

  do {
    HAL_Delay(20);
    lsm6dso_sh_status_get(&ag_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL (trigger). */
  lsm6dso_sh_master_set(&ag_ctx, PROPERTY_DISABLE);
  lsm6dso_xl_data_rate_set(&ag_ctx, LSM6DSO_XL_ODR_OFF);
  return ret;
}

/*
 * @brief  Read lsm2mdl device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t lsm6dso_read_lis2mdl_cx(void *ctx, uint8_t reg,
                                       uint8_t *data, uint16_t len)
{
  lsm6dso_sh_cfg_read_t sh_cfg_read;
  axis3bit16_t data_raw_acceleration;
  int32_t ret;
  uint8_t drdy;
  lsm6dso_status_master_t master_status;
  /* Disable accelerometer. */
  lsm6dso_xl_data_rate_set(&ag_ctx, LSM6DSO_XL_ODR_OFF);
  /* Configure Sensor Hub to read LIS2MDL. */
  sh_cfg_read.slv_add = LIS2MDL_I2C_ADD; /* 8bit I2C address */
  sh_cfg_read.slv_subadd = reg;
  sh_cfg_read.slv_len = len;
  ret = lsm6dso_sh_slv_cfg_read(&ag_ctx, 0, &sh_cfg_read);
  lsm6dso_sh_slave_connected_set(&ag_ctx, LSM6DSO_SLV_0);
  /* Enable I2C Master and I2C master. */
  lsm6dso_sh_master_set(&ag_ctx, PROPERTY_ENABLE);
  /* Enable accelerometer to trigger Sensor Hub operation. */
  lsm6dso_xl_data_rate_set(&ag_ctx, LSM6DSO_XL_ODR_104Hz);
  /* Wait Sensor Hub operation flag set. */
  lsm6dso_acceleration_raw_get(&ag_ctx, data_raw_acceleration.i16bit);

  do {
    HAL_Delay(20);
    lsm6dso_xl_flag_data_ready_get(&ag_ctx, &drdy);
  } while (!drdy);

  do {
    //HAL_Delay(20);
    lsm6dso_sh_status_get(&ag_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL(trigger). */
  lsm6dso_sh_master_set(&ag_ctx, PROPERTY_DISABLE);
  lsm6dso_xl_data_rate_set(&ag_ctx, LSM6DSO_XL_ODR_OFF);
  /* Read SensorHub registers. */
  lsm6dso_sh_read_data_raw_get(&ag_ctx, data, len);
  return ret;
}
