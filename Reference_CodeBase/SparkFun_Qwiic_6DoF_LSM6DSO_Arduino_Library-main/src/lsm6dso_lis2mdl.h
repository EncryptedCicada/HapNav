#include <stdint.h>
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>

typedef struct
{
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Component optional fields **/
  stmdev_mdelay_ptr   mdelay;
  /** Customizable optional pointer **/
  void *handle;

  /** private data **/
  void *priv_data;
} stmdev_ctx_t;

typedef struct
{
  uint8_t   slv_add;
  uint8_t   slv_subadd;
  uint8_t   slv_len;
} lsm6dso_sh_cfg_read_t;