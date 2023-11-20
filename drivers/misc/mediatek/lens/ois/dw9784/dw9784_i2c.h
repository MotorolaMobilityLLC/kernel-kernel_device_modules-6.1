#ifndef __DW9784_I2C_H__
#define __DW9784_I2C_H__
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

void dw9784_set_client(struct i2c_client *i2c_client);
int ois_i2c_rd_u16(struct i2c_client *i2c_client, u16 reg, u16 *val);
int ois_i2c_wr_u16(struct i2c_client *i2c_client, u16 reg, u16 val);
void i2c_block_write_reg(unsigned short addr,unsigned short *fwContentPtr,int size);
void i2c_block_read_reg(unsigned short addr,unsigned short *temp,int size);
void I2C_OPERATION_CHECK(int val);
#define DATPKT_SIZE 256
#endif
