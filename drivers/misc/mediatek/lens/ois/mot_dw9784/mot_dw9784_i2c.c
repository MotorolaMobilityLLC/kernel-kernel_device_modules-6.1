#include "mot_dw9784_i2c.h"

#define DW9784_DEBUG
#ifdef DW9784_DEBUG
#define LOG_INF(format, args...) \
	printk(" [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

#define DW9784_OIS_I2C_SLAVE_ADDR 0xE4

static struct i2c_client *dw9784_i2c_client = NULL;

void dw9784_set_i2c_client(struct i2c_client *i2c_client)
{
	dw9784_i2c_client = i2c_client;
}

int ois_i2c_rd_u16(struct i2c_client *i2c_client, u16 reg, u16 *val)
{
	int ret;
	u8 buf[2];
	struct i2c_msg msg[2];
	u16 addr = i2c_client->addr;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	msg[0].addr = addr;
	msg[0].flags = i2c_client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);
	msg[1].addr  = addr;
	msg[1].flags = i2c_client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 2;
	ret = i2c_transfer(i2c_client->adapter, msg, 2);
	if (ret < 0) {
		LOG_INF("i2c transfer failed (%d)\n", ret);
		return ret;
	}
	*val = ((u16)buf[0] << 8) | buf[1];

	return 0;
}
/*
int ois_i2c_rd_u32(struct i2c_client *i2c_client, u16 reg, u32 *val)
{
	int ret;
	u8 buf[4];
	struct i2c_msg msg[2];
	u16 addr = i2c_client->addr;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	msg[0].addr = addr;
	msg[0].flags = i2c_client->flags;
	msg[0].buf = buf;
	msg[0].len = 2;
	msg[1].addr  = addr;
	msg[1].flags = i2c_client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 4;
	ret = i2c_transfer(i2c_client->adapter, msg, 2);
	if (ret < 0) {
		LOG_INF("i2c transfer failed (%d)\n", ret);
		return ret;
	}
	*val = ((u32)buf[0] << 24) | ((u32)buf[1] << 16) |
	  ((u32)buf[2] << 8) | buf[3];

	return 0;
}
*/
int ois_i2c_wr_u16(struct i2c_client *i2c_client, u16 reg, u16 val)
{
	int ret;
	u8 buf[4];
	struct i2c_msg msg;
	u16 addr = i2c_client->addr;

	LOG_INF("OIS write 0x%x: 0x%x (%d)\n", reg, val, val);

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val >> 8;
	buf[3] = val & 0xff;
	msg.addr = addr;
	msg.flags = i2c_client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);
	ret = i2c_transfer(i2c_client->adapter, &msg, 1);
	if (ret < 0) {
		LOG_INF("i2c transfer failed (%d)\n", ret);
		return ret;
	}
	return 0;
}

void I2C_OPERATION_CHECK(int val)
{
	if (val) {
		LOG_INF("%s I2C failure: %d",
			__func__, val);
	}
}

void i2c_block_write_reg(unsigned short addr,unsigned short *fwContentPtr,int size)
{
	char puSendCmd[DATPKT_SIZE*2 + 2];
	int IDX = 0;
	int tosend = 0;
	u16 data;
	dw9784_i2c_client->addr = DW9784_OIS_I2C_SLAVE_ADDR >> 1;
	puSendCmd[tosend++] = (char)(addr >> 8);
	puSendCmd[tosend++] = (char)(addr & 0xff);
	while(size > IDX) {
		data = fwContentPtr[IDX];
		puSendCmd[tosend++] = (char)(data >> 8);
		puSendCmd[tosend++] = (char)(data & 0xff);
		IDX++;
		if(IDX == size) {
			int i4RetValue = 0;
			struct i2c_msg msgs;
			msgs.addr  = DW9784_OIS_I2C_SLAVE_ADDR >> 1;
			msgs.flags = 0;
			msgs.len   = (DATPKT_SIZE * 2 + 2);
			msgs.buf   = puSendCmd;
			i4RetValue = i2c_transfer(dw9784_i2c_client->adapter, &msgs, 1);
			if (i4RetValue != 1) {
				LOG_INF("I2C send failed!!\n");
				return;
			}
			return;
		}
	}
}

void i2c_block_read_reg(unsigned short addr,unsigned short *temp,int size)
{
	int i4RetValue = 0;
	char puSendCmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };
	struct i2c_msg msgs[2];
	dw9784_i2c_client->addr = DW9784_OIS_I2C_SLAVE_ADDR >> 1;
	msgs[0].addr  = DW9784_OIS_I2C_SLAVE_ADDR >> 1;
	msgs[0].flags = 0;
	msgs[0].len   = 2;
	msgs[0].buf   = puSendCmd;

	msgs[1].addr  = DW9784_OIS_I2C_SLAVE_ADDR >> 1;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len   = 2 * size;
	msgs[1].buf   = (u8*)temp;

	i4RetValue = i2c_transfer(dw9784_i2c_client->adapter, msgs, 2);
	if (i4RetValue != 2) {
		LOG_INF("I2C send failed!!\n");
		return;
	}
	return;
}
