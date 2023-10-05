/* -*- mode: c; c-file-style: "linux" -*- */
/*
* I2C Driver for Atmel ATSHA204 over I2C
*
* Copyright (C) 2014 Josh Datko, Cryptotronix, jbd@cryptotronix.com
*
* This program is free software; you can redistribute  it and/or modify it
* under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/kfifo.h>
#include <linux/uaccess.h>
#include <linux/crc16.h>
#include <linux/bitrev.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/printk.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/mutex.h>

#define ATSHA204_I2C_VERSION "0.2"
#define ATSHA204_SLEEP 0x01

/* 500us for ATECC108A */
#define ATECC108_W_HI (500)
/* 2.5ms for ATSHA204 */
#define ATSHA204_W_HI (2500)

struct atsha204_chip {
	struct device *dev;

	int dev_num;
	char devname[7];
	unsigned long is_open;

	struct i2c_client *client;
	struct miscdevice miscdev;
	struct mutex transaction_mutex;
};

struct atsha204_cmd_metadata {
	int expected_rec_len;
	int actual_rec_len;
	unsigned long usleep;
};

struct atsha204_buffer {
	u8 *ptr;
	int len;
};

struct atsha204_file_priv {
	struct atsha204_chip *chip;
	struct atsha204_cmd_metadata meta;

	struct atsha204_buffer buf;
};

static const struct i2c_device_id atsha204_i2c_id[] = {
	{"atsha204a", 0},
	{ }
};

void atsha204_set_params(struct atsha204_cmd_metadata *cmd,
			 int expected_rec_len,
			 unsigned long usleep)
{
	cmd->expected_rec_len = expected_rec_len;
	cmd->usleep = usleep;
}

struct atsha204_chip *global_chip = NULL;
static atomic_t atsha204_avail = ATOMIC_INIT(1);

u16 atsha204_crc16(const u8 *buf, const u8 len)
{
	u8 i;
	u16 crc16 = 0;

	for (i = 0; i < len; i++) {
		u8 shift;

		for (shift = 0x01; shift > 0x00; shift <<= 1) {
			u8 data_bit = (buf[i] & shift) ? 1 : 0;
			u8 crc_bit = crc16 >> 15;

			crc16 <<= 1;

			if ((data_bit ^ crc_bit) != 0)
				crc16 ^= 0x8005;
		}
	}

	return cpu_to_le16(crc16);
}

bool atsha204_crc16_matches(const u8 *buf, const u8 len, const u16 crc)
{
	u16 crc_calc = atsha204_crc16(buf, len);

	return (crc == crc_calc) ? true : false;
}

bool atsha204_check_rsp_crc16(const u8 *buf, const u8 len)
{
	const u16 *rec_crc = (const u16 *)&buf[len - 2];

	return atsha204_crc16_matches(buf, len - 2, cpu_to_le16(*rec_crc));
}

int atsha204_i2c_wakeup(const struct i2c_client *client)
{
	const struct device *dev = &client->dev;
	bool is_awake = false;
	int retval = -ENODEV;
	struct i2c_msg msg;
	u8 buf[4] = {0};
	unsigned short int try_con = 1;

	/* Set wake-up token message */
	msg.addr = 0;
	msg.flags = 0;
	msg.buf = buf;
	msg.len = 1;

	while (!is_awake) {
		dev_dbg(dev, "Send wake-up (%u)\n", try_con);
		/* To wake up the device you need to hold SDA low for at least
		 * 60us (tWLO). There is no way to do this explicitly in
		 * Linux, so attempt to send 0 to address 0. This will hold
		 * SDA low for 8 clock cycles (sending the address), which
		 * will work as long as the I2C clock speed is less than
		 * 133kHz.
		 */
		buf[0] = 0;
		i2c_transfer(client->adapter, &msg, 1);
		/* Delay for tWHI before reading the response. */
		mdelay(client->addr == 0x60 ? ATECC108_W_HI : ATSHA204_W_HI);
		/* Read the response. */
		if (4 == i2c_master_recv(client, buf, 4)) {
			dev_dbg(dev, "Chip is awake\n");
			is_awake = true;

			if (atsha204_check_rsp_crc16(buf, 4)) {
			    if (buf[1] == 0x11) {
				dev_dbg(dev, "Wakeup response OK\n");
				retval = 0;
			    } else {
				dev_err(dev, "Wakeup response incorrect\n");
				retval = EIO;
			    }
			} else {
				dev_err(dev, "Wakeup CRC failure\n");
				retval = EIO;
			}
		} else {
			if (++try_con >= 5) {
				dev_err(dev, "Wakeup failed. No Device\n");
				break;
			}
		}
	}

	return retval;
}

int atsha204_i2c_idle(const struct i2c_client *client)
{
	const struct device *dev = &client->dev;
	int rc;

	u8 idle_cmd[1] = {0x02};

	dev_dbg(dev, "Send idle\n");
	rc = i2c_master_send(client, idle_cmd, 1);

	return rc;
}

int atsha204_i2c_transaction(struct atsha204_chip *chip,
			     const u8 *to_send, size_t to_send_len,
			     struct atsha204_buffer *buf)


{
	int rc;
	u8 status_packet[4];
	u8 *recv_buf;
	int total_sleep = 60;
	int packet_len;

	mutex_lock(&chip->transaction_mutex);

	dev_dbg(chip->dev, "%s\n", "About to send to device.");

	/* Begin i2c transactions */
	rc = atsha204_i2c_wakeup(chip->client);
	if (rc)
		goto out;

	print_hex_dump_bytes("Sending : ", DUMP_PREFIX_OFFSET,
			     to_send, to_send_len);

	rc = i2c_master_send(chip->client, to_send, to_send_len);
	if (rc != to_send_len)
		goto out;

	/* Poll for the response */
	while (4 != i2c_master_recv(chip->client, status_packet, 4)
	       && total_sleep > 0){
		total_sleep = total_sleep - 4;
		msleep(4);
	}

	packet_len = status_packet[0];
	/* The device is awake and we don't want to hit the watchdog
	   timer, so don't allow sleeps here*/
	recv_buf = kmalloc(packet_len, GFP_ATOMIC);
	memcpy(recv_buf, status_packet, sizeof(status_packet));
	rc = i2c_master_recv(chip->client, recv_buf + 4, packet_len - 4);

	/* Store the entire packet. Other functions must check the CRC
	   and strip of the length byte */
	buf->ptr = recv_buf;
	buf->len = packet_len;

	dev_dbg(chip->dev, "%s\n", "Received from device.");
	print_hex_dump_bytes("Received: ", DUMP_PREFIX_OFFSET,
			     recv_buf, packet_len);

	atsha204_i2c_idle(chip->client);

	rc = to_send_len;
out:
	mutex_unlock(&chip->transaction_mutex);
	return rc;

}

int atsha204_i2c_validate_rsp(const struct atsha204_buffer *packet,
			      struct atsha204_buffer *rsp)
{
	int rc;

	if (packet->len < 4)
		goto out_bad_msg;
	else if (atsha204_check_rsp_crc16(packet->ptr, packet->len)) {
		rsp->ptr = packet->ptr + 1;
		rsp->len = packet->len - 3;
		rc = 0;
		goto out;
	} else {
		/* CRC failed */
	}

out_bad_msg:
	rc = -EBADMSG;
out:
	return rc;
}

int atsha204_i2c_sleep(const struct i2c_client *client)
{
	const struct device *dev = &client->dev;
	int retval;
	char to_send[1] = {ATSHA204_SLEEP};

	dev_dbg(dev, "Send sleep\n");
	retval = i2c_master_send(client, to_send, 1);
	if (retval == 1)
		retval = 0;
	else
		dev_err(dev, "Failed to sleep\n");

	return retval;

}

void atsha204_i2c_crc_command(u8 *cmd, int len)
{
	/* The command packet is:
	   [0x03] [Length=1 + command + CRC] [cmd] [crc]

	   The CRC is calculated over:
	   CRC([Len] [cmd]) */
	int crc_data_len = len - 2 - 1;
	u16 crc = atsha204_crc16(&cmd[1], crc_data_len);


	cmd[len - 2] = cpu_to_le16(crc) & 0xFF;
	cmd[len - 1] = cpu_to_le16(crc) >> 8;
}

int validate_write_size(const size_t count)
{
	const int MIN_SIZE = 4;
	/* Header and CRC occupy 4 bytes and the length is a one byte
	   value */
	const int MAX_SIZE = 255 - 4;
	int rc = -EMSGSIZE;

	if (count <= MAX_SIZE && count >= MIN_SIZE)
		rc = 0;

	return rc;

}

ssize_t atsha204_i2c_write(struct file *filep, const char __user *buf,
			   size_t count, loff_t *f_pos)
{
	struct atsha204_file_priv *priv = filep->private_data;
	struct atsha204_chip *chip = priv->chip;
	u8 *to_send;
	int rc;

	/* Add command byte + length + 2 byte crc */
	const int SEND_SIZE = count + 4;
	const u8 COMMAND_BYTE = 0x03;

	rc = validate_write_size(count);
	if (rc)
		return rc;

	to_send = kmalloc(SEND_SIZE, GFP_KERNEL);
	if (!to_send)
		return -ENOMEM;

	/* Write the header */
	to_send[0] = COMMAND_BYTE;
	/* Length byte = user size + crc size + length byte */
	to_send[1] = count + 2 + 1;

	if (copy_from_user(&to_send[2], buf, count)) {
		rc = -EFAULT;
		kfree(to_send);
		return rc;
	}

	atsha204_i2c_crc_command(to_send, SEND_SIZE);

	rc = atsha204_i2c_transaction(chip, to_send, SEND_SIZE,
				      &priv->buf);

	/* Return to the user the number of bytes that the
	   user provided, don't include the extra header / crc
	   bytes */
	if (SEND_SIZE == rc)
		rc = count;

	/* Reset the f_pos, which indicates the read position in the
	   buffer. Byte 1 points at the start of the data */
	*f_pos = 1;

	kfree(to_send);

	return rc;
}

ssize_t atsha204_i2c_read(struct file *filep, char __user *buf, size_t count,
			  loff_t *f_pos)
{
	struct atsha204_file_priv *priv = filep->private_data;
	struct atsha204_chip *chip = priv->chip;
	struct atsha204_buffer *r_buf = &priv->buf;
	ssize_t rc = 0;
	/* r_buf has 3 extra bytes that should not be returned to the
	   user. The first byte (length) and the last two (crc).
	   However, since f_pos is reset to 1 on write, only subtract
	   2 here.
	*/
	const int MAX_REC_LEN = r_buf->len - 2;

	/* Check the CRC on the rec buffer on the first read */
	if (*f_pos == 1 && !atsha204_check_rsp_crc16(r_buf->ptr, r_buf->len)) {
		rc = -EBADMSG;
		dev_err(chip->dev, "%s\n", "CRC on received buffer failed.");
		goto out;
	}

	if (*f_pos >= MAX_REC_LEN)
		goto out;

	if (*f_pos + count > MAX_REC_LEN)
		count = MAX_REC_LEN - *f_pos;

	rc = copy_to_user(buf, &r_buf->ptr[*f_pos], count);
	if (rc) {
		rc = -EFAULT;
	} else{
		*f_pos += count;
		rc = count;
	}

out:
	return rc;
}

int atsha204_i2c_open(struct inode *inode, struct file *filep)
{
	struct miscdevice *misc = filep->private_data;
	struct atsha204_chip *chip = container_of(misc, struct atsha204_chip,
						  miscdev);
	struct atsha204_file_priv *priv;

	if (!atomic_dec_and_test(&atsha204_avail)) {
		atomic_inc(&atsha204_avail);
		return -EBUSY;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (NULL == priv)
		return -ENOMEM;

	priv->chip = chip;

	filep->private_data = priv;

	filep->f_pos = 0;

	return 0;
}

int atsha204_i2c_release(struct inode *inode, struct file *filep)
{

	atomic_inc(&atsha204_avail);

	return 0;
}

static const struct attribute_group atsha204_dev_group;

int atsha204_sysfs_add_device(struct atsha204_chip *chip)
{
	int err;

	err = sysfs_create_group(&chip->dev->kobj,
				 &atsha204_dev_group);

	if (err)
		dev_err(chip->dev,
			"failed to create sysfs attributes, %d\n", err);
	return err;
}

void atsha204_sysfs_del_device(struct atsha204_chip *chip)
{
	sysfs_remove_group(&chip->dev->kobj, &atsha204_dev_group);
}

void atsha204_i2c_remove(struct i2c_client *client)

{
	const struct device *dev = &(client->dev);
	struct atsha204_chip *chip = dev_get_drvdata(dev);

	if (chip) {
		misc_deregister(&chip->miscdev);
		atsha204_sysfs_del_device(chip);
	}

	kfree(chip);

	global_chip = NULL;

	/* The device is in an idle state, where it keeps ephemeral
	 * memory. Wakeup the device and sleep it, which will cause it
	 * to clear its internal memory */

	atsha204_i2c_wakeup(client);
	atsha204_i2c_sleep(client);
}

MODULE_DEVICE_TABLE(i2c, atsha204_i2c_id);

static const struct file_operations atsha204_i2c_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.open = atsha204_i2c_open,
	.read = atsha204_i2c_read,
	.write = atsha204_i2c_write,
	.release = atsha204_i2c_release,
};

int atsha204_i2c_add_device(struct atsha204_chip *chip)
{
	int retval;

	chip->miscdev.fops = &atsha204_i2c_fops;
	chip->miscdev.minor = MISC_DYNAMIC_MINOR;

	chip->miscdev.name = chip->devname;
	chip->miscdev.parent = chip->dev;

	retval = misc_register(&chip->miscdev);
	if (retval != 0) {
		chip->miscdev.name = NULL;
		dev_err(chip->dev,
			"unable to misc_register %s, minor %d err=%d\n",
			chip->miscdev.name,
			chip->miscdev.minor,
			retval);
	}


	return retval;
}

struct atsha204_chip *atsha204_i2c_register_hardware(struct device *dev,
						     struct i2c_client *client)
{
	struct atsha204_chip *chip;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (chip == NULL)
		goto out_null;

	chip->dev_num = 0;
	scnprintf(chip->devname, sizeof(chip->devname), "%s%d",
		  "atsha", chip->dev_num);

	chip->dev = get_device(dev);
	dev_set_drvdata(dev, chip);

	chip->client = client;

	mutex_init(&chip->transaction_mutex);

	if (atsha204_i2c_add_device(chip)) {
		dev_err(dev, "%s\n", "Failed to add device");
		goto put_device;
	}

	global_chip = chip;

	return chip;

put_device:
	put_device(chip->dev);

	kfree(chip);
out_null:
	return NULL;
}

int atsha204_i2c_probe(struct i2c_client *client)
{
	int result = -1;
	struct device *dev = &client->dev;
	struct atsha204_chip *chip;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	result = atsha204_i2c_wakeup(client);
	if (result == 0) {
		atsha204_i2c_idle(client);

		chip = atsha204_i2c_register_hardware(dev, client);
		if (chip == NULL)
			return -ENODEV;

		global_chip = chip;
		result = atsha204_sysfs_add_device(chip);
	} else{
		dev_err(dev, "Device failed to wake\n");
	}

	return result;
}

int atsha204_i2c_read4(struct atsha204_chip *chip, u8 *read_buf,
		       const u16 addr, const u8 param1)
{
	u8 read_cmd[8] = {0};
	u16 crc;
	struct atsha204_buffer rsp, msg;
	int rc, validate_status;

	read_cmd[0] = 0x03; /* Command byte */
	read_cmd[1] = 0x07; /* length */
	read_cmd[2] = 0x02; /* Read command opcode */
	read_cmd[3] = param1;
	read_cmd[4] = cpu_to_le16(addr) & 0xFF;
	read_cmd[5] = cpu_to_le16(addr) >> 8;

	crc = atsha204_crc16(&read_cmd[1], 5);


	read_cmd[6] = cpu_to_le16(crc) & 0xFF;
	read_cmd[7] = cpu_to_le16(crc) >> 8;

	rc = atsha204_i2c_transaction(chip, read_cmd,
				      sizeof(read_cmd), &rsp);

	if (sizeof(read_cmd) == rc) {
		validate_status = atsha204_i2c_validate_rsp(&rsp, &msg);
		if (0 == validate_status) {
			memcpy(read_buf, msg.ptr, msg.len);
			kfree(rsp.ptr);
			rc = msg.len;
		} else
			rc = validate_status;

	}

	return rc;
}

static ssize_t serial_show(struct device *dev,
                           struct device_attribute *attr,
                           char *buf)
{
        struct atsha204_chip *chip = dev_get_drvdata(dev);
        u8 param1 = 1; /* indicates otp region */
        char *str = buf;

        u8 otp[4] = {0};

        if(4 != atsha204_i2c_read4(chip, &otp[0], 0, param1)){
            str += sprintf(str, "N/A\n");
        } else {
            u32 serial = 0;
            serial = ((u32)(otp[0])<<24) | ((u32)(otp[1])<<16) |
                     ((u32)(otp[2])<<8)  | (u32)otp[3];
            str += sprintf(str, "%i ", serial);
        }

        return str - buf;
}
/*static DEVICE_ATTR_RO(serial);*/
struct device_attribute dev_attr_serial = __ATTR_RO(serial);

static ssize_t otp_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct atsha204_chip *chip = dev_get_drvdata(dev);
	int i;
	u16 bytes, word_addr;
	bool keep_going = true;
	u8 param1 = 1; /* indicates otp region */
	char *str = buf;

	u8 otp[64] = {0};

	for (bytes = 0; bytes < sizeof(otp) && keep_going; bytes += 4) {
		word_addr = bytes / 4;
		if(4 != atsha204_i2c_read4(chip,
					   &otp[bytes],
					   word_addr, param1)){
			keep_going = false;
		}
	}

	for (i = 0; i < bytes; i++) {
		str += sprintf(str, "%02X ", otp[i]);
		if((i + 1) % 4 == 0)
			str += sprintf(str, "\n");
	}

	return str - buf;
}
/*static DEVICE_ATTR_RO(otp);*/
struct device_attribute dev_attr_otp = __ATTR_RO(otp);

static struct attribute *atsha204_dev_attrs[] = {
	&dev_attr_otp.attr,
	&dev_attr_serial.attr,
	NULL,
};

static const struct attribute_group atsha204_dev_group = {
	.attrs = atsha204_dev_attrs,
};

static struct i2c_driver atsha204_i2c_driver = {
	.driver = {
		.name = "atsha204-i2c",
		.owner = THIS_MODULE,
	},
	.probe_new = atsha204_i2c_probe,
	.remove = atsha204_i2c_remove,
	.id_table = atsha204_i2c_id,
};
module_i2c_driver(atsha204_i2c_driver);

MODULE_AUTHOR("Josh Datko <jbd@cryptotronix.com");
MODULE_DESCRIPTION("Atmel ATSHA204 driver");
MODULE_VERSION(ATSHA204_I2C_VERSION);
MODULE_LICENSE("GPL");
