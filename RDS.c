/*
 * drivers/media/radio/radio-bcm2048.c
 *
 * Driver for I2C Broadcom BCM2048 FM Radio Receiver:
 *
 * Copyright (C) Nokia Corporation
 * Contact: Eero Nurkkala <ext-eero.nurkkala@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

//#include <termios.h>
//#include <fcntl.h>
#include <errno.h>
//#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>

/* driver definitions */
#define BCM2048_DRIVER_AUTHOR	"Eero Nurkkala <ext-eero.nurkkala@nokia.com>"
#define BCM2048_DRIVER_NAME	BCM2048_NAME
#define BCM2048_DRIVER_VERSION	KERNEL_VERSION(0, 0, 1)
#define BCM2048_DRIVER_CARD	"Broadcom bcm2048 FM Radio Receiver"
#define BCM2048_DRIVER_DESC	"I2C driver for BCM2048 FM Radio Receiver"

/* I2C Control Registers */
#define BCM2048_I2C_FM_RDS_SYSTEM	0x00
#define BCM2048_I2C_FM_CTRL		0x01
#define BCM2048_I2C_RDS_CTRL0		0x02
#define BCM2048_I2C_RDS_CTRL1		0x03
#define BCM2048_I2C_FM_AUDIO_PAUSE	0x04
#define BCM2048_I2C_FM_AUDIO_CTRL0	0x05
#define BCM2048_I2C_FM_AUDIO_CTRL1	0x06
#define BCM2048_I2C_FM_SEARCH_CTRL0	0x07
#define BCM2048_I2C_FM_SEARCH_CTRL1	0x08
#define BCM2048_I2C_FM_SEARCH_TUNE_MODE	0x09
#define BCM2048_I2C_FM_FREQ0		0x0a
#define BCM2048_I2C_FM_FREQ1		0x0b
#define BCM2048_I2C_FM_AF_FREQ0		0x0c
#define BCM2048_I2C_FM_AF_FREQ1		0x0d
#define BCM2048_I2C_FM_CARRIER		0x0e
#define BCM2048_I2C_FM_RSSI		0x0f
#define BCM2048_I2C_FM_RDS_MASK0	0x10
#define BCM2048_I2C_FM_RDS_MASK1	0x11
#define BCM2048_I2C_FM_RDS_FLAG0	0x12
#define BCM2048_I2C_FM_RDS_FLAG1	0x13
#define BCM2048_I2C_RDS_WLINE		0x14
#define BCM2048_I2C_RDS_BLKB_MATCH0	0x16
#define BCM2048_I2C_RDS_BLKB_MATCH1	0x17
#define BCM2048_I2C_RDS_BLKB_MASK0	0x18
#define BCM2048_I2C_RDS_BLKB_MASK1	0x19
#define BCM2048_I2C_RDS_PI_MATCH0	0x1a
#define BCM2048_I2C_RDS_PI_MATCH1	0x1b
#define BCM2048_I2C_RDS_PI_MASK0	0x1c
#define BCM2048_I2C_RDS_PI_MASK1	0x1d
#define BCM2048_I2C_SPARE1		0x20
#define BCM2048_I2C_SPARE2		0x21
#define BCM2048_I2C_FM_RDS_REV		0x28
#define BCM2048_I2C_SLAVE_CONFIGURATION	0x29
#define BCM2048_I2C_RDS_DATA		0x80
#define BCM2048_I2C_FM_BEST_TUNE_MODE	0x90

/* BCM2048_I2C_FM_RDS_SYSTEM */
#define BCM2048_FM_ON			0x01
#define BCM2048_RDS_ON			0x02

/* BCM2048_I2C_FM_CTRL */
#define BCM2048_BAND_SELECT			0x01
#define BCM2048_STEREO_MONO_AUTO_SELECT		0x02
#define BCM2048_STEREO_MONO_MANUAL_SELECT	0x04
#define BCM2048_STEREO_MONO_BLEND_SWITCH	0x08
#define BCM2048_HI_LO_INJECTION			0x10

/* BCM2048_I2C_RDS_CTRL0 */
#define BCM2048_RBDS_RDS_SELECT		0x01
#define BCM2048_FLUSH_FIFO		0x02

/* BCM2048_I2C_FM_AUDIO_PAUSE */
#define BCM2048_AUDIO_PAUSE_RSSI_TRESH	0x0f
#define BCM2048_AUDIO_PAUSE_DURATION	0xf0

/* BCM2048_I2C_FM_AUDIO_CTRL0 */
#define BCM2048_RF_MUTE			0x01
#define BCM2048_MANUAL_MUTE		0x02
#define BCM2048_DAC_OUTPUT_LEFT		0x04
#define BCM2048_DAC_OUTPUT_RIGHT	0x08
#define BCM2048_AUDIO_ROUTE_DAC		0x10
#define BCM2048_AUDIO_ROUTE_I2S		0x20
#define BCM2048_DE_EMPHASIS_SELECT	0x40
#define BCM2048_AUDIO_BANDWIDTH_SELECT	0x80

/* BCM2048_I2C_FM_SEARCH_CTRL0 */
#define BCM2048_SEARCH_RSSI_THRESHOLD	0x7f
#define BCM2048_SEARCH_DIRECTION	0x80

/* BCM2048_I2C_FM_SEARCH_TUNE_MODE */
#define BCM2048_FM_AUTO_SEARCH		0x03

/* BCM2048_I2C_FM_RSSI */
#define BCM2048_RSSI_VALUE		0xff

/* BCM2048_I2C_FM_RDS_MASK0 */
/* BCM2048_I2C_FM_RDS_MASK1 */
#define BCM2048_FM_FLAG_SEARCH_TUNE_FINISHED	0x01
#define BCM2048_FM_FLAG_SEARCH_TUNE_FAIL	0x02
#define BCM2048_FM_FLAG_RSSI_LOW		0x04
#define BCM2048_FM_FLAG_CARRIER_ERROR_HIGH	0x08
#define BCM2048_FM_FLAG_AUDIO_PAUSE_INDICATION	0x10
#define BCM2048_FLAG_STEREO_DETECTED		0x20
#define BCM2048_FLAG_STEREO_ACTIVE		0x40

/* BCM2048_I2C_RDS_DATA */
#define BCM2048_SLAVE_ADDRESS			0x3f
#define BCM2048_SLAVE_ENABLE			0x80

/* BCM2048_I2C_FM_BEST_TUNE_MODE */
#define BCM2048_BEST_TUNE_MODE			0x80

#define BCM2048_FM_FLAG_SEARCH_TUNE_FINISHED	0x01
#define BCM2048_FM_FLAG_SEARCH_TUNE_FAIL	0x02
#define BCM2048_FM_FLAG_RSSI_LOW		0x04
#define BCM2048_FM_FLAG_CARRIER_ERROR_HIGH	0x08
#define BCM2048_FM_FLAG_AUDIO_PAUSE_INDICATION	0x10
#define BCM2048_FLAG_STEREO_DETECTED		0x20
#define BCM2048_FLAG_STEREO_ACTIVE		0x40

#define BCM2048_RDS_FLAG_FIFO_WLINE		0x02
#define BCM2048_RDS_FLAG_B_BLOCK_MATCH		0x08
#define BCM2048_RDS_FLAG_SYNC_LOST		0x10
#define BCM2048_RDS_FLAG_PI_MATCH		0x20

#define BCM2048_RDS_MARK_END_BYTE0		0x7C
#define BCM2048_RDS_MARK_END_BYTEN		0xFF

#define BCM2048_FM_FLAGS_ALL	(FM_FLAG_SEARCH_TUNE_FINISHED | \
				 FM_FLAG_SEARCH_TUNE_FAIL | \
				 FM_FLAG_RSSI_LOW | \
				 FM_FLAG_CARRIER_ERROR_HIGH | \
				 FM_FLAG_AUDIO_PAUSE_INDICATION | \
				 FLAG_STEREO_DETECTED | FLAG_STEREO_ACTIVE)

#define BCM2048_RDS_FLAGS_ALL	(RDS_FLAG_FIFO_WLINE | \
				 RDS_FLAG_B_BLOCK_MATCH | \
				 RDS_FLAG_SYNC_LOST | RDS_FLAG_PI_MATCH)

#define BCM2048_DEFAULT_TIMEOUT		1500
#define BCM2048_AUTO_SEARCH_TIMEOUT	3000


#define BCM2048_FREQDEV_UNIT		10000
#define BCM2048_FREQV4L2_MULTI		625
#define dev_to_v4l2(f)	((f * BCM2048_FREQDEV_UNIT) / BCM2048_FREQV4L2_MULTI)
#define v4l2_to_dev(f)	((f * BCM2048_FREQV4L2_MULTI) / BCM2048_FREQDEV_UNIT)

#define msb(x)                  ((uint8_t)((uint16_t) x >> 8))
#define lsb(x)                  ((uint8_t)((uint16_t) x &  0x00FF))
#define compose_uint16_t(msb, lsb)	(((uint16_t)msb << 8) | lsb)

#define BCM2048_DEFAULT_POWERING_DELAY	20
#define BCM2048_DEFAULT_REGION		0x02
#define BCM2048_DEFAULT_MUTE		0x01
#define BCM2048_DEFAULT_RSSI_THRESHOLD	0x64
#define BCM2048_DEFAULT_RDS_WLINE	0x7E

#define BCM2048_FM_SEARCH_INACTIVE	0x00
#define BCM2048_FM_PRE_SET_MODE		0x01
#define BCM2048_FM_AUTO_SEARCH_MODE	0x02
#define BCM2048_FM_AF_JUMP_MODE		0x03

#define BCM2048_FREQUENCY_BASE		64000

#define BCM2048_POWER_ON		0x01
#define BCM2048_POWER_OFF		0x00

#define BCM2048_ITEM_ENABLED		0x01
#define BCM2048_SEARCH_DIRECTION_UP	0x01

#define BCM2048_DE_EMPHASIS_75us	75
#define BCM2048_DE_EMPHASIS_50us	50

#define BCM2048_SCAN_FAIL		0x00
#define BCM2048_SCAN_OK			0x01

#define BCM2048_FREQ_ERROR_FLOOR	-20
#define BCM2048_FREQ_ERROR_ROOF		20

/* -60 dB is reported as full signal strenght */
#define BCM2048_RSSI_LEVEL_BASE		-60
#define BCM2048_RSSI_LEVEL_ROOF		-100
#define BCM2048_RSSI_LEVEL_ROOF_NEG	100
#define BCM2048_SIGNAL_MULTIPLIER	(0xFFFF / \
					 (BCM2048_RSSI_LEVEL_ROOF_NEG + \
					  BCM2048_RSSI_LEVEL_BASE))

#define BCM2048_RDS_FIFO_DUPLE_SIZE	0x03
#define BCM2048_RDS_CRC_MASK		0x0F
#define BCM2048_RDS_CRC_NONE		0x00
#define BCM2048_RDS_CRC_MAX_2BITS	0x04
#define BCM2048_RDS_CRC_LEAST_2BITS	0x08
#define BCM2048_RDS_CRC_UNRECOVARABLE	0x0C

#define BCM2048_RDS_BLOCK_MASK		0xF0
#define BCM2048_RDS_BLOCK_A		0x00
#define BCM2048_RDS_BLOCK_B		0x10
#define BCM2048_RDS_BLOCK_C		0x20
#define BCM2048_RDS_BLOCK_D		0x30
#define BCM2048_RDS_BLOCK_C_SCORED	0x40
#define BCM2048_RDS_BLOCK_E		0x60

#define BCM2048_RDS_RT			0x20
#define BCM2048_RDS_PS			0x00

#define BCM2048_RDS_GROUP_AB_MASK	0x08
#define BCM2048_RDS_GROUP_A		0x00
#define BCM2048_RDS_GROUP_B		0x08

#define BCM2048_RDS_RT_AB_MASK		0x10
#define BCM2048_RDS_RT_A		0x00
#define BCM2048_RDS_RT_B		0x10
#define BCM2048_RDS_RT_INDEX		0x0F

#define BCM2048_RDS_PS_INDEX		0x03

struct rds_info {
	uint8_t rds_group_b_cnt;
	uint8_t rds_group_0_b;
	uint8_t rds_group_0_TA;
	uint8_t rds_group_0_MS;
	uint8_t rds_group_0_DI;
	uint8_t rds_group_2_b;
	uint8_t rds_group_2_rt_ab;
	uint16_t rds_pi;
#define BCM2048_MAX_RDS_RT (64 + 1)
	uint8_t rds_rt[BCM2048_MAX_RDS_RT];
	uint8_t rds_rt_group_b;
	uint8_t rds_rt_ab;
#define BCM2048_MAX_RDS_PS (8 + 1)
	uint8_t rds_ps[BCM2048_MAX_RDS_PS];
	uint8_t rds_ps_group;
	uint8_t rds_ps_group_cnt;
#define BCM2048_MAX_RDS_RADIO_TEXT 255
	uint8_t radio_text[BCM2048_MAX_RDS_RADIO_TEXT + 3];
	uint8_t text_len;
	uint8_t rds_pty;
};

char *program_DI[8] = {
  "PTY static",
  "PTY dynamic or EON",
  "not compressed transmission",
  "compressed transmission",
  "not artificial head transmission",
  "artificial head transmission",
  "mono broadcasting",
  "stereo broadcasting",
};
  
  
char *program_type[32] = {
  "No programme type", 
  "News",
  "Current affairs",
  "Information",
  "Sport",
  "Education",
  "Drama",
  "Culture",
  "Science",
  "Varied",
  "Pop music",
  "Rock music",
  "Easy listening",
  "Light classical",
  "Serious classical",
  "Other music",
  "Weather",
  "Finance",
  "Children’s programmes",
  "Social affairs",
  "Religion",
  "Phone-in",
  "Travel",
  "Leisure",
  "Jazz music",
  "Country music",
  "National music",
  "Oldies music",
  "Folk music",
  "Documentary",
  "Alarm test",
  "Alarm"
};

struct bcm2048_device {
	struct rds_info rds_info;
	uint16_t frequency;
	uint8_t cache_fm_rds_system;
	uint8_t cache_fm_ctrl;
	uint8_t cache_fm_audio_ctrl0;
	uint8_t cache_fm_search_ctrl0;
	uint8_t power_state;
	uint8_t rds_state;
	uint8_t fifo_size;
	uint8_t scan_state;
	uint8_t mute_state;
};

static void reset_ps(struct bcm2048_device *bdev)
{
	memset(bdev->rds_info.rds_ps,0,sizeof(bdev->rds_info.rds_ps));
}

static void reset_rt(struct bcm2048_device *bdev)
{
	memset(bdev->rds_info.rds_rt,0,sizeof(bdev->rds_info.rds_rt));
}

static int hci_read(int reg, int values, uint8_t *buf)
{
  FILE* fp;
  int i, j;
  char s1[100] = "hcitool cmd 0x3f 0x15 ";
  char stemp[10] = "";
  char starget[100] = "";
  char reading[200] = "";
  char *preading;
  char *pstarget = starget;
  char *returnv;
  int newline = 1;
  int line = 1;

  sprintf(stemp, "0x%x ", reg);
  pstarget=strcat(s1, stemp);

  sprintf(stemp, "0x%x ", 1);
  pstarget=strcat(pstarget, stemp);

  sprintf(stemp, "0x%x ", values);
  pstarget = strcat(pstarget, stemp);

  fp = popen(pstarget,"r");

  if(!fp){
    printf("Could not open pipe for output.\n");
    return 1;
  }

  // Grab data from process execution
  // Skip the first 3 lines
  fgets(reading, 200 , fp);
  fgets(reading, 200 , fp);
  fgets(reading, 200 , fp);
  
  for (i = 0; i < values;){
    if (newline == 1) {
      fgets(reading, 200 , fp);
      newline = 0;
    }
    if (line > 1){ // >1st line has 20 values
      preading = reading;
      for (j = 0; j < 20; j++){
	if (i < values){
	  buf[i] = (uint8_t)(strtoul(preading, &preading, 16));
	  i++;
	}
      }
      newline = 1;
    }
    if (line == 1){
      preading = reading + 20;
      for (j = 0; j < 14; j++){ // 1st line has only 14 values
        if (i < values){
	  buf[i] = (uint8_t)(strtoul(preading, &preading, 16));
	  i++;
	}
      }
      newline = 1;
      line++;
    }
  }
  if (pclose(fp) != 0) printf(" Error: Failed to close command stream \n");
  return 0;
}

static int hci_write(int reg, int values, uint8_t *buf)
{
  int returnval = 0, i = 0;
  char s1[100] = "hcitool cmd 0x3f 0x15 ";
  char stemp[10] = "";
  char starget[100] = "";
  char *pstarget = starget;

  sprintf(stemp, "0x%x ", reg);
  pstarget = strcat(s1, stemp);

  sprintf(stemp, "0x%x ", 0);
  pstarget = strcat(pstarget, stemp);

  for (i=0; i < values; i++) {
    sprintf(stemp, "0x%x ", buf[i]);
    pstarget = strcat(pstarget, stemp);
  }
  
  sprintf(stemp, "|grep wow");
  pstarget = strcat(pstarget, stemp);

  returnval = system(pstarget);
  return returnval;
}

static int bcm2048_get_rds_rt(struct bcm2048_device *bdev, char *data)
{
	int err = 0, i, j = 0, ce = 0, cr = 0;
	char data_buffer[BCM2048_MAX_RDS_RT+1];

	if (!bdev->rds_info.text_len) {
		err = -EINVAL;
		goto unlock;
	}

	for (i = 0; i < BCM2048_MAX_RDS_RT; i++) {
		if (bdev->rds_info.rds_rt[i]) {
			ce = i;
			/* Skip the carriage return */
			if (bdev->rds_info.rds_rt[i] != 0x0d) {
				data_buffer[j++] = bdev->rds_info.rds_rt[i];
			} else {
				cr = i;
				break;
			}
		}
	}

	if (j <= BCM2048_MAX_RDS_RT)
		data_buffer[j] = 0;

	for (i = 0; i < BCM2048_MAX_RDS_RT; i++) {
		if (!bdev->rds_info.rds_rt[i]) {
			if (cr && (i < cr)) {
				err = -EBUSY;
				goto unlock;
			}
			if (i < ce) {
				if (cr && (i >= cr))
					break;
				err = -EBUSY;
				goto unlock;
			}
		}
	}

	memcpy(data, data_buffer, sizeof(data_buffer));

unlock:
	return err;
}

static int bcm2048_get_rds_ps(struct bcm2048_device *bdev, char *data)
{
	int err = 0, i, j = 0;
	char data_buffer[BCM2048_MAX_RDS_PS+1];

	if (!bdev->rds_info.text_len) {
		err = -EINVAL;
		goto unlock;
	}

	for (i = 0; i < BCM2048_MAX_RDS_PS; i++) {
		if (bdev->rds_info.rds_ps[i]) {
			data_buffer[j++] = bdev->rds_info.rds_ps[i];
		} else {
			if (i < (BCM2048_MAX_RDS_PS - 1)) {
				err = -EBUSY;
				goto unlock;
			}
		}
	}

	if (j <= BCM2048_MAX_RDS_PS)
		data_buffer[j] = 0;

	memcpy(data, data_buffer, sizeof(data_buffer));

unlock:
	return err;
}

static void bcm2048_parse_rds_pi(struct bcm2048_device *bdev)
{
	int i, cnt = 0;
	uint16_t pi;

	for (i = 0; i < bdev->fifo_size; i += BCM2048_RDS_FIFO_DUPLE_SIZE) {

		/* Block A match, only data without crc errors taken */
		if (bdev->rds_info.radio_text[i] == BCM2048_RDS_BLOCK_A) {

			pi = ((bdev->rds_info.radio_text[i+1] << 8) +
				bdev->rds_info.radio_text[i+2]);

			if (!bdev->rds_info.rds_pi) {
				bdev->rds_info.rds_pi = pi;
				return;
			}
			if (pi != bdev->rds_info.rds_pi) {
				cnt++;
				if (cnt > 3) {
					bdev->rds_info.rds_pi = pi;
					cnt = 0;
				}
			} else {
				cnt = 0;
			}
		}
	}
}

static int bcm2048_rds_block_crc(struct bcm2048_device *bdev, int i)
{
	return bdev->rds_info.radio_text[i] & BCM2048_RDS_CRC_MASK;
}

static void bcm2048_parse_rds_rt_block(struct bcm2048_device *bdev, int i,
					int index, int crc)
{
	/* Good data will overwrite poor data */
	if (crc) {
		if (!bdev->rds_info.rds_rt[index])
			bdev->rds_info.rds_rt[index] =
				bdev->rds_info.radio_text[i+1];
		if (!bdev->rds_info.rds_rt[index+1])
			bdev->rds_info.rds_rt[index+1] =
				bdev->rds_info.radio_text[i+2];
	} else {
		bdev->rds_info.rds_rt[index] = bdev->rds_info.radio_text[i+1];
		bdev->rds_info.rds_rt[index+1] =
			bdev->rds_info.radio_text[i+2];
	}
}

static int bcm2048_parse_rt_match_b(struct bcm2048_device *bdev, int i)
{
	int crc, rt_id, rt_group_b, rt_ab, index = 0;

	crc = bcm2048_rds_block_crc(bdev, i);

	if (crc == BCM2048_RDS_CRC_UNRECOVARABLE)
		return -EIO;

	if ((bdev->rds_info.radio_text[i] & BCM2048_RDS_BLOCK_MASK) ==
		BCM2048_RDS_BLOCK_B) {

		rt_id = (bdev->rds_info.radio_text[i+1] &
			BCM2048_RDS_BLOCK_MASK);
		rt_group_b = bdev->rds_info.radio_text[i+1] &
			BCM2048_RDS_GROUP_AB_MASK;
		rt_ab = bdev->rds_info.radio_text[i+2] &
				BCM2048_RDS_RT_AB_MASK;

		if (rt_group_b != bdev->rds_info.rds_rt_group_b) {
			memset(bdev->rds_info.rds_rt, 0,
				sizeof(bdev->rds_info.rds_rt));
			bdev->rds_info.rds_rt_group_b = rt_group_b;
		}

		if (rt_id == BCM2048_RDS_RT) {
			/* A to B or (vice versa), means: clear screen */
			if (rt_ab != bdev->rds_info.rds_rt_ab) {
				memset(bdev->rds_info.rds_rt, 0,
					sizeof(bdev->rds_info.rds_rt));
				bdev->rds_info.rds_rt_ab = rt_ab;
			}

			index = bdev->rds_info.radio_text[i+2] &
					BCM2048_RDS_RT_INDEX;

			if (bdev->rds_info.rds_rt_group_b)
				index <<= 1;
			else
				index <<= 2;

			return index;
		}
	}

	return -EIO;
}

static int bcm2048_parse_rt_match_c(struct bcm2048_device *bdev, int i,
					int index)
{
	int crc;

	crc = bcm2048_rds_block_crc(bdev, i);

	if (crc == BCM2048_RDS_CRC_UNRECOVARABLE)
		return 0;

	//BUG_ON((index+2) >= BCM2048_MAX_RDS_RT);

	if ((bdev->rds_info.radio_text[i] & BCM2048_RDS_BLOCK_MASK) ==
		BCM2048_RDS_BLOCK_C) {
		if (bdev->rds_info.rds_rt_group_b)
			return 1;
		bcm2048_parse_rds_rt_block(bdev, i, index, crc);
		return 1;
	}

	return 0;
}

static void bcm2048_parse_rt_match_d(struct bcm2048_device *bdev, int i,
					int index)
{
	int crc;

	crc = bcm2048_rds_block_crc(bdev, i);

	if (crc == BCM2048_RDS_CRC_UNRECOVARABLE)
		return;

	//BUG_ON((index+4) >= BCM2048_MAX_RDS_RT);

	if ((bdev->rds_info.radio_text[i] & BCM2048_RDS_BLOCK_MASK) ==
		BCM2048_RDS_BLOCK_D)
		bcm2048_parse_rds_rt_block(bdev, i, index+2, crc);
}

static int bcm2048_parse_rds_rt(struct bcm2048_device *bdev)
{
	int i, index = 0, crc, match_b = 0, match_c = 0, match_d = 0;
	char rt_text[BCM2048_MAX_RDS_RT +1];

	for (i = 0; i < bdev->fifo_size; i += BCM2048_RDS_FIFO_DUPLE_SIZE) {

		if (match_b) {
			match_b = 0;
			index = bcm2048_parse_rt_match_b(bdev, i);
			if (index >= 0 && index <= (BCM2048_MAX_RDS_RT - 5))
				match_c = 1;
			continue;
		} else if (match_c) {
			match_c = 0;
			if (bcm2048_parse_rt_match_c(bdev, i, index))
				match_d = 1;
			continue;
		} else if (match_d) {
			match_d = 0;
			bcm2048_parse_rt_match_d(bdev, i, index);
			if (index == 60){
			  if (bcm2048_get_rds_rt(bdev, rt_text) == 0) printf("rds_rt = %s",rt_text);
			  reset_rt(bdev);
			}
			continue;
		}

		/* Skip erroneous blocks due to messed up A block altogether */
		if ((bdev->rds_info.radio_text[i] & BCM2048_RDS_BLOCK_MASK)
			== BCM2048_RDS_BLOCK_A) {
			crc = bcm2048_rds_block_crc(bdev, i);
			if (crc == BCM2048_RDS_CRC_UNRECOVARABLE)
				continue;
			/* Syncronize to a good RDS PI */
			if (((bdev->rds_info.radio_text[i+1] << 8) +
				bdev->rds_info.radio_text[i+2]) ==
				bdev->rds_info.rds_pi)
					match_b = 1;
		}
	}

	return 0;
}

static int parse_rds_group_2(struct bcm2048_device *bdev, int i)
{
	int crc, index, rds_group_b, rt_ab;
	char rt_text[BCM2048_MAX_RDS_RT +1];
	
	crc = bcm2048_rds_block_crc(bdev, i);
	if (crc == BCM2048_RDS_CRC_UNRECOVARABLE) return -EIO;
	
	rds_group_b = bdev->rds_info.radio_text[i+1] & BCM2048_RDS_GROUP_AB_MASK;
	rt_ab = bdev->rds_info.radio_text[i+2] & BCM2048_RDS_RT_AB_MASK;

	if (!bdev->rds_info.rds_group_2_b) {
	  bdev->rds_info.rds_group_2_b = rds_group_b;
	} else if (rds_group_b != bdev->rds_info.rds_group_2_b) {
		memset(bdev->rds_info.rds_rt, 0, sizeof(bdev->rds_info.rds_rt));
		bdev->rds_info.rds_group_2_b = rds_group_b;
	}
	
	/* A to B or (vice versa), means: clear screen */
	if (rt_ab != bdev->rds_info.rds_group_2_rt_ab) {
		memset(bdev->rds_info.rds_rt, 0,
			sizeof(bdev->rds_info.rds_rt));
		bdev->rds_info.rds_group_2_rt_ab = rt_ab;
	}

	index = bdev->rds_info.radio_text[i+2] & BCM2048_RDS_RT_INDEX;

	if (bdev->rds_info.rds_group_2_b) {
		index <<= 1;
		crc = bcm2048_rds_block_crc(bdev, i+6);
		if (crc == BCM2048_RDS_CRC_UNRECOVARABLE) return -EIO;
		
		if (crc) {
			if (!bdev->rds_info.rds_rt[index])
				bdev->rds_info.rds_rt[index] = bdev->rds_info.radio_text[i+7];
			if (!bdev->rds_info.rds_rt[index+1])
				bdev->rds_info.rds_rt[index+1] = bdev->rds_info.radio_text[i+8];
		} else {
			bdev->rds_info.rds_rt[index] = bdev->rds_info.radio_text[i+7];
			bdev->rds_info.rds_rt[index+1] = bdev->rds_info.radio_text[i+8];
		}
		if (index == 30){
		  if (bcm2048_get_rds_rt(bdev, rt_text) == 0) printf("\nrds_rt = %s",rt_text);
		  reset_rt(bdev);
		}
	} else {
		index <<= 2;
		crc = bcm2048_rds_block_crc(bdev, i+3);
		if (crc == BCM2048_RDS_CRC_UNRECOVARABLE) return -EIO;
		
		if (crc) {
			if (!bdev->rds_info.rds_rt[index])
				bdev->rds_info.rds_rt[index] = bdev->rds_info.radio_text[i+4];
			if (!bdev->rds_info.rds_rt[index+1])
				bdev->rds_info.rds_rt[index+1] = bdev->rds_info.radio_text[i+5];
		} else {
			bdev->rds_info.rds_rt[index] = bdev->rds_info.radio_text[i+4];
			bdev->rds_info.rds_rt[index+1] = bdev->rds_info.radio_text[i+5];
		}
		
		crc = bcm2048_rds_block_crc(bdev, i+6);
		if (crc == BCM2048_RDS_CRC_UNRECOVARABLE) return -EIO;
		
		if (crc) {
			if (!bdev->rds_info.rds_rt[index+2])
				bdev->rds_info.rds_rt[index+2] = bdev->rds_info.radio_text[i+7];
			if (!bdev->rds_info.rds_rt[index+3])
				bdev->rds_info.rds_rt[index+3] = bdev->rds_info.radio_text[i+8];
		} else {
			bdev->rds_info.rds_rt[index+2] = bdev->rds_info.radio_text[i+7];
			bdev->rds_info.rds_rt[index+3] = bdev->rds_info.radio_text[i+8];
		}
		if (index == 60){
		  if (bcm2048_get_rds_rt(bdev, rt_text) == 0) printf("\nrds_rt = %s",rt_text);
		  reset_rt(bdev);
		}
	}
	return 0;
}

static int parse_rds_group_0(struct bcm2048_device *bdev, int i)
{
	int crc, index, rds_group_b;
	char ps_text[10];
	
	rds_group_b = bdev->rds_info.radio_text[i+1] & BCM2048_RDS_GROUP_AB_MASK;
	
	/* Poor RSSI will lead to RDS data corruption
	 * So using 3 (same) sequential values to justify major changes
	 */
	
	if (!bdev->rds_info.rds_group_0_b) {
	  bdev->rds_info.rds_group_0_b = rds_group_b;
	} else if (rds_group_b != bdev->rds_info.rds_group_0_b) {
		if (crc == BCM2048_RDS_CRC_NONE) {
			bdev->rds_info.rds_group_b_cnt++;
			if (bdev->rds_info.rds_group_b_cnt > 2) {
				bdev->rds_info.rds_group_0_b = rds_group_b;
				bdev->rds_info.rds_group_b_cnt	= 0;
				//no need to take a action for this group
			} else {
				return -EIO;
			}
		} else {
			bdev->rds_info.rds_group_b_cnt = 0;
		}
	}

	crc = bcm2048_rds_block_crc(bdev, i);

	if (crc == BCM2048_RDS_CRC_UNRECOVARABLE) return -EIO;
		
	index = bdev->rds_info.radio_text[i+2] & BCM2048_RDS_PS_INDEX;
	index <<= 1;
	bdev->rds_info.rds_group_0_DI = index | ((bdev->rds_info.radio_text[i+2] & 0x04) >> 2);
	//printf("rds_DI = %s\n", program_DI[bdev->rds_info.rds_group_0_DI]);
	bdev->rds_info.rds_group_0_MS = (bdev->rds_info.radio_text[i+2] & 0x08) >> 3;
	bdev->rds_info.rds_group_0_TA = (bdev->rds_info.radio_text[i+2] & 0x10) >> 4;
	
	crc = bcm2048_rds_block_crc(bdev, i+6);

	if (crc == BCM2048_RDS_CRC_UNRECOVARABLE) return -EIO;
	
	if (crc) {
	  if (!bdev->rds_info.rds_ps[index]) bdev->rds_info.rds_ps[index] = bdev->rds_info.radio_text[i+7];
	  if (!bdev->rds_info.rds_ps[index+1]) bdev->rds_info.rds_ps[index+1] = bdev->rds_info.radio_text[i+8];
	}
	else {
	  bdev->rds_info.rds_ps[index] = bdev->rds_info.radio_text[i+7];
	  bdev->rds_info.rds_ps[index+1] = bdev->rds_info.radio_text[i+8];
	}
	
	if (index == 6){
	  if (bcm2048_get_rds_ps(bdev, ps_text) == 0) printf("\nrds_ps = %s, rds_pty = %s", ps_text, program_type[bdev->rds_info.rds_pty]);
	  reset_ps(bdev);
	}
	return 0;
}

static int parse_rds_block_b(struct bcm2048_device *bdev, int i)
{
	int crc, index, rds_group_b, rds_group;

	crc = bcm2048_rds_block_crc(bdev, i);

	if (crc == BCM2048_RDS_CRC_UNRECOVARABLE)
		return -EIO;
	
	/* Block B, C and D RDS match */
	if (((bdev->rds_info.radio_text[i] & BCM2048_RDS_BLOCK_MASK) == BCM2048_RDS_BLOCK_B) && ((bdev->rds_info.radio_text[i+3] & BCM2048_RDS_BLOCK_MASK) == BCM2048_RDS_BLOCK_C) && ((bdev->rds_info.radio_text[i+6] & BCM2048_RDS_BLOCK_MASK) == BCM2048_RDS_BLOCK_D)) {
		rds_group = (bdev->rds_info.radio_text[i+1] & BCM2048_RDS_BLOCK_MASK) >> 4;
		rds_group_b = bdev->rds_info.radio_text[i+1] & BCM2048_RDS_GROUP_AB_MASK;
		bdev->rds_info.rds_pty = ((bdev->rds_info.radio_text[i+1] & 0x03) << 3) + ((bdev->rds_info.radio_text[i+2] & 0xE0) >> 5);

		if (rds_group == 0) {
			parse_rds_group_0(bdev, i);
		} else if (rds_group == 2) {
			parse_rds_group_2(bdev, i);
		}
	}

	return -EIO;
}

static void parse_rds_data(struct bcm2048_device *bdev)
{
	int i, cnt = 0, crc, match_b = 0;
	uint16_t pi;

	for (i = 0; i < bdev->fifo_size; i += BCM2048_RDS_FIFO_DUPLE_SIZE) {

		if (match_b) {
			match_b = 0;
			parse_rds_block_b(bdev, i);
			continue;
		}
		/* Skip erroneous blocks due to messed up A block altogether */
		if ((bdev->rds_info.radio_text[i] & BCM2048_RDS_BLOCK_MASK) == BCM2048_RDS_BLOCK_A) {
			crc = bcm2048_rds_block_crc(bdev, i);
			if (crc == BCM2048_RDS_CRC_UNRECOVARABLE){
			  continue;
			}
			pi = ((bdev->rds_info.radio_text[i+1] << 8) +
				bdev->rds_info.radio_text[i+2]);
			
			if (!bdev->rds_info.rds_pi) {
				bdev->rds_info.rds_pi = pi;
				match_b = 1;
			}
			if (pi != bdev->rds_info.rds_pi) {
				cnt++;
				if (cnt > 3) {
					//memset(bdev,0,sizeof(bdev)); //don't know if this is needed...
					bdev->rds_info.rds_pi = pi;
					match_b = 1;
					cnt = 0;
				}
			} else {
				match_b = 1;
				cnt = 0;
			}
		}
	}
}

static void bcm2048_rds_fifo_receive(struct bcm2048_device *bdev)
{
	int buf_size = 120;
  
        hci_read(0x80, buf_size, bdev->rds_info.radio_text);

	bdev->fifo_size = buf_size;
	bdev->rds_info.text_len = bdev->fifo_size;
	
	//bcm2048_parse_rds_pi(bdev);
	//bcm2048_parse_rds_rt(bdev);
	//bcm2048_parse_rds_ps(bdev);
	parse_rds_data(bdev);
}

main(int argc, char *argv[])
{
	struct bcm2048_device *bdev;
	bdev = malloc (sizeof (struct bcm2048_device));
	reset_ps(bdev);
	reset_rt(bdev);
	uint8_t flag_lsb, flag_msb;
	char hci_res[3];
	FILE *fp;
	uint8_t buf;
	
	buf=BCM2048_FM_ON | BCM2048_RDS_ON;
	hci_write(BCM2048_I2C_FM_RDS_SYSTEM, 1, &buf);
	buf=BCM2048_RDS_FLAG_FIFO_WLINE;
	hci_write(BCM2048_I2C_FM_RDS_MASK1, 1, &buf);
	buf=0x78;
	hci_write(BCM2048_I2C_RDS_WLINE, 1, &buf);
	

    for (;;){
	hci_read(BCM2048_I2C_FM_RDS_FLAG0, 1, &flag_lsb);
	hci_read(BCM2048_I2C_FM_RDS_FLAG1, 1, &flag_msb);
	
	//if (flag_lsb & BCM2048_FM_FLAG_SEARCH_TUNE_FINISHED) {
	  if (flag_msb & BCM2048_RDS_FLAG_FIFO_WLINE) {
		bcm2048_rds_fifo_receive(bdev);
		/*if (bdev->rds_state) {
			flags =	BCM2048_RDS_FLAG_FIFO_WLINE;
			bcm2048_send_command(bdev, BCM2048_I2C_FM_RDS_MASK1,
						flags);
		}*/

	  }
	//}

    }
    return 0;
}
