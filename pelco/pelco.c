/*
 *  Hamlib Rotator backend - Easycom
 *  Copyright (c) 2001-2003 by Stephane Fillod
 *  Contributed by Francois Retief <fgretief@sun.ac.za>
 *  Copyright (c) 2014 by Alexander Schultze <alexschultze@gmail.com>
 *
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <math.h>

#include "hamlib/rotator.h"
#include "serial.h"
#include "misc.h"
#include "register.h"

/* ************************************************************************* */
/**
 *  pelco_transaction
 *
 *  Assumes rot!=NULL and cmdstr!=NULL
 *
 *  cmdstr   - string to send to rotator
 *  data     - buffer for reply string
 *  data_len - (input) Maximum size of buffer
 *             (output) Number of bytes read.
 */
static int
pelco_transaction (ROT *rot, const char *cmdstr, char *data, size_t data_len)
{
  struct rot_state *rs;
  int retval;

  rig_debug(RIG_DEBUG_TRACE, "%s called: %s\n", __FUNCTION__, cmdstr);

  if (!rot )
    return -RIG_EINVAL;

  rs = &rot->state;
  serial_flush(&rs->rotport);
  retval = write_block(&rs->rotport, cmdstr, 7);
  if (retval != RIG_OK) {
      goto transaction_quit;
  }

  if (data == NULL || data_len <= 0)
    return RIG_OK;  /* don't want a reply */

  memset(data,0,data_len);
  retval = read_block(&rs->rotport, data, data_len);
  if (retval < 0) {
    rig_debug(RIG_DEBUG_TRACE, "%s read_block failed with status %d\n", __FUNCTION__, retval);
    goto transaction_quit;
  } else {
    rig_debug(RIG_DEBUG_TRACE, "%s read_block: %s\n", __FUNCTION__, data);
    retval = RIG_OK;
  }

  transaction_quit:
  return retval;
}

/* ************************************************************************* */
static char checksum(const char * cmd){
    char tmp = 0;
    for(int i=1; i< 6; i++){
         tmp += cmd[i];
    }
    return tmp;
}

static int
pelco_rot_set_position(ROT *rot, azimuth_t az, elevation_t el)
{
    char cmdstr[7];
    int retval;
    rig_debug(RIG_DEBUG_TRACE, "%s called: %f %f\n", __FUNCTION__, az, el);

    cmdstr[0] = 0xFF;
    cmdstr[1] = 0x01;
    cmdstr[2] = 0x00;
    cmdstr[3] = 0x4B;
    
    char lsb = (char)(((short)(az * 100.0)) & 0xFF);
    char msb = (char)(((((short)(az * 100.0)) >> 8)) & 0xFF);

    cmdstr[4] = msb;
    cmdstr[5] = lsb;
    cmdstr[6] = checksum(cmdstr);

    retval = pelco_transaction(rot, cmdstr, NULL, 0);
    if (retval != RIG_OK) {
		return retval;
    }
    
    memset(cmdstr, 0 , sizeof(cmdstr));

    cmdstr[0] = 0xFF;
    cmdstr[1] = 0x01;
    cmdstr[2] = 0x00;
    cmdstr[3] = 0x4D;

    lsb =(char) (((short)(el * 100.0)) & 0xFF);
    msb =(char) ((((short)(el * 100.0) >> 8)) & 0xFF);

    cmdstr[4] = msb;
    cmdstr[5] = lsb;
    cmdstr[6] = checksum(cmdstr);
    
    retval = pelco_transaction(rot, cmdstr, NULL, 0);
    if (retval != RIG_OK) {
	    return retval;
    }

    return RIG_OK;
}

int is_read_az = 0;
azimuth_t last_az;
elevation_t last_el;

static int
pelco_rot_get_position(ROT *rot, azimuth_t *az, elevation_t *el)
{
    int retval = 0;
    *az = 0;
    *el = 0;
    if(is_read_az == 0){
    char azcmd[7] = {0xFF, 0x01, 0x00, 0x51, 0x00,0x00,0x52 };

    char az_angle[7];
    retval = pelco_transaction(rot, azcmd, az_angle, 7);
    if(retval != RIG_OK){
    	return retval;
    }
    char az_cmd = az_angle[3];
    if(az_cmd == 0x59){
    	char lsb = az_angle[5];
	char msb = az_angle[4];
	short tmp_angle = lsb + (msb << 8);
	*az = tmp_angle / 100.0f;
    }
    is_read_az  = 1;
    last_az = *az;
    *el = last_el;
    }else{
    char elcmd[7] = {0xFF, 0x01, 0x00, 0x53, 0x00, 0x00, 0x54};
    char el_angle[7];
    retval = pelco_transaction(rot, elcmd, el_angle, 7);
    if(retval != RIG_OK){
	    return retval;
    }
    char el_cmd = el_angle[3];
    if(el_cmd == 0x5B){
    	char lsb = el_angle[5];
	char msb = el_angle[4];
	short tmp_angle = lsb + (msb << 8);
	*el = tmp_angle / 100.0f;
    }
    is_read_az = 0;
    last_el = *el;
    *az = last_az;
    }
    return RIG_OK;
}

static int
pelco_rot_stop(ROT *rot)
{
    char stop_cmd[7] = {0xFF,0x01,0x00,0x00,0x00,0x00,0x01};
    int retval;

    rig_debug(RIG_DEBUG_TRACE, "%s called\n", __FUNCTION__);

    retval = pelco_transaction(rot, stop_cmd, NULL, 0);
	if (retval != RIG_OK)
		return retval;

	/* TODO: error processing */

	return RIG_OK;
}

static int
pelco_rot_reset(ROT *rot, rot_reset_t rst)
{
	return RIG_OK;
}

static int
pelco_rot_park(ROT *rot)
{
    int retval;
    rig_debug(RIG_DEBUG_TRACE, "%s called\n", __FUNCTION__);

    retval = pelco_rot_set_position(rot, 0.0, 0.0);
    if (retval != RIG_OK)   /* Custom command (not in pelco) */
		return retval;

	return RIG_OK;
}

static int
pelco_rot_move_velocity(ROT *rot, int direction, int speed)
{
    char cmdstr[7];
    int retval;
    rig_debug(RIG_DEBUG_TRACE, "%s called\n", __FUNCTION__);
    if(speed<1 && speed>64) {
       rig_debug(RIG_DEBUG_ERR,"%s: Invalid speed value!(1-64) (%d)\n", __FUNCTION__, speed);
       return -RIG_EINVAL;
    }

  
    cmdstr[0] = 0xFF;
    cmdstr[1] = 0x01;

    /* Speed for pelco 3 */
    switch (direction) {
    case ROT_MOVE_UP:       /* Elevation increase */
	cmdstr[2] = 0x00;
	cmdstr[3] = 0x00;
	cmdstr[4] = speed;
	cmdstr[5] = 0x00;
        break;
    case ROT_MOVE_DOWN:     /* Elevation decrease */
	cmdstr[2] = 0x00;
	cmdstr[3] = 0x00;
	cmdstr[4] = speed;
	cmdstr[5] = 0x00;
        break;
    case ROT_MOVE_LEFT:     /* Azimuth decrease */
	cmdstr[2] = 0x00;
	cmdstr[3] = 0x00;
	cmdstr[4] = 0x00;
	cmdstr[5] = speed;
        break;
    case ROT_MOVE_RIGHT:    /* Azimuth increase */
	cmdstr[2] = 0x00;
	cmdstr[3] = 0x00;
	cmdstr[4] = 0x00;
	cmdstr[5] = speed;
        break;
    default:
        rig_debug(RIG_DEBUG_ERR,"%s: Invalid direction value! (%d)\n", __FUNCTION__, direction);
        return -RIG_EINVAL;
    }
    cmdstr[6] = checksum(cmdstr);

    retval = pelco_transaction(rot, cmdstr, NULL, 0);
    if (retval != RIG_OK)
        return retval;

    return RIG_OK;
}

/* ************************************************************************* */
/*
 * pelco rotator capabilities.
 */

/* pelcoIII provides changes Moving functions and info.
 */
const struct rot_caps pelco_rot_caps = {
  .rot_model =      ROT_MODEL_PELCO,
  .model_name =     "YL3040",
  .mfg_name =       "YAAN",
  .version =        "0.1",
  .copyright = 	 "LGPL",
  .status =         RIG_STATUS_ALPHA,
  .rot_type =       ROT_TYPE_OTHER,
  .port_type =      RIG_PORT_SERIAL,
  .serial_rate_min =  9600,
  .serial_rate_max =  19200,
  .serial_data_bits =  8,
  .serial_stop_bits =  1,
  .serial_parity =  RIG_PARITY_NONE,
  .serial_handshake =  RIG_HANDSHAKE_NONE,
  .write_delay =  0,
  .post_write_delay =  0,
  .timeout =  200,
  .retry =  3,

  .min_az = 	0.0,
  .max_az =  	360.0,
  .min_el = 	0.0,
  .max_el =  	84.0,

  .priv =  NULL,	/* priv */

  .rot_init =  NULL,
  .rot_cleanup =  NULL,
  .rot_open =  NULL,
  .rot_close =  NULL,

  .get_position =  pelco_rot_get_position,
  .set_position =  pelco_rot_set_position,
  .stop = 	pelco_rot_stop,
  .park =  pelco_rot_park,
  .reset =  pelco_rot_reset,
  .move =  pelco_rot_move_velocity,
};

/* ************************************************************************* */

DECLARE_INITROT_BACKEND(pelco)
{
	rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __FUNCTION__);

    rot_register(&pelco_rot_caps);
	return RIG_OK;
}

/* ************************************************************************* */
/* end of file */
