/*
 * framos_sensor_common.c - common logic
 *
 * Copyright (c) 2019. FRAMOS.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//#define DEBUG 1
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>

#include "framos_sensor_common.h"
#include "i2c/framos_i2c_generic.h"


void fm_calc_lshift(u64 *val, u8 *lshift, s64 max){
    *lshift = 1;
    *val = *val >> *lshift;

    while (*val > max) {
        (*lshift)++;
        *val = *val >> *lshift;
    }

    return;
}
EXPORT_SYMBOL(fm_calc_lshift);

void fm_get_gpio_ctrl(struct camera_common_pdata *board_priv_pdata) {
    struct device_node *node = NULL;

    node = of_find_node_by_name(NULL, "framos_platform_adapter");
    if(node) {
        board_priv_pdata->use_cam_gpio = of_property_read_bool(node, "cam,use-cam-gpio"); 
        of_node_put(node);
    }
    else {
        board_priv_pdata->use_cam_gpio = 0;
    }

    pr_info("Board GPIO ctrl [%d]\n", board_priv_pdata->use_cam_gpio);
    return;
}
EXPORT_SYMBOL(fm_get_gpio_ctrl);

void fm_gpio_set(struct camera_common_data *s_data,
			    unsigned int gpio, int val)
{
	struct camera_common_pdata *pdata = s_data->pdata;

	if (pdata && pdata->use_cam_gpio)
		cam_gpio_ctrl(s_data->dev, gpio, val, 1);
	else {
		if (gpio_cansleep(gpio))
			gpio_set_value_cansleep(gpio, val);
		else
			gpio_set_value(gpio, val);
	}
}
EXPORT_SYMBOL(fm_gpio_set);

struct v4l2_ctrl *fm_find_v4l2_ctrl(struct tegracam_device *tc_dev, 
                                                int ctrl_id)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct tegracam_ctrl_handler *handler = s_data->tegracam_ctrl_hdl;
    struct v4l2_ctrl *ctrl;
    int i;

	for (i = 0; i < handler->numctrls; i++) {
       	ctrl = handler->ctrls[i];
 
        if ( ctrl->id == ctrl_id )
            return ctrl;
	}

    dev_warn(dev, "%s: Couldn't find control with [ %x ] id\n", 
                    __func__, ctrl_id);
    return NULL;
       
}
EXPORT_SYMBOL(fm_find_v4l2_ctrl);


void fm_update_ctrl_range(struct tegracam_device *tc_dev,
                                              int ctrl_id, u64 min, u64 max)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct sensor_control_properties *ctrlprops = NULL;
    struct v4l2_ctrl *ctrl;
	const struct tegracam_ctrl_ops *ops = tc_dev->tcctrl_ops;
    int err = 0;

    ctrlprops =
		&s_data->sensor_props.sensor_modes[s_data->mode].control_properties;

    ctrl = fm_find_v4l2_ctrl(tc_dev, ctrl_id);
    if (ctrl == NULL) {
        return;
    }

    switch (ctrl_id) {
    case TEGRA_CAMERA_CID_EXPOSURE:
        s_data->exposure_min_range = min;
        s_data->exposure_max_range = max;

        /* Current value must be recalculated */
        dev_dbg(dev, "%s: recalculate exposure for set frame length.\n", __func__);
        err = ops->set_exposure(tc_dev, *ctrl->p_cur.p_s64);

	    dev_dbg(dev, "%s:  mode: %u, exposure range [%llu, %llu]\n",
                 __func__, s_data->mode, s_data->exposure_min_range, 
                s_data->exposure_max_range);
        break;
    case TEGRA_CAMERA_CID_EXPOSURE_SHORT:
        s_data->short_exposure_min_range = min;
        s_data->short_exposure_max_range = max;

	    dev_dbg(dev, "%s:  mode: %u, short exposure range [%llu, %llu]\n",
                 __func__, s_data->mode, s_data->short_exposure_min_range, 
                s_data->short_exposure_max_range);
        break;
    case TEGRA_CAMERA_CID_FRAME_RATE:
        ctrlprops->min_framerate = min;
        ctrlprops->max_framerate = max;
        /* default value must be in range */
        ctrlprops->default_framerate = clamp_val(ctrlprops->default_framerate,
                                                 ctrlprops->min_framerate,
                                                 ctrlprops->max_framerate);

	    dev_dbg(dev, "%s:  mode: %u, framerate range [%u, %u]\n",
                 __func__, s_data->mode,
                ctrlprops->min_framerate, ctrlprops->max_framerate);
        break;
    }

	if (err) {
		dev_err(dev,
			"%s: ctrl %s range update failed\n", __func__, ctrl->name);
	}
}
EXPORT_SYMBOL(fm_update_ctrl_range);

/**
 * Get I2C broadcast client
 */
int common_get_broadcast_client(struct tegracam_device *tc_dev,
                                struct v4l2_ctrl *ctrl, 
                                const struct regmap_config *sensor_regmap_config)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev; 

    if (!s_data->broadcast_regmap) {
        struct i2c_client *broadcast_client = 
                                        get_broadcast_client(tc_dev->client);

        /* Disable broadcast mode if an error occurs */
        if (IS_ERR_OR_NULL(broadcast_client)) {  
            dev_warn_once(dev, "%s: couldn't get broadcast client\n", __func__);          
            ctrl->val = UNICAST;
            v4l2_ctrl_activate(ctrl, 0);
            return -ENODEV;
        }
      
        s_data->broadcast_regmap = devm_regmap_init_i2c(broadcast_client,
			                 sensor_regmap_config);

        /* Disable broadcast mode if an error occurs */
        if (IS_ERR_OR_NULL(s_data->broadcast_regmap)) {
            ctrl->val = UNICAST;            
            v4l2_ctrl_activate(ctrl, 0);

            dev_err (dev, "regmap init failed: %ld\n",
                PTR_ERR (s_data->broadcast_regmap));
            return -ENODEV;
        }
	}

    return 0;
}
EXPORT_SYMBOL(common_get_broadcast_client);

static int get_pix_bit_from_mbus(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev = s_data->dev;

    /* Bayer MBUS_CODEs */
    if (s_data->colorfmt->code == MEDIA_BUS_FMT_SBGGR8_1X8 
        || s_data->colorfmt->code == MEDIA_BUS_FMT_SGBRG8_1X8
        || s_data->colorfmt->code == MEDIA_BUS_FMT_SGRBG8_1X8
        || s_data->colorfmt->code == MEDIA_BUS_FMT_SRGGB8_1X8)
         return 8;
    else if (s_data->colorfmt->code == MEDIA_BUS_FMT_SBGGR10_1X10 
            || s_data->colorfmt->code == MEDIA_BUS_FMT_SGBRG10_1X10
            || s_data->colorfmt->code == MEDIA_BUS_FMT_SGRBG10_1X10
            || s_data->colorfmt->code == MEDIA_BUS_FMT_SRGGB10_1X10)
         return 10;
    else if (s_data->colorfmt->code == MEDIA_BUS_FMT_SBGGR12_1X12 
            || s_data->colorfmt->code == MEDIA_BUS_FMT_SGBRG12_1X12
            || s_data->colorfmt->code == MEDIA_BUS_FMT_SGRBG12_1X12
            || s_data->colorfmt->code == MEDIA_BUS_FMT_SRGGB12_1X12)
         return 12;
    else {
        dev_err(dev, "%s: unknown media bus format", __func__);
        return -1;
    }
}

static const struct regmap_config common_crosslink_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

/**
 * Get I2C lvds2mipi client
 */
int common_get_lvds2mipi_client(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;

    if (!s_data->crosslink_regmap) {
        struct i2c_client *crosslink_client = 
                                        get_lvds2mipi_client(tc_dev->client);

        /* Disable broadcast mode if an error occurs */
        if (IS_ERR_OR_NULL(crosslink_client)) {  
            dev_warn_once(dev,
                          "%s: couldn't get broadcast client\n", __func__);          
            return -ENODEV;
        }
      
        s_data->crosslink_regmap = devm_regmap_init_i2c (crosslink_client,
			                 &common_crosslink_regmap_config);

        /* Disable broadcast mode if an error occurs */
        if (IS_ERR_OR_NULL(s_data->crosslink_regmap)) {
            dev_err (dev,
               "regmap init failed: %ld\n",
               PTR_ERR (s_data->crosslink_regmap));
            return -ENODEV;
        }
	}

    return 0;
}
EXPORT_SYMBOL(common_get_lvds2mipi_client);

/**
 * Read register
 */
static int crosslink_read_reg(struct tegracam_device *tc_dev, 
                                u16 addr, u8 *val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = s_data->dev;
    int err = 0;

    err = common_get_lvds2mipi_client(tc_dev);
    if (err)
        return err;

    err = regmap_read(s_data->crosslink_regmap, addr, (int *)val);
    if (err) 
        dev_err(dev, "%s: i2c read failed, 0x%x = %x\n",
			__func__, addr, *val);

    return err;
}

/**
 * Write register
 */
static int crosslink_write_reg(struct tegracam_device *tc_dev, 
                                u16 addr, u8 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = s_data->dev;
    int err = 0;

    err = common_get_lvds2mipi_client(tc_dev);
    if (err)
        return err;

    err = regmap_write(s_data->crosslink_regmap, addr, val);
    if (err) 
        dev_err(dev, "%s: i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

    return err;
}

/**
 * Write multiple sequential registers
 */
static int crosslink_write_buffered_reg(struct tegracam_device *tc_dev, 
                                u16 addr_low, u8 number_of_registers, u32 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = s_data->dev;
    int err = 0, i;

    for (i = 0; i < number_of_registers; i++) {
        err = crosslink_write_reg(tc_dev, addr_low + i, (u8)(val >> (i * 8)));
        if (err) {
            dev_err(dev, "%s: buffered register write error\n", __func__);
            return err;
        }
    }

    return err;
}

/**
 * Read multiple sequential registers
 */
static int crosslink_read_buffered_reg(struct tegracam_device *tc_dev, 
                                u16 addr_low, u8 number_of_registers, u32 *val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = s_data->dev;
    int err = 0, i;
    u8 reg;

    *val = 0;

    for (i = 0; i < number_of_registers; i++) {
        err = crosslink_read_reg(tc_dev, addr_low + i, &reg);
        *val += reg << (i * 8);
        if (err) {
            dev_err(dev, "%s: buffered register read error\n", __func__);
            return err;
        }
    }

    return err;
}

static int read_crosslink_fw_name(struct tegracam_device *tc_dev, char data[])
{    
	struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev = s_data->dev;
    int err = 0, i, j;
    u8 val;
    
    for (i = 0, j = 0; i <= CROSSLINK_FW_NAME_REG_LENGTH ; i++){
        err = crosslink_read_reg(tc_dev, 
            (CROSSLINK_FW_NAME_BASE + CROSSLINK_FW_NAME_REG_LENGTH - i), &val);
        if (err){
            dev_err(dev, "error reading crosslink configruation");            
            return err;
        }

        if (!val) continue;

        data[j]=val;
        j++;
    }

    return 0;
}

static int read_crosslink_fw_version(struct tegracam_device *tc_dev, char data[])
{  
    struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev = s_data->dev;
    int err, i;
    u32 val;
    
    err = crosslink_read_buffered_reg(tc_dev, CROSSLINK_FW_VERSION_BASE, 4, &val);
    if (err){
        dev_err(dev, "error reading crosslink configruation");            
        return err;
    }

    for (i = CROSSLINK_FW_VERSION_REG_LENGTH - 1; i >= 0 ; i--){
        data[CROSSLINK_FW_VERSION_REG_LENGTH - 1 - i] = (val >> i * 8) & 0xFF;
    }

    return 0;
}

static int check_crosslink_image_data(struct tegracam_device *tc_dev, 
                                        u32 data[])
{ 
	struct camera_common_data *s_data = tc_dev->s_data;
    struct crosslink_private *cl_priv = s_data->cl_priv;
    struct device *dev = s_data->dev;
    int err = 0;
    u8 pix_format, lvds_line;
    int mbus_pix_format = get_pix_bit_from_mbus(tc_dev);
    
    err = crosslink_read_reg(tc_dev, CROSSLINK_PIX_FORMAT, &pix_format);
    if (err) goto fail;
    data[0] = pix_format;

    err = crosslink_read_reg(tc_dev, CROSSLINK_LVDS_NUM_CH, &lvds_line);
    if (err) goto fail;
    data[1] = lvds_line;

    if (mbus_pix_format != pix_format
        || cl_priv->sensor_numch != lvds_line) {     
        return -1;
    }

    return 0;
fail:
    dev_err(dev, "error reading crosslink configuration");            
    return err;
}

static int crosslink_write_tg_parameters(struct tegracam_device *tc_dev)
{ 
	struct camera_common_data *s_data = tc_dev->s_data;
    struct time_generator_params *tg_params = &s_data->cl_priv->tg_params;
    struct device *dev = s_data->dev;
    int err = 0;
    
    err = crosslink_write_reg(tc_dev, CROSSLINK_TG_MODE, tg_params->tg_mode);
    if (err) goto fail;
    
    if (*tg_params->is_shutter_mode == SEQ_TRIGGER
        && (tg_params->frame_active_width > tg_params->frame_length)) {
            tg_params->frame_active_width = tg_params->frame_length - MIN_TRIGGER_HIGH_WIDTH_TGES;
            tg_params->frame_inactive_width = MIN_TRIGGER_HIGH_WIDTH_TGES;
    }

    if (tg_params->tg_mode == TG_TRIGGER_MODE) {
        tg_params->frame_inactive_width = tg_params->t_tgpd;

        err = crosslink_write_buffered_reg(tc_dev, CROSSLINK_FRAME_ACTIVE_WIDTH, 
                                            4, tg_params->expanded_exposure);
    } else {
        err = crosslink_write_buffered_reg(tc_dev, CROSSLINK_FRAME_ACTIVE_WIDTH, 
                                            4, tg_params->frame_active_width);
    }

    err |= crosslink_write_buffered_reg(tc_dev, CROSSLINK_FRAME_INACTIVE_WIDTH,
                                        4, tg_params->frame_inactive_width);
    if (err) goto fail;

    err = crosslink_write_buffered_reg(tc_dev, CROSSLINK_FRAME_DELAY, 
                                                        4, tg_params->frame_delay );
    if (err) goto fail;                                                                                     

    err = crosslink_write_reg(tc_dev, CROSSLINK_SYNC_LOGIC,tg_params->sync_logic);

    err |= crosslink_write_reg(tc_dev, CROSSLINK_OUT_LOGIC, tg_params->out_logic);
    if (err) goto fail;
    
    return 0;
fail:
    dev_err(dev, "error writing crosslink slave parameters");            
    return err;
}

int common_verify_crosslink_fw_compatibility(struct tegracam_device *tc_dev, 
                                            s32 *val)
{
	struct device *dev = tc_dev->dev;
    int err = 0;
    u32 image_data[4] = {0};
    char fw_name[CROSSLINK_FW_NAME_REG_LENGTH + 1 ] = "";
    char fw_version[CROSSLINK_FW_VERSION_REG_LENGTH + 1] = "";

    err = read_crosslink_fw_name(tc_dev, fw_name);
    if (err) goto fail;

    err = read_crosslink_fw_version(tc_dev, fw_version);
    if (err) goto fail;

    err = check_crosslink_image_data(tc_dev, image_data);
    if (err) {
        dev_err(dev, "Crosslink firmware and sensor mode doesn't match!\n");
        dev_info(dev, "Current firmware configuration:\n");
        dev_info(dev, " - Firmware name: \t%s\n", fw_name);
        dev_info(dev, " - Firmware version: \t%d.%d.%d.%d\n", fw_version[0], 
                                fw_version[1], fw_version[2], fw_version[3]);
        dev_info(dev, " - Pixel format: \t%u bits/pix\n", image_data[0]);
        dev_info(dev, " - Firmware LVDS ch: \t%u \n", image_data[1]);
        dev_err(dev, "RECONFIGURE FIRMWARE!\n");
            
        *val = 2;
    }
    else{
        *val = 1;
    }
    
    return 0;

fail:
    *val = 2;
    dev_err(dev, "%s: failed to verify fw compatibility\n", __func__);
    return err;
}
EXPORT_SYMBOL(common_verify_crosslink_fw_compatibility);

int crosslink_set_readout_mode(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
    struct crosslink_private *cl_priv = s_data->cl_priv;
    struct crosslink_readout_mode *cl_readout_mode = &s_data->cl_priv->cl_readout_mode;
	struct device *dev = tc_dev->dev;
    int err = 0;
    int i;
    int mbus_pix_format = get_pix_bit_from_mbus(tc_dev);
    u32 v_active, h_active, word_count;

    v_active = s_data->fmt_height;
    h_active = s_data->fmt_width / cl_priv->sensor_numch;
    word_count = (s_data->fmt_width * mbus_pix_format) / 8 ;

    cl_readout_mode->v_active_low = v_active & 0xFF;
    cl_readout_mode->v_active_high = (v_active >> 8) & 0xFF;
    cl_readout_mode->h_active_unit_low = h_active & 0xFF;
    cl_readout_mode->h_active_unit_high = (h_active >> 8) & 0xFF;
    cl_readout_mode->word_count_low = word_count & 0xFF;
    cl_readout_mode->word_count_high = (word_count >> 8) & 0xFF;

    for (i = 0; i < sizeof(struct crosslink_readout_mode)/sizeof(u8); i++) {
        err  = crosslink_write_reg(tc_dev, 
                                    (CROSSLINK_LINE_SKIP_COUNT_RW + i),
                                    *((u8*)cl_readout_mode + sizeof(u8)*i));
        if (err) goto fail;
    }
    return 0;
fail:
    dev_err(dev, "%s: failed to set crosslink readout mode\n", __func__);
    return err;
}
EXPORT_SYMBOL(crosslink_set_readout_mode);

/** 
 * Set Image Sensor operation and trigger mode
 *      0 - Master mode, 1 - Slave Normal trigger mode, 2 - Slave Seq. trigger mode 
 */
int crosslink_set_is_operation_mode(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
    struct time_generator_params *tg_params = &s_data->cl_priv->tg_params;
	struct device *dev = tc_dev->dev;
    int err = 0;
    u8 val;

    if (*tg_params->is_operation_mode == SLAVE_MODE
        && *tg_params->is_shutter_mode == NORMAL_EXPO)
        val = 1;
    else if (*tg_params->is_operation_mode == SLAVE_MODE
        && *tg_params->is_shutter_mode == SEQ_TRIGGER)
        val = 2;
    else
        val = 0;

    err = crosslink_write_reg(tc_dev, CROSSLINK_MASTER_SLAVE, val);
    if (err) 
        dev_err(dev, "%s: error setting operation mode\n", __func__);

    return err;
}
EXPORT_SYMBOL(crosslink_set_is_operation_mode);

/** 
 * Set timing generator mode
 *       0 - Disabled, 1 - Master mode, 2 - Slave mode, 3 - Trigger mode,
 */
int tg_set_operation_mode(struct tegracam_device *tc_dev, 
                                    struct v4l2_ctrl *ctrl)
{
	struct camera_common_data *s_data = tc_dev->s_data;
    struct time_generator_params *tg_params = &s_data->cl_priv->tg_params;
	struct device *dev = tc_dev->dev;
    u8 val = *ctrl->p_new.p_u8;
    int err = 0;

    if (val == TG_DISABLED){
        err = crosslink_write_reg(tc_dev, CROSSLINK_TG_ENABLE, 0);                       

        if (err) 
            dev_err(dev, "%s: Trigger generator stop failure\n", __func__);
    }

    if (*tg_params->is_operation_mode == MASTER_MODE && val != TG_DISABLED){
        val = TG_DISABLED;
        __v4l2_ctrl_s_ctrl(ctrl, val);
        dev_warn(dev, 
            "%s: sensor must be in slave mode to select TG mode\n",
             __func__);
    }

    if (*tg_params->is_shutter_mode == NORMAL_EXPO && val == TG_TRIGGER_MODE){
        val = TG_MASTER_MODE;
        __v4l2_ctrl_s_ctrl(ctrl, val);
        dev_warn(dev, 
            "%s: Selected TG mode is disabled in combination with selected shutter mode, switching to default\n",
             __func__);
    }

    tg_params->tg_mode = val;

    return 0;
}
EXPORT_SYMBOL(tg_set_operation_mode);
   

/* Convert us to 1H */
static u32 tg_convert_us_to_1h(struct tegracam_device *tc_dev, u32 usec)
{
	struct camera_common_data *s_data = tc_dev->s_data;
    struct time_generator_params *tg_params = &s_data->cl_priv->tg_params;
	struct device *dev = tc_dev->dev;
    u32 tg_hmax_clk;
    u64 usec_clk;
    u32 usec_1h;
   
    tg_hmax_clk = (*tg_params->line_time * (s_data->def_clk_freq / 1000) ) / 1000000 + tg_params->xhs_clk_offset;
    usec_clk = ((u64)usec * (s_data->def_clk_freq / 1000) ) / 1000;
    /* Round result */
    usec_1h = (usec_clk + (tg_hmax_clk/2)) / tg_hmax_clk;

    dev_dbg(dev, "%s: Value [%u]us, TG line time [%u]ns, Value [%u]1H\n", 
                            __func__, usec, 
    (tg_hmax_clk * 1000000)/ (s_data->def_clk_freq / 1000), usec_1h);
    return usec_1h;
}

/** 
 * Delay frame
 */
int tg_delay_frame(struct tegracam_device *tc_dev, u32 val)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct time_generator_params *tg_params = &s_data->cl_priv->tg_params;
	struct device *dev = tc_dev->dev;
    int err = 0;
    u32 tg_frame_delay_1h = tg_convert_us_to_1h(tc_dev, val);
   
    tg_params->frame_delay = tg_frame_delay_1h;
    
    if (err) 
        dev_err(dev, "%s: error setting delay frame\n", __func__);

    dev_dbg(dev, "%s: delay [%u]us, delay [%u] 1H\n", 
                            __func__, val, tg_frame_delay_1h);

    return err;
}
EXPORT_SYMBOL(tg_delay_frame);

/** 
 * TG 1H length
 */
int tg_set_line_width(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
    struct time_generator_params *tg_params = &s_data->cl_priv->tg_params;
	struct device *dev = tc_dev->dev;
    int err = 0;
    u32 tg_hmax_clk;
    u32 tg_xhs_low_width, tg_xhs_high_width;

    if (*tg_params->is_operation_mode == MASTER_MODE)
        return 0;

    tg_hmax_clk = (*tg_params->line_time * (s_data->def_clk_freq / 1000)) / 1000000 + tg_params->xhs_clk_offset;
    
    tg_xhs_low_width = tg_params->xhs_min_active_width;
    tg_xhs_high_width = tg_hmax_clk - tg_xhs_low_width;

    err = crosslink_write_buffered_reg(tc_dev, CROSSLINK_LINE_ACTIVE_WIDTH,
                                                         2, tg_xhs_low_width);
    err |= crosslink_write_buffered_reg(tc_dev, CROSSLINK_LINE_INACTIVE_WIDTH,
                                                         2, tg_xhs_high_width);
    if (err) 
        dev_err(dev, "%s: error setting TG XHS WIDTH\n", __func__);

    dev_dbg(dev, "%s: tg xhs [%u], xhs low [%u], xhs high [%u]\n", 
                    __func__, tg_hmax_clk, tg_xhs_low_width, tg_xhs_high_width);

    return err;
}
EXPORT_SYMBOL(tg_set_line_width);

/** 
 * Set frame signal
 * val must be in useconds
 */
int tg_set_frame_width(struct tegracam_device *tc_dev, u32 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
    struct time_generator_params *tg_params = &s_data->cl_priv->tg_params;
	struct device *dev = tc_dev->dev;
    int err = 0;
    u32 tg_frame_high_widht;
    u32 min_trigger_high_width = 1;

    if (*tg_params->is_operation_mode == MASTER_MODE)
        return 0;

    /* Write low width only in normal exposure */
    if (*tg_params->is_shutter_mode == NORMAL_EXPO) {
        tg_params->frame_active_width = 2;
        dev_dbg(dev, "%s: frame low [%u]1H\n",
                 __func__, tg_params->frame_active_width);
    } else {
        min_trigger_high_width = MIN_TRIGGER_HIGH_WIDTH_TGES;     
    }
    tg_frame_high_widht = tg_params->frame_length - tg_params->frame_active_width;

    if (tg_frame_high_widht <= min_trigger_high_width)
        tg_frame_high_widht = min_trigger_high_width;

    tg_params->frame_inactive_width = tg_frame_high_widht;

    if (err) 
        dev_err(dev, "%s: error setting TG XVS WIDTH\n", __func__);

    dev_dbg(dev, "%s: tg frame [%u]lines, frame high [%u]1H\n", 
            __func__, tg_params->frame_length, tg_frame_high_widht);

    return err;
}
EXPORT_SYMBOL(tg_set_frame_width);

/** 
 * Set trigger exposure; only for IS TRIGGER mode
 */
int tg_set_trigger_exposure(struct tegracam_device *tc_dev, u32 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
    struct time_generator_params *tg_params = &s_data->cl_priv->tg_params;
	struct device *dev = tc_dev->dev;
    int err = 0;

    /* If Timing generator is in TRIGGER MODE expand exposure */
    if (tg_params->is_operation_mode == MASTER_MODE
        || *tg_params->is_shutter_mode == NORMAL_EXPO
        || tg_params->tg_mode == TG_TRIGGER_MODE) {
        tg_params->frame_active_width = 0;
        return 0;
    }

    tg_params->frame_active_width = tg_convert_us_to_1h(tc_dev, val);

    if (err) 
        dev_err(dev, "%s: error setting trigger exposure\n", __func__);

    dev_dbg(dev, "%s: frame low [%u]usec, frame low [%u]1H\n", 
        __func__, val, tg_params->frame_active_width);

    return err;
}
EXPORT_SYMBOL(tg_set_trigger_exposure);

/** 
 * Expand trigger exposure; only for TG TRIGGER mode
 */
int tg_expand_trigger_exposure(struct tegracam_device *tc_dev, u32 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
    struct time_generator_params *tg_params = &s_data->cl_priv->tg_params;
	struct device *dev = tc_dev->dev;
    int err = 0;

    /* If Timing generator is in TRIGGER MODE expand exposure */
    if (*tg_params->is_shutter_mode != SEQ_TRIGGER 
        || tg_params->tg_mode == TG_SLAVE_MODE) {
        dev_warn(dev, "%s: not applicable in this mode\n", __func__);
        return 0;
    }

    tg_params->expanded_exposure = tg_convert_us_to_1h(tc_dev, val);

    if (err) 
        dev_err(dev, "%s: error setting expand trigger exposure\n", __func__);

    dev_dbg(dev, "%s: frame low expand val [%u]usec, frame low expand value [%u]1H\n", 
        __func__, val, tg_params->expanded_exposure);

    return err;
}
EXPORT_SYMBOL(tg_expand_trigger_exposure);

/** 
 * Out device from software reset
 */
int crosslink_start(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
    int err = 0;

    err = crosslink_write_reg(tc_dev, CROSSLINK_SW_RST_N_RW, 1);
    if (err) 
        dev_err(dev, "%s: Trigger generator software reset failure\n", __func__);

    return err;
}
EXPORT_SYMBOL(crosslink_start);

/** 
 * Set device to software reset
 */
int crosslink_stop(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
    int err = 0;

    err = crosslink_write_reg(tc_dev, CROSSLINK_SW_RST_N_RW, 0);
    if (err) 
        dev_err(dev, "%s: Trigger generator software reset failure\n", __func__);

    return err;
}
EXPORT_SYMBOL(crosslink_stop);

/** 
 * Start timing generator
 */
int tg_start(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
    int err = 0;

    err = crosslink_write_tg_parameters(tc_dev);

    if (err)
        dev_err(dev, "%s: Trigger generator write parameters failure\n", __func__);
    
    err = crosslink_write_reg(tc_dev, CROSSLINK_TG_ENABLE, 1);

    if (err) 
        dev_err(dev, "%s: Trigger generator start failure\n", __func__);
                                         
    return err;
}
EXPORT_SYMBOL(tg_start);

/** 
 * Stop timing generator
 */
int tg_stop(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
    struct time_generator_params *tg_params = &s_data->cl_priv->tg_params;
	struct device *dev = tc_dev->dev;
    int err = 0;

    /* No need to stop timing generator in IS Master mode */
    if (*tg_params->is_operation_mode == MASTER_MODE){
        return 0;
    }
    err = crosslink_write_reg(tc_dev, CROSSLINK_TG_ENABLE, 0);                       

    if (err) 
        dev_err(dev, "%s: Trigger generator stop failure\n", __func__);

    return err;
}
EXPORT_SYMBOL(tg_stop);

const struct crosslink_readout_mode cl_def_readout_mode = {
    .left_trim_unit = 0,
    .left_trim_lane = 0,
};
EXPORT_SYMBOL(cl_def_readout_mode);

MODULE_DESCRIPTION("Framos Image Sensor common logic");
MODULE_AUTHOR("FRAMOS GmbH");
MODULE_LICENSE("GPL v2");
