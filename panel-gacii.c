#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>

#include <video/display_timing.h>
#include <video/mipi_display.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_dsc.h>

#include "panel-gacii.h"

static struct dentry *panel_proc_dir;

struct panel_cmd_header {
	u8 data_type;
	u8 delay;
	u8 payload_length;
} __packed;

struct panel_cmd_desc {
	struct panel_cmd_header header;
	u8 *payload;
};

struct panel_cmd_seq {
	struct panel_cmd_desc *cmds;
	unsigned int cmd_cnt;
};

/**
 * @modes: Pointer to array of fixed modes appropriate for this panel.  If
 *         only one mode then this can just be the address of this the mode.
 *         NOTE: cannot be used with "timings" and also if this is specified
 *         then you cannot override the mode in the device tree.
 * @num_modes: Number of elements in modes array.
 * @timings: Pointer to array of display timings.  NOTE: cannot be used with
 *           "modes" and also these will be used to validate a device tree
 *           override if one is present.
 * @num_timings: Number of elements in timings array.
 * @bpc: Bits per color.
 * @size: Structure containing the physical size of this panel.
 * @delay: Structure containing various delay values for this panel.
 * @bus_format: See MEDIA_BUS_FMT_... defines.
 * @bus_flags: See DRM_BUS_FLAG_... defines.
 */
struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;
	const struct display_timing *timings;
	unsigned int num_timings;

	unsigned int bpc;

	/**
	 * @width: width (in millimeters) of the panel's active display area
	 * @height: height (in millimeters) of the panel's active display area
	 */
	struct {
		unsigned int width;
		unsigned int height;
	} size;

	/**
	 * @prepare: the time (in milliseconds) that it takes for the panel to
	 *           become ready and start receiving video data
	 * @hpd_absent_delay: Add this to the prepare delay if we know Hot
	 *                    Plug Detect isn't used.
	 * @enable: the time (in milliseconds) that it takes for the panel to
	 *          display the first valid frame after starting to receive
	 *          video data
	 * @disable: the time (in milliseconds) that it takes for the panel to
	 *           turn the display off (no content is visible)
	 * @unprepare: the time (in milliseconds) that it takes for the panel
	 *             to power itself down completely
	 * @reset: the time (in milliseconds) that it takes for the panel
	 *         to reset itself completely
	 * @init: the time (in milliseconds) that it takes for the panel to
	 *	  send init command sequence after reset deassert
	 */
	struct {
		unsigned int prepare;
		unsigned int hpd_absent_delay;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
		unsigned int reset;
		unsigned int init;
	} delay;

	u32 bus_format;
	u32 bus_flags;
	int connector_type;

	struct panel_cmd_seq *init_seq;
	struct panel_cmd_seq *exit_seq;
};

struct panel_gacii {
    struct drm_panel base;
    struct mipi_dsi_device *dsi;
    bool prepared;
    bool enabled;
    bool power_invert;
    bool no_hpd;

    const struct panel_desc *desc;
    struct i2c_adapter *ddc;
    struct spi_device *spi;
    struct drm_display_mode override_mode;
    
    struct drm_dsc_picture_parameter_set *pps;
    enum drm_panel_orientation orientation;
};

static struct display_timing wait_test_panel_timing;

static struct panel_desc wait_test_panel = {
    .modes = &wait_test_panel_mode,
    .num_modes = 1,
    .timings = &wait_test_panel_timing,
    .bpc = 6,
    .size = {
        .width = 800,
        .height = 600,
    },
};



static const struct of_device_id dsi_of_match[] = {
	{
		.compatible = "simple-panel-dsi",
		.data = NULL,
	}, {
        /* sentinel */
    }
};

static int panel_simple_dsi_of_get_desc_data(struct device *dev,
					     struct panel_desc_dsi *desc)
{
	struct device_node *np = dev->of_node;
	u32 val;
	int err;

	err = panel_simple_of_get_desc_data(dev, &desc->desc);
	if (err)
		return err;

	if (!of_property_read_u32(np, "dsi,flags", &val))
		desc->flags = val;
	if (!of_property_read_u32(np, "dsi,format", &val))
		desc->format = val;
	if (!of_property_read_u32(np, "dsi,lanes", &val))
		desc->lanes = val;

	return 0;
}

static int panel_gacii_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct panel_simple *panel;
	struct device *dev = &dsi->dev;
	const struct panel_desc_dsi *desc;
	struct panel_desc_dsi *d;
	const struct of_device_id *id;
	int err;

	id = of_match_node(dsi_of_match, dsi->dev.of_node);
	if (!id)
		return -ENODEV;

	if (!id->data) {
		d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
		if (!d)
			return -ENOMEM;

		err = panel_simple_dsi_of_get_desc_data(dev, d);
		if (err) {
			dev_err(dev, "failed to get desc data: %d\n", err);
			return err;
		}
	}

	desc = id->data ? id->data : d;

	err = panel_simple_probe(&dsi->dev, &desc->desc);
	if (err < 0)
		return err;

	panel = dev_get_drvdata(dev);
	panel->dsi = dsi;

	if (!panel->base.backlight) {
		struct backlight_properties props;

		memset(&props, 0, sizeof(props));
		props.type = BACKLIGHT_RAW;
		props.brightness = 255;
		props.max_brightness = 255;

		panel->base.backlight =
			devm_backlight_device_register(dev, "dcs-backlight",
						       dev, panel, &dcs_bl_ops,
						       &props);
		if (IS_ERR(panel->base.backlight)) {
			err = PTR_ERR(panel->base.backlight);
			dev_err(dev, "failed to register dcs backlight: %d\n",
				err);
			return err;
		}
	}

	dsi->mode_flags = desc->flags;
	dsi->format = desc->format;
	dsi->lanes = desc->lanes;

	err = mipi_dsi_attach(dsi);
	if (err) {
		struct panel_simple *panel = dev_get_drvdata(&dsi->dev);

		drm_panel_remove(&panel->base);
	}

	return err;
}

static int panel_gacii_remove(struct device *dev)
{
    struct panel_gacii *panel = dev_get_drvdata(dev);
    
    drm_panel_remove(&panel->base);
    drm_panel_disable(&panel->base);
    drm_panel_unprepare(&panel->base);

    if (panel->ddc)
        put_device(&panel->base->dev);

    return 0;
}

static int panel_gacii_dsi_remove(struct mipi_dsi_device *dsi)
{
	int err;

	err = mipi_dsi_detach(dsi);
	if (err < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", err);

	return panel_gacii_remove(&dsi->dev);
}

static void panel_gacii_dsi_shutdown(struct mipi_dsi_device *dsi)
{
    panel_gacii_shutdown(&dsi->dev);
}

static struct mipi_dsi_driver panel_gacii_dsi_driver = {
	.driver = {
		.name = "panel-gacii-dsi",
		.of_match_table = dsi_of_match,
	},
	.probe = panel_gacii_dsi_probe,
	.remove = panel_gacii_dsi_remove,
	.shutdown = panel_gacii_dsi_shutdown,
};

static ssize_t proc_panel_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
    return -ESRCH;      /* No such process */
}

#define RX_MAX_PRIMITIVE_LENGTH_BYTES   256
static uint8_t rx_primitive_buffer[RX_MAX_PRIMITIVE_LENGTH_BYTES];
static uint16_t rx_primitive_length = 0;
static uint16_t rx_package_length = 0;
static uint8_t rx_state = RXS_WAITING_FOR_HEADER_START;

void rx_package_handle( uint8_t * rx_buffer, uint16_t rx_length )
{
    uint8_t cmd_code;
    cmd_code = rx_buffer[2];
    switch(cmd_code)
    {
        case CMD_PANEL_DISPLAY_MODE:

        break;
        case CMD_PANEL_DESC:
        
        break;
        case CMD_PANEL_DCS_COMANDS:

        break;
        case CMD_I2C_COMMANDS:

        break;
        case CMD_SPI_COMMANDS:

        break;
        case CMD_FPGA_CONTROL:

        break;
        case CMD_POWER_ON:

        break;
        case CMD_POWER_OFF:

        break;
    }
}


static void interface_rx_statemachine(uint8_t rx_byte)
{
    if( rx_primitive_length >= RX_MAX_PRIMITIVE_LENGTH_BYTES ){
        rx_state = RXS_WAITING_FOR_HEADER_START;        
    }
    switch(rx_state){
    case RXS_WAITING_FOR_HEADER_START:
        rx_primitive_length = 0;
        if( rx_byte == RXS_PACKET_HEAD_H ){
            rx_primitive_buffer[rx_primitive_length++] = rx_byte;
            rx_state = RXS_WAITING_FOR_HEAD_L;
        }
    break;
    case RXS_WAITING_FOR_HEAD_L:
        rx_primitive_buffer[rx_primitive_length++] = rx_byte;
        if( rx_byte == RXS_PACKET_HEAD_L ){
            rx_byte = RXS_WAITING_FOR_LENGTH_H;
        }
        else {
            rx_byte = RXS_WAITING_FOR_HEADER_START;
        }
    break;
    case RXS_WAITING_FOR_LENGTH_H:
        rx_primitive_buffer[rx_primitive_length++] = rx_byte;
        rx_package_length = ( rx_byte & 0xff )<<8;
        rx_byte = RXS_WAITING_FOR_LENGTH_L;
    break;
    case RXS_WAITING_FOR_LENGTH_L:
        rx_primitive_buffer[rx_primitive_length++] = rx_byte;
        rx_package_length |= ( rx_byte & 0xff );
    break;
    case RXS_WAITING_FOR_PACKAGE_CONTENT:
        rx_primitive_buffer[rx_primitive_length++] = rx_byte;
        if( rx_package_length+6 == rx_primitive_length ){
            if( rx_primitive_buffer[rx_primitive_length-2] == RXS_PACKET_TAIL_H && 
                rx_primitive_buffer[rx_primitive_length-1] == RXS_PACKET_TAIL_L ){
                /* handle the receive package */
                rx_package_handle( rx_primitive_buffer, rx_primitive_length );
                rx_byte = RXS_WAITING_FOR_HEADER_START;
            } 
        }
    break;
    }
}

static ssize_t proc_panel_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    // 在此处处理写入debugFS目录中文件的逻辑
    /*
        按照协议进行处理。
        /--head--/--cmd--/--length--/--tail--/
        head:0xaa 0xcc
        cmd:0x1 panel display mode
        cmd:0x2 panel desc
        cmd:0x3 panel dcs comands
        cmd:0x4 i2c commands
        cmd:0x5 spi commands
        cmd:0x6 fpga control
        cmd:0x7 power on
        cmd:0x8 power off
        tail:0xdd 0xee
    */
    #define MAX_BUF_SIZE 256
    char user_data[MAX_BUF_SIZE];
    ssize_t retval = -ENOMEM;
    uint32_t index;
    if (count > MAX_BUF_SIZE)
        return -EINVAL;
    
    if (copy_from_user(user_data, buffer, count))
        return -EFAULT;
    
    for( index = 0; index < count; index++ ) {
        interface_rx_statemachine(user_data[index]);
    }
    
    retval = count;
    return retval;
}

static const struct file_operations proc_panel_ops = {
    .owner    = THIS_MODULE,
    .read     = proc_panel_read,
    .write    = proc_panel_write,
};

static int panel_gacii_probe(struct device *dev, const struct panel_desc *desc)
{
    struct panel_gacii *panel;
    struct device_node *ddc;
    
    panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
    if(!panel)
        return -ENOMEM;

    panel->enabled = false;
    panel->prepared = false;
    panel->desc = desc;

    ddc = of_parse_phandle(dev->of_node, "ddc-i2c-bus", 0);
    if (ddc) {
        panel->ddc = of_find_i2c_adapter_by_node(ddc);
        of_node_put(ddc);

        if (!panel->ddc) {
            err = -EPROBE_DEFER;
            dev_err(dev, "failed to find ddc-i2c-bus: %d\n", err);
            return err;
        }
    }
    
    connector_type = desc->connector_type;

    drm_panel_init(&panel->base, dev, NULL, connector_type);
    
    err = drm_panel_of_backlight(&panel->base);
    if (err) {
        dev_err(dev, "failed to find backlight: %d\n", err);
        goto free_ddc;
    }

    drm_panel_add(&panel->base);
    
    dev_set_drvdata(dev, panel);

    return 0;

free_ddc:
    if (panel->ddc)
        put_device(&panel->ddc->dev);

    return err;
}

static int panel_gacii_remove(struct device *dev)
{
    struct panel_gacii *panel = dev_get_drvdata(dev);
    
    drm_panel_remove(&panel->base);
    drm_panel_disable(&panel->base);
    drm_panel_unprepare(&panel->base);

    if (panel->ddc)
        put_device(&panel->ddc->dev);

    return 0;
}

static void panel_gacii_shutdown(struct device *dev)
{
    struct panel_gacii *panel = dev_get_drvdata(dev);
    drm_panel_disable(&panel->base);
    drm_panel_unprepare(&panel->base);
}

static int panel_gacii_platform_probe(struct platform_device *pdev)
{

}

static int panel_gacii_platform_remove(struct platform_device *pdev)
{

}

static int panel_gacii_platform_shutdown(struct platform_device *pdev)
{

}

static struct platform_driver panel_gacii_platform_driver = {
    .driver = {
        .name = "panel-gacii",
        .of_match_table = platform_of_match,
    },
    .probe = panel_gacii_platform_probe,
    .remove = panel_gacii_platform_remove,
    .shutdown = panel_gacii_platform_shutdown,
};

static int __init panel_gacii_init(void)
{
	int err;
    panel_proc_dir = debugfs_create_dir("gacii_panel", NULL);
    if(!panel_proc_dir) {
        printk(KERN_ERR "gacii panel:failed to create debugfs directory\n");
        return -ENOMEM;
    }

    debugfs_create_file("panel_control", 0644, panel_proc_dir, NULL, &proc_panel_ops);

	err = platform_driver_register(&panel_gacii_platform_driver);
	if (err < 0)
		return err;

	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI)) {
		err = mipi_dsi_driver_register(&panel_gacii_dsi_driver);
		if (err < 0)
			return err;
	}

    return 0;
}
module_init(panel_gacii_init);

static void __exit panel_gacii_exit(void)
{
	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
		mipi_dsi_driver_unregister(&panel_gacii_dsi_driver);

	platform_driver_unregister(&panel_gacii_platform_driver);

    debugfs_remove_recursive(panel_proc_dir);
}
module_exit(panel_gacii_exit);

MODULE_AUTHOR("Angelo <zhangba@gaci-o.com>");
MODULE_DESCRIPTION("DRM Driver for Gacii test Panels");
MODULE_LICENSE("GPL");

