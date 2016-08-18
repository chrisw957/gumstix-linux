/*
 * LCD panel driver for Startek KD035FM
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/mutex.h>
#include <linux/gpio.h>

#include <video/omapdss.h>
#include <video/omap-panel-data.h>

#define OVERO_GPIO_LCD_EN 144

struct kd035fm_data {
	struct mutex lock;
};

static struct omap_video_timings kd035fm_timings = {
	.x_res		= 320,                                  // unit pixels
	.y_res		= 480,                                  // unit pixels

	.pixelclock	= 8500,

	.hsw		= 18,                                   // Horizontal sync pulse width in pixel clocks
	.hfp		= 10,                                   // Horizontal front portch in pixel clocks
	.hbp		= 10,                                   // Horizontal back porch in pixel clocks

	.vsw		= 2,                                    // Vertical sync pulse width in line clocks
	.vfp		= 2,                                    // Vertical front porch in line clocks
	.vbp		= 2,                                    // Vertical back porch in line clocks

	.vsync_level	= OMAPDSS_SIG_ACTIVE_LOW,
	.hsync_level	= OMAPDSS_SIG_ACTIVE_LOW,
	.data_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
	.de_level	= OMAPDSS_SIG_ACTIVE_HIGH,
	.sync_pclk_edge = OMAPDSS_DRIVE_SIG_OPPOSITE_EDGES,
};

struct panel_drv_data {
	struct omap_dss_device dssdev;
	struct omap_dss_device *in;

	struct spi_device *spi;

	int data_lines;

	struct omap_video_timings videomode;

	/* used for non-DT boot, to be removed */
	int backlight_gpio;

	struct gpio_desc *reset_gpio;
};

#define to_panel_data(p) container_of(p, struct panel_drv_data, dssdev)

static void kd035fm_transfer(struct spi_device *spi, int cmd,
			     const u16 *wbuf, int wlen, u8 *rbuf, int rlen)
{
	struct spi_message m;
	struct spi_transfer     *x, xfer[5];
	int r;
	u16 w;

	spi_message_init(&m);

	memset(xfer, 0, sizeof(xfer));
	x = &xfer[0];

	cmd &=  0xff;
	x->tx_buf = &cmd;
	x->bits_per_word = 9;
	x->len = 2;

	if (wlen)
		x->cs_change = 1;

	spi_message_add_tail(x, &m);


	if (wlen) {
		x++;
		x->tx_buf = wbuf;
		x->len = wlen * 2;
		x->bits_per_word = 9;
		x->cs_change = 1;
		spi_message_add_tail(x, &m);
	}


	if (rlen) {
		x++;
		x->tx_buf   = NULL;
		x->rx_buf       = &w;
		x->len          = 1;
		x->bits_per_word = 8;
		x->cs_change = 1;
		spi_message_add_tail(x, &m);

		if (rlen > 1) {
			x->bits_per_word = 9;
			x->len = 2;
			x->cs_change = 0;

			x++;
			x->bits_per_word = 8;
			x->rx_buf = &rbuf[1];
			x->len = rlen - 1;
			x->cs_change = 1;
			spi_message_add_tail(x, &m);
		}
	}

	r = spi_sync(spi, &m);
	if (r < 0)
		printk("kd035fm spi_sync returned error\n.");

	if (rlen)
		rbuf[0] = w & 0xff;
}


static inline void kd035fm_cmd(struct spi_device *spi, int cmd)
{
	kd035fm_transfer(spi, cmd, NULL, 0, NULL, 0);
}


static inline void kd035fm_write(struct spi_device *spi,
				 int reg, const u16 *buf, int len)
{
	kd035fm_transfer(spi, reg, buf, len, NULL, 0);
}


static inline void kd035fm_read(struct spi_device *spi,
				int reg, u8 *buf, int len)
{
	kd035fm_transfer(spi, reg, NULL, 0, buf, len);
}

static void init_kd035fm_panel(struct spi_device *spi)
{
	u16 buffer[32];
	u8 rbuf[32] = { 0x00 };

	printk("kd035fm: init regs\n");

	gpio_set_value(OVERO_GPIO_LCD_EN, 1);
	mdelay(10);

	gpio_set_value(OVERO_GPIO_LCD_EN, 0);
	mdelay(20);

	gpio_set_value(OVERO_GPIO_LCD_EN, 1);
	mdelay(250);

	// Power Control 1
	buffer[0] = 0x08 + 0x100;
	buffer[1] = 0x06 + 0x100;
	kd035fm_write(spi, 0xc0, buffer, 2);

	// Power Control 2
	buffer[0] = 0x43 + 0x100;
	buffer[1] = 0x00 + 0x100;
	kd035fm_write(spi, 0xc1, buffer, 2);

	// VCOM control
	buffer[0] = 0x00 + 0x100;
	buffer[1] = 0x40 + 0x100;
	kd035fm_write(spi, 0xc5, buffer, 2);

	//Power Control 3
	buffer[0] = 0x33 + 0x100;
	kd035fm_write(spi, 0xc2, buffer, 1);

	// Interface Mode Control
	// SDA_EN = 0 (DIN and DOUT are used for 3/4 wire spi)
	// VSPL = 0 (VSYNC = low level sync clock)
	// HSPL = 0 (HSYNC = low level sync clock)
	// DPL = 1 (Data deteched at the falling time)
	// EPL = 0 (DE is high enble)
	buffer[0] = 0x02 + 0x100;
	kd035fm_write(spi, 0xb0, buffer, 1);

	// Frame Rate Control (normal mode/full colors)
	buffer[0] = 0xb0 + 0x100;
	buffer[1] = 0x12 + 0x100;
	kd035fm_write(spi, 0xb1, buffer, 2);

	// Display Inversion Control
	buffer[0] = 0x02 + 0x100;
	kd035fm_write(spi, 0xb4, buffer, 1);

	// Entry Mode Set
	buffer[0] = 0x86 + 0x100;
	kd035fm_write(spi, 0xb7, buffer, 1);

	// Positive Gamma Control
	buffer[0] = 0x0f + 0x100;
	buffer[1] = 0x29 + 0x100;
	buffer[2] = 0x25 + 0x100;
	buffer[3] = 0x0b + 0x100;
	buffer[4] = 0x0e + 0x100;
	buffer[5] = 0x07 + 0x100;
	buffer[6] = 0x42 + 0x100;
	buffer[7] = 0x87 + 0x100;
	buffer[8] = 0x2c + 0x100;
	buffer[9] = 0x06 + 0x100;
	buffer[10] = 0x0f + 0x100;
	buffer[11] = 0x02 + 0x100;
	buffer[12] = 0x0b + 0x100;
	buffer[13] = 0x07 + 0x100;
	buffer[14] = 0x00 + 0x100;
	kd035fm_write(spi, 0xe0, buffer, 15);

	// Negative Gamma Control
	buffer[0] = 0x0f + 0x100;
	buffer[1] = 0x38 + 0x100;
	buffer[2] = 0x34 + 0x100;
	buffer[3] = 0x0d + 0x100;
	buffer[4] = 0x10 + 0x100;
	buffer[5] = 0x09 + 0x100;
	buffer[6] = 0x53 + 0x100;
	buffer[7] = 0x87 + 0x100;
	buffer[8] = 0x3d + 0x100;
	buffer[9] = 0x08 + 0x100;
	buffer[10] = 0x11 + 0x100;
	buffer[11] = 0x04 + 0x100;
	buffer[12] = 0x1a + 0x100;
	buffer[13] = 0x16 + 0x100;
	buffer[14] = 0x00 + 0x100;
	kd035fm_write(spi, 0xe1, buffer, 15);

	// ??
	buffer[0] = 0x1e + 0x100;
	buffer[1] = 0xa3 + 0x100;
	buffer[2] = 0x32 + 0x100;
	buffer[3] = 0x02 + 0x100;
	buffer[4] = 0xb2 + 0x100;
	buffer[5] = 0x52 + 0x100;
	buffer[6] = 0xff + 0x100;
	buffer[7] = 0x10 + 0x100;
	buffer[8] = 0x00 + 0x100;
	kd035fm_write(spi, 0xf2, buffer, 9);

	// ??
	buffer[0] = 0x21 + 0x100;
	buffer[1] = 0x04 + 0x100;
	kd035fm_write(spi, 0xf8, buffer, 2);

	// Memory Access Control
	buffer[0] = 0x08 + 0x100;
	kd035fm_write(spi, 0x36, buffer, 1);

	// Interface Pixel Format (18bit)
	buffer[0] = 0x66 + 0x100;
	kd035fm_write(spi, 0x3a, buffer, 1);

	// ??
	buffer[0] = 0x00 + 0x100;
	buffer[1] = 0x08 + 0x100;
	kd035fm_write(spi, 0xf9, buffer, 2);

	// ??
	buffer[0] = 0x36 + 0x100;
	buffer[1] = 0x04 + 0x100;
	buffer[2] = 0x00 + 0x100;
	buffer[3] = 0x3c + 0x100;
	buffer[4] = 0x0f + 0x100;
	buffer[5] = 0x8f + 0x100;
	kd035fm_write(spi, 0xf1, buffer, 6);

	// Display Function Control
	buffer[0] = 0x30 + 0x100;
	buffer[1] = 0x22 + 0x100;
	buffer[2] = 0x3b + 0x100;
	kd035fm_write(spi, 0xb6, buffer, 3);

	// Column Address Set
	buffer[0] = 0x00 + 0x100;
	buffer[1] = 0x00 + 0x100;
	buffer[2] = 0x01 + 0x100;
	buffer[3] = 0x3f + 0x100;
	kd035fm_write(spi, 0x2a, buffer, 4);

	// Page Address Set
	buffer[0] = 0x00 + 0x100;
	buffer[1] = 0x00 + 0x100;
	buffer[2] = 0x01 + 0x100;
	buffer[3] = 0xdf + 0x100;
	kd035fm_write(spi, 0x2b, buffer, 4);

	// Display Inversion On
	kd035fm_cmd(spi, 0x21);

	// Turn off sleep mode
	kd035fm_cmd(spi, 0x11);

	//delay 120 msec? usec? minutes?
	mdelay(200);

	// Display On
	kd035fm_cmd(spi, 0x29);

	// Memory Write
	kd035fm_cmd(spi, 0x2c);
}

static int kd035fm_connect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	if (omapdss_device_is_connected(dssdev))
		return 0;

	r = in->ops.dpi->connect(in, dssdev);
	if (r)
		return r;

	init_kd035fm_panel(ddata->spi);

	return 0;
}

static void kd035fm_disconnect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (!omapdss_device_is_connected(dssdev))
		return;

	in->ops.dpi->disconnect(in, dssdev);
}

static int kd035fm_enable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	if (!omapdss_device_is_connected(dssdev))
		return -ENODEV;

	if (omapdss_device_is_enabled(dssdev))
		return 0;

	if (ddata->data_lines)
		in->ops.dpi->set_data_lines(in, ddata->data_lines);
	in->ops.dpi->set_timings(in, &ddata->videomode);

	r = in->ops.dpi->enable(in);
	if (r)
		return r;

	if (ddata->reset_gpio)
		gpiod_set_value_cansleep(ddata->reset_gpio, 1);

	if (gpio_is_valid(ddata->backlight_gpio))
		gpio_set_value_cansleep(ddata->backlight_gpio, 1);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void kd035fm_disable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (!omapdss_device_is_enabled(dssdev))
		return;

	if (ddata->reset_gpio)
		gpiod_set_value_cansleep(ddata->reset_gpio, 0);

	if (gpio_is_valid(ddata->backlight_gpio))
		gpio_set_value_cansleep(ddata->backlight_gpio, 0);

	in->ops.dpi->disable(in);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}


static void kd035fm_set_timings(struct omap_dss_device *dssdev,
				struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	ddata->videomode = *timings;
	dssdev->panel.timings = *timings;

	in->ops.dpi->set_timings(in, timings);
}


static void kd035fm_get_timings(struct omap_dss_device *dssdev,
				struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	*timings = ddata->videomode;
}


static int kd035fm_check_timings(struct omap_dss_device *dssdev,
				 struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (!timings || timings->x_res != dssdev->panel.timings.x_res ||
	    timings->y_res != dssdev->panel.timings.y_res)
		return -EINVAL;

	return 0;
}

static struct omap_dss_driver kd035fm_ops = {
	.connect	= kd035fm_connect,
	.disconnect	= kd035fm_disconnect,

	.enable		= kd035fm_enable,
	.disable	= kd035fm_disable,

	.set_timings	= kd035fm_set_timings,
	.get_timings	= kd035fm_get_timings,
	.check_timings	= kd035fm_check_timings,

	.get_resolution = omapdss_default_get_resolution,
};

static int kd035fm_probe_pdata(struct spi_device *spi)
{
	const struct panel_kd035fm_platform_data *pdata;
	struct panel_drv_data *ddata = dev_get_drvdata(&spi->dev);
	struct omap_dss_device *dssdev, *in;
	int r;


	pdata = dev_get_platdata(&spi->dev);

	in = omap_dss_find_output(pdata->source);
	if (in == NULL) {
		dev_err(&spi->dev, "failed to find video source '%s'\n",
			pdata->source);
		return -EPROBE_DEFER;
	}

	ddata->in = in;

	ddata->data_lines = pdata->data_lines;

	dssdev = &ddata->dssdev;
	dssdev->name = pdata->name;

	r = devm_gpio_request_one(&spi->dev, pdata->reset_gpio,
				  GPIOF_OUT_INIT_HIGH, "panel enable");
	if (r)
		goto err_gpio;

	ddata->reset_gpio = gpio_to_desc(pdata->reset_gpio);

	ddata->backlight_gpio = pdata->backlight_gpio;

	return 0;
err_gpio:
	omap_dss_put_device(ddata->in);
	return r;
}


static int kd035fm_probe_of(struct spi_device *spi)
{
	struct device_node *node = spi->dev.of_node;
	struct panel_drv_data *ddata = dev_get_drvdata(&spi->dev);
	struct omap_dss_device *in;
	struct gpio_desc *gpio;

	gpio = devm_gpiod_get(&spi->dev, "reset");
	if (IS_ERR(gpio)) {
		dev_err(&spi->dev, "failed to parse reset gpio\n");
		return PTR_ERR(gpio);
	} else {
		gpiod_direction_output(gpio, 1);
		ddata->reset_gpio = gpio;
	}

	ddata->backlight_gpio = -ENOENT;

	in = omapdss_of_find_source_for_first_ep(node);
	if (IS_ERR(in)) {
		dev_err(&spi->dev, "failed to find video source\n");
		return PTR_ERR(in);
	}

	ddata->in = in;

	return 0;
}


static int kd035fm_panel_spi_probe(struct spi_device *spi)
{
	struct panel_drv_data *ddata;
	struct omap_dss_device *dssdev;
	int r;

	ddata = devm_kzalloc(&spi->dev, sizeof(*ddata), GFP_KERNEL);
	if (ddata == NULL)
		return -ENOMEM;

	dev_set_drvdata(&spi->dev, ddata);

	ddata->spi = spi;

	if (dev_get_platdata(&spi->dev)) {
		r = kd035fm_probe_pdata(spi);
		if (r)
			return r;
	} else if (spi->dev.of_node) {
		r = kd035fm_probe_of(spi);
		if (r)
			return r;
	} else
		return -ENODEV;

	if (gpio_is_valid(ddata->backlight_gpio)) {
		r = devm_gpio_request_one(&spi->dev, ddata->backlight_gpio,
					  GPIOF_OUT_INIT_LOW, "panel backlight");
		if (r)
			goto err_gpio;
	}

	ddata->videomode = kd035fm_timings;

	dssdev = &ddata->dssdev;
	dssdev->dev = &spi->dev;
	dssdev->driver = &kd035fm_ops;
	dssdev->type = OMAP_DISPLAY_TYPE_DPI;
	dssdev->owner = THIS_MODULE;
	dssdev->panel.timings = ddata->videomode;
	dssdev->phy.dpi.data_lines = ddata->data_lines;

	r = omapdss_register_display(dssdev);
	if (r) {
		dev_err(&spi->dev, "Failed to register panel\n");
		goto err_reg;
	}

	return 0;

err_reg:
err_gpio:
	omap_dss_put_device(ddata->in);
	return r;
}


static int kd035fm_panel_spi_remove(struct spi_device *spi)
{
	struct panel_drv_data *ddata = dev_get_drvdata(&spi->dev);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	struct omap_dss_device *in = ddata->in;

	omapdss_unregister_display(dssdev);

	kd035fm_disable(dssdev);
	kd035fm_disconnect(dssdev);

	omap_dss_put_device(in);

	return 0;
}


static const struct of_device_id kd035fm_of_match[] = {
	{ .compatible = "omapdss,startek,kd035fm", },
	{},
};

MODULE_DEVICE_TABLE(of, kd035fm_of_match);

static struct spi_driver kd035fm_spi_driver = {
	.probe				= kd035fm_panel_spi_probe,
	.remove				= kd035fm_panel_spi_remove,
	.driver				= {
		.name			= "panel_startek_kd035fm",
		.owner			= THIS_MODULE,
		.of_match_table		= kd035fm_of_match,
		.suppress_bind_attrs	= true,
	},
};

module_spi_driver(kd035fm_spi_driver);

MODULE_ALIAS("spi:startek,kd035fm");
MODULE_AUTHOR("Chris Whittenburg <whittenburg@gmail.com>");
MODULE_DESCRIPTION("Startek KD035FM LCD Panel driver");
MODULE_LICENSE("GPL");
