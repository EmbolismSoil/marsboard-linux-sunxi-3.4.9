/*
 * ASOC架构的板载ADC/DAC音频驱动
 */
#include "linux/platform_device.h"

#define SUNXI_I7_ONBOARD_CODEC_RATE SNDRV_PCM_RATE_8000_96000

static snd_soc_dai_driver sunxi_i7_soc_dai_driver = {
	.name = "sunxi-onboard-codec",
	/*
	const char *stream_name;
	u64 formats;			
	unsigned int rates;		
	unsigned int rate_min;	
	unsigned int rate_max;		
	unsigned int channels_min;	
	unsigned int channels_max;	
	unsigned int sig_bits;		// number of bits of content
	*/
	.playback = {
		.stream_name = "playback",
		.rates = SUNXI_I7_ONBOARD_CODEC_RATE,
		.channels_max = 1,
		.channels_min = 1,
		
	}
};

static int sunxi_i7_codec_platform_probe(struct platform_device* pdev)
{
	//在这里注册soc_codec和soc_codec_driver
	return 0;
}

static struct platform_device sunxi_i7_soc_codec_device = {
	.name = "sunxi_i7_soc_codec"
};

static struct platform_driver sunxi_i7_soc_codec_driver = {
	.name = "sunxi_i7_soc_codec"
};

static int __init sunxi_i7_soc_codec_init(void)
{
	int err = 0;
	if ((err = platform_device_register(&sunxi_i7_soc_codec_device)) != 0){
		return err;
	}

	if ((err = platform_driver_register(&sunxi_i7_soc_codec_driver)) != 0){
		return err;
	}

	return err;
}

static int __exit sunxi_i7_soc_codec_exit(void)
{
	return platform_driver_unregister(&sunxi_i7_soc_codec_driver);
}