#include <linux/init.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/initval.h>
#include "sunxi-onboard-codec.h"

static void sunxi_i7_chip_free(struct snd_card* card)
{
	sunxi_i7_chip* chip = card->private_data;
	kzfree(chip);
}


static int sunxi_i7_chip_create(struct snd_card* card, struct platform_device* pdev, struct sunxi_i7_chip** chip)
{
	static struct 
	*chip = NULL;
	*chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (*chip == NULL){
		return -ENOMEM;
	}

	struct sunxi_i7_chip* pchip = *chip;
	pchip->pcard = card;
	pchip->pdev = pdev;
	card->private_data = chip;
	card->private_free = sunxi_i7_chip_free;
	
	return 0;
}

static int __devinit sunxi_onboard_codec_probe(struct platform_device* pdev)
{
	struct snd_card* card = NULL;
	int ret = snd_card_create(0, "sunxi-codec", THIS_MODULE, 0, &card);
	if (ret < 0){
		return ret;
	}

	struct sunxi_i7_chip* chip;
	ret = sunxi_i7_chip_create(card, pdev, &chip);
	if (ret < 0){
		goto failed_chip;
	}

	//创建控制接口
	
	//创建pcm

	strcpy(card->driver, "sunxi-i7-onboard-codec");
	strcpy(card->shortname, "sunxi-i7-onboard-codec");
	strcpy(card->longname, "sunxi-i7-onboard-codec Audio Codec");
	snd_card_set_dev(card, &pdev->dev);
	
	ret = snd_card_register(card);
	if (ret < 0){
		goto failed_card;
	}else{
		platform_set_drvdata(pdev, card);
		return 0;
	}

failed_card:
	sunxi_i7_chip_free(card);
	
failed_chip:
	snd_card_free(card);
	return ret;
}

struct platform_driver sunxi_i7_onboard_codec_drv = 
{
	.name = "sunxi-i7-onboard-codec",
	.probe = NULL
};

struct platform_device sunxi_i7_onboard_codec_dev = {
	.name = "sunxi-i7-onboard-codec"
};

static int __init sunxi_onboard_codec_init(void)
{
	int ret = 0;
	ret = platform_device_register(&sunxi_i7_onboard_codec_dev);
	if (ret < 0){
		return ret;
	}

	ret = platform_driver_register(&sunxi_i7_onboard_codec_drv);
	if (ret < 0){
		platform_device_unregister(&sunxi_i7_onboard_codec_dev);
		return ret;
	}

	return 0;
}

static int __exit sunxi_onboard_codec_exit(void)
{
	platform_device_unregister(&sunxi_i7_onboard_codec_dev);
	platform_driver_unregister(&sunxi_i7_onboard_codec_drv);
}

module_init(sunxi_onboard_codec_init);
module_exit(sunxi_onboard_codec_exit);
MODULE_DESCRIPTION("sunxi-i7-onboard-codec-driver");
MODULE_LICENSE("GPL");
