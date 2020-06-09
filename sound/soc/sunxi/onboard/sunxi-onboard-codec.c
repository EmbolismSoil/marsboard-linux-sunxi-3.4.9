#include <linux/init.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/initval.h>

static struct sunxi_i7_chip
{
	struct snd_card* card;
}

static int sunxi_i7_chip_free(struct snd_device* dev)
{
	sunxi_i7_chip* chip = (sunxi_i7_chip*)dev->device_data;
	kzfree(chip);
}


static struct snd_device_ops dev_ops = {
	.dev_free = sunxi_i7_chip_free
};

static int sunxi_i7_chip_create(struct snd_card* card, struct platform_device* pdev, struct sunxi_i7_chip** chip)
{
	static struct 
	*chip = NULL;
	*chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (*chip == NULL){
		return -ENOMEM;
	}

	struct sunxi_i7_chip* pchip = *chip;
	pchip->card = card;

	//TODO: 这里开始分配私有资源

	int err = snd_device_new(card, SNDRV_DEV_CODEC, pchip, &dev_ops);
	if (err < 0){
		sunxi_i7_chip_free(chip);
		return err;
	}

	return 0;
}

struct platform_driver sunxi_i7_onboard_codec = 
{
	.name = "sunxi-i7-onboard-codec",
	.probe = NULL
};