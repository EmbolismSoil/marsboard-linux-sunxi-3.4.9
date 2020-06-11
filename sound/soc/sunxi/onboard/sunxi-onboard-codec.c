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

static int sunxi_i7_onboard_codec_playback_open(struct snd_pcm_substream* pcm)
{
	struct sunxi_i7_chip* chip = snd_pcm_substream_chip(pcm);
	struct snd_pcm_runtime* runtime = pcm->runtime;
	static struct snd_pcm_hardware hw = {
		/*1. 左右声道数据是交替传输的 2. 数据传输是块传输 3. 硬件支持mmap(寄存器控制) 4. 支持暂停 5.支持唤醒*/
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
				   SNDRV_PCM_INFO_BLOCK_TRANSFER |
				   SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
				   SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME),
	     //带符号的16bit小端数据格式
	    .formats = SNDRV_PCM_FMTBIT_S16_LE,
	    //支持的采样率多种多样
	    .rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |SNDRV_PCM_RATE_11025 |\
				   SNDRV_PCM_RATE_22050| SNDRV_PCM_RATE_32000 |\
				   SNDRV_PCM_RATE_44100| SNDRV_PCM_RATE_48000 |SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000 |\
				   SNDRV_PCM_RATE_KNOT),
		.rate_min = 8000,
		.rate_max = 192000,
		.channels_min		= 1,
		.channels_max		= 2,
		.buffer_bytes_max	= 128*1024,//最大的缓冲区大小
		.period_bytes_min	= 1024*4,//最小周期bytes数
		.period_bytes_max	= 1024*32,//最大周期bytes数
		.periods_min		= 4,//最小周期数
		.periods_max		= 8,//最大周期数
		.fifo_size	     	= 32,//fifo字节数
	};
    /*User空间不停向缓冲区buffer写入数据，这个buffer由substream维护，其size为.buffer_bytes_max，
      用户空间的写入是按照frame进行的，每个周期写入一个frame，并且alsa层保证buffer内数据最少由periods_min个周期，
      也就是periods_min个frame*/
	
	static unsigned int rates[] = {
		8000,11025,12000,16000,
		22050,24000,24000,32000,
		44100,48000,96000,192000
	};
	
	static struct snd_pcm_hw_constraint_list hw_constraints_rates = {
		.count	= ARRAY_SIZE(rates),
		.list	= rates,
		.mask	= 0,
	};

	
	runtime->hw = hw;

	int err = 0;
	if ((err = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS)) < 0){
		return err;
	}
	if ((err = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE, &hw_constraints_rates)) < 0){
		return err;
	}

	return 0;

}

static int sunxi_i7_onboard_playback_close(struct snd_pcm_substream* pcm){
	return 0;
}

static int sunxi_i7_onboard_capture_close(struct snd_pcm_substream* pcm){
	return 0;
}

static int sunxi_i7_onboard_playback_hw_params(struct snd_pcm_substream* pcm, struct snd_pcm_hw_params* params)
{
	unsigned long paly_buf_size = params_buffer_bytes(params);
	
}

static struct snd_pcm_ops playback_ops = {
	.open = sunxi_i7_onboard_codec_playback_open,
	.close = sunxi_i7_onboard_playback_close,
	.ioctl = snd_pcm_lib_ioctl,
};

static struct snd_pcm_ops capture_ops = {
	.open = sunxi_i7_onboard_codec_capture_open,
	.close = sunxi_i7_onboard_capture_close,
	.ioctl = snd_pcm_lib_ioctl,
};

static int __devinit sunxi_onboard_codec_pcm_new(struct sunxi_i7_chip* chip)
{
	struct snd_card* card = chip->pcard;
	struct snd_pcm* pcm;
	int err = 0;
	if ((err = snd_pcm_new(card, "ONBOARD-M1 PCM", 0, 1, 1, &pcm)) < 0){
		return err;
	}

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &playback_ops);	
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &capture_ops);
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

static int sunxi_onboard_codec_remove(struct platform_device* pdev)
{
	struct snd_card* card = platform_get_drvdata(pdev);
	snd_card_free(card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

struct platform_driver sunxi_i7_onboard_codec_drv = 
{
	.name = "sunxi-i7-onboard-codec",
	.probe = sunxi_onboard_codec_probe,
	.remove = sunxi_onboard_codec_remove
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
