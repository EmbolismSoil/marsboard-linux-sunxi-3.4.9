#include <linux/init.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <plat/dma_compat.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/initval.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <plat/sys_config.h>
#include <mach/system.h>
#include "sunxi-onboard-codec.h"

static void sunxi_i7_chip_free(struct snd_card* card)
{
	struct sunxi_i7_chip* chip = card->private_data;
	kzfree(chip);
}


static int sunxi_i7_chip_create(struct snd_card* card, struct platform_device* pdev, struct sunxi_i7_chip** chip)
{
	*chip = NULL;
	*chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (*chip == NULL){
		return -ENOMEM;
	}

	struct sunxi_i7_chip* pchip = *chip;

	//设置采样率和时钟
	struct clk* codec_pll2clk;
	struct clk* codec_moduleclk;
	struct clk* codec_apbclk;
	codec_apbclk = clk_get(NULL,"apb_audio_codec");
	clk_enable(codec_apbclk);
	
	codec_pll2clk = clk_get(NULL,"audio_pll");
	codec_moduleclk = clk_get(NULL,"audio_codec");
	clk_set_rate(codec_moduleclk, 24576000);
	clk_enable(codec_pll2clk);
	clk_set_parent(codec_moduleclk, codec_pll2clk);	
	clk_enable(codec_moduleclk);

	struct resource* res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	void* baseaddr = ioremap(res->start, res->end - res->start);
	pchip->baseaddr = baseaddr;

	pchip->gpio_pa = gpio_request_ex("audio_para", "audio_pa_ctrl");
	
	pchip->pcard = card;
	pchip->pdev = pdev;
	pchip->codec_pll2clk = codec_pll2clk;
	pchip->codec_moduleclk = codec_moduleclk;
	pchip->codec_apbclk = codec_apbclk;
	
	card->private_data = chip;
	card->private_free = sunxi_i7_chip_free;
	
	return 0;
}

static void sunxi_i7_rtd_free(struct snd_pcm_runtime* rtd)
{
	if (rtd->private_data != NULL){
		struct sunxi_i7_chip* chip = rtd->private_data;
		clk_put(chip->codec_apbclk);
		clk_put(chip->codec_moduleclk);	
		clk_put(chip->codec_pll2clk);
		iounmap(chip->baseaddr);
		kzfree(rtd->private_data);
	}
}

static int sunxi_i7_onboard_codec_playback_open(struct snd_pcm_substream* pcm)
{
	//struct sunxi_i7_chip* chip = snd_pcm_substream_chip(pcm);
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
		.period_bytes_min	= 1024*4,//最小周期bytes数, 一个周期处理1024个frame
		.period_bytes_max	= 1024*32,//最大周期bytes数
		.periods_min		= 4,//最小周期数, 缓冲区内最少要有4个period数据，也就是4*4*1024字节
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
	struct sunxi_i7_stream_runtime* rtd = kzalloc(sizeof(struct sunxi_i7_stream_runtime), GFP_KERNEL);
	if (rtd == NULL){
		return -ENOMEM;
	}

	memset(rtd, 0, sizeof(struct sunxi_i7_stream_runtime));

	pcm->runtime->private_data = rtd;
	pcm->runtime->private_free = sunxi_i7_rtd_free;
	spin_lock_init(&rtd->lock);

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

static int sunxi_i7_dma_push(struct sunxi_i7_stream_runtime* rtd)
{
	//将缓冲区的数据通过dma发送
	dma_addr_t pos = rtd->pos;
	struct sunxi_dma_params* dma = rtd->dma_params;
	
	int ret = 0;
	while(rtd->periods < rtd->period_min){
		unsigned int len = min(rtd->period_bytes, rtd->dma_end - pos);
		ret = sunxi_dma_enqueue(dma, pos, len, 0);
		if (ret == 0){
			pos = pos + len;
			pos = pos < rtd->dma_end ? pos : rtd->dma_base_addr;
			rtd->periods += 1;
		}else{
			break;
		}
	}

	rtd->pos = pos;
	return ret;
}

static void sunxi_i7_play_dma_callback(struct sunxi_dma_params* dma, void* arg)
{
	struct snd_pcm_substream* pcm = arg;
	snd_pcm_period_elapsed(pcm);

	struct sunxi_i7_stream_runtime* rtd = pcm->runtime->private_data;
	spin_lock(&rtd->lock);
	if (rtd->periods > 0){
		rtd->periods -= 1;		
	}
	sunxi_i7_dma_push(rtd);
	spin_unlock(&rtd->lock);
}

static int sunxi_i7_onboard_playback_hw_params(struct snd_pcm_substream* pcm, struct snd_pcm_hw_params* params)
{
	//分配好内存然后初始化dma
	static struct sunxi_dma_params dma_params = {
		.client.name = "sunxi-i7-codec play",
		.dma_addr = SUNXI_I7_CODEC_BASE_ADDR + SUNXI_I7_DAC_TXDATA
	};

	struct sunxi_i7_stream_runtime* rtd = pcm->runtime->private_data;
	if (rtd->dma_params == NULL){
		rtd->dma_params = &dma_params;
	}

	//初始化dma相关
	unsigned long buf_bytes = params_buffer_bytes(params);
	snd_pcm_lib_malloc_pages(pcm, buf_bytes);
	int ret = sunxi_dma_request(rtd->dma_params, 0);
	
	if (ret < 0){
		printk(KERN_ERR"failed to request dma. ret = %d\n", ret);
		return ret;
	}

	ret = sunxi_dma_set_callback(rtd->dma_params, sunxi_i7_play_dma_callback, pcm);

	if (ret < 0){
		printk(KERN_ERR"failed to set dma callback, ret = %d\n", ret);
		sunxi_dma_release(rtd->dma_params);
		rtd->dma_params = NULL;
		return ret;
	}
	

	struct snd_pcm_runtime* pcm_rtd = pcm->runtime;
	spin_lock_irq(&rtd->lock);
	rtd->periods = 0;
	rtd->period_bytes = params_period_bytes(params);
	rtd->period_min = pcm_rtd->hw.periods_min;
	rtd->dma_base_addr = pcm_rtd->dma_addr;
	rtd->pos = rtd->dma_base_addr;
	rtd->dma_end = rtd->dma_base_addr + buf_bytes;
	spin_unlock_irq(&rtd->lock);

	return 0;
}

static void sunxi_i7_codec_clk_set(struct snd_pcm_substream* pcm)
{
	struct sunxi_i7_chip* chip = snd_pcm_substream_chip(pcm);
	struct clk* codec_pll2clk = chip->codec_pll2clk;
	struct clk* codec_moduleclk = chip->codec_moduleclk;
        void* baseaddr = chip->baseaddr;

	unsigned int rate = pcm->runtime->rate;
	unsigned int reg_val = 0;
	switch(rate){
		case 44100:
			clk_set_rate(codec_pll2clk, 22579200);
			clk_set_rate(codec_moduleclk, 22579200);
			reg_val = readl(baseaddr + SUNXI_I7_DAC_FIFOC);
			reg_val &=~(7<<29);
			reg_val |=(0<<29);
			writel(reg_val, baseaddr + SUNXI_I7_DAC_FIFOC);
		
			break;
		case 22050:
			clk_set_rate(codec_pll2clk, 22579200);
			clk_set_rate(codec_moduleclk, 22579200);
			reg_val = readl(baseaddr + SUNXI_I7_DAC_FIFOC);
			reg_val &=~(7<<29);
			reg_val |=(2<<29);
			writel(reg_val, baseaddr + SUNXI_I7_DAC_FIFOC);
			break;
		case 11025:
			clk_set_rate(codec_pll2clk, 22579200);
			clk_set_rate(codec_moduleclk, 22579200);
			reg_val = readl(baseaddr + SUNXI_I7_DAC_FIFOC);
			reg_val &=~(7<<29);
			reg_val |=(4<<29);
			writel(reg_val, baseaddr + SUNXI_I7_DAC_FIFOC);
			break;
		case 48000:
			clk_set_rate(codec_pll2clk, 24576000);
			clk_set_rate(codec_moduleclk, 24576000);
			reg_val = readl(baseaddr + SUNXI_I7_DAC_FIFOC);
			reg_val &=~(7<<29);
			reg_val |=(0<<29);
			writel(reg_val, baseaddr + SUNXI_I7_DAC_FIFOC);
			break;
		case 96000:
			clk_set_rate(codec_pll2clk, 24576000);
			clk_set_rate(codec_moduleclk, 24576000);
			reg_val = readl(baseaddr + SUNXI_I7_DAC_FIFOC);
			reg_val &=~(7<<29);
			reg_val |=(7<<29);
			writel(reg_val, baseaddr + SUNXI_I7_DAC_FIFOC);
			break;
		case 192000:
			clk_set_rate(codec_pll2clk, 24576000);
			clk_set_rate(codec_moduleclk, 24576000);
			reg_val = readl(baseaddr + SUNXI_I7_DAC_FIFOC);
			reg_val &=~(7<<29);
			reg_val |=(6<<29);
			writel(reg_val, baseaddr + SUNXI_I7_DAC_FIFOC);
			break;
		case 32000:
			clk_set_rate(codec_pll2clk, 24576000);
			clk_set_rate(codec_moduleclk, 24576000);
			reg_val = readl(baseaddr + SUNXI_I7_DAC_FIFOC);
			reg_val &=~(7<<29);
			reg_val |=(1<<29);
			writel(reg_val, baseaddr + SUNXI_I7_DAC_FIFOC);
			break;
		case 24000:
			clk_set_rate(codec_pll2clk, 24576000);
			clk_set_rate(codec_moduleclk, 24576000);
			reg_val = readl(baseaddr + SUNXI_I7_DAC_FIFOC);
			reg_val &=~(7<<29);
			reg_val |=(2<<29);
			writel(reg_val, baseaddr + SUNXI_I7_DAC_FIFOC);
			break;
		case 16000:
			clk_set_rate(codec_pll2clk, 24576000);
			clk_set_rate(codec_moduleclk, 24576000);
			reg_val = readl(baseaddr + SUNXI_I7_DAC_FIFOC);
			reg_val &=~(7<<29);
			reg_val |=(3<<29);
			writel(reg_val, baseaddr + SUNXI_I7_DAC_FIFOC);
			break;
		case 12000:
			clk_set_rate(codec_pll2clk, 24576000);
			clk_set_rate(codec_moduleclk, 24576000);
			reg_val = readl(baseaddr + SUNXI_I7_DAC_FIFOC);
			reg_val &=~(7<<29);
			reg_val |=(4<<29);
			writel(reg_val, baseaddr + SUNXI_I7_DAC_FIFOC);
			break;
		case 8000:
			clk_set_rate(codec_pll2clk, 24576000);
			clk_set_rate(codec_moduleclk, 24576000);
			reg_val = readl(baseaddr + SUNXI_I7_DAC_FIFOC);
			reg_val &=~(7<<29);
			reg_val |=(5<<29);
			writel(reg_val, baseaddr + SUNXI_I7_DAC_FIFOC);
			break;
		default:
			clk_set_rate(codec_pll2clk, 24576000);
			clk_set_rate(codec_moduleclk, 24576000);
			reg_val = readl(baseaddr + SUNXI_I7_DAC_FIFOC);
			reg_val &=~(7<<29);
			reg_val |=(0<<29);
			writel(reg_val, baseaddr + SUNXI_I7_DAC_FIFOC);
			break;			
	}

}

static void sunxi_i7_codec_channel_set(struct snd_pcm_substream* pcm)
{
	unsigned int regval;
	struct sunxi_i7_chip* chip = snd_pcm_substream_chip(pcm);
	void* addr = chip->baseaddr;
	void* fifoc = addr + SUNXI_I7_DAC_FIFOC;	
	regval = readl(fifoc);
	if (pcm->runtime->channels == 1){
		regval |= (1 << 6);
	}else{
		regval &=~ (1 << 6);
	}
	writel(regval, fifoc);

}

static void sunxi_i7_set_bit(uint32_t* addr, uint32_t const bit)
{
	uint32_t regval = readl(addr);
	uint32_t opval = regval;	
	opval |= (1 << bit);
	if (opval != regval){
		writel(opval, addr);
	}
}

static void sunxi_i7_clear_bit(uint32_t* addr, uint32_t const bit)
{
	uint32_t regval = readl(addr);
	uint32_t opval = regval;
	opval &=~ (1 << bit);
	if (regval != opval){
		writel(opval, addr);
	}
}

static void sunxi_i7_codec_cmd(struct snd_pcm_substream* pcm, sunxi_i7_codec_cmd_t const cmd)
{
	struct sunxi_i7_chip* chip = snd_pcm_substream_chip(pcm);
	const void* baseaddr = chip->baseaddr;
	void* reg = NULL;
	switch(cmd){
		case SUNXI_I7_CODEC_FLUSH_FIFO_CMD:
			reg = baseaddr + SUNXI_I7_DAC_FIFOC;
			sunxi_i7_set_bit(reg, 0);
			break;
			
		case SUNXI_I7_CODEC_ENABLE_DRQ_CMD:
			reg = baseaddr + SUNXI_I7_DAC_FIFOC;
			sunxi_i7_set_bit(reg, 4);
		
		default:
			break;
	}
}

static void sunxi_i7_codec_hw_open(struct snd_pcm_substream* pcm)
{
	struct sunxi_i7_chip* chip = snd_pcm_substream_chip(pcm);
	void *baseaddr = chip->baseaddr;
	
	//DAC ENABLE
	uint32_t* reg = baseaddr + SUNXI_I7_DAC_DPC;
	sunxi_i7_set_bit(reg, 31);

	reg = baseaddr + SUNXI_I7_DAC_FIFOC;
	
	//FIFO刷新
	sunxi_i7_codec_cmd(pcm, SUNXI_I7_CODEC_FLUSH_FIFO_CMD);

	//设置DRQ LEVEL
	//uint32_t regval = readl(reg);
	//regval |= (0xf << 8);

	if(pcm->runtime->rate > 32000){
		sunxi_i7_clear_bit(reg, 28);
	}else{
		sunxi_i7_set_bit(reg, 28);
	}

	sunxi_i7_set_bit(reg, 24);
	sunxi_i7_clear_bit(reg, 26);


	reg = baseaddr + SUNXI_I7_DAC_ACTRL;
	sunxi_i7_set_bit(reg, 30);
	sunxi_i7_set_bit(reg, 31);

	sunxi_i7_set_bit(reg, 8); //DAC链接内部运放
}

static int sunxi_i7_codec_dma_config(struct snd_pcm_substream* pcm)
{
	struct sunxi_i7_stream_runtime* rtd = pcm->runtime->private_data;
	struct sunxi_dma_params* dma = rtd->dma_params;
	dma_config_t conf;
	
	memset(&conf, 0, sizeof(conf));
	conf.xfer_type.src_data_width = DATA_WIDTH_16BIT; //16bit
	conf.xfer_type.src_bst_len = DATA_BRST_4; //BRST 4
	conf.xfer_type.dst_data_width = DATA_WIDTH_16BIT;
	conf.xfer_type.dst_bst_len = DATA_BRST_4;
	
	conf.address_type.src_addr_mode = NDMA_ADDR_INCREMENT;
	conf.address_type.dst_addr_mode = NDMA_ADDR_NOCHANGE;
	conf.src_drq_type = N_SRC_SDRAM;
	conf.dst_drq_type = N_DST_AUDIO_CODEC_DA;
	conf.bconti_mode = false;
	conf.irq_spt = CHAN_IRQ_FD;
	
	int ret = sunxi_dma_config(dma, &conf, 0);
	if (ret < 0){
		return ret;
	}

	rtd->periods = 0;
	if (sunxi_dma_flush(dma) == 0){
		rtd->pos = rtd->dma_base_addr;
	}

	return 0;
}

static int sunxi_i7_onboard_playback_prepare(struct snd_pcm_substream* pcm)
{
	//硬件打开codec
	sunxi_i7_codec_hw_open(pcm);

	//设置采样频率
	sunxi_i7_codec_clk_set(pcm);

	//设置通道	
	sunxi_i7_codec_channel_set(pcm);
	
	//DMA配置
	int ret = sunxi_i7_codec_dma_config(pcm);
	if(ret < 0){
		printk(KERN_ERR"config dma failed.\n");
		return ret;
	}

	//加载dma数据
	struct sunxi_i7_stream_runtime* rtd = pcm->runtime->private_data;
	sunxi_i7_dma_push(rtd);
}

static snd_pcm_uframes_t sunxi_i7_onboard_playback_pointer(struct snd_pcm_substream* pcm)
{
	struct sunxi_i7_stream_runtime* rtd = pcm->runtime->private_data;
	dma_addr_t src;
	dma_addr_t dest;	
	unsigned long res = 0;

	spin_lock(&rtd->lock);
	sunxi_dma_getcurposition(rtd->dma_params, &src, &dest);
	res = src + rtd->period_bytes - rtd->dma_base_addr;
	spin_unlock(&rtd->lock);
	return bytes_to_frames(pcm->runtime, res);
}

static int sunxi_i7_onboard_playback_hw_start(struct snd_pcm_substream* pcm)
{
	struct sunxi_i7_chip* chip = snd_pcm_substream_chip(pcm);
	gpio_write_one_pin_value(chip->gpio_pa, 1, "audio_pa_ctrl");

	sunxi_i7_codec_cmd(pcm, SUNXI_I7_CODEC_FLUSH_FIFO_CMD);
	sunxi_i7_codec_cmd(pcm, SUNXI_I7_CODEC_ENABLE_DRQ_CMD);
	return 0;
}

static int sunxi_i7_onboard_playback_trigger(struct snd_pcm_substream* pcm, int cmd)
{
	switch(cmd){
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			sunxi_i7_onboard_playback_hw_start(pcm);			
	}
	return 0;
}

static struct snd_pcm_ops playback_ops = {
	.open = sunxi_i7_onboard_codec_playback_open,
	.close = sunxi_i7_onboard_playback_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = sunxi_i7_onboard_playback_hw_params,
	.prepare = sunxi_i7_onboard_playback_prepare,
	.pointer = sunxi_i7_onboard_playback_pointer
};

#if 0
static struct snd_pcm_ops capture_ops = {
	.open = sunxi_i7_onboard_codec_capture_open,
	.close = sunxi_i7_onboard_capture_close,
	.ioctl = snd_pcm_lib_ioctl
};
#endif

static int __devinit sunxi_onboard_codec_pcm_new(struct sunxi_i7_chip* chip)
{
	struct snd_card* card = chip->pcard;
	struct snd_pcm* pcm;
	int err = 0;
	if ((err = snd_pcm_new(card, "ONBOARD-M1 PCM", 0, 1, 0, &pcm)) < 0){
		return err;
	}

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &playback_ops);	
	//snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &capture_ops);
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

static struct platform_driver sunxi_i7_onboard_codec_drv = 
{
	.probe = sunxi_onboard_codec_probe,
	.remove = sunxi_onboard_codec_remove,
	.driver = {
		.name = "sunxi-i7-onboard-codec",
	}
};
	
static struct resource sunxi_codec_resource[] = {
	[0] = {
		.start = SUNXI_I7_CODEC_BASE_ADDR,
		.end = SUNXI_I7_CODEC_BASE_ADDR + 0x40,
		.flags = IORESOURCE_MEM
	}
};

static struct platform_device sunxi_i7_onboard_codec_dev = {
	.name = "sunxi-i7-onboard-codec",
	.num_resources = 1,
	.resource = sunxi_codec_resource
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

static void __exit sunxi_onboard_codec_exit(void)
{
	platform_device_unregister(&sunxi_i7_onboard_codec_dev);
	platform_driver_unregister(&sunxi_i7_onboard_codec_drv);
}


module_init(sunxi_onboard_codec_init);
module_exit(sunxi_onboard_codec_exit);

MODULE_DESCRIPTION("sunxi ONBOARD CODEC ALSA codec driver");
MODULE_AUTHOR("software");
MODULE_LICENSE("GPL");
