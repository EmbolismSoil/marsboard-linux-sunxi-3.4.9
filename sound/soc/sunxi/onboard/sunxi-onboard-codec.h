#ifndef __SUNXI_ONBOARD_CODEC_H__
#define __SUNXI_ONBOARD_CODEC_H__


#define SUNXI_I7_CODEC_BASE_ADDR         (0x01c22c00)
#define SUNXI_I7_DAC_TXDATA		(0x0c)
#define SUNXI_I7_DAC_FIFOC (0x04)
#define SUNXI_I7_DAC_DPC (0x00)
#define SUNXI_I7_DAC_ACTRL (0x10)

struct sunxi_i7_chip
{
	struct snd_card* pcard;
	struct platform_device* pdev;
	struct clk* codec_pll2clk, codec_moduleclk, codec_apbclk;
	void* baseaddr;
};

struct sunxi_i7_stream_runtime
{
	sunxi_dma_params* dma_params;
	unsigned int periods; //当前dma buffer中有多少个周期
	unsigned int period_bytes; //一个周期有多少byte
	unsigned int period_min; //最少有多少个周期
	dma_addr_t dma_base_addr;
	dma_addr_t pos; //当前写位置
	dma_addr_t dma_end; //dma结束地址
	spinlock_t lock;
};

#endif
