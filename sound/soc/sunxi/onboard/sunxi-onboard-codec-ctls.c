#include <sound/control.h>
#include "sunxi-onboard-codec.h"

#if 0
static const struct snd_kcontrol_new sun7i_dac_ctls[] = {
	/*SUNXI_DAC_ACTL = 0x10,PAVOL*/
	CODEC_SINGLE("Master Playback Volume", SUNXI_DAC_ACTL, 0, 0x3f, 0),
	CODEC_SINGLE("Playback Switch", SUNXI_DAC_ACTL,6,1,0),//全局输出开关
	CODEC_SINGLE("FmL Switch",SUNXI_DAC_ACTL,17,1,0),//Fm左开关
	CODEC_SINGLE("FmR Switch",SUNXI_DAC_ACTL,16,1,0),//Fm右开关
	CODEC_SINGLE("LineL Switch",SUNXI_DAC_ACTL,19,1,0),//Line左开关
	CODEC_SINGLE("LineR Switch",SUNXI_DAC_ACTL,18,1,0),//Line右开关
	CODEC_SINGLE("Ldac Left Mixer",SUNXI_DAC_ACTL,15,1,0),
	CODEC_SINGLE("Rdac Right Mixer",SUNXI_DAC_ACTL,14,1,0),
	CODEC_SINGLE("Ldac Right Mixer",SUNXI_DAC_ACTL,13,1,0),
	CODEC_SINGLE("Mic Input Mux",SUNXI_DAC_ACTL,9,15,0),//from bit 9 to bit 12.Mic（麦克风）输入静音
	CODEC_SINGLE("MIC output volume", SUNXI_DAC_ACTL, 20, 7, 0),
	/*	FM Input to output mixer Gain Control
	 * 	From -4.5db to 6db,1.5db/step,default is 0db
	 *	-4.5db:0x0,-3.0db:0x1,-1.5db:0x2,0db:0x3
	 *	1.5db:0x4,3.0db:0x5,4.5db:0x6,6db:0x7
	 */
	CODEC_SINGLE("Fm output Volume", SUNXI_DAC_ACTL, 23,  7, 0),
	/*	Line-in gain stage to output mixer Gain Control
	 *	0:-1.5db,1:0db
	 */
	CODEC_SINGLE("Line output Volume", SUNXI_DAC_ACTL, 26, 1, 0),
};
#endif

static unsigned long encode_property(int isbool, int writable, int stereo, int max, int step)
{
	return type | access << 1 | count << 2 | max << 3 | step << 35;
}

#define __KCONTROL_VALUE_TYPE(value) (((value) & 0x01) ? SNDRV_CTL_ELEM_TYPE_BOOLEAN, SNDRV_CTL_ELEM_TYPE_INTEGER)
#define __KCONTROL_VALUE_ACCESS(value) (((value) & (0x1 << 1)) ? SNDRV_CTL_ELEM_ACCESS_READWRITE : SNDRV_CTL_ELEM_ACCESS_READ)
#define __KCONTROL_VALUE_COUNT(value) (((value) & (0x1 << 2)) ? 2 : 1)
#define __KCONTROL_VALUE_MAX(value) (((value) & (0xffffffff << 3)) >> 3)
#define __KCONTROL_VALUE_STEP(value) (((value) & (0x6ffff << 35)) >> 35) 

static int sunxi_7i_ctls_info(struct snd_kcontrol *kcontrol, struct	snd_ctl_elem_info	*uinfo)
{
	struct sunxi_i7_chip* chip = snd_kcontrol_chip(kcontrol);
	unsigned long pri = kcontrol->private_value;

	uinfo->access = __KCONTROL_VALUE_ACCESS(pri);
	uinfo->count = __KCONTROL_VALUE_COUNT(pri);
	uinfo->type = __KCONTROL_VALUE_TYPE(pri);

	if (uinfo->type == SNDRV_CTL_ELEM_TYPE_INTEGER){
		uinfo->value.integer.min = 0;		
		uinfo->value.integer.max = __KCONTROL_VALUE_MAX(pri);
		uinfo->value.integer.step = __KCONTROL_VALUE_STEP(pri);
	}

	return 0;
}

static int sunxi_7i_ctls_get(struct snd_kcontrol	*kcontrol, struct	snd_ctl_elem_value	*ucontrol)
{
	return 0;
}


static int sunxi_7i_ctls_put(struct snd_kcontrol	*kcontrol, struct	snd_ctl_elem_value	*ucontrol)
{
	return 0;
}

#define SUNXI_ONBOARD_CODEC_CTL_NEW_ITEM(name, pri) \
{\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,\
	.name = name,\
	.info = sunxi_7i_ctls_info,\
	.put = sunxi_7i_ctls_put,\
	.get = sunxi_7i_ctls_get,\
	.private_value = pri\
}


//主要是播放控制
static struct snd_kcontrol_new sunxi_i7_onboard_codec_play_ctls = {
	SUNXI_ONBOARD_CODEC_CTL_NEW_ITEM("Master Playback Volume", encode_property(0, 1, 0, 100, 0)),
}
