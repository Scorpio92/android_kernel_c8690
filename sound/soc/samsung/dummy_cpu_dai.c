#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <plat/audio.h>
#include <mach/map.h>

static int dummy_cpu_dai_probe(struct snd_soc_dai *dai)
{
       return 0;
}

static int dummy_cpu_dai_remove(struct snd_soc_dai *dai)
{
       return 0;
}

static struct snd_soc_dai_driver dummy_cpu_dai_drv = {
       .name = "dummy_cpu_dai",
       .symmetric_rates = 1,
       .probe = dummy_cpu_dai_probe,
       .remove = dummy_cpu_dai_remove,
       .ops = NULL,
       .playback.channels_min = 2,
       .playback.channels_max = 2,
       .playback.rates = SNDRV_PCM_RATE_8000_96000,
       .playback.formats = (SNDRV_PCM_FMTBIT_S8 |      \
                            SNDRV_PCM_FMTBIT_S16_LE |  \
                            SNDRV_PCM_FMTBIT_S24_LE),
};

static __devinit int dummy_cpu_dai_dev_probe(struct platform_device *pdev)
{
       snd_soc_register_dai(&pdev->dev, &dummy_cpu_dai_drv);

       return 0;
}

static __devexit int dummy_cpu_dai_dev_remove(struct platform_device *pdev)
{
       snd_soc_unregister_dai(&pdev->dev);

       return 0;
}

static struct platform_driver dummy_cpu_dai_dev_driver = {
       .probe  = dummy_cpu_dai_dev_probe,
       .remove = dummy_cpu_dai_dev_remove,
       .driver = {
               .name = "dummy_cpu_dai",
               .owner = THIS_MODULE,
       },
};

static struct platform_device dummy_cpu_dai_dev_device = {
       .name = "dummy_cpu_dai",
       .id = -1,
};

static int __init dummy_cpu_dai_dev_init(void)
{
       int ret = platform_device_register(&dummy_cpu_dai_dev_device);
       if (ret) {
               printk("[wenpin] dummy cpu dai device register failed\n");
               return ret;
       }
       return platform_driver_register(&dummy_cpu_dai_dev_driver);
}
module_init(dummy_cpu_dai_dev_init);

static void __exit dummy_cpu_dai_dev_exit(void)
{
       platform_driver_unregister(&dummy_cpu_dai_dev_driver);
}
module_exit(dummy_cpu_dai_dev_exit);

MODULE_AUTHOR("wenpin cui <wenpin.cui@samsung.com>");
MODULE_DESCRIPTION("Dummy CPU DAI");
MODULE_LICENSE("GPL");
