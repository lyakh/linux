// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * Copyright(c) 2017 Intel Corporation. All rights reserved.
 *
 * Author: Libin Yang <libin.yang@intel.com>
 *         Luo Xionghu <xionghu.luo@intel.com>
 *         Liam Girdwood <liam.r.girdwood@linux.intel.com>
 */

/*
 * virt IO FE driver
 *
 * The SOF driver thinks this driver is another audio DSP, however the calls
 * made by the SOF driver core do not directly go to HW, but over a virtIO
 * message Q to the virtIO BE driver.
 *
 * The virtIO message Q will use the *exact* same IPC structures as we currently
 * use in the mailbox.
 *
 * The mailbox IO and TX/RX msg functions below will do IO on the virt IO Q.
 */

#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_ring.h>
#include <sound/sof.h>
#include <sound/sof/virtio.h>
//#include <uapi/sound/sof-fw.h>
//#include <uapi/sound/sof-ipc.h>

#include "virtio-fe.h"
#include "ops.h"
#include "sof-priv.h"
#include "intel/hda.h"

static const char *const sof_vq_names[SOF_VIRTIO_NUM_OF_VQS] = {
	SOF_VIRTIO_IPC_CMD_TX_VQ_NAME,
	SOF_VIRTIO_IPC_CMD_RX_VQ_NAME,
	SOF_VIRTIO_IPC_NOT_TX_VQ_NAME,
	SOF_VIRTIO_IPC_NOT_RX_VQ_NAME,
};

static const struct sof_dev_desc virt_desc = {
	.nocodec_fw_filename = NULL,
	.nocodec_tplg_filename = "intel/sof-apl-nocodec.tplg",
	.resindex_lpe_base	= -1,
	.resindex_pcicfg_base	= -1,
	.resindex_imr_base	= -1,
	.irqindex_host_ipc	= -1,
	.resindex_dma_base	= -1,
};

//struct sof_virtio_priv {
//	struct snd_sof_pdata *sof_pdata;
//	struct platform_device *pdev_pcm;
//};

struct sof_vfe {
//	struct sof_virtio_priv *priv;
	struct snd_sof_dev *sdev;

	/* IPC cmd from frontend to backend */
	struct virtqueue *ipc_cmd_tx_vq;

	/* IPC cmd reply from backend to frontend */
	struct virtqueue *ipc_cmd_rx_vq;

	/* IPC notification from backend to frontend */
	struct virtqueue *ipc_not_rx_vq;

	/* IPC notification reply from frontend to backend */
	struct virtqueue *ipc_not_tx_vq;

	/* position update work */
	struct work_struct posn_update_work;

	/* current pending cmd message */
	struct snd_sof_ipc_msg *msg;

	/* current and pending notification */
	struct snd_sof_ipc_msg *not;
	struct sof_ipc_stream_posn *posn;
};

/*
 * IPC Firmware ready.
 */
static int vfe_fw_ready(struct snd_sof_dev *sdev, u32 msg_id)
{
	return 0;
};

/* used to send IPC to BE */
static int vfe_send_msg(struct snd_sof_dev *sdev,
			struct snd_sof_ipc_msg *msg)
{
	struct sof_vfe *vfe;
	struct scatterlist sgs[2];
	int ret = 0;

	vfe = sdev->vfe;

	sg_init_table(sgs, 2);

	sg_set_buf(&sgs[SOF_VIRTIO_IPC_MSG],
		   msg->msg_data, msg->msg_size);
	sg_set_buf(&sgs[SOF_VIRTIO_IPC_REPLY],
		   msg->reply_data, msg->reply_size);

	vfe->msg = msg;

	ret = virtqueue_add_outbuf(vfe->ipc_cmd_tx_vq, sgs, 2,
				   msg->msg_data, GFP_KERNEL);
	if (ret < 0)
		dev_err(sdev->dev, "error: could not send IPC %d\n", ret);

	virtqueue_kick(vfe->ipc_cmd_tx_vq);

	return ret;
}

/* get IPC reply from BE */
//static int vfe_get_reply(struct snd_sof_dev *sdev,
//			 struct snd_sof_ipc_msg *msg)
//{
//	struct sof_vfe *vfe = sdev->vfe;
//
//	vfe->msg = NULL;
//	return 0;
//}

/* get stream message from virtio */
static int vfe_get_stream_message(struct snd_sof_dev *sdev)
{
	struct sof_vfe *vfe = sdev->vfe;
	void *buf = NULL;
	unsigned int buflen = 0;

	buf = virtqueue_get_buf(vfe->ipc_not_rx_vq, &buflen);
	if (unlikely(!buf)) {
		dev_err(sdev->dev, "error rx not from virtio:%d!\n", buflen);
		return -ENOMEM;
	}

	return 0;
}

/* Send the IPC message completed. This means vBE has received the cmd */
static void vfe_cmd_tx_done(struct virtqueue *vq)
{
	struct sof_vfe *vfe = vq->vdev->priv;
	struct snd_sof_ipc_msg *msg = vfe->msg;

	msg->ipc_complete = true;
	wake_up(&msg->waitq);
}

static void vfe_cmd_handle_rx(struct virtqueue *vq)
{
	struct sof_vfe *vfe = vq->vdev->priv;

	/*
	 * normally you'd call .get_reply() from RX (IRQ thread), not clear what
	 * sense setting .msg = NULL makes
	 */
	vfe->msg = NULL;
}

static void vfe_not_tx_done(struct virtqueue *vq)
{
}

static void vfe_posn_update(struct work_struct *work)
{
	struct sof_ipc_stream_posn *posn = NULL;
	struct sof_vfe *vfe =
		container_of(work, struct sof_vfe, posn_update_work);
	struct snd_sof_pcm *spcm;
	struct scatterlist sg;
	struct snd_sof_dev *sdev;
	struct virtqueue *vq;
	unsigned int buflen;
	int direction;

	vq = vfe->ipc_not_rx_vq;
	sdev = vfe->sdev;

	/* virtio protects and make sure no re-entry */
	while ((posn = virtqueue_get_buf(vq, &buflen)) != NULL) {
		spcm = snd_sof_find_spcm_comp(sdev, posn->comp_id, &direction);
		if (!spcm) {
			dev_err(sdev->dev,
				"err: period elapsed for unused component %d\n",
					posn->comp_id);
		} else {
			/*
			 * The position update requirement is valid.
			 * Let's update the position now.
			 */
			memcpy(&spcm->stream[direction].posn, posn, sizeof(*posn));
			snd_pcm_period_elapsed(spcm->stream[direction].substream);
		}

		/* kick back the empty posn buffer immediately */
		sg_init_one(&sg, posn, sizeof(*posn));
		virtqueue_add_inbuf(vq, &sg, 1, posn, GFP_KERNEL);
		virtqueue_kick(vq);
	}
}

/*
 * handle the pos_update, receive the posn and send to up layer, then
 * resend the buffer to BE
 */
static void vfe_not_handle_rx(struct virtqueue *vq)
{
	struct sof_vfe *vfe;

	vfe = vq->vdev->priv;

	schedule_work(&vfe->posn_update_work);
}

static int vfe_register(struct snd_sof_dev *sdev)
{
	return 0;
}

static int vfe_unregister(struct snd_sof_dev *sdev)
{
	return 0;
}

#define SKL_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | \
		     SNDRV_PCM_FMTBIT_S32_LE)

struct snd_soc_dai_driver virtio_dai[] = {
	{
	.name = "SSP4 Pin",
	.playback = SOF_DAI_STREAM("ssp4 Tx", 1, 8,
				   SNDRV_PCM_RATE_8000_192000, SKL_FORMATS),
	.capture = SOF_DAI_STREAM("ssp4 Rx", 1, 8,
				  SNDRV_PCM_RATE_8000_192000, SKL_FORMATS),
	},
};

/* virtio fe ops */
struct snd_sof_dsp_ops snd_sof_vfe_ops = {
	/* device init */
	.probe		= vfe_register,
	.remove		= vfe_unregister,

	/* IPC */
	.send_msg	= vfe_send_msg,
//	.get_reply	= vfe_get_reply,
	.fw_ready	= vfe_fw_ready,

	/* DAI drivers */
	.drv		= virtio_dai,
	.num_drv	= 1,
};

static int sof_vfe_init(struct virtio_device *vdev)
{
	struct device *dev;
	struct snd_soc_acpi_mach *mach;
	struct snd_sof_pdata *sof_pdata;
//	struct sof_virtio_priv *priv;
	int ret;

	dev = &vdev->dev;

//	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
//	if (!priv)
//		return NULL;

	sof_pdata = devm_kzalloc(dev, sizeof(*sof_pdata), GFP_KERNEL);
	if (!sof_pdata)
		return -ENOMEM;

	sof_pdata->drv_name = "sof-nocodec";
//	sof_pdata->is_vfe = 1; /* This is audio vFE device */
	mach = devm_kzalloc(dev, sizeof(*mach), GFP_KERNEL);
	if (!mach)
		return -ENOMEM;

#if IS_ENABLED(CONFIG_SND_SOC_SOF_NOCODEC)
	ret = sof_nocodec_setup(dev, sof_pdata, mach, &virt_desc,
				&snd_sof_vfe_ops);
	if (ret < 0)
		return ret;
#else
	return -ENODEV;
#endif

	mach->pdata = &snd_sof_vfe_ops;

	/*
	 * FIXME:currently, we use the guest local tplg file loading for easy
	 * debug, should swich to service request later.
	 */

	sof_pdata->name = dev_name(&vdev->dev);
	sof_pdata->machine = mach;
	sof_pdata->desc = &virt_desc;
	sof_pdata->dev = dev;
	sof_pdata->vfe = vdev->priv;

	/* register machine driver */
//	sof_pdata->pdev_mach =
//		platform_device_register_data(dev, mach->drv_name, -1,
//					      sof_pdata, sizeof(*sof_pdata));
//	if (IS_ERR(sof_pdata->pdev_mach)) {
//		pr_debug("creating sof machine driver failed\n");
//		return NULL;
//	}

	dev_dbg(dev, "created machine %s\n",
		dev_name(&sof_pdata->pdev_mach->dev));

//	dev_set_drvdata(dev, priv);
//
//	priv->sof_pdata = sof_pdata;

	ret = snd_sof_device_probe(dev, sof_pdata);
	if (ret < 0)
		dev_err(dev, "Cannot register device sof-audio. Error %d\n",
			ret);

	return ret;
}

/* Probe and remove. */
static int vfe_probe(struct virtio_device *vdev)
{
	struct virtqueue *vqs[SOF_VIRTIO_NUM_OF_VQS];
	struct device *dev;
	struct scatterlist sg;
	struct sof_vfe *vfe;
	int ret;

	/* the processing callback number must be the same as the vqueues.*/
	vq_callback_t *cbs[SOF_VIRTIO_NUM_OF_VQS] =	{
		vfe_cmd_tx_done,
		vfe_cmd_handle_rx,
		vfe_not_tx_done,
		vfe_not_handle_rx
	};

	dev = &vdev->dev;

	/*
	 * The below two shouldn't be necessary, it's done in
	 * virtio_pci_modern_probe() by calling dma_set_mask_and_coherent()
	 */
//	dev->coherent_dma_mask = DMA_BIT_MASK(64);
//	dev->dma_mask = &dev->coherent_dma_mask;
	vfe = devm_kzalloc(dev, sizeof(*vfe), GFP_KERNEL);
	if (!vfe)
		return -ENOMEM;

	vdev->priv = vfe;

	INIT_WORK(&vfe->posn_update_work, vfe_posn_update);

	/* create virt queue for vfe to send/receive IPC message. */
	ret = virtio_find_vqs(vdev, SOF_VIRTIO_NUM_OF_VQS,
			      vqs, cbs, sof_vq_names, NULL);
	if (ret) {
		dev_err(dev, "error: find vqs fail with %d\n", ret);
		return ret;
	}

	/* virtques */
	vfe->ipc_cmd_tx_vq = vqs[SOF_VIRTIO_IPC_CMD_TX_VQ];
	vfe->ipc_cmd_rx_vq = vqs[SOF_VIRTIO_IPC_CMD_RX_VQ];
	vfe->ipc_not_tx_vq = vqs[SOF_VIRTIO_IPC_NOT_TX_VQ];
	vfe->ipc_not_rx_vq = vqs[SOF_VIRTIO_IPC_NOT_RX_VQ];

	virtio_device_ready(vdev);

	vfe->posn = kmalloc(sizeof(*vfe->posn), GFP_KERNEL);
	sg_init_one(&sg, vfe->posn, sizeof(struct sof_ipc_stream_posn));
	if (vfe->ipc_not_rx_vq) {
		ret = virtqueue_add_inbuf(vfe->ipc_not_rx_vq,
					  &sg, 1, vfe->posn, GFP_KERNEL);
	}
	virtqueue_kick(vfe->ipc_not_rx_vq);

	/*
	 * add the SOF related functions here, to load the
	 * topology, generate the components, and send IPC
	 */
	return sof_vfe_init(vdev);
}

static void vfe_remove(struct virtio_device *vdev)
{
	/* free virtio resurces and unregister device */
	struct sof_vfe *vfe = vdev->priv;

	vdev->config->reset(vdev);
	vdev->config->del_vqs(vdev);
	cancel_work_sync(&vfe->posn_update_work);
	kfree(vfe->posn);

	/* unregister the SOF device */
	snd_sof_device_remove(&vdev->dev);

	return;
}

static void virtaudio_config_changed(struct virtio_device *vdev)
{
}

void sof_virtio_vfe_init(struct snd_sof_dev *sdev,
			 struct snd_sof_pdata *plat_data)
{
//	sdev->is_vfe = plat_data->is_vfe;
	sdev->is_vfe = IS_ENABLED(CONFIG_SND_SOC_SOF_VIRTIO_FE);

	/*
	 * Currently we only support one VM. comp_id from 0 to
	 * SOF_VIRTIO_MAX_UOS_COMPS - 1 is for SOS. Other comp_id numbers
	 * are for VM1.
	 * TBD: comp_id number range should be dynamically assigned when
	 * multiple VMs are supported.
	 */
	if (sdev->is_vfe) {
		sdev->next_comp_id = SOF_VIRTIO_MAX_UOS_COMPS;
		sdev->vfe = plat_data->vfe;
		sdev->vfe->sdev = sdev;
	}
}

/*
 * Need to patch QEMU to create a virtio audio device, e.g. per
 * -device virtio-snd-pci,snd=snd0 where Device ID must be
 * 0x1040 + VIRTIO_ID_AUDIO and Vendor ID = PCI_VENDOR_ID_REDHAT_QUMRANET
 */
const struct virtio_device_id id_table[] = {
	{VIRTIO_ID_AUDIO, VIRTIO_DEV_ANY_ID},
	{0},
};

/*
 * TODO: There still need a shutdown to handle the case the UOS
 * is poweroff, restart.
 */

static struct virtio_driver vfe_audio_driver = {
	.driver.name	= KBUILD_MODNAME,
	.driver.owner	= THIS_MODULE,
	.id_table	= id_table,
	.probe		= vfe_probe,
	.remove		= vfe_remove,
	.config_changed	= virtaudio_config_changed,
};

EXPORT_SYMBOL(snd_sof_vfe_ops);
module_virtio_driver(vfe_audio_driver);

MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("Sound Open Firmware Virtio FE");
MODULE_LICENSE("Dual BSD/GPL");
