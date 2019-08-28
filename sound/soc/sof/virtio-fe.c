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

	struct sof_vfe_ipc_tplg_resp tplg;
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
	struct sof_vfe *vfe = sdev->pdata->vfe;
	int ret;
#if 1
	struct scatterlist sgs[2];

	sg_init_table(sgs, 2);
	sg_set_buf(&sgs[SOF_VIRTIO_IPC_MSG],
		   msg->msg_data, msg->msg_size);
	sg_set_buf(&sgs[SOF_VIRTIO_IPC_REPLY],
		   msg->reply_data, msg->reply_size);

	ret = virtqueue_add_outbuf(vfe->ipc_cmd_tx_vq, sgs, 2,
				   msg->msg_data, GFP_KERNEL);
#else
	struct scatterlist sg_out, sg_in, *sgs[] = {&sg_out, &sg_in};

	sg_init_one(&sg_out, msg->msg_data, msg->msg_size);
	sg_init_one(&sg_in, msg->reply_data, msg->reply_size);

	ret = virtqueue_add_sgs(vfe->ipc_cmd_tx_vq, sgs, 1, 1, msg->msg_data,
				GFP_KERNEL);
#endif
	if (ret < 0)
		dev_err(sdev->dev, "error: could not send IPC %d\n", ret);

	vfe->msg = msg;

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
//static int vfe_get_stream_message(struct snd_sof_dev *sdev)
//{
//	struct sof_vfe *vfe = sdev->vfe;
//	void *buf = NULL;
//	unsigned int buflen = 0;
//
//	buf = virtqueue_get_buf(vfe->ipc_not_rx_vq, &buflen);
//	if (unlikely(!buf)) {
//		dev_err(sdev->dev, "error rx not from virtio:%d!\n", buflen);
//		return -ENOMEM;
//	}
//
//	return 0;
//}

/* Send the IPC message completed. This means vBE has received the cmd */
static void vfe_cmd_tx_done(struct virtqueue *vq)
{
	struct sof_vfe *vfe = vq->vdev->priv;
	struct snd_sof_ipc_msg *msg = vfe->msg;
	struct sof_ipc_reply *reply = msg->reply_data;

	msg->reply_error = reply->error;

	/* Firmware panic? */
	if (msg->reply_error == -ENODEV)
		vfe->sdev->ipc->disable_ipc_tx = true;

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

static int vfe_run(struct snd_sof_dev *sdev)
{
	sdev->boot_complete = true;
	wake_up(&sdev->boot_wait);
	return 0;
}

static void vfe_block_read(struct snd_sof_dev *sdev, u32 bar,
			   u32 offset, void *dest,
			   size_t size)
{
}

static void vfe_block_write(struct snd_sof_dev *sdev, u32 bar,
			    u32 offset, void *src,
			    size_t size)
{
}

static int vfe_load_firmware(struct snd_sof_dev *sdev)
{
	return 0;
}

static void vfe_ipc_msg_data(struct snd_sof_dev *sdev,
			     struct snd_pcm_substream *substream,
			     void *p, size_t sz)
{
}

static int vfe_ipc_pcm_params(struct snd_sof_dev *sdev,
			      struct snd_pcm_substream *substream,
			      const struct sof_ipc_pcm_params_reply *reply)
{
	return 0;
}

//struct sof_ipc_ctrl_data {
//	struct sof_ipc_reply rhdr;
//	u8 data[SOF_IPC_MSG_MAX_SIZE - sizeof(struct sof_ipc_reply)];
//};

static int vfe_request_topology(struct snd_sof_dev *sdev, const char *name,
				struct firmware *fw)
{
	struct sof_vfe_ipc_tplg_req rq = {
		.hdr = {
			.size = sizeof(rq),
			.cmd = SOF_IPC_GLB_TPLG_MSG | SOF_IPC_TPLG_VFE_GET,
		},
	};
	struct sof_vfe *vfe = sdev->pdata->vfe;
	struct sof_vfe_ipc_tplg_resp *partdata = kmalloc(SOF_IPC_MSG_MAX_SIZE,
							 GFP_KERNEL);
//	void *reply = &vfe->tplg;
	size_t part_size = SOF_IPC_MSG_MAX_SIZE - sizeof(partdata->reply);
//	struct sof_ipc_ctrl_data data;
//	unsigned int i, n_msg;
	int ret;

	if (!partdata)
		return -ENOMEM;

	strncpy(rq.file_name, name, sizeof(rq.file_name));

	mutex_lock(&sdev->ipc->tx_mutex);

	do {
		size_t size;

		ret = sof_ipc_tx_message_unlocked(sdev->ipc, rq.hdr.cmd,
						  &rq, sizeof(rq), partdata,
						  SOF_IPC_MSG_MAX_SIZE);
		if (ret < 0)
			goto free;

		size = min_t(size_t, partdata->reply.hdr.size, part_size);
		memcpy(vfe->tplg.data + rq.offset, partdata->data, size);
//		dev_dbg(sdev->dev, "copy 0x%x 0x%x %zu bytes at offset %zu\n",
//			*(u32 *)(vfe->tplg.data + rq.offset),
//			*(u32 *)(vfe->tplg.data + rq.offset + size - 4),
//			size, rq.offset);
		if (!rq.offset)
			fw->size = partdata->reply.hdr.size;
		rq.offset += part_size;
	} while (partdata->reply.hdr.size > part_size);

	rq.hdr.cmd = SOF_IPC_GLB_TPLG_MSG | SOF_IPC_TPLG_VFE_COMP_ID;
	rq.hdr.size = sizeof(rq.hdr);
	ret = sof_ipc_tx_message_unlocked(sdev->ipc, rq.hdr.cmd,
					  &rq, rq.hdr.size, partdata,
					  sizeof(partdata->reply) + sizeof(u32));
	if (ret < 0)
		goto free;

	sdev->next_comp_id = *(u32 *)partdata->data;

	fw->data = vfe->tplg.data;
	fw->pages = NULL;

free:
	mutex_unlock(&sdev->ipc->tx_mutex);

	kfree(partdata);
	return 0;
}

static int vfe_trace_init(struct snd_sof_dev *sdev, u32 *stream_tag)
{
	return -ENODEV;
}

static int vfe_sof_runtime_suspend(struct snd_sof_dev *sof_dev,
				   int state)
{
	return 0;
}

static int vfe_sof_runtime_resume(struct snd_sof_dev *sof_dev)
{
	return 0;
}

/* virtio fe ops */
struct snd_sof_dsp_ops snd_sof_vfe_ops = {
	/* device init */
	.probe		= vfe_register,
	.remove		= vfe_unregister,

	/* PM */
	.runtime_suspend = vfe_sof_runtime_suspend,
	.runtime_resume = vfe_sof_runtime_resume,

	/* IPC */
	.send_msg	= vfe_send_msg,
//	.get_reply	= vfe_get_reply,
	.fw_ready	= vfe_fw_ready,

	/* DAI drivers */
	.drv		= virtio_dai,
	.num_drv	= 1,

	.run		= vfe_run,
	.block_read	= vfe_block_read,
	.block_write	= vfe_block_write,
	.load_firmware	= vfe_load_firmware,
	.ipc_msg_data	= vfe_ipc_msg_data,
	.ipc_pcm_params	= vfe_ipc_pcm_params,

	.trace_init	= vfe_trace_init,

	.request_topology = vfe_request_topology,
};

static const struct sof_dev_desc virt_desc = {
	.nocodec_fw_filename	= NULL,
	.nocodec_tplg_filename	= "sof-apl-uos0.tplg",
	.default_tplg_path	= "intel/sof-tplg",
	.resindex_lpe_base	= -1,
	.resindex_pcicfg_base	= -1,
	.resindex_imr_base	= -1,
	.irqindex_host_ipc	= -1,
	.resindex_dma_base	= -1,
	.ops			= &snd_sof_vfe_ops,
};

static void sof_virtio_vfe_init(struct snd_sof_dev *sdev,
				struct sof_vfe *vfe)
{
	sdev->is_vfe = IS_ENABLED(CONFIG_SND_SOC_SOF_VIRTIO_FE);

	/*
	 * Currently we only support one VM. comp_id from 0 to
	 * SOF_VIRTIO_MAX_UOS_COMPS - 1 is for SOS. Other comp_id numbers
	 * are for VM1.
	 * TBD: comp_id number range should be dynamically assigned when
	 * multiple VMs are supported.
	 */
	sdev->next_comp_id = SOF_VIRTIO_MAX_UOS_COMPS;
	sdev->vfe = vfe;
	vfe->sdev = sdev;
}

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

//	sof_pdata->drv_name = "sof-nocodec";
//	sof_pdata->is_vfe = 1; /* This is audio vFE device */
	mach = devm_kzalloc(dev, sizeof(*mach), GFP_KERNEL);
	if (!mach)
		return -ENOMEM;

#if !IS_ENABLED(CONFIG_SND_SOC_SOF_NOCODEC)
	return -ENODEV;
#endif

	ret = sof_nocodec_setup(dev, sof_pdata, mach, &virt_desc,
				&snd_sof_vfe_ops);
	if (ret < 0)
		return ret;

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
	sof_pdata->tplg_filename_prefix = virt_desc.default_tplg_path;

//	dev_set_drvdata(dev, priv);
//
//	priv->sof_pdata = sof_pdata;

	ret = snd_sof_device_probe(dev, sof_pdata);
	if (ret < 0)
		dev_err(dev, "Cannot register device sof-audio. Error %d\n",
			ret);
	else {
		sof_virtio_vfe_init(dev_get_drvdata(dev), vdev->priv);

		dev_dbg(dev, "created machine %s\n",
			dev_name(&sof_pdata->pdev_mach->dev));
	}

	/* allow runtime_pm */
	pm_runtime_set_autosuspend_delay(dev, SND_SOF_SUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	return ret;
}

/* Probe and remove. */
static int vfe_probe(struct virtio_device *vdev)
{
	struct virtqueue *vqs[SOF_VIRTIO_NUM_OF_VQS];
	/* the processing callback number must be the same as the vqueues.*/
	vq_callback_t *cbs[SOF_VIRTIO_NUM_OF_VQS] =	{
		vfe_cmd_tx_done,
		vfe_cmd_handle_rx,
		vfe_not_tx_done,
		vfe_not_handle_rx
	};
	struct device *dev = &vdev->dev;
	struct scatterlist sg;
	struct sof_vfe *vfe;
	int ret;

	/*
	 * The below two shouldn't be necessary, it's done in
	 * virtio_pci_modern_probe() by calling dma_set_mask_and_coherent()
	 */
//	dev->coherent_dma_mask = DMA_BIT_MASK(64);
//	dev->dma_mask = &dev->coherent_dma_mask;

	ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(64));
	if (ret < 0)
		ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (ret < 0)
		dev_warn(dev, "failed to set DMA mask: %d\n", ret);

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

	vfe->posn = devm_kmalloc(dev, sizeof(*vfe->posn), GFP_KERNEL);
	if (!vfe->posn)
		return -ENOMEM;

	virtio_device_ready(vdev);

	sg_init_one(&sg, vfe->posn, sizeof(struct sof_ipc_stream_posn));
	if (vfe->ipc_not_rx_vq)
		ret = virtqueue_add_inbuf(vfe->ipc_not_rx_vq,
					  &sg, 1, vfe->posn, GFP_KERNEL);

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

static int vfe_runtime_suspend(struct device *dev)
{
	dev_info(dev, "%s()\n", __func__);
	return 0;
}

static int vfe_runtime_resume(struct device *dev)
{
	struct snd_sof_dev *sdev = dev_get_drvdata(dev);
	int ret;

//	WARN_ON(true);
	dev_info(dev, "restore pipelines for resume\n");

	/* restore pipelines */
	ret = sof_restore_pipelines(sdev);
	if (ret < 0)
		dev_err(dev,
			"error: failed to restore pipeline after resume %d\n",
			ret);

	return ret;
}

static const struct dev_pm_ops vfe_pm = {
//	SET_SYSTEM_SLEEP_PM_OPS(vfe_suspend, vfe_resume)
	SET_RUNTIME_PM_OPS(vfe_runtime_suspend, vfe_runtime_resume,
			   NULL)
};

static struct virtio_driver vfe_audio_driver = {
	.driver = {
		.name	= KBUILD_MODNAME,
		.owner	= THIS_MODULE,
		.pm	= &vfe_pm,
	},
	.id_table	= id_table,
	.probe		= vfe_probe,
	.remove		= vfe_remove,
	.config_changed	= virtaudio_config_changed,
};

module_virtio_driver(vfe_audio_driver);

MODULE_DEVICE_TABLE(virtio, id_table);
