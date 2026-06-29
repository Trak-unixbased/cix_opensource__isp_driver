// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021-2021, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "armcb_v4l2_core.h"
#include "armcb_camera_io_drv.h"
#include "armcb_isp.h"
#include "armcb_isp_driver.h"
#include "armcb_platform.h"
#include "armcb_register.h"
#include "armcb_v4l2_config.h"
#include "armcb_v4l2_stream.h"
#include "armcb_v4l_sd.h"
#include "armcb_vb2.h"
#include "armcb_isp_execstart.h"
#include "asm-generic/errno-base.h"
#include "isp_hw_ops.h"
#include "linux/types.h"
#include "media/media-device.h"
#include "media/media-entity.h"
#include "system_dma.h"
#include "system_logger.h"
#include <linux/errno.h>
#include <linux/font.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>
#include <linux/soc/cix/rdr_pub.h>
#include <linux/ktime.h>
#include <linux/soc/cix/cix_ddr_lp.h>

#ifdef LOG_MODULE
#undef LOG_MODULE
#define LOG_MODULE LOG_MODULE_ISP
#endif

#define ARMCB_MODULE_NAME "armcb_isp_v4l2"
#define MAX_PLANES_NUM 2
#define MAX_DEV_OPEN_NUM 3
#define WAIT_TIMEOUT 20000

#define BT601_COLORSPACE    V4L2_COLORSPACE_SMPTE170M
#define BT601_YCBCR_ENC     V4L2_YCBCR_ENC_601
#define BT601_QUANTIZATION  V4L2_QUANTIZATION_LIM_RANGE

// To match different kernel version
#ifdef V4L2_XFER_FUNC_SMPTE170M
#define BT601_XFER_FUNC     V4L2_XFER_FUNC_SMPTE170M
#elif defined(V4L2_XFER_FUNC_SMPTE240M)
#define BT601_XFER_FUNC     V4L2_XFER_FUNC_SMPTE240M
#else
#define BT601_XFER_FUNC     V4L2_XFER_FUNC_DEFAULT
#endif


static wait_queue_head_t daemon_ready_queue;
extern int armcb_multi_cam;
static atomic_t armcb_daemon_ready;

void * discard_buf_addr;
u32 discard_buf_size;
dma_addr_t discard_buf_addr_dma;
dma_addr_t discard_dma_handle;

static armcb_v4l2_dev_t *g_isp_v4l2_devs[ARMCB_MAX_DEVS] = { 0 };
static uint32_t outport_array[ARMCB_MAX_DEVS][V4L2_STREAM_TYPE_MAX];
static int g_adev_idx;
armcb_v4l2_stream_t
	*g_outport_map[ARMCB_MAX_DEVS][ISP_OUTPUT_PORT_MAX] = { 0 };


static const int vout_idx[5] = {
	ISP_OUTPUT_PORT_VOUT0,
	ISP_OUTPUT_PORT_VOUT1,
	ISP_OUTPUT_PORT_VOUT3,
	ISP_OUTPUT_PORT_VOUT5,
	ISP_OUTPUT_PORT_VOUT7
};

/// define isp port token list
static char *const g_IspPortToken[] = {
	[ISP_OUTPUT_PORT_VIN] = (char *const) "VIN",
	[ISP_OUTPUT_PORT_3A] = (char *const) "3A",
	[ISP_OUTPUT_PORT_VOUT0] = (char *const) "VOUT0",
	[ISP_OUTPUT_PORT_VOUT1] = (char *const) "VOUT1",
	[ISP_OUTPUT_PORT_VOUT2] = (char *const) "VOUT2",
	[ISP_OUTPUT_PORT_VOUT3] = (char *const) "VOUT3",
	[ISP_OUTPUT_PORT_VOUT4] = (char *const) "VOUT4",
	[ISP_OUTPUT_PORT_VOUT5] = (char *const) "VOUT5",
	[ISP_OUTPUT_PORT_VOUT6] = (char *const) "VOUT6",
	[ISP_OUTPUT_PORT_VOUT7] = (char *const) "VOUT7",
	[ISP_OUTPUT_PORT_VOUT8] = (char *const) "VOUT8",
	[ISP_OUTPUT_PORT_VOUT9] = (char *const) "VOUT9",

	[ISP_OUTPUT_PORT_MAX] = NULL,
};

static int armcb_isp_vb2_streamoff(struct file *file);

pid_t isp_getpid(void)
{
	return current->tgid;
}

struct isp_v4l2_fmt isp_src_formats[] = {
	{
		.name = "NV12M",
		.fourcc = V4L2_PIX_FMT_NV12M,
		.depth = { 8, 8 },
		.memplanes = 2,
		.mbus_code = MEDIA_BUS_FMT_YUYV8_1_5X8,
	},
	{
		.name = "RGB888",
		.fourcc = V4L2_PIX_FMT_RGB24,
		.depth = { 24 },
		.memplanes = 1,
		.mbus_code = MEDIA_BUS_FMT_RGB888_1X24,
	}
};

armcb_v4l2_stream_t *armcb_v4l2_get_stream(uint32_t ctx_id, int stream_id)
{
	armcb_v4l2_stream_t *pstream = NULL;

	pstream = g_isp_v4l2_devs[ctx_id]->pstreams[stream_id];

	return pstream;
}

/* ----------------------------------------------------------------
 * stream finder utility function
 */
int armcb_v4l2_find_stream(armcb_v4l2_stream_t **ppstream, uint32_t ctx_id,
			   int stream_type)
{
	int stream_id = 0;
	*ppstream = NULL;

	if (stream_type >= V4L2_STREAM_TYPE_MAX || stream_type < 0 ||
		ctx_id >= ARMCB_MAX_DEVS) {
		LOG(LOG_ERR, "stream_id=%d, ctx_id=%d", stream_id, ctx_id);
		return -EINVAL;
	}

	if (g_isp_v4l2_devs[ctx_id] == NULL) {
		LOG(LOG_ERR, "ctx %d dev is NULL", ctx_id);
		return -EBUSY;
	}

	stream_id = g_isp_v4l2_devs[ctx_id]->stream_id_index[stream_type];
	if (stream_id < 0 || stream_id >= V4L2_STREAM_TYPE_MAX ||
		g_isp_v4l2_devs[ctx_id]->pstreams[stream_id] == NULL) {
		LOG(LOG_DEBUG, "stream_type:%d stream_id:%d", stream_type,
			stream_id);
		return -ENODEV;
	}

	*ppstream = g_isp_v4l2_devs[ctx_id]->pstreams[stream_id];
	LOG(LOG_DEBUG, "ctx_id=%d stream_id=%d stream=%p", ctx_id, stream_id,
		*ppstream);

	return 0;
}

int armcb_v4l2_find_ctx_stream_by_outport(uint32_t outport, uint32_t *p_ctx_id,
					  uint32_t *p_stream_id)
{
	uint32_t ctx_id = 0;
	uint32_t stream_id = 0;

	if (outport < 0 || p_ctx_id == NULL || p_stream_id == NULL) {
		LOG(LOG_ERR, "invalid parameter");
		return -EINVAL;
	}

	for (ctx_id = 0; ctx_id < ARMCB_MAX_DEVS; ctx_id++) {
		for (stream_id = 0; stream_id < V4L2_STREAM_TYPE_MAX;
			 stream_id++) {
			if (outport & outport_array[ctx_id][stream_id])
				goto exit;
		}
	}

	if (ctx_id == ARMCB_MAX_DEVS) {
		*p_ctx_id = -1;
		*p_stream_id = -1;
		LOG(LOG_ERR,
			"failed to find a valid ctx_id and stream_id for outport:%d",
			outport);
		return -EINVAL;
	}

exit:
	*p_ctx_id = ctx_id;
	*p_stream_id = stream_id;

	LOG(LOG_DEBUG, "success find ctx_id:%d stream_id:%d for outport:%d",
		ctx_id, stream_id, outport);
	return 0;
}

int armcb_v4l2_find_stream_by_outport_ctx(uint32_t outport, uint32_t ctx_id,
					  uint32_t *p_stream_id)
{
	uint32_t stream_id = 0;

	if (outport < 0 || ctx_id < 0 || p_stream_id == NULL) {
		LOG(LOG_ERR, "invalid parameter");
		return -EINVAL;
	}

	for (stream_id = 0; stream_id < V4L2_STREAM_TYPE_MAX; stream_id++) {
		if (outport & outport_array[ctx_id][stream_id])
			goto exit;
	}

	if (stream_id == V4L2_STREAM_TYPE_MAX) {
		*p_stream_id = -1;
		LOG(LOG_DEBUG,
			"failed to find a valid stream_id for outport:%d and ctx_id:%d",
			outport, ctx_id);
		return -EINVAL;
	}

exit:
	*p_stream_id = stream_id;

	LOG(LOG_DEBUG, "success find stream_id:%d for outport:%d and ctx_id:%d",
		stream_id, outport, ctx_id);
	return 0;
}

extern struct device *mem_dev;
void armcb_isp_invalid_cache (struct vb2_buffer *vb,
					armcb_v4l2_buffer_t *pbuf)
{
	struct sg_table sgt0;
	struct sg_table sgt1;
	struct sg_table *sgt = NULL;
	struct device *dev = mem_dev;

	void* dma_vaddr_0;
	void* dma_vaddr_1;
	dma_addr_t dma_addr_0;
	dma_addr_t dma_addr_1;
	int ret = 0;

	if ((!dev) || (pbuf->vvb.vb2_buf.num_planes > MAX_PLANES_NUM))
		return;

	if ((!vb) || (!pbuf))
		return;

	dma_vaddr_0 = vb2_plane_vaddr(&pbuf->vvb.vb2_buf, 0);
	dma_addr_0 = vb2_dma_contig_plane_dma_addr(vb, 0);

	if (!dma_vaddr_0 || dma_addr_0 == 0) {
		LOG(LOG_ERR, "plane 0 dma address invalid !");
		return;
	}

	ret = dma_get_sgtable_attrs(dev, &sgt0, dma_vaddr_0, dma_addr_0,
			vb->planes[0].length, 0);
	if (ret) {
		LOG(LOG_ERR, "dma_get_sgtable_attrs plane0 failed: %d\n", ret);
		return;
	}

	dma_sync_sgtable_for_cpu(dev, &sgt0, DMA_FROM_DEVICE);
	sgt = &sgt0;
	if (!sgt->sgl || sgt->nents == 0) {
		LOG(LOG_ERR, "sgt0 is invalid");
		sg_free_table(&sgt0);
		return;
	}
	/* free the sg table after flush cache*/
	sg_free_table(&sgt0);

	if (pbuf->vvb.vb2_buf.num_planes > 1) {
		dma_vaddr_1 = vb2_plane_vaddr(&pbuf->vvb.vb2_buf, 1);
		dma_addr_1 = vb2_dma_contig_plane_dma_addr(vb, 1);

		if (!dma_vaddr_1 || dma_addr_1 == 0) {
			LOG(LOG_ERR, "plane 1 dma address invalid !");
			return;
		}

		ret = dma_get_sgtable_attrs(dev, &sgt1, dma_vaddr_1, dma_addr_1,
				vb->planes[1].length, 0);
		if (ret) {
			LOG(LOG_ERR, "dma_get_sgtable_attrs plane0 failed: %d\n", ret);
			return;
		}

		dma_sync_sgtable_for_cpu(dev, &sgt1, DMA_FROM_DEVICE);
		sgt = &sgt1;
		if (!sgt->sgl || sgt->nents == 0) {
			sg_free_table(&sgt1);
			return;
		}

		/* free the sg table after flush cache*/
		sg_free_table(&sgt1);
	}

	return;
}

void armcb_isp_put_frame(uint32_t ctx_id, int stream_id, isp_output_port_t port)
{
	int rc = 0;
	//unsigned long flags;
	armcb_v4l2_stream_t *pstream = NULL;
	armcb_v4l2_buffer_t *pbuf = NULL;
	struct vb2_buffer *vb = NULL;
	armcb_v4l2_dev_t *dev = NULL;

	static armcb_v4l2_buffer_t *splastbuf;
	armcb_v4l2_buffer_t *plastbuf = NULL;
	unsigned int vout_reg1 = 0;
	unsigned int vout_reg2 = 0;
	unsigned int startaddr = 0;

	dev = armcb_v4l2_core_get_dev(ctx_id);

	if (stream_id < 0 && port < ISP_OUTPUT_PORT_MAX) {
		pstream = g_outport_map[ctx_id][port];
	} else {
		/* find stream pointer */
		rc = armcb_v4l2_find_stream(&pstream, ctx_id, stream_id);
		if (rc < 0) {
			LOG(LOG_WARN,
				"can't find stream on ctx %d stream_id %d (errno = %d)",
				ctx_id, stream_id, rc);
			return;
		}
	}

	LOG(LOG_DEBUG,
		"ctx_id:%d Stream#%d fmt(%d*%d %d %d) outport(%d %s) streamType(%d) "
		"reserved_buf_addr(0x%x)",
		ctx_id, pstream->stream_id, pstream->cur_v4l2_fmt.fmt.pix_mp.width,
		pstream->cur_v4l2_fmt.fmt.pix_mp.height,
		pstream->cur_v4l2_fmt.fmt.pix_mp.pixelformat,
		pstream->cur_v4l2_fmt.type, pstream->outport, g_IspPortToken[port],
		pstream->stream_type, pstream->reserved_buf_addr);

	/* check if stream is on */
	if (!pstream || !pstream->stream_started) {
		LOG(LOG_DEBUG, "[Stream#%d] is not started yet on ctx %d",
			stream_id, ctx_id);
		return;
	}

	LOG(LOG_DEBUG, "ctx_id:%d [Stream#%d] %p", ctx_id, pstream->stream_id,
		pstream);

	if (atomic_read(&dev->upload_streamoff) == 1)
		goto exit;

	if (dev->is_streaming == 0)
		goto exit;

	spin_lock(&dev->slock);
	plastbuf = list_last_entry(&(pstream->stream_buffer_list_busy),
				   armcb_v4l2_buffer_t, list);

	/* try to get an active buffer from vb2 queue  */
	if (!list_empty(&pstream->stream_buffer_list_busy)) {
		if (!list_is_singular(&pstream->stream_buffer_list_busy) ||
			(plastbuf == splastbuf)) {
			pbuf = list_entry(pstream->stream_buffer_list_busy.next,
					  armcb_v4l2_buffer_t, list);
			if(pbuf)
				list_del(&pbuf->list);
		}
	}

	if (!pbuf) {
		/// Use reserved buffer to hw output.
		LOG(LOG_DEBUG, "[Stream#%d] type: %d no buffers, use reserved buffer",
			pstream->stream_id, V4L2_STREAM_TYPE_VIDEO);

		if (armcb_get_output_addr_by_outport_i7(pstream->outport,
					&vout_reg1, &vout_reg2) < 0) {
			LOG(LOG_DEBUG,
				"ctx_id:%d stream_id:%d failed to get reg for outport:%d",
				pstream->ctx_id, pstream->stream_id, pstream->outport);
			spin_unlock(&dev->slock);
			return;
		}

		startaddr = discard_buf_addr_dma;
		if (vout_reg1 && startaddr) {
			armcb_isp_write_reg(vout_reg1, startaddr);
		}
		if (vout_reg2 && startaddr) {
			armcb_isp_write_reg(vout_reg2, startaddr);
		}

		spin_unlock(&dev->slock);
		goto exit;
	}

	splastbuf = list_last_entry(&(pstream->stream_buffer_list_busy),
					armcb_v4l2_buffer_t, list);

	vb = &pbuf->vvb.vb2_buf;
	spin_unlock(&dev->slock);

	/*To sync the write done interrupts and the active buffer status*/
	if((vb->state != VB2_BUF_STATE_ACTIVE) ||
		(vb->memory != VB2_MEMORY_MMAP))
	{
		LOG(LOG_ERR, "VB buffer is not active");
		goto exit;
	}

	if (!pbuf->vvb.vb2_buf.vb2_queue)
		goto exit;

	if ((atomic_read(&dev->stream_on_cnt) == STREAM_OFF) ||
			(atomic_read(&dev->stream_on_cnt) == STREAM_DEFAULT))
		goto exit;

	vb->planes[0].bytesused = vb->planes[0].length;
	vb->planes[1].bytesused = vb->planes[1].length;

	/* Invalid cache before CPU read buffer to avoid the cache
	 * line issue */
	//armcb_isp_invalid_cache(vb, pbuf);
	vb->timestamp = ktime_get_ns();
	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
	LOG(LOG_DEBUG, "%s put frame success ctx_id:%d stream_id:%d",
               g_IspPortToken[port], ctx_id, stream_id);
exit:
	return;
}

static int armcb_v4l2_querycap(struct file *file, void *priv,
				   struct v4l2_capability *cap)
{
	armcb_v4l2_dev_t *dev = video_drvdata(file);

	strcpy(cap->driver, "arm-china-isp");
	strcpy(cap->card, "linlon isp v4l2");
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev->v4l2_dev.name);

	/* V4L2_CAP_VIDEO_CAPTURE_MPLANE */
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING |
			   V4L2_CAP_READWRITE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	// cap->capabilities = dev->vid_cap_caps | V4L2_CAP_DEVICE_CAPS;
	LOG(LOG_DEBUG, "capabilities(0x%x)", cap->capabilities);

	return 0;
}

static int armcb_v4l2_log_status(struct file *file, void *fh)
{
	return v4l2_ctrl_log_status(file, fh);
}

static int armcb_v4l2_fh_release(struct file *file)
{
	struct armcb_isp_v4l2_fh *sp = fh_to_private(file->private_data);
	armcb_v4l2_dev_t *dev = video_drvdata(file);
	struct video_device *vdev = &dev->vid_cap_dev;
	struct v4l2_event_subscription sub;
	int ret = 0;

	LOG(LOG_DEBUG, "isp_v4l2 close: ctx_id: %d, called for sid:%d.",
		dev->ctx_id, sp->stream_id);
	if (sp) {
		/* unsubscribe event when close file */
		memset(&sub, 0, sizeof(sub));
		sub.type = V4L2_EVENT_ALL;
		ret = v4l2_event_unsubscribe(&sp->fh, &sub);
		LOG(LOG_DEBUG,
				"armcb_fop_release v4l2_event_unsubscribe, ret = %d", ret);
		/* Release vb2 queue on close. When vdev->queue==NULL: if s_ctrl
		 * already released (queue_released_by_s_ctrl), skip to avoid double-release;
		 * else release our queue to avoid leak when vdev->queue was wrongly set null. */
		mutex_lock(&dev->queue_lock);
		if (sp->vb2_q.dev) {
			if (vdev->queue == &sp->vb2_q) {
				vdev->queue = NULL;
				dev->queue_released_by_s_ctrl = 0;
				vb2_queue_release(&sp->vb2_q);
			} else if (vdev->queue != NULL) {
				dev->queue_released_by_s_ctrl = 0;
				vb2_queue_release(&sp->vb2_q); /* not owner, release our queue */
			} else {
				/* vdev->queue == NULL */
				if (dev->queue_released_by_s_ctrl) {
					dev->queue_released_by_s_ctrl = 0; /* already released by s_ctrl, skip */
				} else {
					/* Abnormal: queue never released, avoid leak */
					vb2_queue_release(&sp->vb2_q);
					dev->queue_released_by_s_ctrl = 0;
				}
			}
		}
		mutex_unlock(&dev->queue_lock);
		v4l2_fh_del(&sp->fh);
		v4l2_fh_exit(&sp->fh);
		kfree(sp);
	}

	return ret;
}

static int armcb_release_output_port(void)
{
    int rc = 0, i = 0, j = 0;

	for(i = 0; i < ARMCB_MAX_DEVS; i++  )
	{
		for(j = 0; j < ISP_OUTPUT_PORT_MAX; j++)
			g_outport_map[i][j] = NULL;
	}

    LOG(LOG_INFO, "#### Armcb release all port resource !!!");
    return rc;
}

// Get current application PID

static int find_user_process_by_name(const char *name)
{
	int rc = -1;
	struct task_struct *curr;
	curr = current;

	if (curr->mm != NULL) {
		LOG(LOG_DEBUG, "The PID is tgid:%d, user_pid: %s\n",
               curr->tgid, curr->comm);
		rc = 0;
	} else {
		LOG(LOG_DEBUG, "The PID is kernel_pid: %s\n", curr->comm);
		rc = -1;
	}

	rc = strncmp(curr->comm, name, 7);
	LOG(LOG_DEBUG, "The pid is %s, and the rc:%d \n",curr->comm, rc);

	return rc;
}

static int armcb_v4l2_post_streamoff_event(armcb_v4l2_dev_t *dev)
{
	struct video_device *vdev = &dev->vid_cap_dev;
	int rc = 0;

	unsigned int *pData = NULL;
	struct v4l2_event ev;
	memset(&ev, 0, sizeof(struct v4l2_event));
	ev.id = ISP_DAEMON_EVENT_STREAM_OFF;
	ev.type = V4L2_EVENT_CTRL;

	/* Queue a empty event to notify userspace */
	pData = (unsigned int *)ev.u.data;
	pData[0] = 0xff;
	v4l2_event_queue(&dev->vid_cap_dev, &ev);
	LOG(LOG_INFO, "%s, Upload streamoff event", dev_name(&vdev->dev));

	atomic_set(&dev->upload_streamoff, 1);

	/* release file handle */
	rc = wait_event_interruptible_timeout(dev->wait_streamoff_queue,
				dev->streamoff_done, msecs_to_jiffies(400000));

	if (rc == -ERESTARTSYS) {
		LOG(LOG_ERR, "Wait interrupt by signal, will not enter sleep!");
		rc = 0;
	}

	LOG(LOG_DEBUG, " ## wait streamoff done : %d", dev->streamoff_done);

	dev->streamoff_done = 0;
	/* post streamoff and set the ready -1 */
	atomic_set(&armcb_daemon_ready, -1);

	return 0;
}

/**
 * Release one stream slot: clear g_outport_map, stream_id_index, deinit stream,
 * clear pstreams[stream_id] and stream_mask. Caller must hold dev->queue_lock.
 */
static void armcb_v4l2_release_stream_slot(armcb_v4l2_dev_t *dev, int stream_id)
{
	armcb_v4l2_stream_t *pstream = dev->pstreams[stream_id];
	int outport_idx;

	if (!pstream)
		return;
	outport_idx = armcb_outport_bits_to_idx(pstream->outport);
	if (outport_idx >= 0 && outport_idx < ISP_OUTPUT_PORT_MAX)
		g_outport_map[dev->ctx_id][outport_idx] = NULL;
	if (pstream->stream_type < V4L2_STREAM_TYPE_MAX)
		dev->stream_id_index[pstream->stream_type] = -1;
	armcb_v4l2_stream_deinit(pstream, dev);
	dev->pstreams[stream_id] = NULL;
	dev->stream_mask &= ~(1 << stream_id);
	dev->is_streaming = 0;
}

static int armcb_v4l2_fop_release(struct file *file)
{
	armcb_v4l2_dev_t *dev = video_drvdata(file);
	struct video_device *vdev = &dev->vid_cap_dev;
	struct vb2_queue *q = vdev->queue;
	struct armcb_isp_v4l2_fh *sp = fh_to_private(file->private_data);
	armcb_v4l2_stream_t *pstream = dev->pstreams[sp->stream_id];
	int outport_idx = -1;
	int rc = 0;

	atomic_sub_return(1, &dev->opened);
	/*There the isp_app and the 3rd application will do close fd, and will
	 * meet follow cases when 3rd app close the fd,
	 * default: 3rd app just close, the release file handle.
	 * case 1: 3rd app is streaming, and close the fd, upload event to isp_app.
	 * case 2: 3rd app is s_fmt, but not start streaming, release buf and clear flags.
	 * case 3: 3rd app is streamoff, and close the fd, clear the flags.
	 * */

	/* case1. The stream is started, and need to close the fd,
	 * report the event to isp_app to do deinit.
	 * */
	if ((atomic_read(&dev->stream_on_cnt) == STREAM_ON) && (vdev->queue))
	{
		if (armcb_multi_cam == 0) {
			/* work around for play cheese video recore and preview exit*/
			rc = find_user_process_by_name("matroskademux0");
			if (!rc)
				goto exit;

			rc = atomic_read(&dev->opened);
			LOG(LOG_DEBUG, "%s fd num_cnt: %d \n",
						dev_name(&vdev->dev), rc);
			/* The isp_app open the video node twice, and the v4l2 app
			 * open the video node once, so if the really close, there
			 * should left 3 times for close the fd */
			if ((rc > MAX_DEV_OPEN_NUM) && (dev->online_mode))
				goto exit;

			rc = armcb_v4l2_post_streamoff_event(dev);

		} else {
			rc = strncmp(dev->v4l2_dev.name, "armcb_isp_v4l2-00",
					sizeof(dev->v4l2_dev.name));
			if (!rc) {
				/* When do V4L2 streamon case, the 3rd app open the fd,
				 * and then close the fd, it should not impact the stream
				 * on pipeline*/
				rc = atomic_read(&dev->opened);
				LOG(LOG_DEBUG, "%s fd num_cnt: %d \n",
						dev_name(&vdev->dev), rc);
				if ((rc > MAX_DEV_OPEN_NUM) && (dev->online_mode))
					goto exit;

				rc = armcb_v4l2_post_streamoff_event(dev);
			}
		}

		/* Point only this stream's output to discard buffer to avoid SMMU errors.
		 * Do not call armcb_v4l2_config_init_update_stream_hw_addr(dev) here:
		 * that updates all vout ports and would break the other camera's stream. */
		if (pstream && pstream->outport && discard_buf_addr_dma) {
			u32 vout_r1 = 0, vout_r2 = 0;

			if (armcb_get_output_addr_by_outport_i7(pstream->outport,
								&vout_r1, &vout_r2) >= 0) {
				if (vout_r1)
					armcb_isp_write_reg(vout_r1, discard_buf_addr_dma);
				if (vout_r2)
					armcb_isp_write_reg(vout_r2, discard_buf_addr_dma);
			}
		}
		/* Clear this stream's state so the next open (e.g. gstreamer re-enter)
		 * can succeed: otherwise stream_mask / g_outport_map stay set and
		 * s_fmt fails with outport busy or wrong stream reuse. */
		mutex_lock(&dev->queue_lock);
		armcb_v4l2_release_stream_slot(dev, sp->stream_id);
		mutex_unlock(&dev->queue_lock);
		goto exit;
	}

	LOG(LOG_INFO, "##### %s, stream_on_cnt:%d, port_idx_release:%d\n",
			dev_name(&vdev->dev),
			atomic_read(&dev->stream_on_cnt),
			atomic_read(&dev->port_idx_release));

	/* case2. The stream is just s_fmt but not started, and need to close
	 * the fd, just release buf and outport port, clear the atomic flags
	 * to avoid the outport busy and wrong status.
	 */
	if((atomic_read(&dev->stream_on_cnt) == STREAM_DEFAULT)
			&& (vdev->queue)
			//&& (armcb_multi_cam == 0)) {
			&& (atomic_read(&dev->port_idx_release) == OUTPORT_IDX_IS_FREE)) {
		rc = find_user_process_by_name("isp_app");
		if(!rc) {
			LOG(LOG_INFO, "#### The pid is isp_app !");
			goto exit;
		}

		if(dev->buf_type == q->type) {
			LOG(LOG_DEBUG, " ##### %s, vb2 release\n", dev_name(&vdev->dev));
			mutex_lock(&dev->queue_lock);
			/* Always stop queue before release to avoid __vb2_queue_cancel WARNING */
			rc = vb2_streamoff(q, q->type);
			if (rc == 0) {
				armcb_v4l2_stream_clear_buffer_lists(pstream);
			} else {
				rc = destroy_buf_queue(q, VB2_BUF_STATE_ERROR);
			}
			if (!rc) {
				vb2_queue_release(q);
				vdev->queue = NULL;
				dev->queue_released_by_s_ctrl = 1; /* so fh_release skips, avoid double-release */
			}

			armcb_v4l2_release_stream_slot(dev, sp->stream_id);
			mutex_unlock(&dev->queue_lock);
			goto deinit;
		}
	}
	/* When the stream is stream off and the buffer is released, it should
	 * set the g_outport_map to 0 to avoid the next loop outport is busy
	 */
	/* case3. The stream is streamoff, and need to close the fd, clear
	 * the outport port, clear the atomic flags to avoid the outport busy.
	 */
	if ((atomic_read(&dev->stream_on_cnt) == STREAM_OFF)
			&& (atomic_read(&dev->port_idx_release) == OUTPORT_IDX_IS_BUSY))
	{
		atomic_set(&dev->upload_streamoff, 0);
		goto deinit;
	}

	if ((atomic_read(&dev->stream_on_cnt) == STREAM_OFF)
			&& (armcb_multi_cam == 0)
			&& (atomic_read(&dev->port_idx_release) == OUTPORT_IDX_IS_FREE))
	{
		atomic_set(&dev->upload_streamoff, 0);
		goto deinit;
	}

	armcb_v4l2_fh_release(file);
	if (dev->ctx_id == 0)
		armcb_isp_execstart_note_ctx0_release();
	LOG(LOG_INFO, "release v4l2 fp success");
	return rc;

deinit:
	armcb_release_output_port();
	atomic_set(&dev->stream_on_cnt, STREAM_DEFAULT);
	atomic_set(&dev->port_idx_release, OUTPORT_IDX_DEFAULT);

exit:
	/* release file handle */
	armcb_v4l2_fh_release(file);
	if (dev->ctx_id == 0)
		armcb_isp_execstart_note_ctx0_release();
	LOG(LOG_INFO, "release v4l2 fp success");
	return rc;
}

static int armcb_v4l2_check_daemon_status(struct video_device *vdev)
{
	int ret = 0;

	ret = wait_event_interruptible_timeout(daemon_ready_queue,
			(atomic_read(&armcb_daemon_ready) == 1),
			msecs_to_jiffies(200000));
	if(ret == 0)
		LOG(LOG_ERR,"Wait daemon ready timeout.");
	if (ret > 0)
		LOG(LOG_ERR,"The daemon is ready.");
	if(ret == -ERESTARTSYS)
		LOG(LOG_ERR,"Wait daemon interrupt by signal, will not sleep.");

	return ret;
}

static int armcb_v4l2_fh_open(struct file *file)
{
	armcb_v4l2_dev_t *dev = video_drvdata(file);
	struct armcb_isp_v4l2_fh *sp = NULL;
	struct video_device *vdev = &dev->vid_cap_dev;
	int stream_opened = 0;

	sp = kzalloc(sizeof(struct armcb_isp_v4l2_fh), GFP_KERNEL);
	if (!sp)
		return -ENOMEM;

	stream_opened = atomic_read(&dev->opened);
	if (stream_opened >= V4L2_STREAM_TYPE_MAX) {
		LOG(LOG_ERR,
			"too many open streams, stream_opened: %d, max: %d.",
			stream_opened, V4L2_STREAM_TYPE_MAX);
		kfree(sp);
		return -EBUSY;
	}

	file->private_data = &sp->fh;

	v4l2_fh_init(&sp->fh, &dev->vid_cap_dev);
	v4l2_fh_add(&sp->fh);

	LOG(LOG_INFO, "%s open v4l2 fp success", dev_name(&vdev->dev));
	return 0;
}

static int armcb_v4l2_fop_open(struct file *filp)
{
	armcb_v4l2_dev_t *dev = video_drvdata(filp);
	struct video_device *vdev = &dev->vid_cap_dev;
	int ret = -1;

	/* To check the PID is isp_app */
	ret = find_user_process_by_name("isp_app");
	if (!ret) {
		goto normal_mode;
	} else {
		if (atomic_read(&armcb_daemon_ready) == -1)
		{
			LOG(LOG_INFO, " %s v4l2 fop open, daemon is %d \n",
				dev_name(&vdev->dev), atomic_read(&armcb_daemon_ready));
			ret = armcb_v4l2_check_daemon_status(vdev);
			goto normal_mode;
		} else {
			goto normal_mode;
		}
	}

normal_mode:
	/*
	 * Newest running isp_app cmdline -> allowed armcb_isp_v4l2-00..03 (ctx_id).
	 * Multi-camera (-m 2/3/4): open -00 before other nodes.
	 */
	armcb_isp_execstart_refresh_if_needed();
	if (!armcb_isp_execstart_ctx_allowed(dev->ctx_id)) {
		LOG(LOG_ERR,
		    "%s: camera is not support (isp_app cmdline policy, ctx_id=%u)\n",
		    dev_name(&vdev->dev), dev->ctx_id);
		return -EPERM;
	}
	if (!armcb_isp_execstart_multi_open_order_ok(dev->ctx_id)) {
		LOG(LOG_ERR,
		    "%s: camera is not support (open armcb_isp_v4l2-00 first, multi-camera)\n",
		    dev_name(&vdev->dev));
		return -EPERM;
	}

	ret = armcb_v4l2_fh_open(filp);
	if (ret < 0) {
		LOG(LOG_ERR, "Error, file handle open fail (rc=%d)", ret);
		goto open_fail;
	}

	atomic_add(1, &dev->opened);
	if (dev->ctx_id == 0)
		armcb_isp_execstart_note_ctx0_open();

open_fail:
	return ret;
}

static ssize_t armcb_v4l2_fop_write(struct file *filep, const char __user *buf,
					size_t count, loff_t *ppos)
{
	struct armcb_isp_v4l2_fh *sp = fh_to_private(filep->private_data);
	int rc = 0;

	if (sp->vb2_q.lock && mutex_lock_interruptible(sp->vb2_q.lock))
		return -ERESTARTSYS;

	rc = vb2_write(&sp->vb2_q, buf, count, ppos,
			   filep->f_flags & O_NONBLOCK);

	if (sp->vb2_q.lock)
		mutex_unlock(sp->vb2_q.lock);
	return rc;
}

static ssize_t armcb_v4l2_fop_read(struct file *filep, char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct armcb_isp_v4l2_fh *sp = fh_to_private(filep->private_data);
	int rc = 0;

	if (sp->vb2_q.lock && mutex_lock_interruptible(sp->vb2_q.lock))
		return -ERESTARTSYS;

	rc = vb2_read(&sp->vb2_q, buf, count, ppos,
			  filep->f_flags & O_NONBLOCK);

	if (sp->vb2_q.lock)
		mutex_unlock(sp->vb2_q.lock);
	return rc;
}

static unsigned int armcb_v4l2_fop_poll(struct file *filep,
					struct poll_table_struct *wait)
{
	struct armcb_isp_v4l2_fh *sp = fh_to_private(filep->private_data);
	int rc = 0;

	if (sp->vb2_q.lock && mutex_lock_interruptible(sp->vb2_q.lock))
		return POLLERR;

	rc = vb2_poll(&sp->vb2_q, filep, wait);

	if (sp->vb2_q.lock)
		mutex_unlock(sp->vb2_q.lock);

	return rc;
}

static int armcb_v4l2_fop_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct armcb_isp_v4l2_fh *sp = fh_to_private(file->private_data);
	int rc = 0;

	rc = vb2_mmap(&sp->vb2_q, vma);

	return rc;
}

static int armcb_v4l2_sync_s_fmt_event(armcb_v4l2_dev_t *dev)
{

	struct video_device *vdev = &dev->vid_cap_dev;
	int rc = 0;

	rc = wait_event_interruptible_timeout(dev->wait_fmt_queue,
			dev->s_fmt_done, msecs_to_jiffies(WAIT_TIMEOUT));

	if(rc == 0)
		LOG(LOG_ERR,"Wait isp set fmt timeout.");
	if (rc > 0)
		LOG(LOG_ERR,"The isp set fmt is successful.");
	if(rc == -ERESTARTSYS)
		LOG(LOG_ERR,"Wait set fmt interrupt by signal, will not sleep.");

	LOG(LOG_INFO, "%s, wait set fmt done *_*: %d", dev_name(&vdev->dev),
			dev->s_fmt_done);

	dev->s_fmt_done = 0;

	return rc;
}

static int armcb_v4l2_sync_streamon_event(armcb_v4l2_dev_t *dev)
{
	struct video_device *vdev = &dev->vid_cap_dev;
	int rc = 0;

	LOG(LOG_INFO, "#### %s sync streamon, StreamOn_done:%d\n",
			dev_name(&vdev->dev), dev->streamon_done);

	rc = wait_event_interruptible_timeout(dev->wait_streamon_queue,
			dev->streamon_done, msecs_to_jiffies(WAIT_TIMEOUT));

	if(rc == 0)
		LOG(LOG_ERR,"Wait isp streamon timeout.");
	if (rc > 0)
		LOG(LOG_ERR,"The isp streamon is successful.");
	if(rc == -ERESTARTSYS)
		LOG(LOG_ERR,"Wait streamon interrupt by signal, will not sleep.");

	LOG(LOG_INFO, "%s, wait streamon done *_*: %d", dev_name(&vdev->dev),
			dev->streamon_done);

	dev->streamon_done = 0;

	return rc;
}

static long armcb_v4l2_fop_ioctl2(struct file *file, unsigned int cmd, unsigned long arg)
{
	armcb_v4l2_dev_t *dev = video_drvdata(file);
	void __user *argp = (void __user *)arg;
	struct video_device *vdev = &dev->vid_cap_dev;
	struct mutex *req_mutex = &vdev->v4l2_dev->mdev->req_queue_mutex;
	int ret = -ENOIOCTLCMD;

	if ((!dev) || (!vdev)) {
		LOG(LOG_ERR, "The vdev and dev is null !");
		return 0;
	}

	if ((!vdev->v4l2_dev) ||(!vdev->v4l2_dev->mdev) || (!req_mutex)) {
		LOG(LOG_ERR, "The vdev v4l2 dev and mdev is null !");
		return 0;
	}

	mutex_lock(&dev->ctl_lock);
	/* To check the PID is isp_app */
	ret = find_user_process_by_name("isp_app");
	if(!ret) {
		goto ioctl_exc;
	} else {
		/*The PID is not */
		goto ioctl_exc_wait;
	}

ioctl_exc:
	ret = video_ioctl2(file, cmd, (unsigned long int)argp);
	if (ret < 0) {
		LOG(LOG_DEBUG, "Failed cmd(%d) and ret(%d)", cmd, ret);
		mutex_unlock(&dev->ctl_lock);
		return ret;
	}
	mutex_unlock(&dev->ctl_lock);
	return ret;

ioctl_exc_wait:
	ret = video_ioctl2(file, cmd, (unsigned long int)argp);
	if (ret < 0) {
		LOG(LOG_DEBUG, "Failed cmd(%d) and ret(%d)", cmd, ret);
		mutex_unlock(&dev->ctl_lock);
		return ret;
	}
	mutex_unlock(&dev->ctl_lock);

	mutex_lock(&dev->wait_lock);
	if (cmd == VIDIOC_S_FMT) {
		if (armcb_multi_cam == 0) {
			armcb_v4l2_sync_s_fmt_event(dev);
		} else {
			ret = strncmp(dev->v4l2_dev.name, "armcb_isp_v4l2-00",
							sizeof(dev->v4l2_dev.name));
			if (!ret)
				armcb_v4l2_sync_s_fmt_event(dev);
		}
	}

	if (cmd == VIDIOC_STREAMON)
			armcb_v4l2_sync_streamon_event(dev);

	mutex_unlock(&dev->wait_lock);
	return ret;
}

static const struct v4l2_file_operations armcb_fops = {
	.owner = THIS_MODULE,
	.open = armcb_v4l2_fop_open,
	.release = armcb_v4l2_fop_release,
	.read = armcb_v4l2_fop_read,
	.write = armcb_v4l2_fop_write,
	.poll = armcb_v4l2_fop_poll,
	.unlocked_ioctl = armcb_v4l2_fop_ioctl2,
	.mmap = armcb_v4l2_fop_mmap,
};

/* Per-stream control operations */
static inline bool armcb_v4l2_is_q_busy(struct vb2_queue *queue,
					struct file *file)
{
	if (queue->owner && queue->owner != file->private_data)
		LOG(LOG_ERR, "vb2_queue %p is busy!", queue);

	return queue->owner && queue->owner != file->private_data;
}

static int armcb_v4l2_streamon(struct file *file, void *priv,
				   enum v4l2_buf_type buf_type)
{
	int rc = -1;
	struct v4l2_event ev;
	unsigned int *pData = NULL;
	armcb_v4l2_dev_t *dev = video_drvdata(file);
	struct video_device *vdev = &dev->vid_cap_dev;

	struct armcb_isp_v4l2_fh *sp = fh_to_private(priv);
	armcb_v4l2_stream_t *pstream = dev->pstreams[sp->stream_id];

	if (dev->is_streaming) {
		LOG(LOG_ERR, "%s streaming is busy\n", dev_name(&vdev->dev));
		return -EBUSY;
	}

	if (armcb_v4l2_is_q_busy(&sp->vb2_q, file))
		return -EBUSY;

	LOG(LOG_DEBUG, "buf_type:%d\n", buf_type);
	memset(&ev, 0, sizeof(struct v4l2_event));
	ev.id = ISP_DAEMON_EVENT_STREAM_ON;
	ev.type = V4L2_EVENT_CTRL;

	pData = (unsigned int *)ev.u.data;
	pData[0] = 0xff;
	v4l2_event_queue(&dev->vid_cap_dev, &ev);

	mutex_lock(&dev->queue_lock);
	dev->buf_type = buf_type;
	mutex_unlock(&dev->queue_lock);

	rc = vb2_streamon(&sp->vb2_q, buf_type);
	if (rc != 0) {
		LOG(LOG_ERR, "fail to vb2_streamon :%d. (rc=%d)", sp->stream_id, rc);
		return rc;
	}

	/*config first frame output address*/
#ifndef V4L2_OPT
	rc = armcb_v4l2_config_update_stream_vin_addr(pstream);
	if (rc != 0) {
		LOG(LOG_ERR,
			"fail to update stream vin addr. (stream_id = %d, rc=%d)",
			sp->stream_id, rc);
	}

	rc = armcb_v4l2_config_update_stream_hw_addr(pstream);
	if (rc != 0) {
		LOG(LOG_ERR,
			"fail to update stream output addr. (stream_id = %d, rc=%d)",
			sp->stream_id, rc);
	}
#endif

	rc = armcb_v4l2_stream_on(pstream);
	if (rc != 0) {
		LOG(LOG_ERR, "fail to isp_stream_on. (stream_id = %d, rc=%d)",
			sp->stream_id, rc);
		return rc;
	}

	dev->is_streaming = 1;
	dev->streaming_pid = isp_getpid();

	atomic_set(&dev->upload_streamoff, 0);

	if (dev->ddr_lp_mode == 1) {
		cix_set_ddrlp(0);
		dev->ddr_lp_mode = 0;
		usleep_range(1000,1000 + 100);
	}

	return rc;
}

static int armcb_isp_vb2_streamoff(struct file *file)
{
	armcb_v4l2_dev_t *dev = video_drvdata(file);
	struct video_device *vdev = &dev->vid_cap_dev;
	struct vb2_queue *q;
	struct armcb_isp_v4l2_fh *sp = fh_to_private(file->private_data);
	armcb_v4l2_stream_t *pstream = dev->pstreams[sp->stream_id];
	int outport_idx = -1;
	int loop;
	int ret = 0;

	mutex_lock(&dev->queue_lock);
	/* Read queue under lock so we don't use a stale pointer if close() already released it. */
	q = vdev->queue;

	if (!pstream) {
		dev->is_streaming = 0;
		goto exit;
	}

	/* To keep the q is not null before do release buffer*/
	if((!q) || (!q->ops) || (!q->dev)) {
		dev->is_streaming = 0;
		LOG(LOG_ERR, "vb2_queue dev is NULL\n");
		goto exit;
	}

	pstream->stream_started = 0;

	/* s_ctrl(STREAM_OFF) calls this when queue exists (do_vb2). Always run streamoff
	 * and release so that if the app (e.g. gstreamer) never closes or crashes,
	 * isp_app can release the queue and avoid leak. fh_release will skip when
	 * queue_released_by_s_ctrl is set. */
	if ((atomic_read(&dev->stream_on_cnt) == STREAM_ON) && (vdev->queue) &&
	    (dev->buf_type == q->type)) {
			/* destroy buf queue*/
			if (!dev->is_streaming) //stream is release.
				goto exit;

			/* Always stop vb2 queue before release to avoid __vb2_queue_cancel WARNING
			 * (queue must leave streaming state before vb2_queue_release).
			 * vb2_streamoff is safe when not streaming (returns -EINVAL, we still release). */
			ret = vb2_streamoff(q, q->type);
			if (ret == 0) {
				/* Buffers already completed by vb2_streamoff; clear driver lists
				 * without calling vb2_buffer_done again. */
				armcb_v4l2_stream_clear_buffer_lists(pstream);
			} else {
				/* Not streaming or streamoff failed (e.g. -EBUSY owner mismatch):
				 * complete buffers then release; may trigger __vb2_queue_cancel WARNING
				 * if queue was still streaming. Ensure same fd does STREAMON and s_ctrl(STREAM_OFF). */
				if (ret != -EINVAL)
					LOG(LOG_ERR, "vb2_streamoff failed %d (owner?); release may WARN\n", ret);
				ret = destroy_buf_queue(q, VB2_BUF_STATE_ERROR);
			}
			if (!ret && vdev->queue == q) {
				/* Always release so leak is avoided if the app never closes (e.g. crash).
				 * fh_release will skip when queue_released_by_s_ctrl is set. */
				LOG(LOG_INFO, " ##### %s, vb2 release\n", dev_name(&vdev->dev));
				vdev->queue = NULL;
				dev->queue_released_by_s_ctrl = 1;
				vb2_queue_release(q);
			}

			if (dev->ddr_lp_mode == 0) {
				cix_set_ddrlp(1);
				dev->ddr_lp_mode = 1;
				usleep_range(100,100 + 100);
			}

		/* deinit all stream slots */
		for (loop = 0; loop < V4L2_STREAM_TYPE_MAX; loop++)
			armcb_v4l2_release_stream_slot(dev, loop);
	}

exit:
	mutex_unlock(&dev->queue_lock);
	return ret;
}

static int armcb_v4l2_streamoff(struct file *file, void *priv,
				enum v4l2_buf_type buf_type)
{
	int ret = 0;

	return ret;
}

static int armcb_v4l2_config_bt601(struct v4l2_pix_format_mplane *pix)
{
	int ret = 0;

	if ((pix->pixelformat == V4L2_PIX_FMT_NV12) ||
        (pix->pixelformat == V4L2_PIX_FMT_NV12M)) {
        if ((pix->width == 640) && (pix->height == 480))
        {
            pix->colorspace = BT601_COLORSPACE;
            pix->ycbcr_enc = BT601_YCBCR_ENC;
            pix->quantization = BT601_QUANTIZATION;
            pix->xfer_func = BT601_XFER_FUNC;
        }
    }

	return ret;
}

int armcb_v4l2_g_fmt_vid_cap_mplane(struct file *file, void *priv,
					struct v4l2_format *f)
{
	int ret = 0;

	/*we get fmt from userspace*/
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;

	LOG(LOG_DEBUG,
		"GET FMT: width = %d, height = %d, colorspace = %d, pixel_format = %d, " \
		"sizeimage0 = %d, sizeimage1 = %d, bytesperline0 = %d, bytesperline1 = %d",
		pix->width, pix->height, pix->colorspace, pix->pixelformat,
		pix->plane_fmt[0].sizeimage, pix->plane_fmt[1].sizeimage,
		pix->plane_fmt[0].bytesperline, pix->plane_fmt[1].bytesperline);

	/* Cheese or gstreamer support the VGA(640x480) need to support the
	 * BT601, so define the VGA support BT601 */
	ret = armcb_v4l2_config_bt601(pix);

	return ret;
}

int armcb_v4l2_s_fmt_vid_cap_mplane(struct file *file, void *priv,
					struct v4l2_format *f)
{
	armcb_v4l2_dev_t *dev = video_drvdata(file);
	struct video_device *vdev = &dev->vid_cap_dev;
	struct armcb_isp_v4l2_fh *sp = fh_to_private(file->private_data);
	armcb_v4l2_stream_t *pstream = dev->pstreams[sp->stream_id];
	struct vb2_queue *q = &sp->vb2_q;

	int outport_idx = -1;
	int i;
	int rc = 0;
	struct v4l2_event ev;
	unsigned int *pData = NULL;

	LOG(LOG_INFO, "### %s ctx_id:%d stream_id:%d", dev_name(&vdev->dev),
			dev->ctx_id, sp->stream_id);

	if (vb2_is_busy(q)) {
		LOG(LOG_INFO, "Buffer is busy and set fmt failed!");
		return -EBUSY;
	}

	mutex_lock(&dev->queue_lock);
	vdev->queue = &sp->vb2_q;

	for (i = 0; i < V4L2_STREAM_TYPE_MAX; i++) {
		if ((dev->stream_mask & (1 << i)) == 0) {
			dev->stream_mask |= (1 << i);
			sp->stream_id = i;
			sp->ctx_id = dev->ctx_id;
			break;
		}
	}

	LOG(LOG_INFO, "ctx_id: %d, called for stream id:%d.",
						dev->ctx_id, sp->stream_id);

	/* init stream */
	armcb_v4l2_stream_init(&dev->pstreams[sp->stream_id], sp->stream_id,
				   dev->ctx_id);
	pstream = dev->pstreams[sp->stream_id];
	if (pstream == NULL) {
		LOG(LOG_ERR, "stream alloc failed\n");
		mutex_unlock(&dev->queue_lock);
		return -ENOMEM;
	}

	/* init vb2 queue */
	rc = isp_vb2_queue_init(&sp->vb2_q, &dev->mutex,
				dev->pstreams[sp->stream_id],
				dev->v4l2_dev.dev);
	if (rc < 0) {
		LOG(LOG_ERR, "Error, vb2 queue init fail (rc=%d)", rc);
		//goto vb2_q_fail;
	}

	/* update stream pointer */
	rc = armcb_v4l2_stream_set_format(pstream, f);
	if (rc < 0) {
		LOG(LOG_ERR, "set format failed.");
		mutex_unlock(&dev->queue_lock);
		return rc;
	}

	atomic_set(&dev->port_idx_release, OUTPORT_IDX_IS_FREE);
	/* update stream pointer index */
	dev->stream_id_index[pstream->stream_type] = pstream->stream_id;
	outport_array[sp->ctx_id][sp->stream_id] = pstream->outport;
	outport_idx = armcb_outport_bits_to_idx(pstream->outport);
	if (outport_idx < 0 || outport_idx >= ISP_OUTPUT_PORT_MAX) {
		rc = -EINVAL;
		LOG(LOG_ERR, "invalid outport idx:%d, bits:%#x\n", outport_idx,
			pstream->outport);
		mutex_unlock(&dev->queue_lock);
		return rc;
	}

	if (g_outport_map[sp->ctx_id][outport_idx]) {
		rc = -EINVAL;
		LOG(LOG_ERR, "busy outport idx:%d, bits:%#x\n", outport_idx,
			pstream->outport);
		mutex_unlock(&dev->queue_lock);
		return rc;
	}

	/* Cheese or gstreamer support the VGA(640x480) need to support the
	 * BT601, so define the VGA support BT601 */
	rc = armcb_v4l2_config_bt601(&f->fmt.pix_mp);

	g_outport_map[sp->ctx_id][outport_idx] = pstream;
	memset(&ev, 0, sizeof(struct v4l2_event));
	ev.id = ISP_DAEMON_EVENT_SET_IMG_SIZE;
	ev.type = V4L2_EVENT_CTRL;

	pData = (unsigned int *)ev.u.data;
	pData[0] = dev->ctx_id;
	pData[1] = f->fmt.pix_mp.width;
	pData[2] = f->fmt.pix_mp.height;
	pData[3] = f->fmt.pix_mp.pixelformat;
	pData[4] = 30;
	v4l2_event_queue(&dev->vid_cap_dev, &ev);

    LOG(LOG_DEBUG, "pstream:%p, ctx_id:%d stream_id:%d stream_type:%d, outport_idx:%d, outport:%d",
		pstream, pstream->ctx_id, pstream->stream_id,
		pstream->stream_type, outport_idx, pstream->outport);

	mutex_unlock(&dev->queue_lock);

	return 0;
}

int armcb_v4l2_try_fmt_vid_cap_mplane(struct file *file, void *priv,
					  struct v4l2_format *f)
{
	/*empty function*/
	/*we get fmt from userspace*/
	return 0;
}

/* vb2 customization for multi-stream support */
int armcb_v4l2_reqbufs(struct file *file, void *priv,
				  struct v4l2_requestbuffers *p)
{
	struct armcb_isp_v4l2_fh *sp = fh_to_private(file->private_data);
	struct video_device *vdev = video_devdata(file);
	int rc = 0;

	vdev->queue = &sp->vb2_q;

	if (armcb_v4l2_is_q_busy(&sp->vb2_q, file))
		return -EBUSY;

	if (p)
		p->flags |= V4L2_MEMORY_FLAG_NON_COHERENT;

	rc = vb2_reqbufs(&sp->vb2_q, p);
	if (rc == 0)
		sp->vb2_q.owner = p->count ? file->private_data : NULL;

	return rc;
}

static int armcb_v4l2_querybuf(struct file *file, void *priv,
				   struct v4l2_buffer *p)
{
	struct armcb_isp_v4l2_fh *sp = fh_to_private(file->private_data);
	int rc = 0;

	rc = vb2_querybuf(&sp->vb2_q, p);
	LOG(LOG_DEBUG, "sid:%d querybuf p->type:%d p->index:%d , rc %d",
		sp->stream_id, p->type, p->index, rc);
	return rc;
}

static int armcb_v4l2_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct video_device *vdev = video_devdata(file);
	int rc = 0;
#ifndef V4L2_OPT
	armcb_v4l2_stream_t *pstream = NULL;
#endif
#if (KERNEL_VERSION(5, 10, 0) <= LINUX_VERSION_CODE)
	armcb_v4l2_dev_t *dev = video_drvdata(file);
#endif
	struct armcb_isp_v4l2_fh *sp = fh_to_private(file->private_data);

	LOG(LOG_DEBUG, "ctx_id:%d stream_id = %d, stream_q: %p, device_q: %p", sp->ctx_id,
	sp->stream_id, &sp->vb2_q, vdev->queue);

#ifndef V4L2_OPT
	//find stream pointer
	pstream = armcb_v4l2_get_stream(sp->ctx_id, sp->stream_id);
	if (pstream) {
		if (pstream->stream_started == 0) {
			if (p->reserved) {
				LOG(LOG_WARN,
					"set reserved buffer %p userptr:%p", p,
					p->m.planes->m.userptr);
				pstream->reserved_buf_addr =
					(u32)p->m.planes->m.userptr;
				return 0;
			}
		}
	}
#endif

	LOG(LOG_DEBUG, "ctx_id:%d stream_id = %d, ownermatch=%d", sp->ctx_id,
		sp->stream_id, armcb_v4l2_is_q_busy(&sp->vb2_q, file));
	if (armcb_v4l2_is_q_busy(&sp->vb2_q, file))
		return -EBUSY;
#if (KERNEL_VERSION(5, 10, 0) <= LINUX_VERSION_CODE)
	rc = vb2_qbuf(&sp->vb2_q, dev->v4l2_dev.mdev, p);
#else
	rc = vb2_qbuf(&sp->vb2_q, p);
#endif
	LOG(LOG_DEBUG, "ctx_id:%d stream_id:%d qbuf p->type:%d p->index:%d, rc %d",
		sp->ctx_id, sp->stream_id, p->type, p->index, rc);
	return rc;
}

static int armcb_v4l2_dqbuf(struct file *file, void *priv,
				struct v4l2_buffer *p)
{
	struct armcb_isp_v4l2_fh *sp = fh_to_private(file->private_data);
	struct video_device *vdev = video_devdata(file);
	int rc = 0;

	LOG(LOG_DEBUG, "ctx_id:%d stream_id = %d, stream_q: %p, device_q: %p", sp->ctx_id,
		sp->stream_id, &sp->vb2_q, vdev->queue);

	if (armcb_v4l2_is_q_busy(&sp->vb2_q, file))
		return -EBUSY;

	rc = vb2_dqbuf(&sp->vb2_q, p, file->f_flags & O_NONBLOCK);
	LOG_RATELIMITED(
		LOG_DEBUG,
		"ctx_id:%d stream_id:%d dqbuf p->type:%d p->index:%d, rc %d",
		sp->ctx_id, sp->stream_id, p->type, p->index, rc);

	return rc;
}

int armcb_v4l2_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	struct isp_v4l2_fmt *fmt;

	if (f->index >= (int)ARRAY_SIZE(isp_src_formats))
		return -EINVAL;

	fmt = &isp_src_formats[f->index];
	strncpy(f->description, fmt->name, sizeof(f->description) - 1);
	f->pixelformat = fmt->fourcc;
	f->mbus_code = fmt->mbus_code;

	LOG(LOG_INFO, "function exit : %d", f->pixelformat);

	return 0;
}

static int armcb_v4l2_enum_framesizes(struct file *file, void *priv,
					 struct v4l2_frmsizeenum *fsize)
{
	static const struct v4l2_frmsize_discrete frame_sizes[] = {
		{ .width = 3264, .height = 2160 },
		{ .width = 1920, .height = 1080 },
		{ .width = 1280, .height = 720 },
		{ .width = 640, .height = 480 },
	};

	if (fsize->index >= ARRAY_SIZE(frame_sizes))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete = frame_sizes[fsize->index];

	return 0;
}

/* Create a buffer for discard operation, before frist frame we need to an
 * address for all the vout port
 * */
int armcb_v4l2_alloc_discard_buffer(u32 reserved_size, struct device *dev)
{
	struct device *dma_dev = mem_dev;

	dma_addr_t dma_handle;
	void *cpu_buf_addr = NULL;
	int rc = 0;

	LOG(LOG_DEBUG, "Allocating %u bytes\n", reserved_size);

	if ((!dma_dev) || (!dev)) {
		LOG(LOG_ERR, "DMA device not initialized\n");
		return -ENODEV;
	}

	/* Alloc DMA buffer for reserved buffer */
	cpu_buf_addr = dma_alloc_coherent(mem_dev,PAGE_ALIGN(reserved_size),
			&dma_handle, GFP_KERNEL);
	if (!cpu_buf_addr) {
		LOG(LOG_ERR, "DMA alloc failed for size %u\n", reserved_size);
		return -ENOMEM;
	}

	/* Update the buffer information */

	discard_buf_addr_dma = dma_handle ;
	discard_buf_addr = cpu_buf_addr;
	discard_buf_size = PAGE_ALIGN(reserved_size);
	discard_dma_handle = dma_handle;

	LOG(LOG_DEBUG, "Allocated: virt=%p, dma=0x%0x, size=%u\n",
			discard_buf_addr, discard_buf_addr_dma, reserved_size);

	return rc;
}

int armcb_v4l2_release_discard_buffer(u32 reserved_size, struct device *dev)
{
	struct device *dma_dev = mem_dev;
	int ret = 0;

	dma_free_coherent(dma_dev, reserved_size,
					discard_buf_addr, discard_dma_handle);
	return ret;
}

int armcb_v4l2_config_init_update_stream_hw_addr(armcb_v4l2_dev_t *dev)
{
	u32 vout_reg1 = 0;
	u32 vout_reg2 = 0;
	u32 startaddr = 0;
	int i = 0, rc = 0;

	for(i = 0; i < sizeof(vout_idx)/sizeof(vout_idx[0]); i++) {
		armcb_init_output_addr_by_i7(&vout_reg1, &vout_reg2, vout_idx[i]);
		startaddr = discard_buf_addr_dma;
		if (vout_reg1 && startaddr) {
			armcb_isp_write_reg(vout_reg1, startaddr);
		}

		if (vout_reg2 && startaddr) {
			armcb_isp_write_reg(vout_reg2, startaddr);
		}
	}

	return rc;
}

//Sync the daemon and other application
static int armcb_v4l2_s_ctrl(struct file *file, void *fh,
                            struct v4l2_control *ctrl)
{
	int rc = -1;
	armcb_v4l2_dev_t *dev = video_drvdata(file);
	struct armcb_isp_v4l2_fh *sp = fh_to_private(fh);
	armcb_v4l2_stream_t *pstream = dev->pstreams[sp->stream_id];
	struct video_device *vdev = &dev->vid_cap_dev;

	switch (ctrl->id) {
		case ISP_DAEMON_SET_STREAM_ON:
			LOG(LOG_INFO, "devname:%s online_mode:%d, ISP_DAEMON_SET_STREAM_ON\n",
					dev_name(&vdev->dev), ctrl->value);
			dev->online_mode = ctrl->value;
			if (ctrl->value) {
				/* Live sensor: discard buffers on vout until app queues real buffers */
				armcb_v4l2_config_init_update_stream_hw_addr(dev);
				rc = armcb_isp_hw_apply_list(CMD_TYPE_STREAMON);
				if (rc < 0) {
					LOG(LOG_ERR, "armcb_isp_hw_apply_list failed ret(%d)", rc);
				} else {
					LOG(LOG_INFO, "pstream:%p, armcb_isp_hw_apply_list success",
							pstream);
				}
				atomic_set(&dev->stream_on_cnt, STREAM_ON);
			} else if (armcb_multi_cam) {
				/* Multi-cam only: offline ctx wakes GStreamer without touching all VOUTs */
				rc = 0;
				atomic_set(&dev->stream_on_cnt, STREAM_ON);
			} else {
				/* Single-cam: preserve legacy offline STREAMON path */
				armcb_v4l2_config_init_update_stream_hw_addr(dev);
				rc = armcb_isp_hw_apply_list(CMD_TYPE_STREAMON);
				atomic_set(&dev->stream_on_cnt, STREAM_ON);
				if (rc < 0)
					LOG(LOG_ERR, "armcb_isp_hw_apply_list failed ret(%d)", rc);
			}
			dev->streamon_done = 1;
			dev->streamoff_done = 0;
			wake_up_interruptible(&dev->wait_streamon_queue);
			break;
		case ISP_DAEMON_SET_FMT:
			/* Set the streamon done flag to wake the s_fmt queue */
			dev->s_fmt_done = ctrl->value;
			LOG(LOG_INFO,"set fmt done %d", dev->s_fmt_done);
			wake_up_interruptible(&dev->wait_fmt_queue);
			rc = 0;
			break;
		case ISP_DAEMON_SET_MULTI_CAM:
			armcb_multi_cam = ctrl->value;
			LOG(LOG_INFO,"set the multi camera %d", armcb_multi_cam);
			rc = 0;
			break;
		case ISP_DAEMON_SET_STREAM_OFF: {
			struct vb2_queue *q_stop;
			bool do_vb2 = false;

			LOG(LOG_INFO, "ISP_DAEMON_SET_STREAM_OFF enter, devname:%s\n",
					dev_name(&vdev->dev));
			/* Only isp_app calls this. If the queue (e.g. applied by gstreamer) is not yet
			 * released, run vb2_streamoff/release here; if already released (queue is null),
			 * skip vb2. armcb_isp_vb2_streamoff internally checks owner and only releases
			 * when appropriate. Hardware streamoff is always run. */
			mutex_lock(&dev->queue_lock);
			q_stop = vdev->queue;
			do_vb2 = (q_stop != NULL);
			if (!q_stop)
				LOG(LOG_INFO, "ISP_DAEMON_SET_STREAM_OFF: queue already released, hw streamoff only, skip vb2\n");
			else
				LOG(LOG_INFO, "ISP_DAEMON_SET_STREAM_OFF: queue not released yet, run hw streamoff + vb2 release\n");
			mutex_unlock(&dev->queue_lock);

			/* Always do hardware streamoff so that ISP is powered down. */
			LOG(LOG_INFO, "ISP_DAEMON_SET_STREAM_OFF: run hardware streamoff (disable_irq, STREAMOFF, POWERDOWN)\n");
			armcb_i7_disable_interrupt();
			rc = armcb_isp_hw_apply_list(CMD_TYPE_STREAMOFF);
			if (rc < 0) {
				LOG(LOG_ERR, "armcb_isp_hw_apply_list failed ret(%d)", rc);
				return rc;
			}
			rc = armcb_isp_hw_apply_list(CMD_TYPE_POWERDOWN);
			if (rc < 0)
			{
				LOG(LOG_ERR, "armcb_isp_hw_apply_list failed ret(%d)", rc);
				return rc;
			}
			/* Wait for hardware to complete any in-flight DMA transactions. */
			usleep_range(50000, 55000);
			LOG(LOG_INFO, "Hardware streamoff/powerdown completed");
			if (do_vb2) {
				armcb_isp_vb2_streamoff(file);
				LOG(LOG_INFO, "ISP_DAEMON_SET_STREAM_OFF: vb2 streamoff done\n");
			}
			atomic_set(&dev->stream_on_cnt, STREAM_OFF);
			atomic_set(&dev->port_idx_release, OUTPORT_IDX_IS_BUSY);
			dev->streamoff_done = 1;
			dev->streamon_done = 0;
			wake_up_interruptible(&dev->wait_streamoff_queue);
			break;
		}
		case ISP_DAEMON_SET_EXIT:
			LOG(LOG_INFO, "devname:%s, ISP_DAEMON_SET_EXIT",
						dev_name(&vdev->dev));
			atomic_set(&dev->daemon_exit, 1);
			wake_up_interruptible(&dev->daemon_exit_queue);
			break;
		case ISP_DAEMON_SET_READY:
			atomic_set(&armcb_daemon_ready, ctrl->value);
			atomic_set(&dev->daemon_exit, 0);
			wake_up_interruptible(&daemon_ready_queue);
			LOG(LOG_INFO,"set the deamon ready %d",
						atomic_read(&armcb_daemon_ready));
			rc = 0;
			break;
		default:
			break;
	}

	return rc;
}

static int
armcb_v4l2_core_subscribe_event(struct v4l2_fh *fh,
				  const struct v4l2_event_subscription *sub)
{
	int ret = -1;

	LOG(LOG_DEBUG, "type:%d id:%d", sub->type, sub->id);
	ret = v4l2_event_subscribe(fh, sub, CORE_NEVENTS, NULL);
	if (ret < 0)
		LOG(LOG_ERR, "armcb_v4l2_subscribe_event failed ret(%d)", ret);
	return ret;
}

static int
armcb_v4l2_core_unsubscribe_event(struct v4l2_fh *fh,
					const struct v4l2_event_subscription *sub)
{
	struct v4l2_event ev;

	memset(&ev, 0, sizeof(struct v4l2_event));
	ev.id = ISP_MIXTURE_INT;
	ev.type = V4L2_EVENT_CTRL;
	// Queue a empty event to notify userspace
	v4l2_event_queue(fh->vdev, &ev);

	LOG(LOG_INFO, "unsubscribe event: type:%d id:%d", sub->type, sub->id);
	v4l2_event_unsubscribe(fh, sub);

	return 0;
}


static int armcb_v4l2_g_selection(struct file *file, void *fh,
				 struct v4l2_selection *s)
{
	LOG(LOG_INFO, "enter: target = %d, type = %d",
		s->target, s->type);

	/* need fix: hard code for NV12M 1080p fmt */
	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = 1920;
		s->r.height = 1080;
		s->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		LOG(LOG_DEBUG, "function exit");
		return 0;
	}

	return -EINVAL;
}

static int armcb_v4l2_s_selection(struct file *file, void *fh,
				 struct v4l2_selection *s)
{
	LOG(LOG_DEBUG, "function enter");
	return 0;
}

static int armcb_v4l2_enum_frameintervals(struct file *file, void *fh,
                     struct v4l2_frmivalenum *interval) {
    /* Check the supporting formats*/
    if ((interval->pixel_format == V4L2_PIX_FMT_NV12M)
			|| (interval->pixel_format == V4L2_PIX_FMT_RGB24)) {
		switch (interval->index) {
			case 0:
				interval->type = V4L2_FRMIVAL_TYPE_DISCRETE;
				interval->discrete.numerator = 1;
				interval->discrete.denominator = 30; // 30fps
				return 0;
			case 1:
				interval->type = V4L2_FRMIVAL_TYPE_DISCRETE;
				interval->discrete.numerator = 1;
				interval->discrete.denominator = 15; // 15fps
				return 0;
			default:
				return -EINVAL;
		}
	}

	LOG(LOG_ERR, "Unsupported resolution: %dx%d",
			interval->width, interval->height);

	return -EINVAL;
}

static int armcb_g_param(struct file *file, void *fh,
			      struct v4l2_streamparm *a)

{
	LOG(LOG_INFO, "g_param enter");
	return 0;
}

static const struct v4l2_ioctl_ops armcb_ioctl_ops = {
	.vidioc_querycap = armcb_v4l2_querycap,

	.vidioc_g_fmt_vid_cap_mplane = armcb_v4l2_g_fmt_vid_cap_mplane,
	.vidioc_s_fmt_vid_cap_mplane = armcb_v4l2_s_fmt_vid_cap_mplane,
	.vidioc_try_fmt_vid_cap_mplane = armcb_v4l2_try_fmt_vid_cap_mplane,

	.vidioc_reqbufs = armcb_v4l2_reqbufs,
	.vidioc_querybuf = armcb_v4l2_querybuf,
	.vidioc_qbuf = armcb_v4l2_qbuf,
	.vidioc_dqbuf = armcb_v4l2_dqbuf,

	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_expbuf = NULL,

	.vidioc_enum_fmt_vid_cap = armcb_v4l2_enum_fmt_vid_cap,
	.vidioc_enum_framesizes = armcb_v4l2_enum_framesizes,

	.vidioc_streamon = armcb_v4l2_streamon,
	.vidioc_streamoff = armcb_v4l2_streamoff,
	.vidioc_log_status = armcb_v4l2_log_status,
	.vidioc_subscribe_event = armcb_v4l2_core_subscribe_event,
	.vidioc_unsubscribe_event = armcb_v4l2_core_unsubscribe_event,

	.vidioc_g_selection = armcb_v4l2_g_selection,
	.vidioc_s_selection = armcb_v4l2_s_selection,
	.vidioc_enum_frameintervals = armcb_v4l2_enum_frameintervals,
	.vidioc_g_parm = armcb_g_param,

	//sync
	.vidioc_s_ctrl = armcb_v4l2_s_ctrl,
};

/*-----------------------------------------------------------------
 *	Initialization and module stuff
 *  ---------------------------------------------------------------
 */
armcb_v4l2_dev_t *armcb_v4l2_core_get_dev(uint32_t ctx_id)
{
	LOG(LOG_DEBUG, "get ctx_id:%d pdev:%p", ctx_id,
		g_isp_v4l2_devs[ctx_id]);
	return g_isp_v4l2_devs[ctx_id];
}

uint32_t armcb_v4l2_core_find_1st_opened_dev(void)
{
	uint32_t i = 0;
	armcb_v4l2_dev_t *pdev = NULL;

	for (i = 0; i < ARMCB_MAX_DEVS; i++) {
		pdev = armcb_v4l2_core_get_dev(i);
		if (pdev && atomic_read(&pdev->opened) > 0)
			break;
	}

	if (i == ARMCB_MAX_DEVS)
		LOG(LOG_WARN, "No v4l2 device opened");
	return i;
}

static void armcb_v4l2_dev_release(struct v4l2_device *v4l2_dev)
{
	armcb_v4l2_dev_t *dev =
		container_of(v4l2_dev, armcb_v4l2_dev_t, v4l2_dev);

	v4l2_device_unregister(&dev->v4l2_dev);
	kfree(dev);
}

static armcb_v4l2_dev_t *
armcb_v4l2_create_instance(struct platform_device *pdev, int ctx_id,
			   struct device *devnode)
{
	armcb_v4l2_dev_t *dev = NULL;
	struct video_device *vfd = NULL;
	int ret = 0;
	int i = 0;

	LOG(LOG_INFO, " ctx_id(%d) +", ctx_id);
	/* allocate main vivid state structure */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		LOG(LOG_ERR, "failed to alloc memory for armcb dev.");
		return NULL;
	}

	dev->ctx_id = ctx_id;
	if (pdev)
		dev->pvdev = pdev;
	else if (devnode)
		dev->pvdev = (struct platform_device *)devnode;
	else {
		LOG(LOG_ERR, "invalid device for drivers.");
		kfree(dev);
		return NULL;
	}

#ifdef CONFIG_MEDIA_CONTROLLER
	dev->v4l2_dev.mdev = &dev->mdev;

	/* Initialize media device */
	(void)strscpy(dev->mdev.model, ARMCB_MODULE_NAME,
			  sizeof(dev->mdev.model));
	snprintf(dev->mdev.bus_info, sizeof(dev->mdev.bus_info),
		 "platform:%s-%03d", ARMCB_MODULE_NAME, ctx_id);
	dev->mdev.dev = devnode;
	devnode->coherent_dma_mask = -1;
	media_device_init(&dev->mdev);
#endif

	/* register v4l2_device */
	snprintf(dev->v4l2_dev.name, sizeof(dev->v4l2_dev.name), "%s-%02d",
		 ARMCB_MODULE_NAME, ctx_id);
	LOG(LOG_INFO, "dev->v4l2_dev.name[%s]", dev->v4l2_dev.name);
	ret = v4l2_device_register(devnode, &dev->v4l2_dev);
	if (ret) {
		kfree(dev);
		return NULL;
	}
	dev->v4l2_dev.release = armcb_v4l2_dev_release;
	dev->v4l2_dev.notify = armcb_v4l2_subdev_notify;

	/* set up the capabilities of the video capture device */
	dev->vid_cap_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING |
				V4L2_CAP_READWRITE;
	/* initialize locks */
	spin_lock_init(&dev->slock);
	spin_lock_init(&dev->v4l2_event_slock);

	dev->ddr_lp_mode = 1;

	mutex_init(&dev->mutex);
	mutex_init(&dev->v4l2_event_mutex);
	mutex_init(&dev->ordered_sd_mutex);
	mutex_init(&dev->queue_lock);
	mutex_init(&dev->ctl_lock);
	mutex_init(&dev->wait_lock);

	/* init subdev list */
	INIT_LIST_HEAD(&dev->ordered_sd_list);

	/* defualt video device*/
	dev->has_vid_cap = 1;

	/* init the queue for wait queue */
	init_waitqueue_head(&dev->wait_fmt_queue);
	init_waitqueue_head(&dev->wait_streamon_queue);
	init_waitqueue_head(&dev->wait_streamoff_queue);
	init_waitqueue_head(&daemon_ready_queue);
	init_waitqueue_head(&dev->daemon_exit_queue);

	/* initialize stream id table */
	for (i = 0; i < V4L2_STREAM_TYPE_MAX; i++)
		dev->stream_id_index[i] = -1;
	/* initialize open counter */
	atomic_set(&dev->stream_on_cnt, STREAM_DEFAULT);
	atomic_set(&dev->port_idx_release, OUTPORT_IDX_DEFAULT);
	atomic_set(&dev->opened, 0);
	atomic_set(&dev->upload_streamoff, 0);
	atomic_set(&armcb_daemon_ready, 0);
	atomic_set(&dev->daemon_exit, 0);

	dev->s_fmt_done = 0;
	dev->streamon_done = 0;
	dev->streamoff_done = 0;
	/* Buf queue status*/
	dev->is_streaming = 0;

	/* finally start creating the device nodes */
	if (dev->has_vid_cap) {
		vfd = &dev->vid_cap_dev;
		snprintf(vfd->name, sizeof(vfd->name), "armcb-%02d-vid-cap",
			 ctx_id);
		vfd->fops = &armcb_fops;
		vfd->ioctl_ops = &armcb_ioctl_ops;
		vfd->device_caps = dev->vid_cap_caps;
		vfd->release = video_device_release_empty;
		vfd->v4l2_dev = &dev->v4l2_dev;
		vfd->queue = &dev->vb2_q;
		vfd->tvnorms = 0;
		/*
		 * Provide a mutex to v4l2 core. It will be used to protect
		 * all fops and v4l2 ioctls.
		 */
		vfd->lock = &dev->mutex;
		video_set_drvdata(vfd, dev);
#if (KERNEL_VERSION(5, 10, 0) <= LINUX_VERSION_CODE)
		ret = video_register_device(vfd, VFL_TYPE_VIDEO, (9 - ctx_id));
#else
		ret = video_register_device(vfd, VFL_TYPE_GRABBER, (9 - ctx_id));
#endif
		if (ret < 0)
			goto unreg_dev;
		LOG(LOG_INFO, "V4L2 capture device registered as %s",
			video_device_node_name(vfd));
		LOG(LOG_INFO,
			"[has_vid_cap] vfd->name[%s] v4l2_dev.name[%s] dev_name[%s]",
			vfd->name, dev->v4l2_dev.name, video_device_node_name(vfd));
	}

	/* Now that everything is fine, let's add it to device list */
	LOG(LOG_INFO, "create video device instance success");

	return dev;

unreg_dev:
	video_unregister_device(&dev->vid_cap_dev);
	v4l2_device_put(&dev->v4l2_dev);
	kfree(dev);
	LOG(LOG_ERR, "create video device instance failed. ret = %d", ret);
	return NULL;
}

armcb_v4l2_dev_t *armcb_register_instance(struct platform_device *pdev,
					  struct device *devnode,
					  unsigned int cam_id)
{
	armcb_v4l2_dev_t *adev = NULL;

	if (cam_id + 1 >= ARMCB_MAX_DEVS) {
		LOG(LOG_ERR, "too many instance, current is %d.", cam_id);
		return NULL;
	}
	if (g_isp_v4l2_devs[cam_id] != NULL) {
		LOG(LOG_ERR, "camera %d has probe.", cam_id);
		return NULL;
	}

    adev = armcb_v4l2_create_instance(pdev, cam_id, devnode);
	if (adev == NULL) {
		LOG(LOG_ERR, "too many instance, current is %d.", cam_id);
		return NULL;
	}

	LOG(LOG_INFO, "register v4l2 video instance %d %p", cam_id, adev);
	g_isp_v4l2_devs[cam_id] = adev;
	return adev;
}

void armcb_cam_instance_destroy(void)
{
	int i = 0;

	for (; i < ARMCB_MAX_DEVS; i++) {
		if (g_isp_v4l2_devs[i] == NULL)
			continue;
		v4l2_async_nf_unregister(&g_isp_v4l2_devs[i]->dts_notifier);
#if (KERNEL_VERSION(4, 17, 0) <= LINUX_VERSION_CODE)
		v4l2_async_nf_cleanup(&g_isp_v4l2_devs[i]->dts_notifier);
#endif
		video_unregister_device(&g_isp_v4l2_devs[i]->vid_cap_dev);
		v4l2_device_put(&g_isp_v4l2_devs[i]->v4l2_dev);
		media_device_unregister(&g_isp_v4l2_devs[i]->mdev);
		LOG(LOG_INFO, "release armcb instance %d (%p)", i,
			g_isp_v4l2_devs[i]);
		g_isp_v4l2_devs[i] = NULL;
	}
	g_adev_idx = 0;
}
