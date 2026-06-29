/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Parse running isp_app process command line arguments and derive allowed V4L2
 * instance mask (ctx_id 0..3 -> armcb_isp_v4l2-00 .. -03). When the mask is
 * multi-camera (ctx0 and at least one other ctx allowed), ctx 1..3 require an
 * open fd on -00 first. See armcb_isp_execstart.c.
 */
#ifndef __ARMCB_ISP_EXECSTART_H__
#define __ARMCB_ISP_EXECSTART_H__

#include <linux/types.h>

void armcb_isp_execstart_refresh_if_needed(void);
void armcb_isp_execstart_force_reload(void);
bool armcb_isp_execstart_ctx_allowed(unsigned int ctx_id);
bool armcb_isp_execstart_multi_open_order_ok(unsigned int ctx_id);
void armcb_isp_execstart_note_ctx0_open(void);
void armcb_isp_execstart_note_ctx0_release(void);
unsigned int armcb_isp_execstart_get_allowed_mask(void);

#endif /* __ARMCB_ISP_EXECSTART_H__ */
