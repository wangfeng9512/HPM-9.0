/* ====================================================================================================================

  The copyright in this software is being made available under the License included below.
  This software may be subject to other third party and contributor rights, including patent rights, and no such
  rights are granted under this license.

  Copyright (c) 2018, HUAWEI TECHNOLOGIES CO., LTD. All rights reserved.
  Copyright (c) 2018, SAMSUNG ELECTRONICS CO., LTD. All rights reserved.
  Copyright (c) 2018, PEKING UNIVERSITY SHENZHEN GRADUATE SCHOOL. All rights reserved.
  Copyright (c) 2018, PENGCHENG LABORATORY. All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, are permitted only for
  the purpose of developing standards within Audio and Video Coding Standard Workgroup of China (AVS) and for testing and
  promoting such standards. The following conditions are required to be met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
      the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
      the following disclaimer in the documentation and/or other materials provided with the distribution.
    * The name of HUAWEI TECHNOLOGIES CO., LTD. or SAMSUNG ELECTRONICS CO., LTD. may not be used to endorse or promote products derived from
      this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

* ====================================================================================================================
*/

#ifndef _COM_DF_H_
#define _COM_DF_H_

#include "com_def.h"

#ifdef __cplusplus
extern "C"
{
#endif

void create_edge_filter_avs2(int w, int h, int ***edge_filter);
void clear_edge_filter_avs2(int x, int y, int w, int h, int ***edge_filter);
void delete_edge_filter_avs2(int ***edge_filter, int h);
void set_edge_filter_param_hor_avs2(COM_PIC * pic, int ***edge_filter, int x_pel, int y_pel, int cuw, int cuh, int edge_condition);
void set_edge_filter_param_ver_avs2(COM_PIC * pic, int ***edge_filter, int x_pel, int y_pel, int cuw, int cuh, int edge_condition);
void set_edge_filter_avs2(COM_INFO *info, COM_MAP *map, COM_PIC * pic, int*** edge_filter);
void set_edge_filter_one_scu_avs2(COM_INFO *info, COM_MAP *map, COM_PIC * pic, int ***edge_filter, int x, int y, int cuw, int cuh, int cud, int cup, BOOL b_recurse);
void deblock_block_avs2(COM_INFO *info, COM_MAP *map, COM_PIC * pic, COM_REFP refp[MAX_NUM_REF_PICS][REFP_NUM], int*** edge_filter, int pel_x, int pel_y, int cuw, int cuh);

void deblock_frame_avs2(COM_INFO *info, COM_MAP *map, COM_PIC * pic, COM_REFP refp[MAX_NUM_REF_PICS][REFP_NUM], int*** edge_filter);

#if DBR
void deblock_mb_avs2(COM_INFO *info, COM_MAP *map, COM_PIC *pic, COM_REFP refp[MAX_NUM_REF_PICS][REFP_NUM], int*** edge_filter, int mb_y, int mb_x, int edge_dir, COM_PIC *pic_org, int enc, DBR_PARAM *dbr_picture_param
#if RDO_DBK_LUMA_ONLY
    , int only_luma
#endif
);
#endif

#ifdef __cplusplus
}
#endif




#endif /* _COM_DF_H_ */
