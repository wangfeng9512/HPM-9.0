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

#ifndef DEC_DECADAPTIVELOOPFILTER_H
#define DEC_DECADAPTIVELOOPFILTER_H

#include "com_def.h"
#include "com_ComAdaptiveLoopFilter.h"
#include  "dec_def.h"
#include <math.h>

void create_alf_global_buffer(DEC_CTX *ctx);
void release_alf_global_buffer(DEC_CTX *ctx);

void alf_process_dec(DEC_CTX* ctx, ALF_PARAM **alf_param, COM_PIC* pic_rec, COM_PIC* pic_dec);
void filter_one_ctb(DEC_ALF_VAR * dec_alf, pel *rec, pel *dec, int stride, int comp_idx, int bit_depth, ALF_PARAM *alf_param
    , int lcu_y_pos, int lcu_height, int lcu_x_pos, int lcu_width
    , BOOL is_above_avail, BOOL is_below_avail, BOOL is_left_avail, BOOL is_right_avail, BOOL is_above_left_avail
    , BOOL is_above_right_avail, int sample_bit_depth
#if ALF_SHAPE
    , BOOL alf_shape_flag
#endif
);
void derive_alf_boundary_availibility(DEC_CTX *ctx, int num_lcu_in_pic_width, int num_lcu_in_pic_height, int lcu_idx,
    BOOL *is_left_avail, BOOL *is_right_avail, BOOL *is_above_avail, BOOL *is_below_avail);

void derive_boundary_avail(DEC_CTX *ctx, int num_lcu_in_pic_width, int num_lcu_in_pic_height, int ctu, BOOL *is_left_avail,
    BOOL *is_right_avail, BOOL *is_above_avail, BOOL *is_below_avail, BOOL *is_above_left_avail, BOOL *is_above_right_avail);

void allocate_alf_param(ALF_PARAM **alf_param, int comp_idx
#if ALF_SHAPE
    , int num_coef
#endif
);
void free_alf_param(ALF_PARAM *alf_param, int comp_idx);

#endif