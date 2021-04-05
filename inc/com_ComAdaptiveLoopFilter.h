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

#ifndef COM_COMADAPTIVELOOPFILTER_H
#define COM_COMADAPTIVELOOPFILTER_H

#include "com_def.h"

extern int tbl_weights_shape1_sym[ALF_MAX_NUM_COEF + 1];
#if ALF_SHAPE
extern int tbl_weights_shape2_sym[ALF_MAX_NUM_COEF_SHAPE2 + 1];
#endif
void set_filter_image(pel *dec_Y, pel *dec_U, pel *dec_V, int in_stride, pel *img_Y_out, pel * img_U_out, pel * img_V_out, int out_stride, int img_width, int img_height);
void reconstruct_coefficients(ALF_PARAM *alf_param, int **filter_coeff);
void reconstruct_coef_info(int comp_idx, ALF_PARAM *alf_param, int **filter_coeff, int *var_ind_tab);
void check_filter_coeff_value(int *filter, int filter_length);
void copy_alf_param(ALF_PARAM *dst, ALF_PARAM *src
#if ALF_SHAPE
    , int num_coef
#endif
);
void filter_one_comp_region(pel *img_res, pel *img_pad, int stride, BOOL is_chroma, int y_pos, int lcu_height, int x_pos,
    int lcu_width, int **filter_set, int *merge_table, pel **var_img,
    int sample_bit_depth, int is_left_avail, int is_right_avail, int is_above_avail, int is_below_avail, int is_above_left_avail,
    int isAboveRightAvail
#if ALF_SHAPE
    , BOOL alf_shape_flag
#endif
);
void extend_pic_border(pel *img, int height, int width, int margin_y, int margin_x, pel *img_ext);
int get_lcu_ctrl_ctx_idx(int ctu, int num_lcu_in_pic_width, int num_lcu_in_pic_height, int comp_idx, BOOL **alf_lcu_enabled);
int check_filtering_unit_boundary_extension(int x, int y, int lcu_pos_x, int lcu_pos_y, int start_x, int start_y, int end_x,
    int end_y, int is_above_left_avail, int is_left_avail, int is_above_right_avail, int is_right_avail);

int get_mem_2D_int(int ***array2D, int rows, int columns);
int get_mem_1D_int(int **array1D, int num);
void free_mem_1D_int(int *array1D);
void free_mem_2D_int(int **array2D);
void copy_frame_for_alf(COM_PIC *pic_dst, COM_PIC *pic_src);

#endif