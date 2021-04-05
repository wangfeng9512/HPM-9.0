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

#include "com_ComAdaptiveLoopFilter.h"

int tbl_weights_shape1_sym[ALF_MAX_NUM_COEF + 1] =
{
    2,
    2,
    2, 2, 2,
    2, 2, 2, 1,
    1
};
#if ALF_SHAPE
int tbl_weights_shape2_sym[ALF_MAX_NUM_COEF_SHAPE2 + 1] =
{
    2,
    2, 2, 2, 2, 2,
    2, 2, 2, 2, 2,
    2, 2, 2, 1,
    1
};
#endif

void reconstruct_coefficients(ALF_PARAM *alf_param, int **filter_coeff)
{
    int g, sum, i, coeff_pred;
    for (g = 0; g < alf_param->filters_per_group; g++)
    {
        sum = 0;
        for (i = 0; i < alf_param->num_coeff - 1; i++)
        {
            sum += (2 * alf_param->coeff_multi[g][i]);
            filter_coeff[g][i] = alf_param->coeff_multi[g][i];
        }
        coeff_pred = (1 << ALF_NUM_BIT_SHIFT) - sum;
        filter_coeff[g][alf_param->num_coeff - 1] = coeff_pred + alf_param->coeff_multi[g][alf_param->num_coeff - 1];
    }
}

void reconstruct_coef_info(int comp_idx, ALF_PARAM *alf_param, int **filter_coeff, int *var_ind_tab)
{
    int i;
    if (comp_idx == Y_C)
    {
        memset(var_ind_tab, 0, NO_VAR_BINS * sizeof(int));
        if (alf_param->filters_per_group > 1)
        {
            for (i = 1; i < NO_VAR_BINS; ++i)
            {
                if (alf_param->filter_pattern[i])
                {
                    var_ind_tab[i] = var_ind_tab[i - 1] + 1;
                }
                else
                {
                    var_ind_tab[i] = var_ind_tab[i - 1];
                }
            }
        }
    }
    reconstruct_coefficients(alf_param, filter_coeff);
}

void extend_pic_border(pel *img, int height, int width, int margin_y, int margin_x, pel *img_ext)
{
    int   x, y, stride;
    pel  *pin;
    pel  *pout;
    pin = img;
    pout = img_ext + margin_y * (width + 2 * margin_x) + margin_x;
    for (y = 0; y < height; y++)
    {
        memcpy(pout, pin, width * sizeof(pel));
        pin = pin + width;
        pout = pout + (width + 2 * margin_x);
    }
    pout = img_ext + margin_y * (width + 2 * margin_x) + margin_x;
    stride = (width + 2 * margin_x);
    for (y = 0; y < height; y++)
    {
        for (x = 0; x < margin_x; x++)
        {
            pout[-margin_x + x] = pout[0];
            pout[width + x] = pout[width - 1];
        }
        pout += stride;
    }
    pout -= (stride + margin_x);
    for (y = 0; y < margin_y; y++)
    {
        memcpy(pout + (y + 1)*stride, pout, sizeof(pel) * (width + (margin_x << 1)));
    }
    pout -= ((height - 1) * stride);
    for (y = 0; y < margin_y; y++)
    {
        memcpy(pout - (y + 1)*stride, pout, sizeof(pel) * (width + (margin_x << 1)));
    }
}

//(x,y): the pixel used for filtering
//(lcu_pos_x, lcu_pos_y): the LCU position within the picture
//start_x: x start position of the current filtering unit
//start_y: y start position of the current filtering unit
//end_x: x end position of the current filtering unit
//end_y: y end position of the current filtering unit
int check_filtering_unit_boundary_extension(int x, int y, int lcu_pos_x, int lcu_pos_y, int start_x, int start_y, int end_x,
    int end_y, int is_above_left_avail, int is_left_avail, int is_above_right_avail, int is_right_avail)
{
    int modified_x;
    if (x < lcu_pos_x)
    {
        if (y < start_y)
        {
            modified_x = lcu_pos_x;
        }
        else if (y < lcu_pos_y)
        {
            if (is_above_left_avail)
            {
                modified_x = x;
            }
            else
            {
                modified_x = lcu_pos_x;
            }
        }
        else if (y <= end_y)
        {
            if (is_left_avail)
            {
                modified_x = x;
            }
            else
            {
                modified_x = lcu_pos_x;
            }
        }
        else
        {
            modified_x = lcu_pos_x;
        }
    }
    else if (x <= end_x)
    {
        modified_x = x;
    }
    else
    {
        // x>endX
        if (y < start_y)
        {
            modified_x = end_x;
        }
        else if (y < lcu_pos_y)
        {
            if (is_above_right_avail)
            {
                modified_x = x;
            }
            else
            {
                modified_x = end_x;
            }
        }
        else if (y <= end_y)
        {
            if (is_right_avail)
            {
                modified_x = x;
            }
            else
            {
                modified_x = end_x;
            }
        }
        else
        {
            modified_x = end_x;
        }
    }
    return modified_x;
}

void filter_one_comp_region(pel *img_res, pel *img_pad, int stride, BOOL is_chroma, int y_pos, int lcu_height, int x_pos,
    int lcu_width, int **filter_set, int *merge_table, pel **var_img,
    int sample_bit_depth, int is_left_avail, int is_right_avail, int is_above_avail, int is_below_avail, int is_above_left_avail,
    int isAboveRightAvail
#if ALF_SHAPE
    , BOOL alf_shape_flag
#endif
)
{
    int offset = (1 << ((int)ALF_NUM_BIT_SHIFT - 1));
    pel *img_pad1, *img_pad2, *img_pad3, *img_pad4, *img_pad5, *img_pad6;
    pel *var = var_img[y_pos >> LOG2_VAR_SIZE_H] + (x_pos >> LOG2_VAR_SIZE_W);
    int i, j, pixel_int;
    int *coef = (is_chroma) ? filter_set[0] : filter_set[merge_table[*(var)]];
    int start_pos = is_above_avail ? (y_pos - 4) : y_pos;
    int end_pos = is_below_avail ? (y_pos + lcu_height - 4) : (y_pos + lcu_height);
    int x_offset_left = is_left_avail ? -3 : 0;
    int x_offset_right = is_right_avail ? 3 : 0;
    int x_pos_end = x_pos + lcu_width;
    int y_up, y_bottom;
    int x_left, x_right;
    img_pad += (start_pos * stride);
    img_res += (start_pos * stride);
#if ALF_SHAPE
    if (!alf_shape_flag)
    {
#endif
    for (i = start_pos; i < end_pos; i++)
    {
#if ALF_DEC_OPT
        y_up = COM_MAX(start_pos, i - 1);
        y_bottom = COM_MIN(end_pos - 1, i + 1);
        img_pad1 = img_pad + (y_bottom - i) * stride;
        img_pad2 = img_pad + (y_up - i) * stride;
        y_up = COM_MAX(start_pos, i - 2);
        y_bottom = COM_MIN(end_pos - 1, i + 2);
        img_pad3 = img_pad + (y_bottom - i) * stride;
        img_pad4 = img_pad + (y_up - i) * stride;
        y_up = COM_MAX(start_pos, i - 3);
        y_bottom = COM_MIN(end_pos - 1, i + 3);
        img_pad5 = img_pad + (y_bottom - i) * stride;
        img_pad6 = img_pad + (y_up - i) * stride;
#else
        y_up = COM_CLIP3(start_pos, end_pos - 1, i - 1);
        y_bottom = COM_CLIP3(start_pos, end_pos - 1, i + 1);
        img_pad1 = img_pad + (y_bottom - i) * stride;
        img_pad2 = img_pad + (y_up - i) * stride;
        y_up = COM_CLIP3(start_pos, end_pos - 1, i - 2);
        y_bottom = COM_CLIP3(start_pos, end_pos - 1, i + 2);
        img_pad3 = img_pad + (y_bottom - i) * stride;
        img_pad4 = img_pad + (y_up - i) * stride;
        y_up = COM_CLIP3(start_pos, end_pos - 1, i - 3);
        y_bottom = COM_CLIP3(start_pos, end_pos - 1, i + 3);
        img_pad5 = img_pad + (y_bottom - i) * stride;
        img_pad6 = img_pad + (y_up - i) * stride;
#endif
        for (j = x_pos; j < x_pos_end; j++)
        {
            pixel_int = coef[0] * (img_pad5[j] + img_pad6[j]);
            pixel_int += coef[1] * (img_pad3[j] + img_pad4[j]);
            pixel_int += coef[3] * (img_pad1[j] + img_pad2[j]);
#if ALF_DEC_OPT
            if (i >= y_pos + 3 && i <= end_pos - 4)
            {
                x_left = COM_MAX(x_pos + x_offset_left, j - 1);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 1);
                pixel_int += coef[2] * (img_pad2[x_left] + img_pad1[x_right]);
                pixel_int += coef[4] * (img_pad2[x_right] + img_pad1[x_left]);
                pixel_int += coef[7] * (img_pad[x_right] + img_pad[x_left]);
                x_left = COM_MAX(x_pos + x_offset_left, j - 2);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 2);
                pixel_int += coef[6] * (img_pad[x_right] + img_pad[x_left]);
                x_left = COM_MAX(x_pos + x_offset_left, j - 3);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 3);
                pixel_int += coef[5] * (img_pad[x_right] + img_pad[x_left]);
            }
            else
            {
#endif
                // upper left c2
                x_left = check_filtering_unit_boundary_extension(j - 1, i - 1, x_pos, y_pos, x_pos, start_pos, x_pos_end - 1, end_pos - 1,
                    is_above_left_avail, is_left_avail, isAboveRightAvail, is_right_avail);
                pixel_int += coef[2] * img_pad2[x_left];
                // upper right c4
                x_right = check_filtering_unit_boundary_extension(j + 1, i - 1, x_pos, y_pos, x_pos, start_pos, x_pos_end - 1, end_pos - 1,
                    is_above_left_avail, is_left_avail, isAboveRightAvail, is_right_avail);
                pixel_int += coef[4] * img_pad2[x_right];
                // lower left c4
                x_left = check_filtering_unit_boundary_extension(j - 1, i + 1, x_pos, y_pos, x_pos, start_pos, x_pos_end - 1, end_pos - 1,
                    is_above_left_avail, is_left_avail, isAboveRightAvail, is_right_avail);
                pixel_int += coef[4] * img_pad1[x_left];
                // lower right c2
                x_right = check_filtering_unit_boundary_extension(j + 1, i + 1, x_pos, y_pos, x_pos, start_pos, x_pos_end - 1, end_pos - 1,
                    is_above_left_avail, is_left_avail, isAboveRightAvail, is_right_avail);
                pixel_int += coef[2] * img_pad1[x_right];
#if ALF_DEC_OPT
                x_left = COM_MAX(x_pos + x_offset_left, j - 1);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 1);
                pixel_int += coef[7] * (img_pad[x_right] + img_pad[x_left]);
                x_left = COM_MAX(x_pos + x_offset_left, j - 2);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 2);
                pixel_int += coef[6] * (img_pad[x_right] + img_pad[x_left]);
                x_left = COM_MAX(x_pos + x_offset_left, j - 3);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 3);
                pixel_int += coef[5] * (img_pad[x_right] + img_pad[x_left]);
            }
#else
            x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 1);
            x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 1);
            pixel_int += coef[7] * (img_pad[x_right] + img_pad[x_left]);
            x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 2);
            x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 2);
            pixel_int += coef[6] * (img_pad[x_right] + img_pad[x_left]);
            x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 3);
            x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 3);
            pixel_int += coef[5] * (img_pad[x_right] + img_pad[x_left]);
#endif
            pixel_int += coef[8] * (img_pad[j]);
            pixel_int = (int)((pixel_int + offset) >> ALF_NUM_BIT_SHIFT);
            img_res[j] = (pel)COM_CLIP3(0, ((1 << sample_bit_depth) - 1), pixel_int);
        }
        img_pad += stride;
        img_res += stride;
    }
#if ALF_SHAPE
    }
    else
    {
        for (i = start_pos; i < end_pos; i++)
        {
#if ALF_DEC_OPT
            y_up = COM_MAX(start_pos, i - 1);
            y_bottom = COM_MIN(end_pos - 1, i + 1);
            img_pad1 = img_pad + (y_bottom - i) * stride;
            img_pad2 = img_pad + (y_up - i) * stride;
            y_up = COM_MAX(start_pos, i - 2);
            y_bottom = COM_MIN(end_pos - 1, i + 2);
            img_pad3 = img_pad + (y_bottom - i) * stride;
            img_pad4 = img_pad + (y_up - i) * stride;
            y_up = COM_MAX(start_pos, i - 3);
            y_bottom = COM_MIN(end_pos - 1, i + 3);
            img_pad5 = img_pad + (y_bottom - i) * stride;
            img_pad6 = img_pad + (y_up - i) * stride;
#else
            y_up = COM_CLIP3(start_pos, end_pos - 1, i - 1);
            y_bottom = COM_CLIP3(start_pos, end_pos - 1, i + 1);
            img_pad1 = img_pad + (y_bottom - i) * stride;
            img_pad2 = img_pad + (y_up - i) * stride;
            y_up = COM_CLIP3(start_pos, end_pos - 1, i - 2);
            y_bottom = COM_CLIP3(start_pos, end_pos - 1, i + 2);
            img_pad3 = img_pad + (y_bottom - i) * stride;
            img_pad4 = img_pad + (y_up - i) * stride;
            y_up = COM_CLIP3(start_pos, end_pos - 1, i - 3);
            y_bottom = COM_CLIP3(start_pos, end_pos - 1, i + 3);
            img_pad5 = img_pad + (y_bottom - i) * stride;
            img_pad6 = img_pad + (y_up - i) * stride;
#endif
            for (j = x_pos; j < x_pos_end; j++)
            {
                pixel_int = coef[0] * (img_pad5[j] + img_pad6[j]);
                pixel_int += coef[3] * (img_pad3[j] + img_pad4[j]);
                pixel_int += coef[8] * (img_pad1[j] + img_pad2[j]);
#if ALF_DEC_OPT
                if (i >= y_pos + 3 && i <= end_pos - 4)
                {
                    x_left = COM_MAX(x_pos + x_offset_left, j - 1);
                    x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 1);
                    pixel_int += coef[4] * (img_pad4[x_right] + img_pad3[x_left]);
                    pixel_int += coef[2] * (img_pad4[x_left] + img_pad3[x_right]);
                    pixel_int += coef[7] * (img_pad2[x_left] + img_pad1[x_right]);
                    pixel_int += coef[9] * (img_pad2[x_right] + img_pad1[x_left]);
                    pixel_int += coef[13] * (img_pad[x_right] + img_pad[x_left]);
                    x_left = COM_MAX(x_pos + x_offset_left, j - 2);
                    x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 2);
                    pixel_int += coef[5] * (img_pad4[x_right] + img_pad3[x_left]);
                    pixel_int += coef[1] * (img_pad4[x_left] + img_pad3[x_right]);
                    pixel_int += coef[6] * (img_pad2[x_left] + img_pad1[x_right]);
                    pixel_int += coef[10] * (img_pad2[x_right] + img_pad1[x_left]);
                    pixel_int += coef[12] * (img_pad[x_right] + img_pad[x_left]);
                    x_left = COM_MAX(x_pos + x_offset_left, j - 3);
                    x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 3);
                    pixel_int += coef[11] * (img_pad[x_right] + img_pad[x_left]);
                }
                else
                {
#endif
                // upper left c2
                x_left = check_filtering_unit_boundary_extension(j - 1, i - 1, x_pos, y_pos, x_pos, start_pos, x_pos_end - 1, end_pos - 1,
                    is_above_left_avail, is_left_avail, isAboveRightAvail, is_right_avail);
                pixel_int += coef[7] * img_pad2[x_left];
                // upper right c4
                x_right = check_filtering_unit_boundary_extension(j + 1, i - 1, x_pos, y_pos, x_pos, start_pos, x_pos_end - 1, end_pos - 1,
                    is_above_left_avail, is_left_avail, isAboveRightAvail, is_right_avail);
                pixel_int += coef[9] * img_pad2[x_right];
                // lower left c4
                x_left = check_filtering_unit_boundary_extension(j - 1, i + 1, x_pos, y_pos, x_pos, start_pos, x_pos_end - 1, end_pos - 1,
                    is_above_left_avail, is_left_avail, isAboveRightAvail, is_right_avail);
                pixel_int += coef[9] * img_pad1[x_left];
                // lower right c2
                x_right = check_filtering_unit_boundary_extension(j + 1, i + 1, x_pos, y_pos, x_pos, start_pos, x_pos_end - 1, end_pos - 1,
                    is_above_left_avail, is_left_avail, isAboveRightAvail, is_right_avail);
                pixel_int += coef[7] * img_pad1[x_right];

#if ALF_DEC_OPT
                x_left = COM_MAX(x_pos + x_offset_left, j - 1);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 1);
#else
                x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 1);
                x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 1);
#endif
                pixel_int += coef[4] * (img_pad3[x_left] + img_pad4[x_right]);
                pixel_int += coef[2] * (img_pad3[x_right] + img_pad4[x_left]);
                pixel_int += coef[13] * (img_pad[x_right] + img_pad[x_left]);
#if ALF_DEC_OPT
                x_left = COM_MAX(x_pos + x_offset_left, j - 2);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 2);
#else
                x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 2);
                x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 2);
#endif
                pixel_int += coef[6] * (img_pad1[x_right] + img_pad2[x_left]);
                pixel_int += coef[10] * (img_pad1[x_left] + img_pad2[x_right]);
                pixel_int += coef[5] * (img_pad3[x_left] + img_pad4[x_right]);
                pixel_int += coef[1] * (img_pad3[x_right] + img_pad4[x_left]);
                pixel_int += coef[12] * (img_pad[x_right] + img_pad[x_left]);
#if ALF_DEC_OPT
                x_left = COM_MAX(x_pos + x_offset_left, j - 3);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 3);
#else
                x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 3);
                x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 3);
#endif
                pixel_int += coef[11] * (img_pad[x_right] + img_pad[x_left]);
#if ALF_DEC_OPT
                }
#endif
                pixel_int += coef[14] * (img_pad[j]);
                pixel_int = (int)((pixel_int + offset) >> ALF_NUM_BIT_SHIFT);
                img_res[j] = (pel)COM_CLIP3(0, ((1 << sample_bit_depth) - 1), pixel_int);
            }
            img_pad += stride;
            img_res += stride;
        }

    }
#endif
}

void check_filter_coeff_value(int *filter, int filter_length)
{
    int i;
    int max_value_non_center;
    int min_value_non_center;
    int max_value_center;
    int min_value_center;
    max_value_non_center = 1 * (1 << ALF_NUM_BIT_SHIFT) - 1;
    min_value_non_center = 0 - 1 * (1 << ALF_NUM_BIT_SHIFT);
    max_value_center = 2 * (1 << ALF_NUM_BIT_SHIFT) - 1;
    min_value_center = 0;
    for (i = 0; i < filter_length - 1; i++)
    {
        filter[i] = COM_CLIP3(min_value_non_center, max_value_non_center, filter[i]);
    }
    filter[filter_length - 1] = COM_CLIP3(min_value_center, max_value_center, filter[filter_length - 1]);
}

void copy_alf_param(ALF_PARAM *dst, ALF_PARAM *src
#if ALF_SHAPE
                  , int num_coef
#endif
)
{
    int j;
    dst->alf_flag = src->alf_flag;
    dst->component_id = src->component_id;
    dst->filters_per_group = src->filters_per_group;
    dst->num_coeff = src->num_coeff;
    switch (src->component_id)
    {
    case Y_C:
        for (j = 0; j < NO_VAR_BINS; j++)
        {
#if ALF_SHAPE
            memcpy(dst->coeff_multi[j], src->coeff_multi[j], num_coef * sizeof(int));
#else
            memcpy(dst->coeff_multi[j], src->coeff_multi[j], ALF_MAX_NUM_COEF * sizeof(int));
#endif
        }
        memcpy(dst->filter_pattern, src->filter_pattern, NO_VAR_BINS * sizeof(int));
        break;
    case U_C:
    case V_C:
        for (j = 0; j < 1; j++)
        {
#if ALF_SHAPE
            memcpy(dst->coeff_multi[j], src->coeff_multi[j], num_coef * sizeof(int));
#else
            memcpy(dst->coeff_multi[j], src->coeff_multi[j], ALF_MAX_NUM_COEF * sizeof(int));
#endif
        }
        break;
    default:
        printf("Not a legal component ID\n");
        assert(0);
        exit(-1);
    }
}

int get_lcu_ctrl_ctx_idx(int ctu, int num_lcu_in_pic_width, int num_lcu_in_pic_height, int comp_idx, BOOL **alf_lcu_enabled)
{
    int row, col, a, b;
    row = ctu / num_lcu_in_pic_width;
    col = ctu % num_lcu_in_pic_width;
    if (row == 0)
    {
        b = 0;
    }
    else
    {
        b = (int)alf_lcu_enabled[ctu - num_lcu_in_pic_width][comp_idx];
    }
    if (col == 0)
    {
        a = 0;
    }
    else
    {
        a = (int)alf_lcu_enabled[ctu - 1][comp_idx];
    }
    return (a + 2 * b);
}

void set_filter_image(pel *dec_Y, pel *dec_U, pel *dec_V, int in_stride, pel *img_Y_out, pel * img_U_out, pel * img_V_out, int out_stride, int img_width, int img_height)
{
    int j, i, width_cr, in_stride_cr, out_stride_cr, height_cr;
    pel *src;
    src = dec_Y;
    for (j = 0; j < img_height; j++)
    {
        for (i = 0; i < img_width; i++)
        {
            img_Y_out[i] = src[i];
        }
        img_Y_out += out_stride;
        src += in_stride;
    }
    width_cr = img_width >> 1;
    height_cr = img_height >> 1;
    in_stride_cr = in_stride >> 1;
    out_stride_cr = out_stride >> 1;
    src = dec_U;
    for (j = 0; j < height_cr; j++)
    {
        for (i = 0; i < width_cr; i++)
        {
            img_U_out[i] = src[i];
        }
        img_V_out += out_stride_cr;
        src += in_stride_cr;
    }
    src = dec_V;
    for (j = 0; j < height_cr; j++)
    {
        for (i = 0; i < width_cr; i++)
        {
            img_V_out[i] = src[i];
        }
        img_V_out += out_stride_cr;
        src += in_stride_cr;
    }
}

int get_mem_1D_int(int **array1D, int num)
{
    if ((*array1D = (int *)calloc(num, sizeof(int))) == NULL)
    {
        assert(0);
    }
    return num * sizeof(int);
}

int get_mem_2D_int(int ***array2D, int rows, int columns)
{
    int i;
    if ((*array2D = (int **)calloc(rows, sizeof(int *))) == NULL)
    {
        assert(0);
    }
    if (((*array2D)[0] = (int *)calloc(rows * columns, sizeof(int))) == NULL)
    {
        assert(0);
    }
    for (i = 1; i < rows; i++)
    {
        (*array2D)[i] = (*array2D)[i - 1] + columns;
    }
    return rows * columns * sizeof(int);
}

void free_mem_1D_int(int *array1D)
{
    if (array1D)
    {
        free(array1D);
    }
    else
    {
        assert(0);
    }
}

void free_mem_2D_int(int **array2D)
{
    if (array2D)
    {
        if (array2D[0])
        {
            free(array2D[0]);
            array2D[0] = NULL;
        }
        else
        {
            assert(0);
        }
        free(array2D);
    }
    else
    {
        assert(0);
    }
}


void copy_frame_for_alf(COM_PIC *pic_dst, COM_PIC *pic_src)
{
    int i, j;
    int src_stride, dst_stride;
    pel* src;
    pel* dst;
    src_stride = pic_src->stride_luma;
    dst_stride = pic_dst->stride_luma;
    src = pic_src->y;
    dst = pic_dst->y;
    for (j = 0; j < pic_src->height_luma; j++)
    {
        for (i = 0; i < pic_src->width_luma; i++)
        {
            dst[i] = src[i];
        }
        dst += dst_stride;
        src += src_stride;
    }
    src_stride = pic_src->stride_chroma;
    dst_stride = pic_dst->stride_chroma;
    src = pic_src->u;
    dst = pic_dst->u;
    for (j = 0; j < pic_src->height_chroma; j++)
    {
        for (i = 0; i < pic_src->width_chroma; i++)
        {
            dst[i] = src[i];
        }
        dst += dst_stride;
        src += src_stride;
    }
    src = pic_src->v;
    dst = pic_dst->v;
    for (j = 0; j < pic_src->height_chroma; j++)
    {
        for (i = 0; i < pic_src->width_chroma; i++)
        {
            dst[i] = src[i];
        }
        dst += dst_stride;
        src += src_stride;
    }
}
