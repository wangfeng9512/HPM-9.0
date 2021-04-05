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

#include "com_sao.h"

void init_stat_data(SAO_STAT_DATA *stat_data)
{
    int i;
    for (i = 0; i < MAX_NUM_SAO_CLASSES; i++)
    {
        stat_data->diff[i] = 0;
        stat_data->count[i] = 0;
    }
}


long long int get_distortion(int compIdx, int type, SAO_STAT_DATA **sao_stat_data, SAO_BLK_PARAM *sao_cur_param)
{
    int class_idx, band_idx;
    long long int dist = 0;
    switch (type)
    {
    case SAO_TYPE_EO_0:
    case SAO_TYPE_EO_90:
    case SAO_TYPE_EO_135:
    case SAO_TYPE_EO_45:
    {
        for (class_idx = 0; class_idx < NUM_SAO_EO_CLASSES; class_idx++)
        {
            dist += distortion_cal(sao_stat_data[compIdx][type].count[class_idx], sao_cur_param[compIdx].offset[class_idx],
                                   sao_stat_data[compIdx][type].diff[class_idx]);
        }
    }
    break;
    case SAO_TYPE_BO:
    {
        for (class_idx = 0; class_idx < NUM_BO_OFFSET; class_idx++)
        {
            band_idx = class_idx % NUM_SAO_BO_CLASSES;
            dist += distortion_cal(sao_stat_data[compIdx][type].count[band_idx], sao_cur_param[compIdx].offset[band_idx],
                                   sao_stat_data[compIdx][type].diff[band_idx]);
        }
    }
    break;
    default:
    {
        printf("Not a supported type");
        assert(0);
        exit(-1);
    }
    }
    return dist;
}

long long int distortion_cal(long long int count, int offset, long long int diff)
{
    return (count * (long long int)offset * (long long int)offset - diff * offset * 2);
}

void off_sao(SAO_BLK_PARAM *sao_blk_param)
{
    int i;
    for (i = 0; i < N_C; i++)
    {
        sao_blk_param[i].mode_idc = SAO_MODE_OFF;
        sao_blk_param[i].type_idc = -1;
        sao_blk_param[i].start_band = -1;
        sao_blk_param[i].start_band2 = -1;
        sao_blk_param[i].delta_band = -1;
        memset(sao_blk_param[i].offset, 0, sizeof(int)*MAX_NUM_SAO_CLASSES);
    }
}
void copy_sao_param_for_blk(SAO_BLK_PARAM *sao_param_dst, SAO_BLK_PARAM *sao_param_src)
{
    int i, j;
    for (i = 0; i < N_C; i++)
    {
        sao_param_dst[i].mode_idc = sao_param_src[i].mode_idc;
        sao_param_dst[i].type_idc = sao_param_src[i].type_idc;
        sao_param_dst[i].start_band = sao_param_src[i].start_band;
        sao_param_dst[i].start_band2 = sao_param_src[i].start_band2;
        sao_param_dst[i].delta_band = sao_param_src[i].delta_band;
        for (j = 0; j < MAX_NUM_SAO_CLASSES; j++)
        {
            sao_param_dst[i].offset[j] = sao_param_src[i].offset[j];
        }
    }
}
void copy_sao_param_for_blk_one_component(SAO_BLK_PARAM *saopara_dst, SAO_BLK_PARAM *saopara_src)
{
    int  j;
    saopara_dst->mode_idc = saopara_src->mode_idc;
    saopara_dst->type_idc = saopara_src->type_idc;
    saopara_dst->start_band = saopara_src->start_band;
    saopara_dst->start_band2 = saopara_src->start_band2;
    saopara_dst->delta_band = saopara_src->delta_band;
    for (j = 0; j < MAX_NUM_SAO_CLASSES; j++)
    {
        saopara_dst->offset[j] = saopara_src->offset[j];
    }
}

void copy_ctu_for_sao(COM_PIC * pic_dst, COM_PIC * pic_src, int pix_y, int pix_x, int lcu_pix_height, int lcu_pix_width)
{
    int j;
    pel* src;
    pel* dst;
    int src_stride, dst_stride;
    int src_offset, dst_offset;
    src_stride = pic_src->stride_luma;
    dst_stride = pic_dst->stride_luma;
    src_offset = pix_y*src_stride + pix_x;
    dst_offset = pix_y*dst_stride + pix_x;
    src = pic_src->y + src_offset;
    dst = pic_dst->y + dst_offset;
    for (j = 0; j < lcu_pix_height; j++)
    {
        for (int i = 0; i < lcu_pix_width; i++)
        {
            dst[i] = src[i];
        }
        dst += dst_stride;
        src += src_stride;
    }
    src_stride = pic_src->stride_chroma;
    dst_stride = pic_dst->stride_chroma;
    src_offset = (pix_y >> 1)*src_stride + (pix_x >> 1);
    dst_offset = (pix_y >> 1)*dst_stride + (pix_x >> 1);
    src = pic_src->u + src_offset;
    dst = pic_dst->u + dst_offset;
    for (j = 0; j < (lcu_pix_height >> 1); j++)
    {
        for (int i = 0; i < (lcu_pix_width >> 1); i++)
        {
            dst[i] = src[i];
        }
        dst += dst_stride;
        src += src_stride;
    }
    src = pic_src->v + src_offset;
    dst = pic_dst->v + dst_offset;
    for (j = 0; j < (lcu_pix_height >> 1); j++)
    {
        for (int i = 0; i < (lcu_pix_width >> 1); i++)
        {
            dst[i] = src[i];
        }
        dst += dst_stride;
        src += src_stride;
    }
}

void copy_frame_for_sao(COM_PIC * pic_dst, COM_PIC * pic_src)
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
    //assert(pic_src->stride_luma == pic_dst->stride_luma);
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

BOOL is_same_patch(s8* map_patch_idx, int mb_nr1, int mb_nr2)
{
    assert(mb_nr1 >= 0);
    assert(mb_nr2 >= 0);
    return (map_patch_idx[mb_nr1] == map_patch_idx[mb_nr2]);
}

void get_sao_merge_neighbor(COM_INFO *info, s8* map_patch_idx, int pic_width_scu, int pic_width_lcu, int lcu_pos, int mb_y, int mb_x,
                         SAO_BLK_PARAM **rec_sao_blk_param, int *merge_avail,
                         SAO_BLK_PARAM sao_merge_param[][N_C])
{
    int mb_nr;
    int merge_up_avail, merge_left_avail;
    SAO_BLK_PARAM *sao_left_param;
    SAO_BLK_PARAM *sao_up_param;
    mb_nr = mb_y * pic_width_scu + mb_x;
    merge_up_avail   = (mb_y == 0) ? 0 : is_same_patch(map_patch_idx, mb_nr, mb_nr - pic_width_scu) ? 1 : 0;
    merge_left_avail = (mb_x == 0) ? 0 : is_same_patch(map_patch_idx, mb_nr, mb_nr - 1) ? 1 : 0;
    if (merge_left_avail)
    {
        sao_left_param = rec_sao_blk_param[lcu_pos - 1];
        copy_sao_param_for_blk(sao_merge_param[SAO_MERGE_LEFT], sao_left_param);
    }
    if (merge_up_avail)
    {
        sao_up_param = rec_sao_blk_param[lcu_pos - pic_width_lcu];
        copy_sao_param_for_blk(sao_merge_param[SAO_MERGE_ABOVE], sao_up_param);
    }
    merge_avail[SAO_MERGE_LEFT] = merge_left_avail;
    merge_avail[SAO_MERGE_ABOVE] = merge_up_avail;
}

void check_boundary_param(COM_INFO *info, COM_MAP *map, int mb_y, int mb_x, int lcu_pix_height, int lcu_pix_width,
                       int *lcu_process_left, int *lcu_process_right, int *lcu_process_up, int *lcu_process_down, int *lcu_process_upleft,
                       int *lcu_process_upright, int *lcu_process_leftdown, int *lcu_process_rightdown, int filter_on)
{
    int pic_width_scu = info->pic_width_in_scu;
    s8* map_patch_idx = map->map_patch_idx;
    int mb_nr = mb_y * pic_width_scu + mb_x;
    int lcu_mb_width = lcu_pix_width >> MIN_CU_LOG2;
    int lcu_mb_height = lcu_pix_height >> MIN_CU_LOG2;
    int pic_mb_height = info->pic_height >> MIN_CU_LOG2;
    int pic_mb_width = info->pic_width >> MIN_CU_LOG2;

    *lcu_process_up = (mb_y == 0) ? 0 : 1;
    *lcu_process_down = (mb_y >= pic_mb_height - lcu_mb_height) ? 0 : 1;
    *lcu_process_left = (mb_x == 0) ? 0 : 1;
    *lcu_process_right = (mb_x >= pic_mb_width - lcu_mb_width) ? 0 : 1;
    *lcu_process_upleft = (mb_x == 0 || mb_y == 0) ? 0 : 1;
    *lcu_process_upright = (mb_x >= pic_mb_width - lcu_mb_width || mb_y == 0) ? 0 : 1;
    *lcu_process_leftdown = (mb_x == 0 || mb_y >= pic_mb_height - lcu_mb_height) ? 0 : 1;
    *lcu_process_rightdown = (mb_x >= pic_mb_width - lcu_mb_width || mb_y >= pic_mb_height - lcu_mb_height) ? 0 : 1;
    if (!filter_on)
    {
        *lcu_process_down = 0;
        *lcu_process_right = 0;
        *lcu_process_leftdown = 0;
        *lcu_process_rightdown = 0;
    }
}

void check_boundary_proc(COM_INFO *info, COM_MAP *map, int pix_y, int pix_x, int lcu_pix_height, int lcu_pix_width, int comp,
                       int *lcu_process_left, int *lcu_process_right, int *lcu_process_up, int *lcu_process_down,
                       int *lcu_process_upleft, int *lcu_process_upright, int *lcu_process_leftdown, int *lcu_process_rightdown, int filter_on)
{
    int pic_width = comp ? (info->pic_width >> 1) : info->pic_width;
    int pic_height = comp ? (info->pic_height >> 1) : info->pic_height;
    int pic_mb_width = info->pic_width_in_scu;
    int mb_size_in_bit = comp ? (MIN_CU_LOG2 - 1) : MIN_CU_LOG2;
    int mb_nr_cur, mb_nr_up, mb_nr_down, mb_nr_left, mb_nr_right, mb_nr_upleft, mb_nr_upright, mb_nr_leftdown,
        mb_nr_rightdown;
    s8 *map_patch_idx = map->map_patch_idx;
    int cross_patch_flag = info->sqh.cross_patch_loop_filter;

    mb_nr_cur = (pix_y >> mb_size_in_bit) * pic_mb_width + (pix_x >> mb_size_in_bit);
    mb_nr_up = ((pix_y - 1) >> mb_size_in_bit) * pic_mb_width + (pix_x >> mb_size_in_bit);
    mb_nr_down = ((pix_y + lcu_pix_height) >> mb_size_in_bit) * pic_mb_width + (pix_x >> mb_size_in_bit);
    mb_nr_left = (pix_y >> mb_size_in_bit) * pic_mb_width + ((pix_x - 1) >> mb_size_in_bit);
    mb_nr_right = (pix_y >> mb_size_in_bit) * pic_mb_width + ((pix_x + lcu_pix_width) >> mb_size_in_bit);
    mb_nr_upleft = ((pix_y - 1) >> mb_size_in_bit) * pic_mb_width + ((pix_x - 1) >> mb_size_in_bit);
    mb_nr_upright = ((pix_y - 1) >> mb_size_in_bit) * pic_mb_width + ((pix_x + lcu_pix_width) >> mb_size_in_bit);
    mb_nr_leftdown = ((pix_y + lcu_pix_height) >> mb_size_in_bit) * pic_mb_width + ((pix_x - 1) >> mb_size_in_bit);
    mb_nr_rightdown = ((pix_y + lcu_pix_height) >> mb_size_in_bit) * pic_mb_width + ((pix_x + lcu_pix_width) >>
                      mb_size_in_bit);
    *lcu_process_up = (pix_y == 0) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_up) ? 1 :
                      cross_patch_flag;
    *lcu_process_down = (pix_y >= pic_height - lcu_pix_height) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_down) ? 1 : cross_patch_flag;
    *lcu_process_left = (pix_x == 0) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_left) ? 1 :
                        cross_patch_flag;
    *lcu_process_right = (pix_x >= pic_width - lcu_pix_width) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_right) ? 1 : cross_patch_flag;
    *lcu_process_upleft = (pix_x == 0 ||
                           pix_y == 0) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_upleft) ? 1 : cross_patch_flag;
    *lcu_process_upright = (pix_x >= pic_width - lcu_pix_width ||
                            pix_y == 0) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_upright) ? 1 : cross_patch_flag;
    *lcu_process_leftdown = (pix_x == 0 ||
                             pix_y >= pic_height - lcu_pix_height) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_leftdown)
                            ? 1 : cross_patch_flag;
    *lcu_process_rightdown = (pix_x >= pic_width - lcu_pix_width ||
                              pix_y >= pic_height - lcu_pix_height) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_rightdown)
                             ? 1 : cross_patch_flag;
    if (!filter_on)
    {
        *lcu_process_down = 0;
        *lcu_process_right = 0;
        *lcu_process_leftdown = 0;
        *lcu_process_rightdown = 0;
    }
}

void SAO_on_block(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_rec, COM_PIC  *pic_sao, SAO_BLK_PARAM *sao_blk_param, int comp_idx, int pix_y, int pix_x, int lcu_pix_height,
                  int lcu_pix_width, int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down,
                  int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdown,
                  int sample_bit_depth)
{
    int type;
    int start_x, end_x, start_y, end_y;
    int start_x_r0, end_x_r0, start_x_r, end_x_r, start_x_rn, end_x_rn;
    int x, y;
    pel *src, *dst;
    char left_sign, right_sign, up_sign, down_sign;
    long diff;
    char *sign_up_line, *sign_up_line1;
    int reg = 0;
    int edge_type, band_type;
    int offset0, offset1, offset2;
    int src_stride, dst_stride;
    src = dst = NULL;
    offset0 = offset1 = offset2 = 0;

    if ((lcu_pix_height <= 0) || (lcu_pix_width <= 0))
    {
        return;
    }

    switch (comp_idx)
    {
    case Y_C:
        src_stride = pic_sao->stride_luma;
        dst_stride = pic_rec->stride_luma;
        src = pic_sao->y;
        dst = pic_rec->y;
        break;
    case U_C:
        src_stride = pic_sao->stride_chroma;
        dst_stride = pic_rec->stride_chroma;
        src = pic_sao->u;
        dst = pic_rec->u;
        break;
    case V_C:
        src_stride = pic_sao->stride_chroma;
        dst_stride = pic_rec->stride_chroma;
        src = pic_sao->v;
        dst = pic_rec->v;
        break;
    default:
        src_stride = 0;
        dst_stride = 0;
        src = NULL;
        dst = NULL;
        assert(0);
    }
    sign_up_line = (char *)malloc((lcu_pix_width + 1) * sizeof(char));
    assert(sao_blk_param->mode_idc == SAO_MODE_NEW);
    type = sao_blk_param->type_idc;
    switch (type)
    {
    case SAO_TYPE_EO_0:
    {
        start_y = 0;
        end_y = lcu_pix_height;
        start_x = lcu_available_left ? 0 : 1;
        end_x = lcu_available_right ? lcu_pix_width : (lcu_pix_width - 1);
        for (y = start_y; y < end_y; y++)
        {
            diff = src[(pix_y + y) * src_stride + pix_x + start_x] - src[(pix_y + y) * src_stride + pix_x + start_x - 1];
            left_sign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
            for (x = start_x; x < end_x; x++)
            {
                diff = src[(pix_y + y) * src_stride + pix_x + x] - src[(pix_y + y) * src_stride + pix_x + x + 1];
                right_sign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                edge_type = left_sign + right_sign;
                left_sign = -right_sign;
                dst[(pix_y + y)*dst_stride + pix_x + x] = (pel)COM_CLIP3(0, ((1 << sample_bit_depth) - 1),
                        src[(pix_y + y) * src_stride + pix_x + x] + sao_blk_param->offset[edge_type + 2]);
            }
        }
    }
    break;
    case SAO_TYPE_EO_90:
    {
        start_x = 0;
        end_x = lcu_pix_width;
        start_y = lcu_available_up ? 0 : 1;
        end_y = lcu_available_down ? lcu_pix_height : (lcu_pix_height - 1);
        for (x = start_x; x < end_x; x++)
        {
            diff = src[(pix_y + start_y) * src_stride + pix_x + x] - src[(pix_y + start_y - 1) * src_stride + pix_x + x];
            up_sign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
            for (y = start_y; y < end_y; y++)
            {
                diff = src[(pix_y + y) * src_stride + pix_x + x] - src[(pix_y + y + 1) * src_stride + pix_x + x];
                down_sign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                edge_type = down_sign + up_sign;
                up_sign = -down_sign;
                dst[(pix_y + y)*dst_stride + pix_x + x] = (pel)COM_CLIP3(0, ((1 << sample_bit_depth) - 1),
                        src[(pix_y + y) * src_stride + pix_x + x] + sao_blk_param->offset[edge_type + 2]);
            }
        }
    }
    break;
    case SAO_TYPE_EO_135:
    {
        start_x_r0 = lcu_available_upleft ? 0 : 1;
        end_x_r0 = lcu_available_up ? (lcu_available_right ? lcu_pix_width : (lcu_pix_width - 1)) : 1;
        start_x_r = lcu_available_left ? 0 : 1;
        end_x_r = lcu_available_right ? lcu_pix_width : (lcu_pix_width - 1);
        start_x_rn = lcu_available_down ? (lcu_available_left ? 0 : 1) : (lcu_pix_width - 1);
        end_x_rn = lcu_available_rightdown ? lcu_pix_width : (lcu_pix_width - 1);
        //init the line buffer
        for (x = start_x_r; x < end_x_r + 1; x++)
        {
            diff = src[(pix_y + 1) * src_stride + pix_x + x] - src[pix_y * src_stride + pix_x + x - 1];
            up_sign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
            sign_up_line[x] = up_sign;
        }
        //first row
        for (x = start_x_r0; x < end_x_r0; x++)
        {
            diff = src[pix_y * src_stride + pix_x + x] - src[(pix_y - 1) * src_stride + pix_x + x - 1];
            up_sign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
            edge_type = up_sign - sign_up_line[x + 1];
            dst[pix_y * dst_stride + pix_x + x] = (pel)COM_CLIP3(0, ((1 << sample_bit_depth) - 1),
                                                 src[pix_y * src_stride + pix_x + x] + sao_blk_param->offset[edge_type + 2]);
        }
        //middle rows
        for (y = 1; y < lcu_pix_height - 1; y++)
        {
            for (x = start_x_r; x < end_x_r; x++)
            {
                if (x == start_x_r)
                {
                    diff = src[(pix_y + y) * src_stride + pix_x + x] - src[(pix_y + y - 1) * src_stride + pix_x + x - 1];
                    up_sign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                    sign_up_line[x] = up_sign;
                }
                diff = src[(pix_y + y) * src_stride + pix_x + x] - src[(pix_y + y + 1) * src_stride + pix_x + x + 1];
                down_sign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                edge_type = down_sign + sign_up_line[x];
                dst[(pix_y + y)*dst_stride + pix_x + x] = (pel)COM_CLIP3(0, ((1 << sample_bit_depth) - 1),
                        src[(pix_y + y) * src_stride + pix_x + x] + sao_blk_param->offset[edge_type + 2]);
                sign_up_line[x] = (char)reg;
                reg = -down_sign;
            }
        }
        //last row
        for (x = start_x_rn; x < end_x_rn; x++)
        {
            if (x == start_x_r)
            {
                diff = src[(pix_y + lcu_pix_height - 1) * src_stride + pix_x + x] - src[(pix_y + lcu_pix_height - 2) * src_stride + pix_x
                        + x - 1];
                up_sign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                sign_up_line[x] = up_sign;
            }
            diff = src[(pix_y + lcu_pix_height - 1) * src_stride + pix_x + x] - src[(pix_y + lcu_pix_height) * src_stride + pix_x + x
                    + 1];
            down_sign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
            edge_type = down_sign + sign_up_line[x];
            dst[(pix_y + lcu_pix_height - 1)*dst_stride + pix_x + x] = (pel)COM_CLIP3(0, ((1 << sample_bit_depth) - 1),
                    src[(pix_y + lcu_pix_height - 1) * src_stride + pix_x + x] + sao_blk_param->offset[edge_type + 2]);
        }
    }
    break;
    case SAO_TYPE_EO_45:
    {
        start_x_r0 = lcu_available_up ? (lcu_available_left ? 0 : 1) : (lcu_pix_width - 1);
        end_x_r0 = lcu_available_upright ? lcu_pix_width : (lcu_pix_width - 1);
        start_x_r = lcu_available_left ? 0 : 1;
        end_x_r = lcu_available_right ? lcu_pix_width : (lcu_pix_width - 1);
        start_x_rn = lcu_available_leftdown ? 0 : 1;
        end_x_rn = lcu_available_down ? (lcu_available_right ? lcu_pix_width : (lcu_pix_width - 1)) : 1;
        //init the line buffer
        sign_up_line1 = sign_up_line + 1;
        for (x = start_x_r - 1; x < end_x_r; x++)
        {
            diff = src[(pix_y + 1) * src_stride + pix_x + x] - src[pix_y * src_stride + pix_x + x + 1];
            up_sign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
            sign_up_line1[x] = up_sign;
        }
        //first row
        for (x = start_x_r0; x < end_x_r0; x++)
        {
            diff = src[pix_y * src_stride + pix_x + x] - src[(pix_y - 1) * src_stride + pix_x + x + 1];
            up_sign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
            edge_type = up_sign - sign_up_line1[x - 1];
            dst[pix_y * dst_stride + pix_x + x] = (pel)COM_CLIP3(0, ((1 << sample_bit_depth) - 1),
                                                 src[pix_y * src_stride + pix_x + x] + sao_blk_param->offset[edge_type + 2]);
        }
        //middle rows
        for (y = 1; y < lcu_pix_height - 1; y++)
        {
            for (x = start_x_r; x < end_x_r; x++)
            {
                if (x == end_x_r - 1)
                {
                    diff = src[(pix_y + y) * src_stride + pix_x + x] - src[(pix_y + y - 1) * src_stride + pix_x + x + 1];
                    up_sign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                    sign_up_line1[x] = up_sign;
                }
                diff = src[(pix_y + y) * src_stride + pix_x + x] - src[(pix_y + y + 1) * src_stride + pix_x + x - 1];
                down_sign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                edge_type = down_sign + sign_up_line1[x];
                dst[(pix_y + y)*dst_stride + pix_x + x] = (pel)COM_CLIP3(0, ((1 << sample_bit_depth) - 1),
                        src[(pix_y + y) * src_stride + pix_x + x] + sao_blk_param->offset[edge_type + 2]);
                sign_up_line1[x - 1] = -down_sign;
            }
        }
        for (x = start_x_rn; x < end_x_rn; x++)
        {
            if (x == end_x_r - 1)
            {
                diff = src[(pix_y + lcu_pix_height - 1) * src_stride + pix_x + x] - src[(pix_y + lcu_pix_height - 2) * src_stride + pix_x
                        + x + 1];
                up_sign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
                sign_up_line1[x] = up_sign;
            }
            diff = src[(pix_y + lcu_pix_height - 1) * src_stride + pix_x + x] - src[(pix_y + lcu_pix_height) * src_stride + pix_x + x
                    - 1];
            down_sign = diff > 0 ? 1 : (diff < 0 ? -1 : 0);
            edge_type = down_sign + sign_up_line1[x];
            dst[(pix_y + lcu_pix_height - 1)*dst_stride + pix_x + x] = (pel)COM_CLIP3(0, ((1 << sample_bit_depth) - 1),
                    src[(pix_y + lcu_pix_height - 1) * src_stride + pix_x + x] + sao_blk_param->offset[edge_type + 2]);
        }
    }
    break;
    case SAO_TYPE_BO:
    {
        start_x = 0;
        end_x = lcu_pix_width;
        start_y = 0;
        end_y = lcu_pix_height;
        for (x = start_x; x < end_x; x++)
        {
            for (y = start_y; y < end_y; y++)
            {
                band_type = src[(pix_y + y) * src_stride + pix_x + x] >> (sample_bit_depth - NUM_SAO_BO_CLASSES_IN_BIT);
                dst[(pix_y + y)*dst_stride + pix_x + x] = (pel)COM_CLIP3(0, ((1 << sample_bit_depth) - 1),
                        src[(pix_y + y) * src_stride + pix_x + x] + sao_blk_param->offset[band_type]);
            }
        }
    }
    break;
    default:
    {
        printf("Not a supported SAO types\n");
        assert(0);
        exit(-1);
    }
    }
    free(sign_up_line);
}

void SAO_on_smb(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_rec, COM_PIC  *pic_sao, int pix_y, int pix_x, int lcu_pix_width, int lcu_pix_height,
                SAO_BLK_PARAM *sao_blk_param, int sample_bit_depth)
{
    int comp_idx;
    int mb_y = pix_y >> MIN_CU_LOG2;
    int mb_x = pix_x >> MIN_CU_LOG2;
    int is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail, is_below_right_avail;
    int lcu_pix_height_t, lcu_pix_width_t, pix_x_t, pix_y_t;
    int is_left_proc, is_right_proc, is_above_proc, is_below_proc, is_above_left_proc, is_above_right_proc, is_below_left_proc, is_below_right_proc;
    check_boundary_param(info, map, mb_y, mb_x, lcu_pix_height, lcu_pix_width, &is_left_avail, &is_right_avail,
                      &is_above_avail, &is_below_avail, &is_above_left_avail, &is_above_right_avail, &is_below_left_avail, &is_below_right_avail, 1);

    if ((sao_blk_param[Y_C].mode_idc == SAO_MODE_OFF) && (sao_blk_param[U_C].mode_idc == SAO_MODE_OFF) &&
            (sao_blk_param[V_C].mode_idc == SAO_MODE_OFF))
    {
        return;
    }
    for (comp_idx = 0; comp_idx < N_C; comp_idx++)
    {
        if (sao_blk_param[comp_idx].mode_idc != SAO_MODE_OFF)
        {
            /**
            * HSUAN: AREA 5
            */
            lcu_pix_width_t = comp_idx ? ((lcu_pix_width >> 1) - SAO_SHIFT_PIX_NUM) : (lcu_pix_width - SAO_SHIFT_PIX_NUM);
            lcu_pix_height_t = comp_idx ? ((lcu_pix_height >> 1) - SAO_SHIFT_PIX_NUM) : (lcu_pix_height - SAO_SHIFT_PIX_NUM);
            pix_x_t = comp_idx ? (pix_x >> 1) : pix_x;
            pix_y_t = comp_idx ? (pix_y >> 1) : pix_y;
            check_boundary_proc(info, map,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, comp_idx, &is_left_proc,
                              &is_right_proc, &is_above_proc, &is_below_proc, &is_above_left_proc, &is_above_right_proc, &is_below_left_proc, &is_below_right_proc, 1);

            //it's supposed that chroma has the same result as luma!!!
            SAO_on_block(info, map, pic_rec, pic_sao, &(sao_blk_param[comp_idx]), comp_idx,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t,
                         is_left_proc /*Left*/, is_right_proc/*Right*/, is_above_proc/*Above*/, is_below_proc/*Below*/, is_above_left_proc/*AboveLeft*/,
                         is_above_right_proc/*AboveRight*/, is_below_left_proc/*BelowLeft*/, is_below_right_proc/*BelowRight*/, sample_bit_depth);
            if (is_above_left_avail)
            {
                /**
                * HSUAN: AREA 1
                */
                lcu_pix_width_t = SAO_SHIFT_PIX_NUM;
                lcu_pix_height_t = SAO_SHIFT_PIX_NUM;
                pix_x_t = comp_idx ? ((pix_x >> 1) - SAO_SHIFT_PIX_NUM) : (pix_x - SAO_SHIFT_PIX_NUM);
                pix_y_t = comp_idx ? ((pix_y >> 1) - SAO_SHIFT_PIX_NUM) : (pix_y - SAO_SHIFT_PIX_NUM);
                check_boundary_proc(info, map,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, comp_idx, &is_left_proc,
                                  &is_right_proc, &is_above_proc, &is_below_proc, &is_above_left_proc, &is_above_right_proc, &is_below_left_proc, &is_below_right_proc, 1);

                //it's supposed that chroma has the same result as luma!!!
                SAO_on_block(info, map, pic_rec, pic_sao, &(sao_blk_param[comp_idx]), comp_idx,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t,
                             is_left_proc /*Left*/, is_right_proc/*Right*/, is_above_proc/*Above*/, is_below_proc/*Below*/, is_above_left_proc/*AboveLeft*/,
                             is_above_right_proc/*AboveRight*/, is_below_left_proc/*BelowLeft*/, is_below_right_proc/*BelowRight*/, sample_bit_depth);
            }
            if (is_left_avail)
            {
                /**
                * HSUAN: AREA 4
                */
                lcu_pix_width_t = SAO_SHIFT_PIX_NUM;
                lcu_pix_height_t = comp_idx ? ((lcu_pix_height >> 1) - SAO_SHIFT_PIX_NUM) : (lcu_pix_height - SAO_SHIFT_PIX_NUM);
                pix_x_t = comp_idx ? ((pix_x >> 1) - SAO_SHIFT_PIX_NUM) : (pix_x - SAO_SHIFT_PIX_NUM);
                pix_y_t = comp_idx ? (pix_y >> 1) : pix_y;
                check_boundary_proc(info, map,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, comp_idx, &is_left_proc,
                                  &is_right_proc, &is_above_proc, &is_below_proc, &is_above_left_proc, &is_above_right_proc, &is_below_left_proc, &is_below_right_proc, 1);

                //it's supposed that chroma has the same result as luma!!!
                SAO_on_block(info, map, pic_rec, pic_sao, &(sao_blk_param[comp_idx]), comp_idx,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t,
                             is_left_proc /*Left*/, is_right_proc/*Right*/, is_above_proc/*Above*/, is_below_proc/*Below*/, is_above_left_proc/*AboveLeft*/,
                             is_above_right_proc/*AboveRight*/, is_below_left_proc/*BelowLeft*/, is_below_right_proc/*BelowRight*/, sample_bit_depth);
            }
            if (is_above_avail)
            {
                /**
                * HSUAN: AREA 2
                */
                lcu_pix_width_t = comp_idx ? ((lcu_pix_width >> 1) - SAO_SHIFT_PIX_NUM) : (lcu_pix_width - SAO_SHIFT_PIX_NUM);
                lcu_pix_height_t = SAO_SHIFT_PIX_NUM;
                pix_x_t = comp_idx ? (pix_x >> 1) : pix_x;
                pix_y_t = comp_idx ? ((pix_y >> 1) - SAO_SHIFT_PIX_NUM) : (pix_y - SAO_SHIFT_PIX_NUM);
                check_boundary_proc(info, map,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, comp_idx, &is_left_proc,
                                  &is_right_proc, &is_above_proc, &is_below_proc, &is_above_left_proc, &is_above_right_proc, &is_below_left_proc, &is_below_right_proc, 1);

                //it's supposed that chroma has the same result as luma!!!
                SAO_on_block(info, map, pic_rec, pic_sao, &(sao_blk_param[comp_idx]), comp_idx,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t,
                             is_left_proc /*Left*/, is_right_proc/*Right*/, is_above_proc/*Above*/, is_below_proc/*Below*/, is_above_left_proc/*AboveLeft*/,
                             is_above_right_proc/*AboveRight*/, is_below_left_proc/*BelowLeft*/, is_below_right_proc/*BelowRight*/, sample_bit_depth);
            }
            if (!is_right_avail)
            {
                if (is_above_avail && !is_above_right_avail)
                {
                    /**
                    * HSUAN: AREA 3
                    */
                    lcu_pix_width_t = SAO_SHIFT_PIX_NUM;
                    lcu_pix_height_t = SAO_SHIFT_PIX_NUM;
                    pix_x_t = comp_idx ? ((pix_x >> 1) + (lcu_pix_width >> 1) - SAO_SHIFT_PIX_NUM) : (pix_x + lcu_pix_width -
                              SAO_SHIFT_PIX_NUM);
                    pix_y_t = comp_idx ? ((pix_y >> 1) - SAO_SHIFT_PIX_NUM) : (pix_y - SAO_SHIFT_PIX_NUM);
                    check_boundary_proc(info, map,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, comp_idx, &is_left_proc,
                                      &is_right_proc, &is_above_proc, &is_below_proc, &is_above_left_proc, &is_above_right_proc, &is_below_left_proc, &is_below_right_proc, 1);

                    //it's supposed that chroma has the same result as luma!!!
                    SAO_on_block(info, map, pic_rec, pic_sao, &(sao_blk_param[comp_idx]), comp_idx,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t,
                                 is_left_proc /*Left*/, is_right_proc/*Right*/, is_above_proc/*Above*/, is_below_proc/*Below*/, is_above_left_proc/*AboveLeft*/,
                                 is_above_right_proc/*AboveRight*/, is_below_left_proc/*BelowLeft*/, is_below_right_proc/*BelowRight*/, sample_bit_depth);
                }
                /**
                * HSUAN: AREA 6
                */
                lcu_pix_width_t = SAO_SHIFT_PIX_NUM;
                lcu_pix_height_t = comp_idx ? ((lcu_pix_height >> 1) - SAO_SHIFT_PIX_NUM) : (lcu_pix_height - SAO_SHIFT_PIX_NUM);
                pix_x_t = comp_idx ? ((pix_x >> 1) + (lcu_pix_width >> 1) - SAO_SHIFT_PIX_NUM) : (pix_x + lcu_pix_width -
                          SAO_SHIFT_PIX_NUM);
                pix_y_t = comp_idx ? (pix_y >> 1) : pix_y;
                check_boundary_proc(info, map,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, comp_idx, &is_left_proc,
                                  &is_right_proc, &is_above_proc, &is_below_proc, &is_above_left_proc, &is_above_right_proc, &is_below_left_proc, &is_below_right_proc, 1);

                //it's supposed that chroma has the same result as luma!!!
                SAO_on_block(info, map, pic_rec, pic_sao, &(sao_blk_param[comp_idx]), comp_idx,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t,
                             is_left_proc /*Left*/, is_right_proc/*Right*/, is_above_proc/*Above*/, is_below_proc/*Below*/, is_above_left_proc/*AboveLeft*/,
                             is_above_right_proc/*AboveRight*/, is_below_left_proc/*BelowLeft*/, is_below_right_proc/*BelowRight*/, sample_bit_depth);
            }
            if (!is_below_avail)
            {
                if (is_left_avail)
                {
                    /**
                    * HSUAN: AREA 7
                    */
                    lcu_pix_width_t = SAO_SHIFT_PIX_NUM;
                    lcu_pix_height_t = SAO_SHIFT_PIX_NUM;
                    pix_x_t = comp_idx ? ((pix_x >> 1) - SAO_SHIFT_PIX_NUM) : (pix_x - SAO_SHIFT_PIX_NUM);
                    pix_y_t = comp_idx ? ((pix_y >> 1) + (lcu_pix_height >> 1) - SAO_SHIFT_PIX_NUM) : (pix_y + lcu_pix_height -
                              SAO_SHIFT_PIX_NUM);
                    assert(!is_below_left_avail);
                    check_boundary_proc(info, map,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, comp_idx, &is_left_proc,
                                      &is_right_proc, &is_above_proc, &is_below_proc, &is_above_left_proc, &is_above_right_proc, &is_below_left_proc, &is_below_right_proc, 1);

                    //it's supposed that chroma has the same result as luma!!!
                    SAO_on_block(info, map, pic_rec, pic_sao, &(sao_blk_param[comp_idx]), comp_idx,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t,
                                 is_left_proc /*Left*/, is_right_proc/*Right*/, is_above_proc/*Above*/, is_below_proc/*Below*/, is_above_left_proc/*AboveLeft*/,
                                 is_above_right_proc/*AboveRight*/, is_below_left_proc/*BelowLeft*/, is_below_right_proc/*BelowRight*/, sample_bit_depth);
                }
                /**
                * HSUAN: AREA 8
                */
                lcu_pix_width_t = comp_idx ? ((lcu_pix_width >> 1) - SAO_SHIFT_PIX_NUM) : (lcu_pix_width - SAO_SHIFT_PIX_NUM);
                lcu_pix_height_t = SAO_SHIFT_PIX_NUM;
                pix_x_t = comp_idx ? (pix_x >> 1) : pix_x;
                pix_y_t = comp_idx ? ((pix_y >> 1) + (lcu_pix_height >> 1) - SAO_SHIFT_PIX_NUM) : (pix_y + lcu_pix_height -
                          SAO_SHIFT_PIX_NUM);
                check_boundary_proc(info, map,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, comp_idx, &is_left_proc,
                                  &is_right_proc, &is_above_proc, &is_below_proc, &is_above_left_proc, &is_above_right_proc, &is_below_left_proc, &is_below_right_proc, 1);

                //it's supposed that chroma has the same result as luma!!!
                SAO_on_block(info, map, pic_rec, pic_sao, &(sao_blk_param[comp_idx]), comp_idx,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t,
                             is_left_proc /*Left*/, is_right_proc/*Right*/, is_above_proc/*Above*/, is_below_proc/*Below*/, is_above_left_proc/*AboveLeft*/,
                             is_above_right_proc/*AboveRight*/, is_below_left_proc/*BelowLeft*/, is_below_right_proc/*BelowRight*/, sample_bit_depth);
            }
            if (!is_below_right_avail && !is_right_avail && !is_below_avail)
            {
                /**
                * HSUAN: AREA 9
                */
                lcu_pix_width_t = SAO_SHIFT_PIX_NUM;
                lcu_pix_height_t = SAO_SHIFT_PIX_NUM;
                pix_x_t = comp_idx ? ((pix_x >> 1) + (lcu_pix_width >> 1) - SAO_SHIFT_PIX_NUM) : (pix_x + lcu_pix_width -
                          SAO_SHIFT_PIX_NUM);
                pix_y_t = comp_idx ? ((pix_y >> 1) + (lcu_pix_height >> 1) - SAO_SHIFT_PIX_NUM) : (pix_y + lcu_pix_height -
                          SAO_SHIFT_PIX_NUM);
                check_boundary_proc(info, map,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, comp_idx, &is_left_proc,
                                  &is_right_proc, &is_above_proc, &is_below_proc, &is_above_left_proc, &is_above_right_proc, &is_below_left_proc, &is_below_right_proc, 1);

                //it's supposed that chroma has the same result as luma!!!
                SAO_on_block(info, map, pic_rec, pic_sao, &(sao_blk_param[comp_idx]), comp_idx,  pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t,
                             is_left_proc /*Left*/, is_right_proc/*Right*/, is_above_proc/*Above*/, is_below_proc/*Below*/, is_above_left_proc/*AboveLeft*/,
                             is_above_right_proc/*AboveRight*/, is_below_left_proc/*BelowLeft*/, is_below_right_proc/*BelowRight*/, sample_bit_depth);
            }
        }
    }
}

void sao_frame(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_rec, COM_PIC  *pic_sao, SAO_BLK_PARAM **rec_sao_blk_param)
{
    int pic_pix_height = info->pic_height;
    int pic_pix_width = info->pic_width;
    int input_max_size_in_bit = info->log2_max_cuwh;
    int bit_depth = info->bit_depth_internal;
    int pix_y, pix_x;
    int lcu_pix_height, lcu_pix_width;

    for (pix_y = 0; pix_y < pic_pix_height; pix_y += lcu_pix_height)
    {
        lcu_pix_height = min(1 << (input_max_size_in_bit), (pic_pix_height - pix_y));
        for (pix_x = 0; pix_x < pic_pix_width; pix_x += lcu_pix_width)
        {
            int x_in_lcu = pix_x >> info->log2_max_cuwh;
            int y_in_lcu = pix_y >> info->log2_max_cuwh;
            int lcu_pos = x_in_lcu + y_in_lcu * info->pic_width_in_lcu;
            lcu_pix_width = min(1 << (input_max_size_in_bit), (pic_pix_width - pix_x));

            SAO_on_smb(info, map, pic_rec, pic_sao, pix_y, pix_x, lcu_pix_width, lcu_pix_height, rec_sao_blk_param[lcu_pos], bit_depth);
        }
    }
}

int com_malloc_3d_sao_stat_data(SAO_STAT_DATA ** **array3D, int num_SMB, int num_comp, int num_class)
{
    int i, j;
    if (((*array3D) = (SAO_STAT_DATA ** *)calloc(num_SMB, sizeof(SAO_STAT_DATA **))) == NULL)
    {
        printf("MALLOC FAILED: get_mem3DSAOstatdate: array3D");
        assert(0);
        exit(-1);
    }
    for (i = 0; i < num_SMB; i++)
    {
        if ((*(*array3D + i) = (SAO_STAT_DATA **)calloc(num_comp, sizeof(SAO_STAT_DATA *))) == NULL)
        {
            printf("MALLOC FAILED: get_mem2DSAOstatdate: array2D");
            assert(0);
            exit(-1);
        }
        for (j = 0; j < num_comp; j++)
        {
            if ((*(*(*array3D + i) + j) = (SAO_STAT_DATA *)calloc(num_class, sizeof(SAO_STAT_DATA))) == NULL)
            {
                printf("MALLOC FAILED: get_mem1DSAOstatdate: arrayD");
                assert(0);
                exit(-1);
            }
        }
    }
    return num_SMB * num_comp * num_class * sizeof(SAO_STAT_DATA);
}

int com_malloc_2d_sao_param(SAO_BLK_PARAM *** array2D, int num_SMB, int num_comp)
{
    int i;
    if (((*array2D) = (SAO_BLK_PARAM **)calloc(num_SMB, sizeof(SAO_BLK_PARAM *))) == NULL)
    {
        printf("MALLOC FAILED: get_mem2DSAOBlkParam: array2D");
        assert(0);
        exit(-1);
    }
    for (i = 0; i < num_SMB; i++)
    {
        if ((*(*array2D + i) = (SAO_BLK_PARAM *)calloc(num_comp, sizeof(SAO_BLK_PARAM))) == NULL)
        {
            printf("MALLOC FAILED: get_mem1DSAOBlkParam: array1D");
            assert(0);
            exit(-1);
        }
    }
    return num_SMB * num_comp * sizeof(SAO_BLK_PARAM);
}

void com_free_3d_sao_stat_data(SAO_STAT_DATA *** array3D, int num_SMB, int num_comp)
{
    int i, j;
    if (array3D)
    {
        for (i = 0; i < num_SMB; i++)
        {
            if (array3D[i])
            {
                for (j = 0; j < num_comp; j++)
                {
                    if (array3D[i][j])
                    {
                        free(array3D[i][j]);
                        array3D[i][j] = NULL;
                    }
                }
                free(array3D[i]);
                array3D[i] = NULL;
            }
        }
        //array3D is not freed totally, and this leads to some memory leak, but not a big deal
    }
}

void com_free_2d_sao_param(SAO_BLK_PARAM **array2D, int num_SMB)
{
    int i;
    if (array2D)
    {
        for (i = 0; i < num_SMB; i++)
        {
            if (array2D[i])
            {
                free(array2D[i]);
                array2D[i] = NULL;
            }
        }
    }
    //array2D is not freed totally, this leads to some memory leak, but not a big deal
}
