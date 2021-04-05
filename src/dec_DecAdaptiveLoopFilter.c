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

#include "dec_DecAdaptiveLoopFilter.h"

#define Clip_post(high,val) ((val > high)? high: val)

/*
*************************************************************************
* Function: ALF decoding process top function
* Input:
*         ctx  : CONTEXT used for decoding process
*     pic_rec  : picture of the the ALF input image
*     pic_dec  : picture of the the ALF reconstructed image
*************************************************************************
*/
void alf_process_dec(DEC_CTX* ctx, ALF_PARAM **alf_param, COM_PIC* pic_rec, COM_PIC* pic_dec)
{
    int bit_depth = ctx->info.bit_depth_internal;
    pel *rec_Y, *dec_Y;
    pel *rec_UV[2], *dec_UV[2];
    int  lcu_idx, num_lcu_in_frame, num_lcu_in_pic_width, num_lcu_in_pic_height;
    int  lcu_y_pos, lcu_x_pos, lcu_height, lcu_width;
    int  comp_idx, luma_stride, chroma_stride, img_height, img_width;
    BOOL is_left_avail, is_right_avail, is_above_avail, is_below_avail;
    BOOL is_above_left_avail, is_above_right_avail;
    lcu_height = 1 << ctx->info.log2_max_cuwh;
    lcu_width  = lcu_height;
    img_height = ctx->info.pic_height;
    img_width  = ctx->info.pic_width;
    num_lcu_in_pic_width  = img_width / lcu_width;
    num_lcu_in_pic_height = img_height / lcu_height;
    num_lcu_in_pic_width  += (img_width % lcu_width) ? 1 : 0;
    num_lcu_in_pic_height += (img_height % lcu_height) ? 1 : 0;
    num_lcu_in_frame = num_lcu_in_pic_height * num_lcu_in_pic_width;
    luma_stride = pic_rec->stride_luma;
    chroma_stride = pic_rec->stride_chroma;
    rec_Y = pic_rec->y;
    rec_UV[0] = pic_rec->u;
    rec_UV[1] = pic_rec->v;
    dec_Y = pic_dec->y;
    dec_UV[0] = pic_dec->u;
    dec_UV[1] = pic_dec->v;
#if ALF_SHAPE
    BOOL alf_shape_flag = ctx->info.sqh.adaptive_filter_shape_enable_flag;
#endif
    for (lcu_idx = 0; lcu_idx < num_lcu_in_frame; lcu_idx++)
    {
        lcu_y_pos = (lcu_idx / num_lcu_in_pic_width) * lcu_height;
        lcu_x_pos = (lcu_idx % num_lcu_in_pic_width) * lcu_width;
        int cur_lcu_height = (lcu_y_pos + lcu_height > img_height) ? (img_height - lcu_y_pos) : lcu_height;
        int cur_lcu_width = (lcu_x_pos + lcu_width  > img_width) ? (img_width - lcu_x_pos) : lcu_width;
        //derive CTU boundary availabilities
        derive_boundary_avail(ctx, num_lcu_in_pic_width, num_lcu_in_pic_height, lcu_idx, &is_left_avail, &is_right_avail, &is_above_avail,
                            &is_below_avail, &is_above_left_avail, &is_above_right_avail);
        for (comp_idx = 0; comp_idx < N_C; comp_idx++)
        {
            if (!ctx->dec_alf->alf_lcu_enabled[lcu_idx][comp_idx])
            {
                continue;
            }
            if (comp_idx == Y_C)
            {
                filter_one_ctb(ctx->dec_alf, rec_Y, dec_Y, (comp_idx == Y_C) ? (luma_stride) : (chroma_stride)
                             , comp_idx, bit_depth, alf_param[comp_idx], lcu_y_pos, cur_lcu_height, lcu_x_pos, cur_lcu_width
                             , is_above_avail, is_below_avail, is_left_avail, is_right_avail, is_above_left_avail, is_above_right_avail, bit_depth
#if ALF_SHAPE
                             , alf_shape_flag
#endif
                );
            }
            else
            {
                filter_one_ctb(ctx->dec_alf, rec_UV[comp_idx - U_C], dec_UV[comp_idx - U_C], (comp_idx == Y_C) ? (luma_stride) : (chroma_stride)
                             , comp_idx, bit_depth, alf_param[comp_idx], lcu_y_pos, cur_lcu_height, lcu_x_pos, cur_lcu_width
                             , is_above_avail, is_below_avail, is_left_avail, is_right_avail, is_above_left_avail, is_above_right_avail, bit_depth
#if ALF_SHAPE
                             , alf_shape_flag
#endif
                );
            }
        }
    }
}

/*
*************************************************************************
* Function: ALF filter on CTB
*************************************************************************
*/
void filter_one_ctb(DEC_ALF_VAR * dec_alf, pel *rec, pel *dec, int stride, int comp_idx, int bit_depth, ALF_PARAM *alf_param
                  , int lcu_y_pos, int lcu_height, int lcu_x_pos, int lcu_width
                  , BOOL is_above_avail, BOOL is_below_avail, BOOL is_left_avail, BOOL is_right_avail, BOOL is_above_left_avail
                  , BOOL is_above_right_avail, int sample_bit_depth
#if ALF_SHAPE
                  , BOOL alf_shape_flag
#endif
                 )
{
    const int  format_shift = (comp_idx == Y_C) ? 0 : 1;
    int y_pos, x_pos, height, width;
    //reconstruct coefficients to m_filterCoeffSym and m_varIndTab
    reconstruct_coef_info(comp_idx, alf_param, dec_alf->filter_coeff_sym,
                        dec_alf->var_ind_tab); //reconstruct ALF coefficients & related parameters
    //derive CTB start positions, width, and height. If the boundary is not available, skip boundary samples.
    y_pos = (lcu_y_pos >> format_shift);
    height = (lcu_height >> format_shift);
    x_pos = (lcu_x_pos >> format_shift);
    width = (lcu_width >> format_shift);
    filter_one_comp_region(rec, dec, stride, (comp_idx != Y_C)
                        , y_pos, height, x_pos, width, dec_alf->filter_coeff_sym, dec_alf->var_ind_tab, dec_alf->var_img, sample_bit_depth,
                        is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail
#if ALF_SHAPE
                        , alf_shape_flag
#endif
    );
}

void derive_alf_boundary_availibility(DEC_CTX *ctx, int num_lcu_in_pic_width, int num_lcu_in_pic_height, int lcu_idx,
        BOOL *is_left_avail, BOOL *is_right_avail, BOOL *is_above_avail, BOOL *is_below_avail)
{
    int num_lcu_in_frame = num_lcu_in_pic_height * num_lcu_in_pic_width;
    BOOL is_above_left_avail;
    BOOL is_above_right_avail;
    BOOL is_below_left_avail;
    BOOL is_below_right_avail;
    int  lcu_height = 1 << ctx->info.log2_max_cuwh;
    int  lcu_width = lcu_height;
    int  img_height = ctx->info.pic_height;
    int  img_width = ctx->info.pic_width;
    int  pic_x;
    int  pic_y;
    int  mb_x;
    int  mb_y;
    int  mb_curr;
    int  smb_mb_width;
    int  smb_mb_height;
    int  pic_mb_width = img_width / MIN_CU_SIZE;
    int  pic_mb_height = img_height / MIN_CU_SIZE;
    int  mb_left;
    int  mb_right;
    int  mb_above;
    int  mb_below;
    int  mb_above_left;
    int  mb_above_right;
    int  mb_below_left;
    int  mb_below_right;
    int  mb_curr_patch_idx, mb_neighbor_patch_idx, mb_neighbor2_patch_idx, mb_neighbor3_patch_idx;
    pic_x = (lcu_idx % num_lcu_in_pic_width) * lcu_width;
    pic_y = (lcu_idx / num_lcu_in_pic_width) * lcu_height;
    pic_mb_width += (img_width  % MIN_CU_SIZE) ? 1 : 0;
    pic_mb_height += (img_height % MIN_CU_SIZE) ? 1 : 0;
    mb_x = pic_x / MIN_CU_SIZE;
    mb_y = pic_y / MIN_CU_SIZE;
    mb_curr = mb_y * pic_mb_width + mb_x;
    smb_mb_width = lcu_width >> MIN_CU_LOG2;
    smb_mb_height = lcu_height >> MIN_CU_LOG2;
    *is_left_avail = (lcu_idx % num_lcu_in_pic_width != 0);
    *is_right_avail = (lcu_idx % num_lcu_in_pic_width != num_lcu_in_pic_width - 1);
    *is_above_avail = (lcu_idx >= num_lcu_in_pic_width);
    *is_below_avail = (lcu_idx  < num_lcu_in_frame - num_lcu_in_pic_width);
    is_above_left_avail = (*is_above_avail && *is_left_avail);
    is_above_right_avail = (*is_above_avail && *is_right_avail);
    is_below_left_avail = (*is_below_avail && *is_left_avail);
    is_below_right_avail = (*is_below_avail && *is_right_avail);
    s8 *map_patch_idx = ctx->map.map_patch_idx;
    mb_left = *is_left_avail ? (mb_curr - 1) : -1;
    mb_right = *is_right_avail ? (mb_curr + 1) : -1;
    mb_above = *is_above_avail ? (mb_curr - pic_mb_width) : -1;
    mb_below = *is_below_avail ? (mb_curr + pic_mb_width) : -1;
    mb_above_left = is_above_left_avail ? (mb_curr - pic_mb_width - 1) : -1;
    mb_above_right = is_above_right_avail ? (mb_curr - pic_mb_width + 1) : -1;
    mb_below_left = is_below_left_avail ? (mb_curr + pic_mb_width - 1) : -1;
    mb_below_right = is_below_right_avail ? (mb_curr + pic_mb_width + 1) : -1;
    *is_left_avail = *is_right_avail = *is_above_avail = *is_below_avail = FALSE;
    mb_curr_patch_idx = map_patch_idx[mb_curr];
    if (mb_left != -1)
    {
        mb_neighbor_patch_idx = map_patch_idx[mb_left];
        if (mb_curr_patch_idx == mb_neighbor_patch_idx)
        {
            *is_left_avail = TRUE;
        }
    }
    if (mb_right != -1)
    {
        mb_neighbor_patch_idx = map_patch_idx[mb_right];
        if (mb_curr_patch_idx == mb_neighbor_patch_idx)
        {
            *is_right_avail = TRUE;
        }
    }
    if (mb_above != -1 && mb_above_left != -1 && mb_above_right != -1)
    {
        mb_neighbor_patch_idx = map_patch_idx[mb_above];
        mb_neighbor2_patch_idx = map_patch_idx[mb_above_left];
        mb_neighbor3_patch_idx = map_patch_idx[mb_above_right];
        if ((mb_curr_patch_idx == mb_neighbor_patch_idx)
                && (mb_neighbor_patch_idx == mb_neighbor2_patch_idx)
                && (mb_neighbor_patch_idx == mb_neighbor3_patch_idx))
        {
            *is_above_avail = TRUE;
        }
    }
    if (mb_below != -1 && mb_below_left != -1 && mb_below_right != -1)
    {
        mb_neighbor_patch_idx = map_patch_idx[mb_below];
        mb_neighbor2_patch_idx = map_patch_idx[mb_below_left];
        mb_neighbor3_patch_idx = map_patch_idx[mb_below_right];
        if ((mb_curr_patch_idx == mb_neighbor_patch_idx)
                && (mb_neighbor_patch_idx == mb_neighbor2_patch_idx)
                && (mb_neighbor_patch_idx == mb_neighbor3_patch_idx))
        {
            *is_below_avail = TRUE;
        }
    }
}

void create_alf_global_buffer(DEC_CTX *ctx)
{
    int lcu_height, lcu_width, img_height, img_width;
    int num_lcu_in_frame, num_lcu_in_pic_width, num_lcu_in_pic_height;
    int n, i, g;
    int tbl_region[NO_VAR_BINS] = { 0, 1, 4, 5, 15, 2, 3, 6, 14, 11, 10, 7, 13, 12,  9,  8 };
    int x_interval;
    int y_interval;
    int y_index, x_index;
    int y_index_offset;
    lcu_height = 1 << (ctx->info.log2_max_cuwh);
    lcu_width = lcu_height;
    img_height = ctx->info.pic_height;
    img_width = ctx->info.pic_width;
    num_lcu_in_pic_width = img_width / lcu_width;
    num_lcu_in_pic_height = img_height / lcu_height;
    num_lcu_in_pic_width += (img_width % lcu_width) ? 1 : 0;
    num_lcu_in_pic_height += (img_height % lcu_height) ? 1 : 0;
    num_lcu_in_frame = num_lcu_in_pic_height * num_lcu_in_pic_width;
    x_interval = ((((img_width + lcu_width - 1) / lcu_width) + 1) / 4 * lcu_width);
    y_interval = ((((img_height + lcu_height - 1) / lcu_height) + 1) / 4 * lcu_height);
    ctx->dec_alf = (DEC_ALF_VAR *)malloc(1 * sizeof(DEC_ALF_VAR));
    ctx->dec_alf->alf_lcu_enabled = (BOOL **)malloc(num_lcu_in_frame * sizeof(BOOL *));
#if ALF_SHAPE
    int num_coef = (ctx->info.sqh.adaptive_filter_shape_enable_flag) ? ALF_MAX_NUM_COEF_SHAPE2 : ALF_MAX_NUM_COEF;
#endif
    for (n = 0; n < num_lcu_in_frame; n++)
    {
        ctx->dec_alf->alf_lcu_enabled[n] = (BOOL *)malloc(N_C * sizeof(BOOL));
        memset(ctx->dec_alf->alf_lcu_enabled[n], FALSE, N_C * sizeof(BOOL));
    }
    ctx->dec_alf->var_img = (pel **)malloc(img_height * sizeof(pel *));
    for (n = 0; n < img_height; n++)
    {
        ctx->dec_alf->var_img[n] = (pel *)malloc(img_width * sizeof(pel));
        memset(ctx->dec_alf->var_img[n], 0, img_width * sizeof(pel));
    }
    ctx->dec_alf->filter_coeff_sym = (int **)malloc(NO_VAR_BINS * sizeof(int *));
    for (g = 0; g < (int)NO_VAR_BINS; g++)
    {
#if ALF_SHAPE
        ctx->dec_alf->filter_coeff_sym[g] = (int *)malloc(num_coef * sizeof(int));
        memset(ctx->dec_alf->filter_coeff_sym[g], 0, num_coef * sizeof(int));
#else
        ctx->dec_alf->filter_coeff_sym[g] = (int *)malloc(ALF_MAX_NUM_COEF * sizeof(int));
        memset(ctx->dec_alf->filter_coeff_sym[g], 0, ALF_MAX_NUM_COEF * sizeof(int));
#endif
    }
    for (i = 0; i < img_height; i = i + 4)
    {
        y_index = (y_interval == 0) ? (3) : (Clip_post(3, i / y_interval));
        y_index_offset = y_index * 4;
        for (g = 0; g < img_width; g = g + 4)
        {
            x_index = (x_interval == 0) ? (3) : (Clip_post(3, g / x_interval));
            ctx->dec_alf->var_img[i >> LOG2_VAR_SIZE_H][g >> LOG2_VAR_SIZE_W] = (pel)tbl_region[y_index_offset + x_index];
        }
    }
    ctx->dec_alf->alf_picture_param = (ALF_PARAM **)malloc((N_C) * sizeof(ALF_PARAM *));
    for (i = 0; i < N_C; i++)
    {
        ctx->dec_alf->alf_picture_param[i] = NULL;
        allocate_alf_param(&(ctx->dec_alf->alf_picture_param[i]), i
#if ALF_SHAPE
                        , num_coef
#endif
        );
    }
}

void release_alf_global_buffer(DEC_CTX *ctx)
{
    int lcu_height, lcu_width, img_height, img_width;
    int num_lcu_in_frame, num_lcu_in_pic_width, num_lcu_in_pic_height;
    int n, g;

    lcu_height = 1 << (ctx->info.log2_max_cuwh);
    lcu_width = lcu_height;
    img_height = ctx->info.pic_height;
    img_width = ctx->info.pic_width;
    num_lcu_in_pic_width = img_width / lcu_width;
    num_lcu_in_pic_height = img_height / lcu_height;
    num_lcu_in_pic_width += (img_width % lcu_width) ? 1 : 0;
    num_lcu_in_pic_height += (img_height % lcu_height) ? 1 : 0;
    num_lcu_in_frame = num_lcu_in_pic_height * num_lcu_in_pic_width;
    if (ctx->dec_alf)
    {
        if (ctx->dec_alf->var_img)
        {
            for (n = 0; n < img_height; n++)
            {
                free(ctx->dec_alf->var_img[n]);
            }
            free(ctx->dec_alf->var_img);
            ctx->dec_alf->var_img = NULL;
        }
        if (ctx->dec_alf->alf_lcu_enabled)
        {
            for (n = 0; n < num_lcu_in_frame; n++)
            {
                free(ctx->dec_alf->alf_lcu_enabled[n]);
            }
            free(ctx->dec_alf->alf_lcu_enabled);
            ctx->dec_alf->alf_lcu_enabled = NULL;
        }
        if (ctx->dec_alf->filter_coeff_sym)
        {
            for (g = 0; g < (int)NO_VAR_BINS; g++)
            {
                free(ctx->dec_alf->filter_coeff_sym[g]);
            }
            free(ctx->dec_alf->filter_coeff_sym);
            ctx->dec_alf->filter_coeff_sym = NULL;
        }
        if (ctx->dec_alf->alf_picture_param)
        {
            for (g = 0; g < (int)N_C; g++)
            {
                free_alf_param(ctx->dec_alf->alf_picture_param[g], g);
            }
            free(ctx->dec_alf->alf_picture_param);
            ctx->dec_alf->alf_picture_param = NULL;
        }
        if (ctx->dec_alf)
        {
            free(ctx->dec_alf);
            ctx->dec_alf = NULL;
        }
    }
}

void allocate_alf_param(ALF_PARAM **alf_param, int comp_idx
#if ALF_SHAPE
                    , int num_coef
#endif
)
{
    *alf_param = (ALF_PARAM *)malloc(sizeof(ALF_PARAM));
    (*alf_param)->alf_flag = 0;
#if ALF_SHAPE
    (*alf_param)->num_coeff = num_coef;
#else
    (*alf_param)->num_coeff = ALF_MAX_NUM_COEF;
#endif
    (*alf_param)->filters_per_group = 1;
    (*alf_param)->component_id = comp_idx;
    (*alf_param)->coeff_multi = NULL;
    (*alf_param)->filter_pattern = NULL;
    switch (comp_idx)
    {
    case Y_C:
#if ALF_SHAPE
        get_mem_2D_int(&((*alf_param)->coeff_multi), NO_VAR_BINS, num_coef);
#else
        get_mem_2D_int(&((*alf_param)->coeff_multi), NO_VAR_BINS, ALF_MAX_NUM_COEF);
#endif
        get_mem_1D_int(&((*alf_param)->filter_pattern), NO_VAR_BINS);
        break;
    case U_C:
    case V_C:
#if ALF_SHAPE
        get_mem_2D_int(&((*alf_param)->coeff_multi), 1, num_coef);
#else
        get_mem_2D_int(&((*alf_param)->coeff_multi), 1, ALF_MAX_NUM_COEF);
#endif
        break;
    default:
        printf("Not a legal component ID\n");
        assert(0);
        exit(-1);
    }
}

void free_alf_param(ALF_PARAM *alf_param, int comp_idx)
{
    switch (comp_idx)
    {
    case Y_C:
        free_mem_2D_int(alf_param->coeff_multi);
        free_mem_1D_int(alf_param->filter_pattern);
        break;
    case U_C:
    case V_C:
        free_mem_2D_int(alf_param->coeff_multi);
        break;
    default:
        printf("Not a legal component ID\n");
        assert(0);
        exit(-1);
    }
    free(alf_param);
    alf_param = NULL;
}

void derive_boundary_avail(DEC_CTX *ctx, int num_lcu_in_pic_width, int num_lcu_in_pic_height, int ctu, BOOL *is_left_avail,
                         BOOL *is_right_avail, BOOL *is_above_avail, BOOL *is_below_avail, BOOL *is_above_left_avail, BOOL *is_above_right_avail)
{
    int  num_lcu_in_frame = num_lcu_in_pic_height * num_lcu_in_pic_width;
    int  lcu_height = 1 << ctx->info.log2_max_cuwh;
    int  lcu_width = lcu_height;
    int  pic_x;
    int  pic_y;
    int  mb_x;
    int  mb_y;
    int  mb_curr;
    int  pic_mb_width = ctx->info.pic_width_in_scu;
    int  mb_left;
    int  mb_right;
    int  mb_above;
    int  mb_curr_patch_idx, mb_neighbor_patch_idx;
    int  mb_above_left;
    int  mb_above_right;
    int cross_patch_flag = ctx->info.sqh.cross_patch_loop_filter;
    pic_x = (ctu % num_lcu_in_pic_width) * lcu_width;
    pic_y = (ctu / num_lcu_in_pic_width) * lcu_height;

    mb_x = pic_x / MIN_CU_SIZE;
    mb_y = pic_y / MIN_CU_SIZE;
    mb_curr = mb_y * pic_mb_width + mb_x;
    *is_left_avail = (ctu % num_lcu_in_pic_width != 0);
    *is_right_avail = (ctu % num_lcu_in_pic_width != num_lcu_in_pic_width - 1);
    *is_above_avail = (ctu >= num_lcu_in_pic_width);
    *is_below_avail = (ctu  < num_lcu_in_frame - num_lcu_in_pic_width);
    *is_above_left_avail = *is_above_avail && *is_left_avail;
    *is_above_right_avail = *is_above_avail && *is_right_avail;
    s8* map_patch_idx = ctx->map.map_patch_idx;
    mb_left = *is_left_avail ? (mb_curr - 1) : -1;
    mb_right = *is_right_avail ? (mb_curr + (lcu_width >> MIN_CU_LOG2)) : -1;
    mb_above = *is_above_avail ? (mb_curr - pic_mb_width) : -1;
    mb_above_left = *is_above_left_avail ? (mb_curr - pic_mb_width - 1) : -1;
    mb_above_right = *is_above_right_avail ? (mb_curr - pic_mb_width + (lcu_width >> MIN_CU_LOG2)) : -1;
    if (!cross_patch_flag)
    {
        *is_left_avail = *is_right_avail = *is_above_avail = FALSE;
        *is_above_left_avail = *is_above_right_avail = FALSE;
        mb_curr_patch_idx = map_patch_idx[mb_curr];
        if (mb_left != -1)
        {
            mb_neighbor_patch_idx = map_patch_idx[mb_left];
            if (mb_curr_patch_idx == mb_neighbor_patch_idx)
            {
                *is_left_avail = TRUE;
            }
        }
        if (mb_right != -1)
        {
            mb_neighbor_patch_idx = map_patch_idx[mb_right];
            if (mb_curr_patch_idx == mb_neighbor_patch_idx)
            {
                *is_right_avail = TRUE;
            }
        }
        if (mb_above != -1)
        {
            mb_neighbor_patch_idx = map_patch_idx[mb_above];
            if (mb_curr_patch_idx == mb_neighbor_patch_idx)
            {
                *is_above_avail = TRUE;
            }
        }
        if (mb_above_left != -1)
        {
            mb_neighbor_patch_idx = map_patch_idx[mb_above_left];
            if (mb_curr_patch_idx == mb_neighbor_patch_idx)
            {
                *is_above_left_avail = TRUE;
            }
        }
        if (mb_above_right != -1)
        {
            mb_neighbor_patch_idx = map_patch_idx[mb_above_right];
            if (mb_curr_patch_idx == mb_neighbor_patch_idx)
            {
                *is_above_right_avail = TRUE;
            }
        }
    }
}