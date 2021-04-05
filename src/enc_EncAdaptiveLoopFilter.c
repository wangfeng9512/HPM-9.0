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

#include "enc_EncAdaptiveLoopFilter.h"

#define ROUND(a)  (((a) < 0)? (int)((a) - 0.5) : (int)((a) + 0.5))
#define REG              0.0001
#define REG_SQR          0.0000001

//ENC_ALF_VAR *Enc_ALF;

#define Clip_post(high,val) ((val > high)? high: val)
/*
*************************************************************************
* Function: ALF encoding process top function
* Input:
*             ctx  : CONTEXT used for encoding process
*     alf_bs_temp  : bitstream structure for ALF RDO
*         pic_rec  : picture of the reconstructed image
*     pic_alf_Org  : picture of the original image
*     pic_alf_Rec  : picture of the the ALF input image
*     lambda_mode  : The lambda value in the ALF-RD decision
* Output:
*    alf_pic_param: The ALF parameter
*              apsId: The ALF parameter index in the buffer
*       isNewApsSent£ºThe New flag index
* Return:
*************************************************************************
*/
void alf_process(ENC_CTX* ctx, COM_BSW *alf_bs_temp, ALF_PARAM **alf_picture_param, COM_PIC * pic_rec, COM_PIC * pic_alf_Org, COM_PIC * pic_alf_Rec, double lambda_mode)
{
    ENC_CORE *core = ctx->core;
    ENC_SBAC* alf_sbac = GET_SBAC_ENC(alf_bs_temp);
    int luma_margin_x;
    int luma_margin_y;
    int chroma_margin_x;
    int chroma_margin_y;
    int luma_stride; // Add to avoid micro trubles
    pel *rec_Y, *dec_Y, *org_Y;
    pel *rec_UV[2], *dec_UV[2], *org_UV[2];
    luma_margin_x = (1 << ctx->info.log2_max_cuwh) + 16;
    luma_margin_y = (1 << ctx->info.log2_max_cuwh) + 16;
    chroma_margin_x = luma_margin_x >> 1;
    chroma_margin_y = luma_margin_y >> 1;
    rec_Y = pic_rec->y;
    rec_UV[0] = pic_rec->u;
    rec_UV[1] = pic_rec->v;
    dec_Y = pic_alf_Rec->y;
    dec_UV[0] = pic_alf_Rec->u;
    dec_UV[1] = pic_alf_Rec->v;
    org_Y = pic_alf_Org->y;
    org_UV[0] = pic_alf_Org->u;
    org_UV[1] = pic_alf_Org->v;
    SBAC_STORE((core->s_alf_initial), (*alf_sbac));
    luma_stride = pic_rec->stride_luma;
    // !!! NOTED: From There on, We use luma_stride instead of (img->width + (luma_margin_x<<1)) to avoid micro trubles
    get_statistics_alf(ctx, org_Y, org_UV, dec_Y, dec_UV, luma_stride);
    set_cur_alf_param(ctx, alf_bs_temp, alf_picture_param, lambda_mode);
    execute_pic_lcu_on_off_decision(ctx, alf_bs_temp, alf_picture_param, lambda_mode, FALSE, NULL, org_Y, org_UV, dec_Y, dec_UV, rec_Y, rec_UV, luma_stride);
}

/*
*************************************************************************
* Function: Calculate the correlation matrix for image
* Input:
*             ctx  : CONTEXT used for encoding process
*     pic_alf_Org  : picture of the original image
*     pic_alf_Rec  : picture of the the ALF input image
*           stride : The stride of Y component of the ALF input picture
*           lambda : The lambda value in the ALF-RD decision
* Output:
* Return:
*************************************************************************
*/
void get_statistics_alf(ENC_CTX* ctx, pel *img_Y_org, pel **img_UV_org, pel *img_Y_dec, pel **img_UV_dec, int stride)
{
    ENC_ALF_VAR *enc_alf = ctx->enc_alf;
    BOOL  is_left_avail, is_right_avail, is_above_avail, is_below_avail;
    BOOL  is_above_left_avail, is_above_right_avail;
    int lcu_height, lcu_width, img_height, img_width;
    int lcu_idx, num_lcu_in_frame, num_lcu_in_pic_width, num_lcu_in_pic_height;
    int lcu_y_pos, lcu_x_pos;
    int comp_idx, format_shift;
    lcu_height = 1 << ctx->info.log2_max_cuwh;
    lcu_width = lcu_height;
    img_height = ctx->info.pic_height;
    img_width = ctx->info.pic_width;
    num_lcu_in_pic_width = img_width / lcu_width;
    num_lcu_in_pic_height = img_height / lcu_height;
    num_lcu_in_pic_width += (img_width % lcu_width) ? 1 : 0;
    num_lcu_in_pic_height += (img_height % lcu_height) ? 1 : 0;
    num_lcu_in_frame = num_lcu_in_pic_height * num_lcu_in_pic_width;
#if ALF_SHAPE
    BOOL alf_shape_flag = ctx->info.sqh.adaptive_filter_shape_enable_flag; 
#endif
    for (lcu_idx = 0; lcu_idx < num_lcu_in_frame; lcu_idx++)
    {
        lcu_y_pos = (lcu_idx / num_lcu_in_pic_width) * lcu_height;
        lcu_x_pos = (lcu_idx % num_lcu_in_pic_width) * lcu_width;
        int cur_lcu_height = (lcu_y_pos + lcu_height > img_height) ? (img_height - lcu_y_pos) : lcu_height;
        int cur_lcu_width = (lcu_x_pos + lcu_width  > img_width) ? (img_width - lcu_x_pos) : lcu_width;
        derive_boundary_avail(ctx, num_lcu_in_pic_width, num_lcu_in_pic_height, lcu_idx, &is_left_avail, &is_right_avail, &is_above_avail, &is_below_avail,
                            &is_above_left_avail, &is_above_right_avail);
        for (comp_idx = 0; comp_idx < N_C; comp_idx++)
        {
            format_shift = (comp_idx == Y_C) ? 0 : 1;
            reset_alf_corr(enc_alf->alf_corr[comp_idx][lcu_idx]
#if ALF_SHAPE
                          , alf_shape_flag ? ALF_MAX_NUM_COEF_SHAPE2 : ALF_MAX_NUM_COEF
#endif
            );
            if (comp_idx == Y_C)
            {
                get_statistics_one_lcu_alf(ctx->enc_alf, comp_idx, lcu_y_pos, lcu_x_pos, cur_lcu_height, cur_lcu_width, is_above_avail,
                                    is_below_avail, is_left_avail, is_right_avail
                                    , is_above_left_avail, is_above_right_avail, enc_alf->alf_corr[comp_idx][lcu_idx], img_Y_org, img_Y_dec, stride, format_shift
#if ALF_SHAPE
                                    , alf_shape_flag
#endif
                );
            }
            else
            {
                get_statistics_one_lcu_alf(ctx->enc_alf, comp_idx, lcu_y_pos, lcu_x_pos, cur_lcu_height, cur_lcu_width, is_above_avail,
                                    is_below_avail, is_left_avail, is_right_avail
                                    , is_above_left_avail, is_above_right_avail, enc_alf->alf_corr[comp_idx][lcu_idx], img_UV_org[comp_idx - U_C],
                                    img_UV_dec[comp_idx - U_C], (stride >> 1), format_shift
#if ALF_SHAPE
                                    , alf_shape_flag
#endif
                );
            }
        }
    }
}

void derive_alf_boundary_availibility(ENC_CTX* ctx, int num_lcu_in_pic_width, int num_lcu_in_pic_height, int lcu_idx,
        BOOL *is_left_avail, BOOL *is_right_avail, BOOL *is_above_avail, BOOL *is_below_avail)
{
    BOOL is_above_left_avail;
    BOOL is_above_right_avail;
    BOOL is_below_left_avail;
    BOOL is_below_right_avail;
    int  lcu_height = 1 << ctx->info.log2_max_cuwh;
    int  lcu_width = lcu_height;
    int  img_height = ctx->info.pic_height;
    int  img_width = ctx->info.pic_width;
    int  num_lcu_in_frame;
    int  pic_x;
    int  pic_y;
    int  mb_x;
    int  mb_y;
    int  smb_mb_width;
    int  smb_mb_height;
    int  pic_mb_width = img_width / MIN_CU_SIZE;
    int  pic_mb_height = img_height / MIN_CU_SIZE;
    int  mb_cur;
    s8  *map_patch_idx = ctx->map.map_patch_idx;
    int  mb_left;
    int  mb_right;
    int  mb_above;
    int  mb_below;
    int  mb_above_left;
    int  mb_above_right;
    int  mb_below_left;
    int  mb_below_right;
    int  mb_cur_patch_idx, mb_neighbor_patch_idx, mb_neighbor2_patch_idx, mb_neighbor3_patch_idx;
    num_lcu_in_frame = num_lcu_in_pic_height * num_lcu_in_pic_width;
    pic_x = (lcu_idx % num_lcu_in_pic_width) * lcu_width;
    pic_y = (lcu_idx / num_lcu_in_pic_width) * lcu_height;
    pic_mb_width += (img_width  % MIN_CU_SIZE) ? 1 : 0;
    pic_mb_height += (img_height % MIN_CU_SIZE) ? 1 : 0;
    mb_x = pic_x / MIN_CU_SIZE;
    mb_y = pic_y / MIN_CU_SIZE;
    mb_cur = mb_y * pic_mb_width + mb_x;
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
    mb_left = *is_left_avail ? (mb_cur - 1) : -1;
    mb_right = *is_right_avail ? (mb_cur + 1) : -1;
    mb_above = *is_above_avail ? (mb_cur - pic_mb_width) : -1;
    mb_below = *is_below_avail ? (mb_cur + pic_mb_width) : -1;
    mb_above_left = is_above_left_avail ? (mb_cur - pic_mb_width - 1) : -1;
    mb_above_right = is_above_right_avail ? (mb_cur - pic_mb_width + 1) : -1;
    mb_below_left = is_below_left_avail ? (mb_cur + pic_mb_width - 1) : -1;
    mb_below_right = is_below_right_avail ? (mb_cur + pic_mb_width + 1) : -1;
    *is_left_avail = *is_right_avail = *is_above_avail = *is_below_avail = FALSE;
    mb_cur_patch_idx = map_patch_idx[mb_cur];
    if (mb_left != -1)
    {
        mb_neighbor_patch_idx = map_patch_idx[mb_left];
        if (mb_cur_patch_idx == mb_neighbor_patch_idx)
        {
            *is_left_avail = TRUE;
        }
    }
    if (mb_right != -1)
    {
        mb_neighbor_patch_idx = map_patch_idx[mb_right];
        if (mb_cur_patch_idx == mb_neighbor_patch_idx)
        {
            *is_right_avail = TRUE;
        }
    }
    if (mb_above != -1 && mb_above_left != -1 && mb_above_right != -1)
    {
        mb_neighbor_patch_idx = map_patch_idx[mb_above];
        mb_neighbor2_patch_idx = map_patch_idx[mb_above_left];
        mb_neighbor3_patch_idx = map_patch_idx[mb_above_right];
        if ((mb_cur_patch_idx == mb_neighbor_patch_idx)
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
        if ((mb_cur_patch_idx == mb_neighbor_patch_idx)
                && (mb_neighbor_patch_idx == mb_neighbor2_patch_idx)
                && (mb_neighbor_patch_idx == mb_neighbor3_patch_idx))
        {
            *is_below_avail = TRUE;
        }
    }
}

void create_alf_global_buffers(ENC_CTX* ctx)
{
    int lcu_height, lcu_width, img_height, img_width;
    int num_lcu_in_frame, num_lcu_in_pic_width, num_lcu_in_pic_height;
    int comp_idx, n, i, g;
#if ALF_SHAPE
    int num_coef = (ctx->info.sqh.adaptive_filter_shape_enable_flag) ? ALF_MAX_NUM_COEF_SHAPE2 : ALF_MAX_NUM_COEF;
#else
    int num_coef = (int)ALF_MAX_NUM_COEF;
#endif
    int tbl_region[NO_VAR_BINS] = { 0, 1, 4, 5, 15, 2, 3, 6, 14, 11, 10, 7, 13, 12,  9,  8 };
    int x_interval;
    int y_interval;
    int x_idx, y_idx;
    int y_idx_offset;
    lcu_height = 1 << ctx->info.log2_max_cuwh;
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

    ctx->enc_alf = (ENC_ALF_VAR *)malloc(1 * sizeof(ENC_ALF_VAR));
    ctx->enc_alf->bit_increment = 0;
#if ALF_SHAPE
    memset(ctx->enc_alf->y_temp, 0, num_coef * sizeof(double));
#else
    memset(ctx->enc_alf->y_temp, 0, ALF_MAX_NUM_COEF * sizeof(double));
#endif
    memset(ctx->enc_alf->pix_acc_merged, 0, NO_VAR_BINS * sizeof(double));
    memset(ctx->enc_alf->var_ind_tab, 0, NO_VAR_BINS * sizeof(int));
    for (comp_idx = 0; comp_idx < N_C; comp_idx++)
    {
        ctx->enc_alf->alf_corr[comp_idx] = (ALF_CORR_DATA **)malloc(num_lcu_in_frame * sizeof(ALF_CORR_DATA *));
        ctx->enc_alf->alf_non_skipped_corr[comp_idx] = (ALF_CORR_DATA **)malloc(num_lcu_in_frame * sizeof(ALF_CORR_DATA *));
        for (n = 0; n < num_lcu_in_frame; n++)
        {
            allocate_alf_corr_data(&(ctx->enc_alf->alf_corr[comp_idx][n]), comp_idx
#if ALF_SHAPE
                                , num_coef
#endif
            );
            allocate_alf_corr_data(&(ctx->enc_alf->alf_non_skipped_corr[comp_idx][n]), comp_idx
#if ALF_SHAPE
                                , num_coef
#endif
            );
        }
        allocate_alf_corr_data(&(ctx->enc_alf->alf_corr_merged[comp_idx]), comp_idx
#if ALF_SHAPE
                            , num_coef
#endif
        );
    }
    for (n = 0; n < NO_VAR_BINS; n++)
    {
        ctx->enc_alf->y_merged[n] = (double *)malloc(num_coef * sizeof(double));
        ctx->enc_alf->E_merged[n] = (double **)malloc(num_coef * sizeof(double *));
        for (i = 0; i < num_coef; i++)
        {
            ctx->enc_alf->E_merged[n][i] = (double *)malloc(num_coef * sizeof(double));
            memset(ctx->enc_alf->E_merged[n][i], 0, num_coef * sizeof(double));
        }
    }
    ctx->enc_alf->E_temp = (double **)malloc(num_coef * sizeof(double *));
    for (n = 0; n < num_coef; n++)
    {
        ctx->enc_alf->E_temp[n] = (double *)malloc(num_coef * sizeof(double));
        memset(ctx->enc_alf->E_temp[n], 0, num_coef * sizeof(double));
    }
    ctx->enc_alf->alf_picture_param = (ALF_PARAM **)malloc(N_C * sizeof(ALF_PARAM *));
    for (comp_idx = 0; comp_idx < N_C; comp_idx++)
    {
        allocate_alf_param(&(ctx->enc_alf->alf_picture_param[comp_idx]), comp_idx
#if ALF_SHAPE
                       , num_coef
#endif
        );
    }
    for (n = 0; n < NO_VAR_BINS; n++)
    {
        ctx->enc_alf->coeff_num_filter[n] = (int *)malloc(num_coef * sizeof(int));
        memset(&(ctx->enc_alf->coeff_num_filter[n][0]), 0, sizeof(int)*num_coef);
        ctx->enc_alf->coeff_num_filter[n][num_coef - 1] = (1 << ALF_NUM_BIT_SHIFT);
    }
    ctx->enc_alf->filter_coeff_sym = (int **)malloc(NO_VAR_BINS * sizeof(int *));
    for (g = 0; g < (int)NO_VAR_BINS; g++)
    {
#if ALF_SHAPE
        ctx->enc_alf->filter_coeff_sym[g] = (int *)malloc(num_coef * sizeof(int));
        memset(ctx->enc_alf->filter_coeff_sym[g], 0, num_coef * sizeof(int));
#else
        ctx->enc_alf->filter_coeff_sym[g] = (int *)malloc(ALF_MAX_NUM_COEF * sizeof(int));
        memset(ctx->enc_alf->filter_coeff_sym[g], 0, ALF_MAX_NUM_COEF * sizeof(int));
#endif
    }
    ctx->enc_alf->var_img = (pel **)malloc(img_height * sizeof(pel *));
    for (n = 0; n < img_height; n++)
    {
        ctx->enc_alf->var_img[n] = (pel *)malloc(img_width * sizeof(pel));
        memset(ctx->enc_alf->var_img[n], 0, img_width * sizeof(pel));
    }
    ctx->enc_alf->alf_lcu_enabled = (BOOL **)malloc(num_lcu_in_frame * sizeof(BOOL *));
    for (n = 0; n < num_lcu_in_frame; n++)
    {
        ctx->enc_alf->alf_lcu_enabled[n] = (BOOL *)malloc(N_C * sizeof(BOOL));
        memset(ctx->enc_alf->alf_lcu_enabled[n], 0, N_C * sizeof(BOOL));
    }
    for (i = 0; i < img_height; i = i + 4)
    {
        x_idx = (y_interval == 0) ? (3) : (Clip_post(3, i / y_interval));
        y_idx_offset = x_idx * 4;
        for (g = 0; g < img_width; g = g + 4)
        {
            y_idx = (x_interval == 0) ? (3) : (Clip_post(3, g / x_interval));
            ctx->enc_alf->var_img[i >> LOG2_VAR_SIZE_H][g >> LOG2_VAR_SIZE_W] = (pel)tbl_region[y_idx_offset + y_idx];
        }
    }
}

void destroy_alf_global_buffers(ENC_CTX* ctx, unsigned int max_size_in_bit)
{
    int lcu_height, lcu_width, img_height, img_width;
    int num_lcu_in_frame, num_lcu_in_pic_width, num_lcu_in_pic_height;
    int comp_idx, n, i, g;
#if ALF_SHAPE
    int num_coef = (ctx->info.sqh.adaptive_filter_shape_enable_flag) ? ALF_MAX_NUM_COEF_SHAPE2 : ALF_MAX_NUM_COEF;
#else
    int num_coef = (int)ALF_MAX_NUM_COEF;
#endif
    lcu_height = 1 << max_size_in_bit;
    lcu_width = lcu_height;
    img_height = ctx->info.pic_height;
    img_width = ctx->info.pic_width;
    num_lcu_in_pic_width = img_width / lcu_width;
    num_lcu_in_pic_height = img_height / lcu_height;
    num_lcu_in_pic_width += (img_width % lcu_width) ? 1 : 0;
    num_lcu_in_pic_height += (img_height % lcu_height) ? 1 : 0;
    num_lcu_in_frame = num_lcu_in_pic_height * num_lcu_in_pic_width;
    for (comp_idx = 0; comp_idx < N_C; comp_idx++)
    {
        for (n = 0; n < num_lcu_in_frame; n++)
        {
            free_alf_corr_data(ctx->enc_alf->alf_corr[comp_idx][n]
#if ALF_SHAPE
                            , num_coef
#endif
            );
            free_alf_corr_data(ctx->enc_alf->alf_non_skipped_corr[comp_idx][n]
#if ALF_SHAPE
                            , num_coef
#endif
            );
        }
        com_mfree(ctx->enc_alf->alf_corr[comp_idx]);
        ctx->enc_alf->alf_corr[comp_idx] = NULL;
        com_mfree(ctx->enc_alf->alf_non_skipped_corr[comp_idx]);
        ctx->enc_alf->alf_non_skipped_corr[comp_idx] = NULL;
        free_alf_corr_data(ctx->enc_alf->alf_corr_merged[comp_idx]
#if ALF_SHAPE
                        , num_coef
#endif
        );
        com_mfree(ctx->enc_alf->alf_corr_merged[comp_idx]);
    }
    for (n = 0; n < NO_VAR_BINS; n++)
    {
        com_mfree(ctx->enc_alf->y_merged[n]);
        ctx->enc_alf->y_merged[n] = NULL;
        for (i = 0; i < num_coef; i++)
        {
            com_mfree(ctx->enc_alf->E_merged[n][i]);
        }
        com_mfree(ctx->enc_alf->E_merged[n]);
        ctx->enc_alf->E_merged[n] = NULL;
    }
    for (i = 0; i < num_coef; i++)
    {
        com_mfree(ctx->enc_alf->E_temp[i]);
    }
    com_mfree(ctx->enc_alf->E_temp);
    ctx->enc_alf->E_temp = NULL;
    for (comp_idx = 0; comp_idx < N_C; comp_idx++)
    {
        free_alf_param(ctx->enc_alf->alf_picture_param[comp_idx], comp_idx);
    }
    com_mfree(ctx->enc_alf->alf_picture_param);

    for (n = 0; n < NO_VAR_BINS; n++)
    {
        com_mfree(ctx->enc_alf->coeff_num_filter[n]);
    }
    for (n = 0; n < img_height; n++)
    {
        com_mfree(ctx->enc_alf->var_img[n]);
    }
    com_mfree(ctx->enc_alf->var_img);
    ctx->enc_alf->var_img = NULL;
    for (n = 0; n < num_lcu_in_frame; n++)
    {
        com_mfree(ctx->enc_alf->alf_lcu_enabled[n]);
    }
    com_mfree(ctx->enc_alf->alf_lcu_enabled);
    ctx->enc_alf->alf_lcu_enabled = NULL;
    for (g = 0; g < (int)NO_VAR_BINS; g++)
    {
        com_mfree(ctx->enc_alf->filter_coeff_sym[g]);
    }
    com_mfree(ctx->enc_alf->filter_coeff_sym);
    ctx->enc_alf->filter_coeff_sym = NULL;
    com_mfree(ctx->enc_alf);
    ctx->enc_alf = NULL;
}

void allocate_alf_corr_data(ALF_CORR_DATA **dst, int cIdx
#if ALF_SHAPE
    , int num_coef
#endif
)
{
#if !ALF_SHAPE
    const int num_coef = ALF_MAX_NUM_COEF;
#endif
    const int max_num_groups = NO_VAR_BINS;
    int num_groups = (cIdx == Y_C) ? (max_num_groups) : (1);
    int i, j, g;
    (*dst) = (ALF_CORR_DATA *)malloc(sizeof(ALF_CORR_DATA));
    (*dst)->component_id = cIdx;
    (*dst)->E_corr = (double ***)malloc(num_groups * sizeof(double **));
    (*dst)->y_corr = (double **)malloc(num_groups * sizeof(double *));
    (*dst)->pix_acc = (double *)malloc(num_groups * sizeof(double));
    for (g = 0; g < num_groups; g++)
    {
        (*dst)->y_corr[g] = (double *)malloc(num_coef * sizeof(double));
        for (j = 0; j < num_coef; j++)
        {
            (*dst)->y_corr[g][j] = 0;
        }
        (*dst)->E_corr[g] = (double **)malloc(num_coef * sizeof(double *));
        for (i = 0; i < num_coef; i++)
        {
            (*dst)->E_corr[g][i] = (double *)malloc(num_coef * sizeof(double));
            for (j = 0; j < num_coef; j++)
            {
                (*dst)->E_corr[g][i][j] = 0;
            }
        }
        (*dst)->pix_acc[g] = 0;
    }
}

void free_alf_corr_data(ALF_CORR_DATA *dst
#if ALF_SHAPE
    , int num_coef
#endif
)
{
#if !ALF_SHAPE
    const int num_coef = ALF_MAX_NUM_COEF;
#endif
    const int max_num_groups = NO_VAR_BINS;
    int num_groups, i, g;
    if (dst->component_id >= 0)
    {
        num_groups = (dst->component_id == Y_C) ? (max_num_groups) : (1);
        for (g = 0; g < num_groups; g++)
        {
            for (i = 0; i < num_coef; i++)
            {
                com_mfree(dst->E_corr[g][i]);
            }
            com_mfree(dst->E_corr[g]);
            com_mfree(dst->y_corr[g]);
        }
        com_mfree(dst->E_corr);
        com_mfree(dst->y_corr);
        com_mfree(dst->pix_acc);
    }
}

void reset_alf_corr(ALF_CORR_DATA *alfCorr
#if ALF_SHAPE
    , int num_coef
#endif
)
{
    if (alfCorr->component_id >= 0)
    {
#if !ALF_SHAPE
        const int num_coef = ALF_MAX_NUM_COEF;
#endif
        int max_num_groups = NO_VAR_BINS;
        int g, j, i;
        int num_groups = (alfCorr->component_id == Y_C) ? (max_num_groups) : (1);
        for (g = 0; g < num_groups; g++)
        {
            alfCorr->pix_acc[g] = 0;
            for (j = 0; j < num_coef; j++)
            {
                alfCorr->y_corr[g][j] = 0;
                for (i = 0; i < num_coef; i++)
                {
                    alfCorr->E_corr[g][j][i] = 0;
                }
            }
        }
    }
}

/*
*************************************************************************
* Function: Calculate the correlation matrix for each LCU
* Input:
*  skipCUBoundaries  : Boundary skip flag
*           comp_idx : Image component index
*            lcu_idx : The address of current LCU
* (lcu_x_pos,lcu_y_pos)  : The LCU position
*          isXXAvail : The Available of neighboring LCU
*            pic_org : The original image buffer
*            pic_src : The distortion image buffer
*           stride : The width of image buffer
* Output:
* Return:
*************************************************************************
*/
void get_statistics_one_lcu_alf(ENC_ALF_VAR *enc_alf, int comp_idx
                         , int lcu_y_pos, int lcu_x_pos, int lcu_height, int lcu_width
                         , BOOL is_above_avail, BOOL is_below_avail, BOOL is_left_avail, BOOL is_right_avail
                         , BOOL is_above_left_avail, BOOL is_above_right_avail, ALF_CORR_DATA *alf_corr, pel *pic_org, pel *pic_src
                         , int stride, int format_shift
#if ALF_SHAPE
                         , BOOL alf_shape_flag
#endif
)
{
    int ypos, xpos, height, width;
    switch (comp_idx)
    {
    case U_C:
    case V_C:
        ypos = (lcu_y_pos >> format_shift);
        xpos = (lcu_x_pos >> format_shift);
        height = (lcu_height >> format_shift);
        width = (lcu_width >> format_shift);
        calc_corr_one_comp_region_chroma(pic_org, pic_src, stride, ypos, xpos, height, width, alf_corr->E_corr[0], alf_corr->y_corr[0],
                                  is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail
#if ALF_SHAPE
                                  , alf_shape_flag
#endif
        );
        break;
    case Y_C:
        ypos = (lcu_y_pos >> format_shift);
        xpos = (lcu_x_pos >> format_shift);
        height = (lcu_height >> format_shift);
        width = (lcu_width >> format_shift);
        calc_corr_one_comp_region_luma(enc_alf, pic_org, pic_src, stride, ypos, xpos, height, width, alf_corr->E_corr, alf_corr->y_corr,
                                  alf_corr->pix_acc, is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail
#if ALF_SHAPE
                                  , alf_shape_flag
#endif
        );
        break;
    default:
        printf("Not a legal component index for ALF\n");
        assert(0);
        exit(-1);
    }
}

/*
*************************************************************************
* Function: Calculate the correlation matrix for Luma
*************************************************************************
*/
void calc_corr_one_comp_region_luma(ENC_ALF_VAR *enc_alf, pel *img_org, pel *img_pad, int stride, int y_pos, int x_pos, int height, int width
                               , double ***E_corr, double **y_corr, double *pix_acc, int is_left_avail, int is_right_avail, int is_above_avail
                               , int is_below_avail, int is_above_left_avail, int is_above_right_avail
#if ALF_SHAPE
                               , BOOL alf_shape_flag
#endif
)
{
    int x_pos_end = x_pos + width;
#if ALF_SHAPE
    int N = alf_shape_flag ? ALF_MAX_NUM_COEF_SHAPE2 : ALF_MAX_NUM_COEF;
    int E_local[ALF_MAX_NUM_COEF_SHAPE2];
#else
    int N = ALF_MAX_NUM_COEF;
    int E_local[ALF_MAX_NUM_COEF];
#endif
    int start_pos_luma = is_above_avail ? (y_pos - 4) : y_pos;
    int end_pos_luma = is_below_avail ? (y_pos + height - 4) : (y_pos + height);
    int x_offset_left = is_left_avail ? -3 : 0;
    int x_offset_right = is_right_avail ? 3 : 0;
    int y_up, y_bottom;
    int x_left, x_right;
    pel *img_pad1, *img_pad2, *img_pad3, *img_pad4, *img_pad5, *img_pad6;
    int i, j, k, l, y_local, var_ind;
    double **E;
    double *yy;
    img_pad += start_pos_luma * stride;
    img_org += start_pos_luma * stride;
    var_ind = enc_alf->var_img[y_pos >> LOG2_VAR_SIZE_H][x_pos >> LOG2_VAR_SIZE_W];
    //loop region height
#if ALF_SHAPE
    if (!alf_shape_flag)
    {
#endif
    for (i = start_pos_luma; i < end_pos_luma; i++)
    {
#if ALF_DEC_OPT
        y_up = COM_MAX(start_pos_luma, i - 1);
        y_bottom = COM_MIN(end_pos_luma - 1, i + 1);
        img_pad1 = img_pad + (y_bottom - i) * stride;
        img_pad2 = img_pad + (y_up - i) * stride;
        y_up = COM_MAX(start_pos_luma, i - 2);
        y_bottom = COM_MIN(end_pos_luma - 1, i + 2);
        img_pad3 = img_pad + (y_bottom - i) * stride;
        img_pad4 = img_pad + (y_up - i) * stride;
        y_up = COM_MAX(start_pos_luma, i - 3);
        y_bottom = COM_MIN(end_pos_luma - 1, i + 3);
        img_pad5 = img_pad + (y_bottom - i) * stride;
        img_pad6 = img_pad + (y_up - i) * stride;
#else
        y_up = COM_CLIP3(start_pos_luma, end_pos_luma - 1, i - 1);
        y_bottom = COM_CLIP3(start_pos_luma, end_pos_luma - 1, i + 1);
        img_pad1 = img_pad + (y_bottom - i) * stride;
        img_pad2 = img_pad + (y_up - i) * stride;
        y_up = COM_CLIP3(start_pos_luma, end_pos_luma - 1, i - 2);
        y_bottom = COM_CLIP3(start_pos_luma, end_pos_luma - 1, i + 2);
        img_pad3 = img_pad + (y_bottom - i) * stride;
        img_pad4 = img_pad + (y_up - i) * stride;
        y_up = COM_CLIP3(start_pos_luma, end_pos_luma - 1, i - 3);
        y_bottom = COM_CLIP3(start_pos_luma, end_pos_luma - 1, i + 3);
        img_pad5 = img_pad + (y_bottom - i) * stride;
        img_pad6 = img_pad + (y_up - i) * stride;
#endif
        //loop current lcu_idx width
        for (j = x_pos; j < x_pos_end; j++)
        {
            memset(E_local, 0, N * sizeof(int));
            E_local[0] = (img_pad5[j] + img_pad6[j]);
            E_local[1] = (img_pad3[j] + img_pad4[j]);
            E_local[3] = (img_pad1[j] + img_pad2[j]);
            // upper left c2
#if ALF_DEC_OPT
            if (i >= y_pos + 3 && i <= end_pos_luma - 4)
            {
                x_left = COM_MAX(x_pos + x_offset_left, j - 1);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 1);
                E_local[4] = (img_pad2[x_right] + img_pad1[x_left]);
                E_local[2] = (img_pad2[x_left] + img_pad1[x_right]);
                E_local[7] = (img_pad[x_right] + img_pad[x_left]);
                x_left = COM_MAX(x_pos + x_offset_left, j - 2);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 2);
                E_local[6] = (img_pad[x_right] + img_pad[x_left]);
                x_left = COM_MAX(x_pos + x_offset_left, j - 3);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 3);
                E_local[5] = (img_pad[x_right] + img_pad[x_left]);
            }
            else
            {
#endif
            x_left = check_filtering_unit_boundary_extension(j - 1, i - 1, x_pos, y_pos, x_pos, start_pos_luma, x_pos_end - 1,
                    end_pos_luma - 1, is_above_left_avail, is_left_avail, is_above_right_avail, is_right_avail);
            E_local[2] = img_pad2[x_left];
            // upper right c4
            x_right = check_filtering_unit_boundary_extension(j + 1, i - 1, x_pos, y_pos, x_pos, start_pos_luma, x_pos_end - 1,
                     end_pos_luma - 1, is_above_left_avail, is_left_avail, is_above_right_avail, is_right_avail);
            E_local[4] = img_pad2[x_right];
            // lower left c4
            x_left = check_filtering_unit_boundary_extension(j - 1, i + 1, x_pos, y_pos, x_pos, start_pos_luma, x_pos_end - 1,
                    end_pos_luma - 1, is_above_left_avail, is_left_avail, is_above_right_avail, is_right_avail);
            E_local[4] += img_pad1[x_left];
            // lower right c2
            x_right = check_filtering_unit_boundary_extension(j + 1, i + 1, x_pos, y_pos, x_pos, start_pos_luma, x_pos_end - 1,
                     end_pos_luma - 1, is_above_left_avail, is_left_avail, is_above_right_avail, is_right_avail);
            E_local[2] += img_pad1[x_right];
#if ALF_DEC_OPT
            x_left = COM_MAX(x_pos + x_offset_left, j - 1);
            x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 1);
            E_local[7] = (img_pad[x_right] + img_pad[x_left]);
            x_left = COM_MAX(x_pos + x_offset_left, j - 2);
            x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 2);
            E_local[6] = (img_pad[x_right] + img_pad[x_left]);
            x_left = COM_MAX(x_pos + x_offset_left, j - 3);
            x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 3);
            E_local[5] = (img_pad[x_right] + img_pad[x_left]);
            }
#else
            x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 1);
            x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 1);
            E_local[7] = (img_pad[x_right] + img_pad[x_left]);
            x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 2);
            x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 2);
            E_local[6] = (img_pad[x_right] + img_pad[x_left]);
            x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 3);
            x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 3);
            E_local[5] = (img_pad[x_right] + img_pad[x_left]);
#endif
            E_local[8] = (img_pad[j]);
            y_local = img_org[j];
            pix_acc[var_ind] += (y_local * y_local);
            E = E_corr[var_ind];
            yy = y_corr[var_ind];
            for (k = 0; k < N; k++)
            {
                for (l = k; l < N; l++)
                {
                    E[k][l] += (double)(E_local[k] * E_local[l]);
                }
                yy[k] += (double)(E_local[k] * y_local);
            }
        }
        img_pad += stride;
        img_org += stride;
    }
#if ALF_SHAPE
    }
    else
    {
        for (i = start_pos_luma; i < end_pos_luma; i++)
        {
#if ALF_DEC_OPT
            y_up = COM_MAX(start_pos_luma, i - 1);
            y_bottom = COM_MIN(end_pos_luma - 1, i + 1);
            img_pad1 = img_pad + (y_bottom - i) * stride;
            img_pad2 = img_pad + (y_up - i) * stride;
            y_up = COM_MAX(start_pos_luma, i - 2);
            y_bottom = COM_MIN(end_pos_luma - 1, i + 2);
            img_pad3 = img_pad + (y_bottom - i) * stride;
            img_pad4 = img_pad + (y_up - i) * stride;
            y_up = COM_MAX(start_pos_luma, i - 3);
            y_bottom = COM_MIN(end_pos_luma - 1, i + 3);
            img_pad5 = img_pad + (y_bottom - i) * stride;
            img_pad6 = img_pad + (y_up - i) * stride;
#else
            y_up = COM_CLIP3(start_pos_luma, end_pos_luma - 1, i - 1);
            y_bottom = COM_CLIP3(start_pos_luma, end_pos_luma - 1, i + 1);
            img_pad1 = img_pad + (y_bottom - i) * stride;
            img_pad2 = img_pad + (y_up - i) * stride;
            y_up = COM_CLIP3(start_pos_luma, end_pos_luma - 1, i - 2);
            y_bottom = COM_CLIP3(start_pos_luma, end_pos_luma - 1, i + 2);
            img_pad3 = img_pad + (y_bottom - i) * stride;
            img_pad4 = img_pad + (y_up - i) * stride;
            y_up = COM_CLIP3(start_pos_luma, end_pos_luma - 1, i - 3);
            y_bottom = COM_CLIP3(start_pos_luma, end_pos_luma - 1, i + 3);
            img_pad5 = img_pad + (y_bottom - i) * stride;
            img_pad6 = img_pad + (y_up - i) * stride;
#endif
            //loop current lcu_idx width
            for (j = x_pos; j < x_pos_end; j++)
            {
                memset(E_local, 0, N * sizeof(int));
                E_local[0] = (img_pad5[j] + img_pad6[j]);
                E_local[3] = (img_pad3[j] + img_pad4[j]);
                E_local[8] = (img_pad1[j] + img_pad2[j]);
#if ALF_DEC_OPT
                if (i >= y_pos + 3 && i <= end_pos_luma - 4)
                {
                    x_left = COM_MAX(x_pos + x_offset_left, j - 1);
                    x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 1);
                    E_local[4] = (img_pad4[x_right] + img_pad3[x_left]);
                    E_local[2] = (img_pad4[x_left] + img_pad3[x_right]);
                    E_local[7] = (img_pad2[x_left] + img_pad1[x_right]);
                    E_local[9] = (img_pad2[x_right] + img_pad1[x_left]);
                    E_local[13] = (img_pad[x_right] + img_pad[x_left]);
                    x_left = COM_MAX(x_pos + x_offset_left, j - 2);
                    x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 2);
                    E_local[5] = (img_pad4[x_right] + img_pad3[x_left]);
                    E_local[1] = (img_pad4[x_left] + img_pad3[x_right]);
                    E_local[6] = (img_pad2[x_left] + img_pad1[x_right]);
                    E_local[10] = (img_pad2[x_right] + img_pad1[x_left]);
                    E_local[12] = (img_pad[x_right] + img_pad[x_left]);
                    x_left = COM_MAX(x_pos + x_offset_left, j - 3);
                    x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 3);
                    E_local[11] = (img_pad[x_right] + img_pad[x_left]);
                }
                else
                {
#endif
                // upper left c2
                x_left = check_filtering_unit_boundary_extension(j - 1, i - 1, x_pos, y_pos, x_pos, start_pos_luma, x_pos_end - 1,
                    end_pos_luma - 1, is_above_left_avail, is_left_avail, is_above_right_avail, is_right_avail);
                E_local[7] = img_pad2[x_left];
                // upper right c4
                x_right = check_filtering_unit_boundary_extension(j + 1, i - 1, x_pos, y_pos, x_pos, start_pos_luma, x_pos_end - 1,
                    end_pos_luma - 1, is_above_left_avail, is_left_avail, is_above_right_avail, is_right_avail);
                E_local[9] = img_pad2[x_right];
                // lower left c4
                x_left = check_filtering_unit_boundary_extension(j - 1, i + 1, x_pos, y_pos, x_pos, start_pos_luma, x_pos_end - 1,
                    end_pos_luma - 1, is_above_left_avail, is_left_avail, is_above_right_avail, is_right_avail);
                E_local[9] += img_pad1[x_left];
                // lower right c2
                x_right = check_filtering_unit_boundary_extension(j + 1, i + 1, x_pos, y_pos, x_pos, start_pos_luma, x_pos_end - 1,
                    end_pos_luma - 1, is_above_left_avail, is_left_avail, is_above_right_avail, is_right_avail);
                E_local[7] += img_pad1[x_right];
#if ALF_DEC_OPT
                x_left = COM_MAX(x_pos + x_offset_left, j - 1);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 1);
#else
                x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 1);
                x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 1);
#endif
                E_local[4] = img_pad4[x_right];
                E_local[4] += img_pad3[x_left];
                E_local[2] = img_pad4[x_left];
                E_local[2] += img_pad3[x_right];
                E_local[13] = (img_pad[x_right] + img_pad[x_left]);
#if ALF_DEC_OPT
                x_left = COM_MAX(x_pos + x_offset_left, j - 2);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 2);
#else
                x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 2);
                x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 2);
#endif
                E_local[6] = img_pad2[x_left];
                E_local[6] += img_pad1[x_right];
                E_local[5] = img_pad4[x_right];
                E_local[5] += img_pad3[x_left];
                E_local[1] = img_pad4[x_left];
                E_local[1] += img_pad3[x_right];
                E_local[10] = img_pad2[x_right];
                E_local[10] += img_pad1[x_left];
                E_local[12] = (img_pad[x_right] + img_pad[x_left]);
#if ALF_DEC_OPT
                x_left = COM_MAX(x_pos + x_offset_left, j - 3);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 3);
#else
                x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 3);
                x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 3);
#endif
                E_local[11] = (img_pad[x_right] + img_pad[x_left]);
#if ALF_DEC_OPT
                }
#endif
                E_local[14] = (img_pad[j]);
                y_local = img_org[j];
                pix_acc[var_ind] += (y_local * y_local);
                E = E_corr[var_ind];
                yy = y_corr[var_ind];
                for (k = 0; k < N; k++)
                {
                    for (l = k; l < N; l++)
                    {
                        E[k][l] += (double)(E_local[k] * E_local[l]);
                    }
                    yy[k] += (double)(E_local[k] * y_local);
                }
            }
            img_pad += stride;
            img_org += stride;
        }
    }
#endif
    for (var_ind = 0; var_ind < NO_VAR_BINS; var_ind++)
    {
        E = E_corr[var_ind];
        for (k = 1; k < N; k++)
        {
            for (l = 0; l < k; l++)
            {
                E[k][l] = E[l][k];
            }
        }
    }
}


/*
*************************************************************************
* Function: Calculate the correlation matrix for Chroma
*************************************************************************
*/
void calc_corr_one_comp_region_chroma(pel *img_org, pel *img_pad, int stride, int y_pos, int x_pos, int height, int width,
                               double **E_corr, double *y_corr, int is_left_avail, int is_right_avail, int is_above_avail, int is_below_avail,
                               int is_above_left_avail, int is_above_right_avail
#if ALF_SHAPE
                               , BOOL alf_shape_flag
#endif
)
{
    int x_pos_end = x_pos + width;
#if ALF_SHAPE
    int N = alf_shape_flag ? ALF_MAX_NUM_COEF_SHAPE2 : ALF_MAX_NUM_COEF;
    int E_local[ALF_MAX_NUM_COEF_SHAPE2];
#else
    int N = ALF_MAX_NUM_COEF;
    int E_local[ALF_MAX_NUM_COEF];
#endif
    int start_pos_chroma = is_above_avail ? (y_pos - 4) : y_pos;
    int end_pos_chroma = is_below_avail ? (y_pos + height - 4) : (y_pos + height);
    int x_offset_left = is_left_avail ? -3 : 0;
    int x_offset_right = is_right_avail ? 3 : 0;
    int y_up, y_bottom;
    int x_left, x_right;
    pel *img_pad1, *img_pad2, *img_pad3, *img_pad4, *img_pad5, *img_pad6;
    int i, j, k, l, y_local;
    img_pad += start_pos_chroma * stride;
    img_org += start_pos_chroma * stride;
#if ALF_SHAPE
    if (!alf_shape_flag)
    {
#endif
    for (i = start_pos_chroma; i < end_pos_chroma; i++)
    {
#if ALF_DEC_OPT
        y_up = COM_MAX(start_pos_chroma, i - 1);
        y_bottom = COM_MIN(end_pos_chroma - 1, i + 1);
        img_pad1 = img_pad + (y_bottom - i) * stride;
        img_pad2 = img_pad + (y_up - i) * stride;
        y_up = COM_MAX(start_pos_chroma, i - 2);
        y_bottom = COM_MIN(end_pos_chroma - 1, i + 2);
        img_pad3 = img_pad + (y_bottom - i) * stride;
        img_pad4 = img_pad + (y_up - i) * stride;
        y_up = COM_MAX(start_pos_chroma, i - 3);
        y_bottom = COM_MIN(end_pos_chroma - 1, i + 3);
        img_pad5 = img_pad + (y_bottom - i) * stride;
        img_pad6 = img_pad + (y_up - i) * stride;
#else
        y_up = COM_CLIP3(start_pos_chroma, end_pos_chroma - 1, i - 1);
        y_bottom = COM_CLIP3(start_pos_chroma, end_pos_chroma - 1, i + 1);
        img_pad1 = img_pad + (y_bottom - i) * stride;
        img_pad2 = img_pad + (y_up - i) * stride;
        y_up = COM_CLIP3(start_pos_chroma, end_pos_chroma - 1, i - 2);
        y_bottom = COM_CLIP3(start_pos_chroma, end_pos_chroma - 1, i + 2);
        img_pad3 = img_pad + (y_bottom - i) * stride;
        img_pad4 = img_pad + (y_up - i) * stride;
        y_up = COM_CLIP3(start_pos_chroma, end_pos_chroma - 1, i - 3);
        y_bottom = COM_CLIP3(start_pos_chroma, end_pos_chroma - 1, i + 3);
        img_pad5 = img_pad + (y_bottom - i) * stride;
        img_pad6 = img_pad + (y_up - i) * stride;
#endif
        for (j = x_pos; j < x_pos_end; j++)
        {
            memset(E_local, 0, N * sizeof(int));
            E_local[0] = (img_pad5[j] + img_pad6[j]);
            E_local[1] = (img_pad3[j] + img_pad4[j]);
            E_local[3] = (img_pad1[j] + img_pad2[j]);
#if ALF_DEC_OPT
            if (i >= y_pos + 3 && i <= end_pos_chroma - 4)
            {
                x_left = COM_MAX(x_pos + x_offset_left, j - 1);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 1);
                E_local[4] = (img_pad2[x_right] + img_pad1[x_left]);
                E_local[2] = (img_pad2[x_left] + img_pad1[x_right]);
                E_local[7] = (img_pad[x_right] + img_pad[x_left]);
                x_left = COM_MAX(x_pos + x_offset_left, j - 2);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 2);
                E_local[6] = (img_pad[x_right] + img_pad[x_left]);
                x_left = COM_MAX(x_pos + x_offset_left, j - 3);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 3);
                E_local[5] = (img_pad[x_right] + img_pad[x_left]);
            }
            else
            {
#endif
                // upper left c2
                x_left = check_filtering_unit_boundary_extension(j - 1, i - 1, x_pos, y_pos, x_pos, start_pos_chroma, x_pos_end - 1,
                    end_pos_chroma - 1, is_above_left_avail, is_left_avail, is_above_right_avail, is_right_avail);
                E_local[2] = img_pad2[x_left];
                // upper right c4
                x_right = check_filtering_unit_boundary_extension(j + 1, i - 1, x_pos, y_pos, x_pos, start_pos_chroma, x_pos_end - 1,
                    end_pos_chroma - 1, is_above_left_avail, is_left_avail, is_above_right_avail, is_right_avail);
                E_local[4] = img_pad2[x_right];
                // lower left c4
                x_left = check_filtering_unit_boundary_extension(j - 1, i + 1, x_pos, y_pos, x_pos, start_pos_chroma, x_pos_end - 1,
                    end_pos_chroma - 1, is_above_left_avail, is_left_avail, is_above_right_avail, is_right_avail);
                E_local[4] += img_pad1[x_left];
                // lower right c2
                x_right = check_filtering_unit_boundary_extension(j + 1, i + 1, x_pos, y_pos, x_pos, start_pos_chroma, x_pos_end - 1,
                    end_pos_chroma - 1, is_above_left_avail, is_left_avail, is_above_right_avail, is_right_avail);
                E_local[2] += img_pad1[x_right];
#if ALF_DEC_OPT
                x_left = COM_MAX(x_pos + x_offset_left, j - 1);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 1);
                E_local[7] = (img_pad[x_right] + img_pad[x_left]);
                x_left = COM_MAX(x_pos + x_offset_left, j - 2);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 2);
                E_local[6] = (img_pad[x_right] + img_pad[x_left]);
                x_left = COM_MAX(x_pos + x_offset_left, j - 3);
                x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 3);
                E_local[5] = (img_pad[x_right] + img_pad[x_left]);
            }
#else
            x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 1);
            x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 1);
            E_local[7] = (img_pad[x_right] + img_pad[x_left]);
            x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 2);
            x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 2);
            E_local[6] = (img_pad[x_right] + img_pad[x_left]);
            x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 3);
            x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 3);
            E_local[5] = (img_pad[x_right] + img_pad[x_left]);
#endif
            E_local[8] = (img_pad[j]);
            y_local = (int)img_org[j];
            for (k = 0; k < N; k++)
            {
                E_corr[k][k] += E_local[k] * E_local[k];
                for (l = k + 1; l < N; l++)
                {
                    E_corr[k][l] += E_local[k] * E_local[l];
                }
                y_corr[k] += y_local * E_local[k];
            }
        }
        img_pad += stride;
        img_org += stride;
    }
#if ALF_SHAPE
    }
    else
    {
        for (i = start_pos_chroma; i < end_pos_chroma; i++)
        {
#if ALF_DEC_OPT
            y_up = COM_MAX(start_pos_chroma, i - 1);
            y_bottom = COM_MIN(end_pos_chroma - 1, i + 1);
            img_pad1 = img_pad + (y_bottom - i) * stride;
            img_pad2 = img_pad + (y_up - i) * stride;
            y_up = COM_MAX(start_pos_chroma, i - 2);
            y_bottom = COM_MIN(end_pos_chroma - 1, i + 2);
            img_pad3 = img_pad + (y_bottom - i) * stride;
            img_pad4 = img_pad + (y_up - i) * stride;
            y_up = COM_MAX(start_pos_chroma, i - 3);
            y_bottom = COM_MIN(end_pos_chroma - 1, i + 3);
            img_pad5 = img_pad + (y_bottom - i) * stride;
            img_pad6 = img_pad + (y_up - i) * stride;
#else
            y_up = COM_CLIP3(start_pos_chroma, end_pos_chroma - 1, i - 1);
            y_bottom = COM_CLIP3(start_pos_chroma, end_pos_chroma - 1, i + 1);
            img_pad1 = img_pad + (y_bottom - i) * stride;
            img_pad2 = img_pad + (y_up - i) * stride;
            y_up = COM_CLIP3(start_pos_chroma, end_pos_chroma - 1, i - 2);
            y_bottom = COM_CLIP3(start_pos_chroma, end_pos_chroma - 1, i + 2);
            img_pad3 = img_pad + (y_bottom - i) * stride;
            img_pad4 = img_pad + (y_up - i) * stride;
            y_up = COM_CLIP3(start_pos_chroma, end_pos_chroma - 1, i - 3);
            y_bottom = COM_CLIP3(start_pos_chroma, end_pos_chroma - 1, i + 3);
            img_pad5 = img_pad + (y_bottom - i) * stride;
            img_pad6 = img_pad + (y_up - i) * stride;
#endif
            for (j = x_pos; j < x_pos_end; j++)
            {
                memset(E_local, 0, N * sizeof(int));
                E_local[0] = (img_pad5[j] + img_pad6[j]);
                E_local[3] = (img_pad3[j] + img_pad4[j]);
                E_local[8] = (img_pad1[j] + img_pad2[j]);
#if ALF_DEC_OPT
                if (i >= y_pos + 3 && i <= end_pos_chroma - 4)
                {
                    x_left = COM_MAX(x_pos + x_offset_left, j - 1);
                    x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 1);
                    E_local[4] = (img_pad4[x_right] + img_pad3[x_left]);
                    E_local[2] = (img_pad4[x_left] + img_pad3[x_right]);
                    E_local[7] = (img_pad2[x_left] + img_pad1[x_right]);
                    E_local[9] = (img_pad2[x_right] + img_pad1[x_left]);
                    E_local[13] = (img_pad[x_right] + img_pad[x_left]);
                    x_left = COM_MAX(x_pos + x_offset_left, j - 2);
                    x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 2);
                    E_local[5] = (img_pad4[x_right] + img_pad3[x_left]);
                    E_local[1] = (img_pad4[x_left] + img_pad3[x_right]);
                    E_local[6] = (img_pad2[x_left] + img_pad1[x_right]);
                    E_local[10] = (img_pad2[x_right] + img_pad1[x_left]);
                    E_local[12] = (img_pad[x_right] + img_pad[x_left]);
                    x_left = COM_MAX(x_pos + x_offset_left, j - 3);
                    x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 3);
                    E_local[11] = (img_pad[x_right] + img_pad[x_left]);
                }
                else
                {
#endif
                    // upper left c2
                    x_left = check_filtering_unit_boundary_extension(j - 1, i - 1, x_pos, y_pos, x_pos, start_pos_chroma, x_pos_end - 1,
                        end_pos_chroma - 1, is_above_left_avail, is_left_avail, is_above_right_avail, is_right_avail);
                    E_local[7] = img_pad2[x_left];
                    // upper right c4
                    x_right = check_filtering_unit_boundary_extension(j + 1, i - 1, x_pos, y_pos, x_pos, start_pos_chroma, x_pos_end - 1,
                        end_pos_chroma - 1, is_above_left_avail, is_left_avail, is_above_right_avail, is_right_avail);
                    E_local[9] = img_pad2[x_right];
                    // lower left c4
                    x_left = check_filtering_unit_boundary_extension(j - 1, i + 1, x_pos, y_pos, x_pos, start_pos_chroma, x_pos_end - 1,
                        end_pos_chroma - 1, is_above_left_avail, is_left_avail, is_above_right_avail, is_right_avail);
                    E_local[9] += img_pad1[x_left];
                    // lower right c2
                    x_right = check_filtering_unit_boundary_extension(j + 1, i + 1, x_pos, y_pos, x_pos, start_pos_chroma, x_pos_end - 1,
                        end_pos_chroma - 1, is_above_left_avail, is_left_avail, is_above_right_avail, is_right_avail);
                    E_local[7] += img_pad1[x_right];
#if ALF_DEC_OPT
                    x_left = COM_MAX(x_pos + x_offset_left, j - 1);
                    x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 1);
#else
                    x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 1);
                    x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 1);
#endif
                    E_local[13] = (img_pad[x_right] + img_pad[x_left]);
                    E_local[4] = img_pad4[x_right] + img_pad3[x_left];
                    E_local[2] = img_pad4[x_left] + img_pad3[x_right];
#if ALF_DEC_OPT
                    x_left = COM_MAX(x_pos + x_offset_left, j - 2);
                    x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 2);
#else
                    x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 2);
                    x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 2);
#endif
                    E_local[5] = img_pad4[x_right] + img_pad3[x_left];
                    E_local[1] = img_pad4[x_left] + img_pad3[x_right];
                    E_local[6] = img_pad2[x_left];
                    E_local[6] += img_pad1[x_right];
                    E_local[10] = img_pad2[x_right];
                    E_local[10] += img_pad1[x_left];
                    E_local[12] = (img_pad[x_right] + img_pad[x_left]);
#if ALF_DEC_OPT
                    x_left = COM_MAX(x_pos + x_offset_left, j - 3);
                    x_right = COM_MIN(x_pos_end - 1 + x_offset_right, j + 3);
#else
                    x_left = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j - 3);
                    x_right = COM_CLIP3(x_pos + x_offset_left, x_pos_end - 1 + x_offset_right, j + 3);
#endif
                    E_local[11] = (img_pad[x_right] + img_pad[x_left]);
#if ALF_DEC_OPT
                }
#endif
                E_local[14] = (img_pad[j]);
                y_local = (int)img_org[j];
                for (k = 0; k < N; k++)
                {
                    E_corr[k][k] += E_local[k] * E_local[k];
                    for (l = k + 1; l < N; l++)
                    {
                        E_corr[k][l] += E_local[k] * E_local[l];
                    }
                    y_corr[k] += y_local * E_local[k];
                }
            }
            img_pad += stride;
            img_org += stride;
        }

    }
#endif
    for (j = 0; j < N - 1; j++)
    {
        for (i = j + 1; i < N; i++)
        {
            E_corr[i][j] = E_corr[j][i];
        }
    }
}


/*
*************************************************************************
* Function: ALF parameter selection
* Input:
*    alf_pic_param: The ALF parameter
*       lambda    : The lambda value in the ALF-RD decision
* Return:
*************************************************************************
*/
void set_cur_alf_param(ENC_CTX *ctx, COM_BSW* alf_bs_temp, ALF_PARAM **alf_pic_param, double lambda)
{
    ENC_ALF_VAR *enc_alf = ctx->enc_alf;
    ALF_CORR_DATA *** alf_lcu_corr = NULL;
    int comp_idx, i;
    ALF_CORR_DATA *alf_pic_corr[N_C];
    double min_cost, cost;
    ALF_PARAM *temp_alf_param[N_C];
    int pic_header_bits = 0;
#if ALF_SHAPE
    int num_coef = (ctx->info.sqh.adaptive_filter_shape_enable_flag) ? ALF_MAX_NUM_COEF_SHAPE2 : ALF_MAX_NUM_COEF;
#endif
    for (comp_idx = 0; comp_idx < N_C; comp_idx++)
    {
        allocate_alf_param(&(temp_alf_param[comp_idx]), comp_idx
#if ALF_SHAPE
                       , num_coef
#endif
        );
    }

    {
        // normal part
        min_cost = 1.7e+308; // max double
        for (comp_idx = 0; comp_idx < N_C; comp_idx++)
        {
            allocate_alf_corr_data(&(alf_pic_corr[comp_idx]), comp_idx
#if ALF_SHAPE
                                , num_coef
#endif
            );
        }
        accumulate_lcu_correlation(ctx, alf_pic_corr, enc_alf->alf_corr, TRUE);
        decide_alf_picture_param(enc_alf, temp_alf_param, alf_pic_corr, lambda);
        for (i = 0; i < ALF_REDESIGN_ITERATION; i++)
        {
            if (i != 0)
            {
                //redesign filter according to the last on off results
                accumulate_lcu_correlation(ctx, alf_pic_corr, enc_alf->alf_corr, FALSE);
                decide_alf_picture_param(enc_alf, temp_alf_param, alf_pic_corr, lambda);
            }
            //estimate cost
            cost = execute_pic_lcu_on_off_decision(ctx, alf_bs_temp, temp_alf_param, lambda, TRUE, enc_alf->alf_corr, NULL, NULL, NULL, NULL, NULL, NULL,
                                              0);
            pic_header_bits = estimate_alf_bitrate_in_pic_header(temp_alf_param);
            cost += (double)pic_header_bits * lambda;
            if (cost < min_cost)
            {
                min_cost = cost;
                for (comp_idx = 0; comp_idx < N_C; comp_idx++)
                {
                    copy_alf_param(alf_pic_param[comp_idx], temp_alf_param[comp_idx]
#if ALF_SHAPE
                                 , num_coef
#endif
                    );
                }
            }
        }
        for (comp_idx = 0; comp_idx < N_C; comp_idx++)
        {
            free_alf_corr_data(alf_pic_corr[comp_idx]
#if ALF_SHAPE
                            , num_coef
#endif
            );
            free(alf_pic_corr[comp_idx]);
        }
        alf_lcu_corr = enc_alf->alf_corr;
    }
    for (comp_idx = 0; comp_idx < N_C; comp_idx++)
    {
        free_alf_param(temp_alf_param[comp_idx], comp_idx);
    }
}

unsigned int uvlc_bit_estimate(int val)
{
    unsigned int length = 1;
    val++;
    assert(val);
    while (1 != val)
    {
        val >>= 1;
        length += 2;
    }
    return ((length >> 1) + ((length + 1) >> 1));
}

unsigned int svlc_bit_estimate(int val)
{
    return uvlc_bit_estimate((val <= 0) ? (-val << 1) : ((val << 1) - 1));
}

unsigned int filter_coeff_bit_estimate(int *coeff
#if ALF_SHAPE
                                        , int num_coef
#endif
)
{
    unsigned int  bits = 0;
    int i;
#if ALF_SHAPE
    for (i = 0; i < (int)num_coef; i++)
#else
    for (i = 0; i < (int)ALF_MAX_NUM_COEF; i++)
#endif
    {
        bits += (svlc_bit_estimate(coeff[i]));
    }
    return bits;
}

unsigned int alf_param_bitrate_estimate(ALF_PARAM *alf_param)
{
    unsigned int  bits = 0; //alf enabled flag
    int num_filters, g;
    if (alf_param->alf_flag == 1)
    {
        if (alf_param->component_id == Y_C)
        {
            num_filters = alf_param->filters_per_group - 1;
            bits += uvlc_bit_estimate(num_filters);
            bits += (4 * num_filters);
        }
        for (g = 0; g < alf_param->filters_per_group; g++)
        {
            bits += filter_coeff_bit_estimate(alf_param->coeff_multi[g]
#if ALF_SHAPE
                                                  , alf_param->num_coeff
#endif
            );
        }
    }
    return bits;
}

unsigned int estimate_alf_bitrate_in_pic_header(ALF_PARAM **alf_pic_param)
{
    //CXCTBD please help to check if the implementation is consistent with syntax coding
    int comp_idx;
    unsigned int bits = 3; // pic_alf_enabled_flag[0,1,2]
    if (alf_pic_param[0]->alf_flag == 1 || alf_pic_param[1]->alf_flag == 1 || alf_pic_param[2]->alf_flag == 1)
    {
        for (comp_idx = 0; comp_idx < N_C; comp_idx++)
        {
            bits += alf_param_bitrate_estimate(alf_pic_param[comp_idx]);
        }
    }
    return bits;
}

long long estimate_filter_distortion(ENC_ALF_VAR *enc_alf, int comp_idx, ALF_CORR_DATA *alf_corr, int **coeff_set, int filter_set_size, int *tbl_merge, BOOL do_pix_acc_merge
#if ALF_SHAPE
                                   , int num_coef
#endif
)
{
#if !ALF_SHAPE
    int num_coef = (int)ALF_MAX_NUM_COEF;
#endif
    ALF_CORR_DATA *alf_merged = enc_alf->alf_corr_merged[comp_idx];
    int f;
    int **coeff = (coeff_set == NULL) ? (enc_alf->coeff_num_filter) : (coeff_set);
    long long dist = 0;
    alf_merge_from(alf_merged, alf_corr, tbl_merge, do_pix_acc_merge
#if ALF_SHAPE
              , num_coef
#endif
    );
    for (f = 0; f < filter_set_size; f++)
    {
        dist += fast_filter_dist_estimate(enc_alf, alf_merged->E_corr[f], alf_merged->y_corr[f], coeff[f], num_coef);
    }
    return dist;
}

long long fast_filter_dist_estimate(ENC_ALF_VAR *enc_alf, double **E, double *y, int *coeff, int filter_length)
{
    //static memory
#if ALF_SHAPE
    double coeff_double[ALF_MAX_NUM_COEF_SHAPE2];
#else
    double coeff_double[ALF_MAX_NUM_COEF];
#endif
    //variable
    int    i, j;
    long long  dist;
    double dist_double, sum;
    unsigned int shift;
    for (i = 0; i < filter_length; i++)
    {
        coeff_double[i] = (double)coeff[i] / (double)(1 << ((int)ALF_NUM_BIT_SHIFT));
    }
    dist_double = 0;
    for (i = 0; i < filter_length; i++)
    {
        sum = ((double)E[i][i]) * coeff_double[i];
#if ALF_SHAPE
        for (j = i + 1; j < COM_MIN(filter_length, ALF_MAX_NUM_COEF_SHAPE2); j++)
#else
        for (j = i + 1; j < filter_length; j++)
#endif
        {
            sum += (double)(2 * E[i][j]) * coeff_double[j];
        }
        dist_double += ((sum - 2.0 * y[i]) * coeff_double[i]);
    }
    shift = enc_alf->bit_increment << 1;
    if (dist_double < 0)
    {
        dist = -(((long long)(-dist_double + 0.5)) >> shift);
    }
    else   //dist_double >=0
    {
        dist = ((long long)(dist_double + 0.5)) >> shift;
    }
    return dist;
}

/*
*************************************************************************
* Function: correlation matrix merge
* Input:
*                pic_src: input correlation matrix
*         tbl_merge: merge table
* Output:
*                dst: output correlation matrix
* Return:
*************************************************************************
*/
void alf_merge_from(ALF_CORR_DATA *dst, ALF_CORR_DATA *src, int *tbl_merge, BOOL do_pix_acc_merge
#if ALF_SHAPE
               , int num_coef
#endif
)
{
#if !ALF_SHAPE
    int num_coef = ALF_MAX_NUM_COEF;
#endif
    double **src_E, **dst_E;
    double *src_y, *dst_y;
    int max_filter_set_size, j, i, var_ind, filter_idx;
    assert(dst->component_id == src->component_id);
    reset_alf_corr(dst
#if ALF_SHAPE
                  , num_coef
#endif
    );
    switch (dst->component_id)
    {
    case U_C:
    case V_C:
        src_E = src->E_corr[0];
        dst_E = dst->E_corr[0];
        src_y = src->y_corr[0];
        dst_y = dst->y_corr[0];
        for (j = 0; j < num_coef; j++)
        {
            for (i = 0; i < num_coef; i++)
            {
                dst_E[j][i] += src_E[j][i];
            }
            dst_y[j] += src_y[j];
        }
        if (do_pix_acc_merge)
        {
            dst->pix_acc[0] = src->pix_acc[0];
        }
        break;
    case Y_C:
        max_filter_set_size = (int)NO_VAR_BINS;
        for (var_ind = 0; var_ind < max_filter_set_size; var_ind++)
        {
            filter_idx = (tbl_merge == NULL) ? (0) : (tbl_merge[var_ind]);
            src_E = src->E_corr[var_ind];
            dst_E = dst->E_corr[filter_idx];
            src_y = src->y_corr[var_ind];
            dst_y = dst->y_corr[filter_idx];
            for (j = 0; j < num_coef; j++)
            {
                for (i = 0; i < num_coef; i++)
                {
                    dst_E[j][i] += src_E[j][i];
                }
                dst_y[j] += src_y[j];
            }
            if (do_pix_acc_merge)
            {
                dst->pix_acc[filter_idx] += src->pix_acc[var_ind];
            }
        }
        break;
    default:
        printf("not a legal component ID\n");
        assert(0);
        exit(-1);
    }
}

/*
*************************************************************************
* Function: ALF On/Off decision for LCU
*************************************************************************
*/
double execute_pic_lcu_on_off_decision(ENC_CTX *ctx, COM_BSW *alf_bs_temp, ALF_PARAM **alf_pic_param, double lambda, BOOL is_rdo_estimate, ALF_CORR_DATA *** alf_corr
                                  , pel *img_Y_org, pel **img_UV_org, pel *img_Y_dec, pel **img_UV_dec, pel *img_Y_rec, pel **img_UV_rec, int stride)
{
    ENC_ALF_VAR *enc_alf = ctx->enc_alf;
    ENC_CORE *core = ctx->core;
    ENC_SBAC *alf_sbac = GET_SBAC_ENC(alf_bs_temp);
    int bit_depth = ctx->info.bit_depth_internal;
    long long  dist_enc, dist_off;
    double  bits_enc, bits_off, cost_enc, cost_off, cost_alf_on, cost_alf_off;
    BOOL is_left_avail, is_right_avail, is_above_avail, is_below_avail;
    BOOL is_above_left_avail, is_above_right_avail;
    long long  dist_best_pic[N_C];
    double bits_best_pic[N_C];
    int comp_idx, lcu_idx, lcu_y_pos, lcu_x_pos, lcu_height, lcu_width, format_shift, n, stride_in, ctx_idx;
    pel *org = NULL;
    pel *dec = NULL;
    pel *rec = NULL;
    double lambda_luma, lambda_chroma;
    /////
    int img_height, img_width;
    int num_lcu_in_frame, num_lcu_in_pic_width, num_lcu_in_pic_height;
    double best_cost = 0;
    SBAC_LOAD((*alf_sbac), (core->s_alf_initial));
    SBAC_STORE((core->s_alf_cu_ctr), (*alf_sbac));
    lcu_height = 1 << ctx->info.log2_max_cuwh;
    lcu_width = lcu_height;
    img_height = ctx->info.pic_height;
    img_width = ctx->info.pic_width;
    num_lcu_in_pic_width = img_width / lcu_width;
    num_lcu_in_pic_height = img_height / lcu_height;
    num_lcu_in_pic_width += (img_width % lcu_width) ? 1 : 0;
    num_lcu_in_pic_height += (img_height % lcu_height) ? 1 : 0;
    num_lcu_in_frame = num_lcu_in_pic_height * num_lcu_in_pic_width;
    lambda_luma = lambda; //VKTBD lambda is not correct
    lambda_chroma = lambda_luma;
#if ALF_SHAPE
    int  num_coef = (ctx->info.sqh.adaptive_filter_shape_enable_flag) ? ALF_MAX_NUM_COEF_SHAPE2 : ALF_MAX_NUM_COEF;
#endif
    for (comp_idx = 0; comp_idx < N_C; comp_idx++)
    {
        dist_best_pic[comp_idx] = 0;
        bits_best_pic[comp_idx] = 0;
    }
    for (lcu_idx = 0; lcu_idx < num_lcu_in_frame; lcu_idx++)
    {
        //derive CTU width and height
        lcu_y_pos = (lcu_idx / num_lcu_in_pic_width) * lcu_height;
        lcu_x_pos = (lcu_idx % num_lcu_in_pic_width) * lcu_width;
        int cur_lcu_height = (lcu_y_pos + lcu_height > img_height) ? (img_height - lcu_y_pos) : lcu_height;
        int cur_lcu_width = (lcu_x_pos + lcu_width  > img_width) ? (img_width - lcu_x_pos) : lcu_width;
        //if the current CTU is the starting CTU at the slice, reset cabac
        if (lcu_idx > 0)
        {
            int prev_lcu_y_pos = ((lcu_idx - 1) / num_lcu_in_pic_width) * lcu_height;
            int prev_lcu_x_pos = ((lcu_idx - 1) % num_lcu_in_pic_width) * lcu_width;
            s8 *map_patch_idx = ctx->map.map_patch_idx;
            int curr_mb_nr = (lcu_y_pos >> MIN_CU_LOG2) * (img_width >> MIN_CU_LOG2) + (lcu_x_pos >> MIN_CU_LOG2);
            int prev_mb_nr = (prev_lcu_y_pos >> MIN_CU_LOG2) * (img_width >> MIN_CU_LOG2) + (prev_lcu_x_pos >> MIN_CU_LOG2);
            int curr_lcu_patch_idx = map_patch_idx[curr_mb_nr];
            int prev_lcu_patch_idx = map_patch_idx[prev_mb_nr];
            if (curr_lcu_patch_idx != prev_lcu_patch_idx)
                enc_sbac_init(alf_bs_temp); // init sbac for alf rdo
        }
        //derive CTU boundary availabilities
        derive_boundary_avail(ctx, num_lcu_in_pic_width, num_lcu_in_pic_height, lcu_idx, &is_left_avail, &is_right_avail, &is_above_avail,
                            &is_below_avail, &is_above_left_avail, &is_above_right_avail);
        for (comp_idx = 0; comp_idx < N_C; comp_idx++)
        {
            //if slice-level enabled flag is 0, set CTB-level enabled flag 0
            if (alf_pic_param[comp_idx]->alf_flag == 0)
            {
                enc_alf->alf_lcu_enabled[lcu_idx][comp_idx] = FALSE;
                continue;
            }
            if (!is_rdo_estimate)
            {
                format_shift = (comp_idx == Y_C) ? 0 : 1;
                org = (comp_idx == Y_C) ? img_Y_org : img_UV_org[comp_idx - U_C];
                dec = (comp_idx == Y_C) ? img_Y_dec : img_UV_dec[comp_idx - U_C];
                rec = (comp_idx == Y_C) ? img_Y_rec : img_UV_rec[comp_idx - U_C];
                stride_in = (comp_idx == Y_C) ? (stride) : (stride >> 1);
            }
            else
            {
                format_shift = 0;
                stride_in = 0;
                org = NULL;
                dec = NULL;
                rec = NULL;
            }

            //ALF on
            if (is_rdo_estimate)
            {
                reconstruct_coef_info(comp_idx, alf_pic_param[comp_idx], enc_alf->filter_coeff_sym, enc_alf->var_ind_tab);
                //dist_enc is the estimated distortion reduction compared with filter-off case
                dist_enc = estimate_filter_distortion(enc_alf, comp_idx, alf_corr[comp_idx][lcu_idx], enc_alf->filter_coeff_sym,
                                                   alf_pic_param[comp_idx]->filters_per_group, enc_alf->var_ind_tab, FALSE
#if ALF_SHAPE
                                                   , num_coef
#endif
                )
                          - estimate_filter_distortion(enc_alf, comp_idx, alf_corr[comp_idx][lcu_idx], NULL, 1, NULL, FALSE
#if ALF_SHAPE
                                                     , num_coef
#endif
                          );
            }
            else
            {
                filter_one_ctb(enc_alf, rec, dec, stride_in, comp_idx, bit_depth, alf_pic_param[comp_idx]
                             , lcu_y_pos, cur_lcu_height, lcu_x_pos, cur_lcu_width, is_above_avail, is_below_avail, is_left_avail, is_right_avail, is_above_left_avail, is_above_right_avail
#if ALF_SHAPE
                             , (ctx->info.sqh.adaptive_filter_shape_enable_flag)
#endif
                );
                dist_enc  = calc_alf_lcu_dist(ctx, 0, comp_idx, lcu_idx, lcu_y_pos, lcu_x_pos, cur_lcu_height, cur_lcu_width, is_above_avail, org, rec, stride_in, format_shift);
                dist_enc -= calc_alf_lcu_dist(ctx, 0, comp_idx, lcu_idx, lcu_y_pos, lcu_x_pos, cur_lcu_height, cur_lcu_width, is_above_avail, org, dec,  stride_in, format_shift);
            }
            SBAC_LOAD((*alf_sbac), (core->s_alf_cu_ctr));
            ctx_idx = 0;
            bits_enc = enc_get_bit_number(alf_sbac);
            enc_eco_alf_lcu_ctrl(alf_sbac, alf_bs_temp, 1);
            bits_enc = enc_get_bit_number(alf_sbac) - bits_enc;
            cost_enc = (double)dist_enc + (comp_idx == 0 ? lambda_luma : lambda_chroma) * bits_enc;
            //ALF off
            dist_off = 0;
            //bits_off = 1;
            SBAC_LOAD((*alf_sbac), (core->s_alf_cu_ctr));
            ctx_idx = 0;
            bits_off = enc_get_bit_number(alf_sbac);
            enc_eco_alf_lcu_ctrl(alf_sbac, alf_bs_temp, 0);
            bits_off = enc_get_bit_number(alf_sbac) - bits_off;
            cost_off = (double)dist_off + (comp_idx == 0 ? lambda_luma : lambda_chroma) * bits_off;
            //set CTB-level on/off flag
            enc_alf->alf_lcu_enabled[lcu_idx][comp_idx] = (cost_enc < cost_off) ? TRUE : FALSE;
            if (!is_rdo_estimate && !enc_alf->alf_lcu_enabled[lcu_idx][comp_idx])
            {
                copy_one_alf_blk(rec, dec, stride_in, lcu_y_pos >> format_shift, lcu_x_pos >> format_shift, cur_lcu_height >> format_shift,
                              cur_lcu_width >> format_shift, is_above_avail, is_below_avail);
            }
            SBAC_LOAD((*alf_sbac), (core->s_alf_cu_ctr));
            ctx_idx = 0;
            bits_off = enc_get_bit_number(alf_sbac);
            enc_eco_alf_lcu_ctrl(alf_sbac, alf_bs_temp, (enc_alf->alf_lcu_enabled[lcu_idx][comp_idx] ? 1 : 0));
            bits_off = enc_get_bit_number(alf_sbac) - bits_off;
            SBAC_STORE((core->s_alf_cu_ctr), (*alf_sbac));
            bits_best_pic[comp_idx] += (enc_alf->alf_lcu_enabled[lcu_idx][comp_idx] ? bits_enc : bits_off);
            dist_best_pic[comp_idx] += (enc_alf->alf_lcu_enabled[lcu_idx][comp_idx] ? dist_enc : dist_off);
        } //CTB
    } //CTU
    
    //if (is_rdo_estimate || 1)
    {
        for (comp_idx = 0; comp_idx < N_C; comp_idx++)
        {
            if (alf_pic_param[comp_idx]->alf_flag == 1)
            {
                cost_alf_on = (double)dist_best_pic[comp_idx] + (comp_idx == 0 ? lambda_luma : lambda_chroma) * (bits_best_pic[comp_idx] +
                            (double)(alf_param_bitrate_estimate(alf_pic_param[comp_idx])));
                cost_alf_off = 0;
                if (cost_alf_on >= cost_alf_off)
                {
                    alf_pic_param[comp_idx]->alf_flag = 0;
                    for (n = 0; n < num_lcu_in_frame; n++)
                    {
                        enc_alf->alf_lcu_enabled[n][comp_idx] = FALSE;
                    }
                    if (!is_rdo_estimate)
                    {
                        format_shift = (comp_idx == Y_C) ? 0 : 1;
                        org = (comp_idx == Y_C) ? img_Y_org : img_UV_org[comp_idx - U_C];
                        dec = (comp_idx == Y_C) ? img_Y_dec : img_UV_dec[comp_idx - U_C];
                        rec = (comp_idx == Y_C) ? img_Y_rec : img_UV_rec[comp_idx - U_C];
                        stride_in = (comp_idx == Y_C) ? (stride) : (stride >> 1);
                        copy_to_image(rec, dec, stride_in, img_height, img_width, format_shift);
                    }
                }
            }
        }
    }
    best_cost = 0;
    for (comp_idx = 0; comp_idx < N_C; comp_idx++)
    {
        if (alf_pic_param[comp_idx]->alf_flag == 1)
        {
            best_cost += (double)dist_best_pic[comp_idx] + (comp_idx == 0 ? lambda_luma : lambda_chroma) * (bits_best_pic[comp_idx]);
        }
    }
    //return the block-level RD cost
    return best_cost;
}

/*
*************************************************************************
* Function: ALF filter on CTB
*************************************************************************
*/
void filter_one_ctb(ENC_ALF_VAR *enc_alf, pel *rec, pel *dec, int stride, int comp_idx, int bit_depth, ALF_PARAM *alf_param, int lcu_y_pos, int lcu_height,
                  int lcu_x_pos, int lcu_width, BOOL is_above_avail, BOOL is_below_avail, BOOL is_left_avail, BOOL is_right_avail, BOOL is_above_left_avail, BOOL is_above_right_avail
#if ALF_SHAPE
                  , BOOL alf_shape_flag
#endif
)
{
    //half size of 7x7cross+ 3x3square
    int format_shift = (comp_idx == Y_C) ? 0 : 1;
    int y_pos, height, x_pos, width;
    //reconstruct coefficients to filter_coeff_sym and var_ind_tab
    reconstruct_coef_info(comp_idx, alf_param, enc_alf->filter_coeff_sym,
                        enc_alf->var_ind_tab); //reconstruct ALF coefficients & related parameters
    //derive CTB start positions, width, and height. If the boundary is not available, skip boundary samples.
    y_pos  = (lcu_y_pos >> format_shift);
    height = (lcu_height >> format_shift);
    x_pos  = (lcu_x_pos >> format_shift);
    width  = (lcu_width >> format_shift);
    filter_one_comp_region(rec, dec, stride, (comp_idx != Y_C), y_pos, height, x_pos, width, enc_alf->filter_coeff_sym,
                        enc_alf->var_ind_tab, enc_alf->var_img, bit_depth, is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail
#if ALF_SHAPE
                        , alf_shape_flag
#endif
    );
}

long long calc_alf_lcu_dist(ENC_CTX *ctx, BOOL skip_lcu_boundary, int comp_idx, int lcu_idx, int lcu_y_pos, int lcu_x_pos, int cur_lcu_height,
                         int cur_lcu_width, BOOL is_above_avail, pel *pic_src, pel *pic_cmp, int stride, int format_shift)
{
    ENC_ALF_VAR *enc_alf = ctx->enc_alf;
    long long dist = 0;
    int  pos_offset, y_pos, x_pos, height, width;
    pel *ptr_cmp;
    pel *ptr_src;
    BOOL not_skip_lines_right_VB;
    BOOL not_skip_lines_below_VB = TRUE;
    int img_height, img_width;
    int num_lcu_in_frame, num_lcu_in_pic_width, num_lcu_in_pic_height;
    int lcu_height = 1 << ctx->info.log2_max_cuwh;
    int lcu_width = lcu_height;
    img_height = ctx->info.pic_height;
    img_width = ctx->info.pic_width;
    num_lcu_in_pic_width = img_width / lcu_width;
    num_lcu_in_pic_height = img_height / lcu_height;
    num_lcu_in_pic_width += (img_width % lcu_width) ? 1 : 0;
    num_lcu_in_pic_height += (img_height % lcu_height) ? 1 : 0;
    num_lcu_in_frame = num_lcu_in_pic_height * num_lcu_in_pic_width;
    if (skip_lcu_boundary)
    {
        if (lcu_idx + num_lcu_in_pic_width < num_lcu_in_frame)
        {
            not_skip_lines_below_VB = FALSE;
        }
    }
    not_skip_lines_right_VB = TRUE;
    if (skip_lcu_boundary)
    {
        if ((lcu_idx + 1) % num_lcu_in_pic_width != 0)
        {
            not_skip_lines_right_VB = FALSE;
        }
    }
    switch (comp_idx)
    {
    case U_C:
    case V_C:
        y_pos = (lcu_y_pos >> format_shift);
        x_pos = (lcu_x_pos >> format_shift);
        height = (cur_lcu_height >> format_shift);
        width = (cur_lcu_width >> format_shift);
        if (!not_skip_lines_below_VB)
        {
            height = height - (int)(DF_CHANGED_SIZE / 2) - (int)(ALF_FOOTPRINT_SIZE / 2);
        }
        if (!not_skip_lines_right_VB)
        {
            width = width - (int)(DF_CHANGED_SIZE / 2) - (int)(ALF_FOOTPRINT_SIZE / 2);
        }
        if (is_above_avail)
        {
            pos_offset = ((y_pos - 4) * stride) + x_pos;
        }
        else
        {
            pos_offset = (y_pos * stride) + x_pos;
        }
        ptr_cmp = pic_cmp + pos_offset;
        ptr_src = pic_src + pos_offset;
        dist += calc_ssd(enc_alf, ptr_src, ptr_cmp, width, height, stride);
        break;
    case Y_C:
        y_pos = (lcu_y_pos >> format_shift);
        x_pos = (lcu_x_pos >> format_shift);
        height = (cur_lcu_height >> format_shift);
        width = (cur_lcu_width >> format_shift);
        if (!not_skip_lines_below_VB)
        {
            height = height - (int)(DF_CHANGED_SIZE)-(int)(ALF_FOOTPRINT_SIZE / 2);
        }
        if (!not_skip_lines_right_VB)
        {
            width = width - (int)(DF_CHANGED_SIZE)-(int)(ALF_FOOTPRINT_SIZE / 2);
        }
        pos_offset = (y_pos * stride) + x_pos;
        ptr_cmp = pic_cmp + pos_offset;
        ptr_src = pic_src + pos_offset;
        dist += calc_ssd(enc_alf, ptr_src, ptr_cmp, width, height, stride);
        break;
    default:
        printf("not a legal component ID for ALF \n");
        assert(0);
        exit(-1);
    }
    return dist;
}

void copy_one_alf_blk(pel *pic_dst, pel *pic_src, int stride, int y_pos, int x_pos, int height, int width, int is_above_avail, int is_below_avail)
{
    int pos_offset = (y_pos * stride) + x_pos;
    pel *dst;
    pel *src;
    int j;
    int start_pos = is_above_avail ? (y_pos - 4) : y_pos;
    int end_pos = is_below_avail ? (y_pos + height - 4) : y_pos + height;
    pos_offset = (start_pos * stride) + x_pos;
    dst = pic_dst + pos_offset;
    src = pic_src + pos_offset;
    for (j = start_pos; j < end_pos; j++)
    {
        memcpy(dst, src, sizeof(pel)*width);
        dst += stride;
        src += stride;
    }
}

void add_alf_corr_data(ALF_CORR_DATA *A, ALF_CORR_DATA *B, ALF_CORR_DATA *C
#if ALF_SHAPE
                     , int num_coef
#endif
)
{
#if !ALF_SHAPE
    int num_coef = ALF_MAX_NUM_COEF;
#endif
    int max_num_groups = NO_VAR_BINS;
    int num_groups;
    int g, j, i;
    if (A->component_id >= 0)
    {
        num_groups = (A->component_id == Y_C) ? (max_num_groups) : (1);
        for (g = 0; g < num_groups; g++)
        {
            C->pix_acc[g] = A->pix_acc[g] + B->pix_acc[g];
            for (j = 0; j < num_coef; j++)
            {
                C->y_corr[g][j] = A->y_corr[g][j] + B->y_corr[g][j];
                for (i = 0; i < num_coef; i++)
                {
                    C->E_corr[g][j][i] = A->E_corr[g][j][i] + B->E_corr[g][j][i];
                }
            }
        }
    }
}

void accumulate_lcu_correlation(ENC_CTX *ctx, ALF_CORR_DATA **alf_corr_acc, ALF_CORR_DATA ***alf_corr_src_lcu, BOOL use_all_lcus)
{
#if ALF_SHAPE
    int num_coef = (ctx->info.sqh.adaptive_filter_shape_enable_flag) ? ALF_MAX_NUM_COEF_SHAPE2 : ALF_MAX_NUM_COEF;
#endif
    ENC_ALF_VAR *enc_alf = ctx->enc_alf;
    int comp_idx, num_stat_lcu, addr;
    ALF_CORR_DATA *alf_corr_acc_comp;
    int lcu_height, lcu_width, img_height, img_width;
    int num_lcu_in_frame, num_lcu_in_pic_width, num_lcu_in_pic_height;
    lcu_height = 1 << ctx->info.log2_max_cuwh;
    lcu_width = lcu_height;
    img_height = ctx->info.pic_height;
    img_width = ctx->info.pic_width;
    num_lcu_in_pic_width = img_width / lcu_width;
    num_lcu_in_pic_height = img_height / lcu_height;
    num_lcu_in_pic_width += (img_width % lcu_width) ? 1 : 0;
    num_lcu_in_pic_height += (img_height % lcu_height) ? 1 : 0;
    num_lcu_in_frame = num_lcu_in_pic_height * num_lcu_in_pic_width;
    for (comp_idx = 0; comp_idx < N_C; comp_idx++)
    {
        alf_corr_acc_comp = alf_corr_acc[comp_idx];
        reset_alf_corr(alf_corr_acc_comp
#if ALF_SHAPE
                      , num_coef
#endif
        );
        if (!use_all_lcus)
        {
            num_stat_lcu = 0;
            for (addr = 0; addr < num_lcu_in_frame; addr++)
            {
                if (enc_alf->alf_lcu_enabled[addr][comp_idx])
                {
                    num_stat_lcu++;
                    break;
                }
            }
            if (num_stat_lcu == 0)
            {
                use_all_lcus = TRUE;
            }
        }
        for (addr = 0; addr < (int)num_lcu_in_frame; addr++)
        {
            if (use_all_lcus || enc_alf->alf_lcu_enabled[addr][comp_idx])
            {
                add_alf_corr_data(alf_corr_src_lcu[comp_idx][addr], alf_corr_acc_comp, alf_corr_acc_comp
#if ALF_SHAPE
                                , num_coef
#endif
                );
            }
        }
    }
}

void decide_alf_picture_param(ENC_ALF_VAR *enc_alf, ALF_PARAM **alf_pic_param, ALF_CORR_DATA **alf_corr, double lambda_luma)
{
    double lambda_weight = 1.0;
    int comp_idx;
    double lambda;
    ALF_PARAM *alf_param;
    ALF_CORR_DATA *pic_corr;
    for (comp_idx = 0; comp_idx < N_C; comp_idx++)
    {
        //VKTBD chroma need different lambdas? lambda_weight needed?
        lambda = lambda_luma * lambda_weight;
        alf_param = alf_pic_param[comp_idx];
        pic_corr = alf_corr[comp_idx];
        alf_param->alf_flag = 1;
        derive_filter_info(enc_alf, comp_idx, pic_corr, alf_param, NO_VAR_BINS, lambda);
    }
}

void derive_filter_info(ENC_ALF_VAR *enc_alf, int comp_idx, ALF_CORR_DATA *alf_corr, ALF_PARAM *alf_param, int max_num_filter, double lambda)
{
#if ALF_SHAPE
    int num_coef = alf_param->num_coeff;
    double *coef = NULL;
#else
    int num_coef = ALF_MAX_NUM_COEF;
    double coef[ALF_MAX_NUM_COEF];
#endif
    int lambda_for_merge, num_filters;
    switch (comp_idx)
    {
    case Y_C:
        lambda_for_merge = ((int)lambda) * (1 << (2 * enc_alf->bit_increment));
        memset(enc_alf->var_ind_tab, 0, sizeof(int)*NO_VAR_BINS);
        find_best_filter_var_pred(enc_alf, alf_corr->y_corr, alf_corr->E_corr, alf_corr->pix_acc, enc_alf->filter_coeff_sym, &num_filters,
                               enc_alf->var_ind_tab, lambda_for_merge, max_num_filter
#if ALF_SHAPE
                               , num_coef
#endif
        );
        code_filter_coeff(enc_alf->filter_coeff_sym, enc_alf->var_ind_tab, num_filters, alf_param);
        break;
    case U_C:
    case V_C:
#if ALF_SHAPE
        coef = (double*)malloc(num_coef * sizeof(double));
#endif
        alf_param->filters_per_group = 1;
        gns_solve_by_Cholesky_decomp(alf_corr->E_corr[0], alf_corr->y_corr[0], coef, num_coef);
        quant_filter_coef(coef, enc_alf->filter_coeff_sym[0]
#if ALF_SHAPE
                         , num_coef
#endif
        );
        memcpy(alf_param->coeff_multi[0], enc_alf->filter_coeff_sym[0], sizeof(int)*num_coef);
        predict_alf_coeff(alf_param->coeff_multi, num_coef, alf_param->filters_per_group);
#if ALF_SHAPE
        free(coef);
#endif
        break;
    default:
        printf("Not a legal component ID\n");
        assert(0);
        exit(-1);
    }
}

void code_filter_coeff(int **filter_coeff, int *var_ind_tab, int num_filters, ALF_PARAM *alf_param)
{
    int filter_pattern[NO_VAR_BINS], start_second_filter = 0, i, g;
    memset(filter_pattern, 0, NO_VAR_BINS * sizeof(int));
#if !ALF_SHAPE
    alf_param->num_coef = (int)ALF_MAX_NUM_COEF;
#endif
    alf_param->filters_per_group = num_filters;
    //merge table assignment
    if (alf_param->filters_per_group > 1)
    {
        for (i = 1; i < NO_VAR_BINS; ++i)
        {
            if (var_ind_tab[i] != var_ind_tab[i - 1])
            {
                filter_pattern[i] = 1;
                start_second_filter = i;
            }
        }
    }
    memcpy(alf_param->filter_pattern, filter_pattern, NO_VAR_BINS * sizeof(int));
    //coefficient prediction
    for (g = 0; g < alf_param->filters_per_group; g++)
    {
        for (i = 0; i < alf_param->num_coeff; i++)
        {
            alf_param->coeff_multi[g][i] = filter_coeff[g][i];
        }
    }
    predict_alf_coeff(alf_param->coeff_multi, alf_param->num_coeff, alf_param->filters_per_group);
}

void quant_filter_coef(double *h, int *quant_h
#if ALF_SHAPE
                      , int num_coef
#endif
)
{
    int i, N;
    int max_value, min_value;
    double dbl_total_gain;
    int total_gain, q_total_gain;
    int upper, lower;
    double *dh;
    int  *nc;
    const int    *filter_mag;
#if ALF_SHAPE
    N = num_coef;
    filter_mag = (N == ALF_MAX_NUM_COEF) ? tbl_weights_shape1_sym : tbl_weights_shape2_sym;
#else
    N = (int)ALF_MAX_NUM_COEF;
    filter_mag = tbl_weights_shape1_sym;
#endif
    dh = (double *)malloc(N * sizeof(double));
    nc = (int *)malloc(N * sizeof(int));
    max_value = (1 << (1 + ALF_NUM_BIT_SHIFT)) - 1;
    min_value = 0 - (1 << (1 + ALF_NUM_BIT_SHIFT));
    dbl_total_gain = 0.0;
    q_total_gain = 0;
    for (i = 0; i < N; i++)
    {
        if (h[i] >= 0.0)
        {
            quant_h[i] = (int)(h[i] * (1 << ALF_NUM_BIT_SHIFT) + 0.5);
        }
        else
        {
            quant_h[i] = -(int)(-h[i] * (1 << ALF_NUM_BIT_SHIFT) + 0.5);
        }
        dh[i] = (double)quant_h[i] / (double)(1 << ALF_NUM_BIT_SHIFT) - h[i];
        dh[i] *= filter_mag[i];
        dbl_total_gain += h[i] * filter_mag[i];
        q_total_gain += quant_h[i] * filter_mag[i];
        nc[i] = i;
    }
    // modification of quantized filter coefficients
    total_gain = (int)(dbl_total_gain * (1 << ALF_NUM_BIT_SHIFT) + 0.5);
    if (q_total_gain != total_gain)
    {
        filter_coef_quick_sort(dh, nc, 0, N - 1);
        if (q_total_gain > total_gain)
        {
            upper = N - 1;
            while (q_total_gain > total_gain + 1)
            {
                i = nc[upper % N];
                quant_h[i]--;
                q_total_gain -= filter_mag[i];
                upper--;
            }
            if (q_total_gain == total_gain + 1)
            {
                if (dh[N - 1] > 0)
                {
                    quant_h[N - 1]--;
                }
                else
                {
                    i = nc[upper % N];
                    quant_h[i]--;
                    quant_h[N - 1]++;
                }
            }
        }
        else if (q_total_gain < total_gain)
        {
            lower = 0;
            while (q_total_gain < total_gain - 1)
            {
                i = nc[lower % N];
                quant_h[i]++;
                q_total_gain += filter_mag[i];
                lower++;
            }
            if (q_total_gain == total_gain - 1)
            {
                if (dh[N - 1] < 0)
                {
                    quant_h[N - 1]++;
                }
                else
                {
                    i = nc[lower % N];
                    quant_h[i]++;
                    quant_h[N - 1]--;
                }
            }
        }
    }
    // set of filter coefficients
    for (i = 0; i < N; i++)
    {
        quant_h[i] = max(min_value, min(max_value, quant_h[i]));
    }
    check_filter_coeff_value(quant_h, N);
    free(dh);
    dh = NULL;
    free(nc);
    nc = NULL;
}

void filter_coef_quick_sort(double *coef_data, int *coef_num, int upper, int lower)
{
    double mid, tmp_data;
    int i, j, tmp_num;
    i = upper;
    j = lower;
    mid = coef_data[(lower + upper) >> 1];
    do
    {
        while (coef_data[i] < mid)
        {
            i++;
        }
        while (mid < coef_data[j])
        {
            j--;
        }
        if (i <= j)
        {
            tmp_data = coef_data[i];
            tmp_num = coef_num[i];
            coef_data[i] = coef_data[j];
            coef_num[i] = coef_num[j];
            coef_data[j] = tmp_data;
            coef_num[j] = tmp_num;
            i++;
            j--;
        }
    }
    while (i <= j);
    if (upper < j)
    {
        filter_coef_quick_sort(coef_data, coef_num, upper, j);
    }
    if (i < lower)
    {
        filter_coef_quick_sort(coef_data, coef_num, i, lower);
    }
}
void find_best_filter_var_pred(ENC_ALF_VAR *enc_alf, double **y_sym, double ***E_sym, double *pix_acc, int **filter_coeff_sym,
                            int *filters_per_fr_best, int var_ind_tab[], double lambda_val, int num_max_filters
#if ALF_SHAPE
                            , int num_Coeff
#endif
)
{
    static BOOL is_first = TRUE;
    static int *filter_coeff_sym_quant[NO_VAR_BINS];
    int filters_per_fr, first_filter, interval[NO_VAR_BINS][2], interval_best[NO_VAR_BINS][2];
    int i, g;
    double  lagrangian, lagrangian_min;
    int sqr_filter_length;
    int *weights;
    double error_force0_coeff_tab[NO_VAR_BINS][2];
    {
        for (g = 0; g < NO_VAR_BINS; g++)
        {
#if ALF_SHAPE
            filter_coeff_sym_quant[g] = (int *)malloc(num_Coeff * sizeof(int));
#else
            filter_coeff_sym_quant[g] = (int *)malloc(ALF_MAX_NUM_COEF * sizeof(int));
#endif
        }
        is_first = FALSE;
    }
#if ALF_SHAPE
    sqr_filter_length = (int)num_Coeff;
    weights = (ALF_MAX_NUM_COEF == num_Coeff)? tbl_weights_shape1_sym: tbl_weights_shape2_sym;
#else
    sqr_filter_length = (int)ALF_MAX_NUM_COEF;
    weights = tbl_weights_shape1_sym;
#endif
    // zero all variables
    memset(var_ind_tab, 0, sizeof(int)*NO_VAR_BINS);
    for (i = 0; i < NO_VAR_BINS; i++)
    {
#if ALF_SHAPE
        memset(filter_coeff_sym[i], 0, sizeof(int)* num_Coeff);
        memset(filter_coeff_sym_quant[i], 0, sizeof(int)* num_Coeff);
#else
        memset(filter_coeff_sym[i], 0, sizeof(int)*ALF_MAX_NUM_COEF);
        memset(filter_coeff_sym_quant[i], 0, sizeof(int)*ALF_MAX_NUM_COEF);
#endif
    }
    first_filter = 1;
    lagrangian_min = 0;
    filters_per_fr = NO_VAR_BINS;
    while (filters_per_fr >= 1)
    {
        merge_filters_greedy(enc_alf, y_sym, E_sym, pix_acc, interval, sqr_filter_length, filters_per_fr);
        find_filter_coeff(enc_alf, E_sym, y_sym, pix_acc, filter_coeff_sym, filter_coeff_sym_quant, interval,
                        var_ind_tab, sqr_filter_length, filters_per_fr, weights, error_force0_coeff_tab);
        lagrangian = find_best_coeff_cod_method(filter_coeff_sym_quant, sqr_filter_length, filters_per_fr,
                                             error_force0_coeff_tab, lambda_val);
        if (lagrangian < lagrangian_min || first_filter == 1 || filters_per_fr == num_max_filters)
        {
            first_filter = 0;
            lagrangian_min = lagrangian;
            (*filters_per_fr_best) = filters_per_fr;
            memcpy(interval_best, interval, NO_VAR_BINS * 2 * sizeof(int));
        }
        filters_per_fr--;
    }
    find_filter_coeff(enc_alf, E_sym, y_sym, pix_acc, filter_coeff_sym, filter_coeff_sym_quant, interval_best,
                    var_ind_tab, sqr_filter_length, (*filters_per_fr_best), weights, error_force0_coeff_tab);
    if (*filters_per_fr_best == 1)
    {
        memset(var_ind_tab, 0, sizeof(int)*NO_VAR_BINS);
    }
    for (g = 0; g < NO_VAR_BINS; g++)
    {
        free(filter_coeff_sym_quant[g]);
    }
}

double merge_filters_greedy(ENC_ALF_VAR *enc_alf, double **y_global_seq, double ***E_global_seq, double *pix_acc_global_seq,
                          int interval_best[NO_VAR_BINS][2], int sqr_filter_length, int num_intervals)
{
    int first, ind, ind1, ind2, i, j, best_to_merge;
    double error, error1, error2, error_min;
    static double pix_acc_temp, error_tab[NO_VAR_BINS], error_comb_tab[NO_VAR_BINS];
    static int index_list[NO_VAR_BINS], available[NO_VAR_BINS], num_remaining;
    if (num_intervals == NO_VAR_BINS)
    {
        num_remaining = NO_VAR_BINS;
        for (ind = 0; ind < NO_VAR_BINS; ind++)
        {
            index_list[ind] = ind;
            available[ind] = 1;
            enc_alf->pix_acc_merged[ind] = pix_acc_global_seq[ind];
            memcpy(enc_alf->y_merged[ind], y_global_seq[ind], sizeof(double)*sqr_filter_length);
            for (i = 0; i < sqr_filter_length; i++)
            {
                memcpy(enc_alf->E_merged[ind][i], E_global_seq[ind][i], sizeof(double)*sqr_filter_length);
            }
        }
    }
    // Try merging different matrices
    if (num_intervals == NO_VAR_BINS)
    {
        for (ind = 0; ind < NO_VAR_BINS; ind++)
        {
            error_tab[ind] = calculate_error_abs(enc_alf->E_merged[ind], enc_alf->y_merged[ind], enc_alf->pix_acc_merged[ind],
                                               sqr_filter_length);
        }
        for (ind = 0; ind < NO_VAR_BINS - 1; ind++)
        {
            ind1 = index_list[ind];
            ind2 = index_list[ind + 1];
            error1 = error_tab[ind1];
            error2 = error_tab[ind2];
            pix_acc_temp = enc_alf->pix_acc_merged[ind1] + enc_alf->pix_acc_merged[ind2];
            for (i = 0; i < sqr_filter_length; i++)
            {
                enc_alf->y_temp[i] = enc_alf->y_merged[ind1][i] + enc_alf->y_merged[ind2][i];
                for (j = 0; j < sqr_filter_length; j++)
                {
                    enc_alf->E_temp[i][j] = enc_alf->E_merged[ind1][i][j] + enc_alf->E_merged[ind2][i][j];
                }
            }
            error_comb_tab[ind1] = calculate_error_abs(enc_alf->E_temp, enc_alf->y_temp, pix_acc_temp,
                                   sqr_filter_length) - error1 - error2;
        }
    }
    while (num_remaining > num_intervals)
    {
        error_min = 0;
        first = 1;
        best_to_merge = 0;
        for (ind = 0; ind < num_remaining - 1; ind++)
        {
            error = error_comb_tab[index_list[ind]];
            if ((error < error_min || first == 1))
            {
                error_min = error;
                best_to_merge = ind;
                first = 0;
            }
        }
        ind1 = index_list[best_to_merge];
        ind2 = index_list[best_to_merge + 1];
        enc_alf->pix_acc_merged[ind1] += enc_alf->pix_acc_merged[ind2];
        for (i = 0; i < sqr_filter_length; i++)
        {
            enc_alf->y_merged[ind1][i] += enc_alf->y_merged[ind2][i];
            for (j = 0; j < sqr_filter_length; j++)
            {
                enc_alf->E_merged[ind1][i][j] += enc_alf->E_merged[ind2][i][j];
            }
        }
        available[ind2] = 0;
        //update error tables
        error_tab[ind1] = error_comb_tab[ind1] + error_tab[ind1] + error_tab[ind2];
        if (index_list[best_to_merge] > 0)
        {
            ind1 = index_list[best_to_merge - 1];
            ind2 = index_list[best_to_merge];
            error1 = error_tab[ind1];
            error2 = error_tab[ind2];
            pix_acc_temp = enc_alf->pix_acc_merged[ind1] + enc_alf->pix_acc_merged[ind2];
            for (i = 0; i < sqr_filter_length; i++)
            {
                enc_alf->y_temp[i] = enc_alf->y_merged[ind1][i] + enc_alf->y_merged[ind2][i];
                for (j = 0; j < sqr_filter_length; j++)
                {
                    enc_alf->E_temp[i][j] = enc_alf->E_merged[ind1][i][j] + enc_alf->E_merged[ind2][i][j];
                }
            }
            error_comb_tab[ind1] = calculate_error_abs(enc_alf->E_temp, enc_alf->y_temp, pix_acc_temp,
                                   sqr_filter_length) - error1 - error2;
        }
        if (index_list[best_to_merge + 1] < NO_VAR_BINS - 1)
        {
            ind1 = index_list[best_to_merge];
            ind2 = index_list[best_to_merge + 2];
            error1 = error_tab[ind1];
            error2 = error_tab[ind2];
            pix_acc_temp = enc_alf->pix_acc_merged[ind1] + enc_alf->pix_acc_merged[ind2];
            for (i = 0; i < sqr_filter_length; i++)
            {
                enc_alf->y_temp[i] = enc_alf->y_merged[ind1][i] + enc_alf->y_merged[ind2][i];
                for (j = 0; j < sqr_filter_length; j++)
                {
                    enc_alf->E_temp[i][j] = enc_alf->E_merged[ind1][i][j] + enc_alf->E_merged[ind2][i][j];
                }
            }
            error_comb_tab[ind1] = calculate_error_abs(enc_alf->E_temp, enc_alf->y_temp, pix_acc_temp,
                                   sqr_filter_length) - error1 - error2;
        }
        ind = 0;
        for (i = 0; i < NO_VAR_BINS; i++)
        {
            if (available[i] == 1)
            {
                index_list[ind] = i;
                ind++;
            }
        }
        num_remaining--;
    }
    error_min = 0;
    for (ind = 0; ind < num_intervals; ind++)
    {
        error_min += error_tab[index_list[ind]];
    }
    for (ind = 0; ind < num_intervals - 1; ind++)
    {
        interval_best[ind][0] = index_list[ind];
        interval_best[ind][1] = index_list[ind + 1] - 1;
    }
    interval_best[num_intervals - 1][0] = index_list[num_intervals - 1];
    interval_best[num_intervals - 1][1] = NO_VAR_BINS - 1;
    return (error_min);
}

double calculate_error_abs(double **A, double *b, double y, int size)
{
    int i;
    double error, sum;
#if ALF_SHAPE
    double c[ALF_MAX_NUM_COEF_SHAPE2];
#else
    double c[ALF_MAX_NUM_COEF];
#endif
    gns_solve_by_Cholesky_decomp(A, b, c, size);
    sum = 0;
    for (i = 0; i < size; i++)
    {
        sum += c[i] * b[i];
    }
    error = y - sum;
    return error;
}

#if ALF_SHAPE
int gns_Cholesky_decomp(double **inp_matrix, double out_matrix[ALF_MAX_NUM_COEF_SHAPE2][ALF_MAX_NUM_COEF_SHAPE2], int num_eq)
#else
int gns_Cholesky_decomp(double **inp_matrix, double out_matrix[ALF_MAX_NUM_COEF][ALF_MAX_NUM_COEF], int num_eq)
#endif
{
    int i, j, k;     /* Looping Variables */
    double scale;    /* scaling factor for each row */
#if ALF_SHAPE
    double inv_diag[ALF_MAX_NUM_COEF_SHAPE2];  /* Vector of the inverse of diagonal entries of out_matrix */
#else
    double inv_diag[ALF_MAX_NUM_COEF];  /* Vector of the inverse of diagonal entries of out_matrix */
#endif
    //  Cholesky decomposition starts
    for (i = 0; i < num_eq; i++)
    {
        for (j = i; j < num_eq; j++)
        {
            /* Compute the scaling factor */
            scale = inp_matrix[i][j];
            if (i > 0)
            {
                for (k = i - 1; k >= 0; k--)
                {
                    scale -= out_matrix[k][j] * out_matrix[k][i];
                }
            }
            /* Compute i'th row of out_matrix */
            if (i == j)
            {
                if (scale <= REG_SQR)    // if(scale <= 0 )  /* If inp_matrix is singular */
                {
                    return 0;
                }
                else
                {
                    /* Normal operation */
                    inv_diag[i] = 1.0 / (out_matrix[i][i] = sqrt(scale));
                }
            }
            else
            {
                out_matrix[i][j] = scale * inv_diag[i]; /* Upper triangular part          */
                out_matrix[j][i] = 0.0;              /* Lower triangular part set to 0 */
            }
        }
    }
    return 1; /* Signal that Cholesky factorization is successfully performed */
}

#if ALF_SHAPE
void gns_transpose_back_substitution(double U[ALF_MAX_NUM_COEF_SHAPE2][ALF_MAX_NUM_COEF_SHAPE2], double rhs[], double x[], int order)
#else
void gns_transpose_back_substitution(double U[ALF_MAX_NUM_COEF][ALF_MAX_NUM_COEF], double rhs[], double x[], int order)
#endif
{
    int i, j;             /* Looping variables */
    double sum;           /* Holds backsubstitution from already handled rows */
    /* Backsubstitution starts */
    x[0] = rhs[0] / U[0][0];               /* First row of U'                   */
    for (i = 1; i < order; i++)
    {
        /* For the rows 1..order-1           */
        for (j = 0, sum = 0.0; j < i; j++)   /* Backsubst already solved unknowns */
        {
            sum += x[j] * U[j][i];
        }
        x[i] = (rhs[i] - sum) / U[i][i];       /* i'th component of solution vect.  */
    }
}

#if ALF_SHAPE
void gns_back_substitution(double R[ALF_MAX_NUM_COEF_SHAPE2][ALF_MAX_NUM_COEF_SHAPE2], double z[ALF_MAX_NUM_COEF_SHAPE2], int R_size, double A[ALF_MAX_NUM_COEF_SHAPE2])
#else
void gns_back_substitution(double R[ALF_MAX_NUM_COEF][ALF_MAX_NUM_COEF], double z[ALF_MAX_NUM_COEF], int R_size, double A[ALF_MAX_NUM_COEF])
#endif
{
    int i, j;
    double sum;
    R_size--;
    A[R_size] = z[R_size] / R[R_size][R_size];
    for (i = R_size - 1; i >= 0; i--)
    {
        for (j = i + 1, sum = 0.0; j <= R_size; j++)
        {
            sum += R[i][j] * A[j];
        }
        A[i] = (z[i] - sum) / R[i][i];
    }
}

int gns_solve_by_Cholesky_decomp(double **LHS, double *rhs, double *x, int num_eq)
{
#if ALF_SHAPE
    double aux[ALF_MAX_NUM_COEF_SHAPE2];     /* Auxiliary vector */
    double U[ALF_MAX_NUM_COEF_SHAPE2][ALF_MAX_NUM_COEF_SHAPE2];    /* Upper triangular Cholesky factor of LHS */
#else
    double aux[ALF_MAX_NUM_COEF];     /* Auxiliary vector */
    double U[ALF_MAX_NUM_COEF][ALF_MAX_NUM_COEF];    /* Upper triangular Cholesky factor of LHS */
#endif
    int  i, singular;          /* Looping variable */
    assert(num_eq > 0);
    /* The equation to be solved is LHSx = rhs */
    /* Compute upper triangular U such that U'*U = LHS */
    if (gns_Cholesky_decomp(LHS, U, num_eq))   /* If Cholesky decomposition has been successful */
    {
        singular = 1;
        /* Now, the equation is  U'*U*x = rhs, where U is upper triangular
        * Solve U'*aux = rhs for aux
        */
        gns_transpose_back_substitution(U, rhs, aux, num_eq);
        /* The equation is now U*x = aux, solve it for x (new motion coefficients) */
        gns_back_substitution(U, aux, num_eq, x);
    }
    else   /* LHS was singular */
    {
        singular = 0;
        /* Regularize LHS */
        for (i = 0; i < num_eq; i++)
        {
            LHS[i][i] += REG;
        }
        /* Compute upper triangular U such that U'*U = regularized LHS */
        singular = gns_Cholesky_decomp(LHS, U, num_eq);
        if (singular == 1)
        {
            /* Solve  U'*aux = rhs for aux */
            gns_transpose_back_substitution(U, rhs, aux, num_eq);
            /* Solve U*x = aux for x */
            gns_back_substitution(U, aux, num_eq, x);
        }
        else
        {
            x[0] = 1.0;
            for (i = 1; i < num_eq; i++)
            {
                x[i] = 0.0;
            }
        }
    }
    return singular;
}

double find_filter_coeff(ENC_ALF_VAR *enc_alf, double ***E_global_seq, double **y_global_seq, double *pix_acc_global_seq, int **filter_coeff_seq,
                       int **filter_coeff_quant_seq, int interval_best[NO_VAR_BINS][2], int var_ind_tab[NO_VAR_BINS], int sqr_filter_length,
                       int filters_per_fr, int *weights, double error_tab_force0_coeff[NO_VAR_BINS][2])
{
    static double pix_acc_temp;
    static BOOL is_first = TRUE;
    static int *filter_coeff_quant = NULL;
    static double *filter_coeff = NULL;
    double error;
    int k, filter_idx;
    {
#if ALF_SHAPE
        get_mem_1D_int(&filter_coeff_quant, ALF_MAX_NUM_COEF_SHAPE2);
        filter_coeff = (double *)malloc(ALF_MAX_NUM_COEF_SHAPE2 * sizeof(double));
#else
        get_mem_1D_int(&filter_coeff_quant, ALF_MAX_NUM_COEF);
        filter_coeff = (double *)malloc(ALF_MAX_NUM_COEF * sizeof(double));
#endif
        is_first = FALSE;
    }
    error = 0;
    for (filter_idx = 0; filter_idx < filters_per_fr; filter_idx++)
    {
        add_A(enc_alf->E_temp, E_global_seq, interval_best[filter_idx][0], interval_best[filter_idx][1], sqr_filter_length);
        add_b(enc_alf->y_temp, y_global_seq, interval_best[filter_idx][0], interval_best[filter_idx][1], sqr_filter_length);
        pix_acc_temp = 0;
        for (k = interval_best[filter_idx][0]; k <= interval_best[filter_idx][1]; k++)
        {
            pix_acc_temp += pix_acc_global_seq[k];
        }
        // Find coefficients
        error_tab_force0_coeff[filter_idx][1] = pix_acc_temp + quantize_integer_filter(filter_coeff, filter_coeff_quant, enc_alf->E_temp,
                                         enc_alf->y_temp, sqr_filter_length, weights);
        error_tab_force0_coeff[filter_idx][0] = pix_acc_temp;
        error += error_tab_force0_coeff[filter_idx][1];
        for (k = 0; k < sqr_filter_length; k++)
        {
            filter_coeff_seq[filter_idx][k] = filter_coeff_quant[k];
            filter_coeff_quant_seq[filter_idx][k] = filter_coeff_quant[k];
        }
    }
    for (filter_idx = 0; filter_idx < filters_per_fr; filter_idx++)
    {
        for (k = interval_best[filter_idx][0]; k <= interval_best[filter_idx][1]; k++)
        {
            var_ind_tab[k] = filter_idx;
        }
    }
    free_mem_1D_int(filter_coeff_quant);
    free(filter_coeff);
    return (error);
}

void add_A(double **A_merged, double ***A, int start, int stop, int size)
{
    int i, j, ind;          /* Looping variable */
    for (i = 0; i < size; i++)
    {
        for (j = 0; j < size; j++)
        {
            A_merged[i][j] = 0;
            for (ind = start; ind <= stop; ind++)
            {
                A_merged[i][j] += A[ind][i][j];
            }
        }
    }
}

void add_b(double *b_merged, double **b, int start, int stop, int size)
{
    int i, ind;          /* Looping variable */
    for (i = 0; i < size; i++)
    {
        b_merged[i] = 0;
        for (ind = start; ind <= stop; ind++)
        {
            b_merged[i] += b[ind][i];
        }
    }
}
void round_filter_coeff(int *filter_coeff_quant, double *filter_coeff, int sqr_filter_length, int factor)
{
    int i;
    double diff;
    int diff_int, sign;
    for (i = 0; i < sqr_filter_length; i++)
    {
        sign = (filter_coeff[i] > 0) ? 1 : -1;
        diff = filter_coeff[i] * sign;
        diff_int = (int)(diff * (double)factor + 0.5);
        filter_coeff_quant[i] = diff_int * sign;
    }
}

double calculate_error_coeff_provided(double **A, double *b, double *c, int size)
{
    int i, j;
    double error, sum = 0;
    error = 0;
    for (i = 0; i < size; i++)   //diagonal
    {
        sum = 0;
        for (j = i + 1; j < size; j++)
        {
            sum += (A[j][i] + A[i][j]) * c[j];
        }
        error += (A[i][i] * c[i] + sum - 2 * b[i]) * c[i];
    }
    return error;
}

double quantize_integer_filter(double *filter_coeff, int *filter_coeff_quant, double **E, double *y, int sqr_filter_length, int *weights)
{
    double error;
    static BOOL is_first = TRUE;
    static int *filter_coeff_quant_mod = NULL;
    int factor = (1 << ((int)ALF_NUM_BIT_SHIFT));
    int i;
    int quant_coeff_sum, min_ind, target_coeff_sum_int, k, diff;
    double target_coeff_sum, error_min;
    {
#if ALF_SHAPE
        get_mem_1D_int(&filter_coeff_quant_mod, ALF_MAX_NUM_COEF_SHAPE2);
#else
        get_mem_1D_int(&filter_coeff_quant_mod, ALF_MAX_NUM_COEF);
#endif
        is_first = FALSE;
    }
    gns_solve_by_Cholesky_decomp(E, y, filter_coeff, sqr_filter_length);
    target_coeff_sum = 0;
    for (i = 0; i < sqr_filter_length; i++)
    {
        target_coeff_sum += (weights[i] * filter_coeff[i] * factor);
    }
    target_coeff_sum_int = ROUND(target_coeff_sum);
    round_filter_coeff(filter_coeff_quant, filter_coeff, sqr_filter_length, factor);
    quant_coeff_sum = 0;
    for (i = 0; i < sqr_filter_length; i++)
    {
        quant_coeff_sum += weights[i] * filter_coeff_quant[i];
    }
    while (quant_coeff_sum != target_coeff_sum_int)
    {
        if (quant_coeff_sum > target_coeff_sum_int)
        {
            diff = quant_coeff_sum - target_coeff_sum_int;
            error_min = 0;
            min_ind = -1;
            for (k = 0; k < sqr_filter_length; k++)
            {
                if (weights[k] <= diff)
                {
                    for (i = 0; i < sqr_filter_length; i++)
                    {
                        filter_coeff_quant_mod[i] = filter_coeff_quant[i];
                    }
                    filter_coeff_quant_mod[k]--;
                    for (i = 0; i < sqr_filter_length; i++)
                    {
                        filter_coeff[i] = (double)filter_coeff_quant_mod[i] / (double)factor;
                    }
                    error = calculate_error_coeff_provided(E, y, filter_coeff, sqr_filter_length);
                    if (error < error_min || min_ind == -1)
                    {
                        error_min = error;
                        min_ind = k;
                    }
                }
            }
            filter_coeff_quant[min_ind]--;
        }
        else
        {
            diff = target_coeff_sum_int - quant_coeff_sum;
            error_min = 0;
            min_ind = -1;
            for (k = 0; k < sqr_filter_length; k++)
            {
                if (weights[k] <= diff)
                {
                    for (i = 0; i < sqr_filter_length; i++)
                    {
                        filter_coeff_quant_mod[i] = filter_coeff_quant[i];
                    }
                    filter_coeff_quant_mod[k]++;
                    for (i = 0; i < sqr_filter_length; i++)
                    {
                        filter_coeff[i] = (double)filter_coeff_quant_mod[i] / (double)factor;
                    }
                    error = calculate_error_coeff_provided(E, y, filter_coeff, sqr_filter_length);
                    if (error < error_min || min_ind == -1)
                    {
                        error_min = error;
                        min_ind = k;
                    }
                }
            }
            filter_coeff_quant[min_ind]++;
        }
        quant_coeff_sum = 0;
        for (i = 0; i < sqr_filter_length; i++)
        {
            quant_coeff_sum += weights[i] * filter_coeff_quant[i];
        }
    }
    check_filter_coeff_value(filter_coeff_quant, sqr_filter_length);
    for (i = 0; i < sqr_filter_length; i++)
    {
        filter_coeff[i] = (double)filter_coeff_quant[i] / (double)factor;
    }
    error = calculate_error_coeff_provided(E, y, filter_coeff, sqr_filter_length);
    free_mem_1D_int(filter_coeff_quant_mod);
    return (error);
}

double find_best_coeff_cod_method(int **filter_coeff_sym_quant, int sqr_filter_length, int filters_per_fr,
                               double error_force0_coeff_tab[NO_VAR_BINS][2], double lambda)
{
    int coeff_bits, i;
    double error = 0, lagrangian;
    static BOOL  is_first = TRUE;
    static int **coeff_multi = NULL;
    int g;
    {
#if ALF_SHAPE
        get_mem_2D_int(&coeff_multi, NO_VAR_BINS, sqr_filter_length);
#else
        get_mem_2D_int(&coeff_multi, NO_VAR_BINS, ALF_MAX_NUM_COEF);
#endif
        is_first = FALSE;
    }
    for (g = 0; g < filters_per_fr; g++)
    {
        for (i = 0; i < sqr_filter_length; i++)
        {
            coeff_multi[g][i] = filter_coeff_sym_quant[g][i];
        }
    }
    predict_alf_coeff(coeff_multi, sqr_filter_length, filters_per_fr);
    coeff_bits = 0;
    for (g = 0; g < filters_per_fr; g++)
    {
        coeff_bits += filter_coeff_bit_estimate(coeff_multi[g]
#if ALF_SHAPE
                                                , sqr_filter_length
#endif
        );
    }
    for (i = 0; i < filters_per_fr; i++)
    {
        error += error_force0_coeff_tab[i][1];
    }
    lagrangian = error + lambda * coeff_bits;
    free_mem_2D_int(coeff_multi);
    return (lagrangian);
}

void predict_alf_coeff(int **coeff, int num_coef, int num_filters)
{
    int g, pred, sum, i;
    for (g = 0; g < num_filters; g++)
    {
        sum = 0;
        for (i = 0; i < num_coef - 1; i++)
        {
            sum += (2 * coeff[g][i]);
        }
        pred = (1 << ALF_NUM_BIT_SHIFT) - (sum);
        coeff[g][num_coef - 1] = coeff[g][num_coef - 1] - pred;
    }
}

long long calc_ssd(ENC_ALF_VAR *enc_alf, pel *org, pel *cmp, int width, int height, int stride)
{
    long long ssd = 0;
    int x, y;
    unsigned int shift = enc_alf->bit_increment << 1;
    int temp;
    for (y = 0; y < height; y++)
    {
        for (x = 0; x < width; x++)
        {
            temp = org[x] - cmp[x];
            ssd += (temp * temp) >> shift;
        }
        org += stride;
        cmp += stride;
    }
    return ssd;
}

void copy_to_image(pel *dst, pel *src, int stride_in, int img_height, int img_width, int format_shift)
{
    int j, width, height;
    pel *ptr_dst;
    pel *ptr_src;
    height = img_height >> format_shift;
    width = img_width >> format_shift;
    ptr_src = src;
    ptr_dst = dst;
    for (j = 0; j < height; j++)
    {
        memcpy(ptr_dst, ptr_src, width * sizeof(pel));
        ptr_dst = ptr_dst + stride_in;
        ptr_src = ptr_src + stride_in;
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
    (*alf_param)->num_coef = ALF_MAX_NUM_COEF;
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

void derive_boundary_avail(ENC_CTX *ctx, int num_lcu_in_pic_width, int num_lcu_in_pic_height, int lcu_idx, BOOL *is_left_avail,
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
    int  mb_above_left;
    int  mb_above_right;
    int  mb_curr_patch_idx, mb_neighbor_patch_idx;
    int  cross_patch_flag = ctx->info.sqh.cross_patch_loop_filter;
    pic_x = (lcu_idx % num_lcu_in_pic_width) * lcu_width;
    pic_y = (lcu_idx / num_lcu_in_pic_width) * lcu_height;

    mb_x = pic_x / MIN_CU_SIZE;
    mb_y = pic_y / MIN_CU_SIZE;
    mb_curr = mb_y * pic_mb_width + mb_x;
    *is_left_avail = (lcu_idx % num_lcu_in_pic_width != 0);
    *is_right_avail = (lcu_idx % num_lcu_in_pic_width != num_lcu_in_pic_width - 1);
    *is_above_avail = (lcu_idx >= num_lcu_in_pic_width);
    *is_below_avail = (lcu_idx  < num_lcu_in_frame - num_lcu_in_pic_width);
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