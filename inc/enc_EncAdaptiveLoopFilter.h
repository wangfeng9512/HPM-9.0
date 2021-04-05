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

#ifndef ENC_ENCADAPTIVELOOPFILTER_H
#define ENC_ENCADAPTIVELOOPFILTER_H
#include "com_def.h"
#include  "enc_def.h"
#include "com_ComAdaptiveLoopFilter.h"
#include <math.h>

void copy_to_image(pel *dst, pel *src, int stride_in, int img_height, int img_width, int format_shift);
void reset_alf_corr(ALF_CORR_DATA *alf_corr
#if ALF_SHAPE
    , int num_coeff
#endif
);
unsigned int uvlc_bit_estimate(int val);
void allocate_alf_corr_data(ALF_CORR_DATA **dst, int idx
#if ALF_SHAPE
    , int num_coeff
#endif
);
void free_alf_corr_data(ALF_CORR_DATA *dst
#if ALF_SHAPE
    , int num_coeff
#endif
);
void set_cur_alf_param(ENC_CTX *ctx, COM_BSW* alf_bs_temp, ALF_PARAM **alf_pic_param, double lambda);
void alf_process(ENC_CTX* ctx, COM_BSW *alf_bs_temp, ALF_PARAM **alf_picture_param, COM_PIC * pic_rec, COM_PIC * pic_alf_Org, COM_PIC * pic_alf_Rec, double lambda_mode);
void get_statistics_alf(ENC_CTX* ctx, pel *img_Y_org, pel **img_UV_org, pel *img_Y_dec, pel **img_UV_dec, int stride);
void derive_alf_boundary_availibility(ENC_CTX* ctx, int num_lcu_in_pic_width, int num_lcu_in_pic_height, int lcu_idx,
    BOOL *is_left_avail, BOOL *is_right_avail, BOOL *is_above_avail, BOOL *is_below_avail);

void derive_boundary_avail(ENC_CTX *ctx, int num_lcu_in_pic_width, int num_lcu_in_pic_height, int lcu_idx, BOOL *is_left_avail,
    BOOL *is_right_avail, BOOL *is_above_avail, BOOL *is_below_avail, BOOL *is_above_left_avail, BOOL *is_above_right_avail);

void create_alf_global_buffers(ENC_CTX* ctx);
void destroy_alf_global_buffers(ENC_CTX* ctx, unsigned int max_size_in_bit);
void get_statistics_one_lcu_alf(ENC_ALF_VAR *enc_alf, int comp_idx
    , int lcu_y_pos, int lcu_x_pos, int lcu_height, int lcu_width
    , BOOL is_above_avail, BOOL is_below_avail, BOOL is_left_avail, BOOL is_right_avail
    , BOOL is_above_left_avail, BOOL is_above_right_avail, ALF_CORR_DATA *alf_corr, pel *pic_org, pel *pic_src
    , int stride, int format_shift
#if ALF_SHAPE
    , BOOL alf_shape_flag
#endif
);
void calc_corr_one_comp_region_luma(ENC_ALF_VAR *enc_alf, pel *img_org, pel *img_pad, int stride, int y_pos, int x_pos, int height, int width
    , double ***E_corr, double **y_corr, double *pix_acc, int is_left_avail, int is_right_avail, int is_above_avail
    , int is_below_avail, int is_above_left_avail, int is_above_right_avail
#if ALF_SHAPE
    , BOOL alf_shape_flag
#endif
);
void calc_corr_one_comp_region_chroma(pel *img_org, pel *img_pad, int stride, int y_pos, int x_pos, int height, int width,
    double **E_corr, double *y_corr, int is_left_avail, int is_right_avail, int is_above_avail, int is_below_avail,
    int is_above_left_avail, int is_above_right_avail
#if ALF_SHAPE
    , BOOL alf_shape_flag
#endif
);

unsigned int estimate_alf_bitrate_in_pic_header(ALF_PARAM **alf_pic_param);
double execute_pic_lcu_on_off_decision(ENC_CTX *ctx, COM_BSW *alf_bs_temp, ALF_PARAM **alf_pic_param, double lambda, BOOL is_rdo_estimate, ALF_CORR_DATA *** alf_corr
    , pel *img_Y_org, pel **img_UV_org, pel *img_Y_dec, pel **img_UV_dec, pel *img_Y_rec, pel **img_UV_rec, int stride);
unsigned int alf_param_bitrate_estimate(ALF_PARAM *alfParam);
long long estimate_filter_distortion(ENC_ALF_VAR *enc_alf, int comp_idx, ALF_CORR_DATA *alf_corr, int **coeff_set, int filter_set_size, int *tbl_merge, BOOL do_pix_acc_merge
#if ALF_SHAPE
    , int num_coef
#endif
);
void alf_merge_from(ALF_CORR_DATA *dst, ALF_CORR_DATA *src, int *tbl_merge, BOOL do_pix_acc_merge
#if ALF_SHAPE
    , int num_coef
#endif
);
long long calc_ssd(ENC_ALF_VAR *enc_alf, pel *org, pel *cmp, int width, int height, int stride);
double find_filter_coeff(ENC_ALF_VAR *enc_alf, double ***E_global_seq, double **y_global_seq, double *pix_acc_global_seq, int **filter_coeff_seq,
    int **filter_coeff_quant_seq, int interval_best[NO_VAR_BINS][2], int var_ind_tab[NO_VAR_BINS], int sqr_filter_length,
    int filters_per_fr, int *weights, double error_tab_force0_coeff[NO_VAR_BINS][2]);
double find_best_coeff_cod_method(int **filter_coeff_sym_quant, int sqr_filter_length, int filters_per_fr,
    double error_force0_coeff_tab[NO_VAR_BINS][2], double lambda);
void filter_one_ctb(ENC_ALF_VAR *enc_alf, pel *rec, pel *dec, int stride, int comp_idx, int bit_depth, ALF_PARAM *alf_param, int lcu_y_pos, int lcu_height,
    int lcu_x_pos, int lcu_width, BOOL is_above_avail, BOOL is_below_avail, BOOL is_left_avail, BOOL is_right_avail, BOOL is_above_left_avail, BOOL is_above_right_avail
#if ALF_SHAPE
    , BOOL alf_shape_flag
#endif
);
void copy_one_alf_blk(pel *pic_dst, pel *pic_src, int stride, int y_pos, int x_pos, int height, int width, int is_above_avail, int is_below_avail);
void add_alf_corr_data(ALF_CORR_DATA *A, ALF_CORR_DATA *B, ALF_CORR_DATA *C
#if ALF_SHAPE
    , int num_coef
#endif
);
void accumulate_lcu_correlation(ENC_CTX *ctx, ALF_CORR_DATA **alf_corr_acc, ALF_CORR_DATA ***alf_corr_src_lcu, BOOL use_all_lcus);
void decide_alf_picture_param(ENC_ALF_VAR *enc_alf, ALF_PARAM **alf_pic_param, ALF_CORR_DATA **alf_corr, double lambda_luma);
void derive_filter_info(ENC_ALF_VAR *enc_alf, int comp_idx, ALF_CORR_DATA *alf_corr, ALF_PARAM *alf_param, int max_num_filter, double lambda);
void code_filter_coeff(int **filter_coeff, int *var_ind_tab, int num_filters, ALF_PARAM *alf_param);
void quant_filter_coef(double *h, int *quant_h
#if ALF_SHAPE
    , int num_coef
#endif
);
void filter_coef_quick_sort(double *coef_data, int *coef_num, int upper, int lower);
void find_best_filter_var_pred(ENC_ALF_VAR *enc_alf, double **y_sym, double ***E_sym, double *pix_acc, int **filter_coeff_sym,
    int *filters_per_fr_best, int var_ind_tab[], double lambda_val, int num_max_filters
#if ALF_SHAPE
    , int num_Coeff
#endif
);
double merge_filters_greedy(ENC_ALF_VAR *enc_alf, double **y_global_seq, double ***E_global_seq, double *pix_acc_global_seq,
    int interval_best[NO_VAR_BINS][2], int sqr_filter_length, int num_intervals);
double calculate_error_abs(double **A, double *b, double y, int size);
void predict_alf_coeff(int **coeff, int num_coef, int num_filters);


double quantize_integer_filter(double *filter_coeff, int *filter_coeff_quant, double **E, double *y, int sqr_filter_length, int *weights);
void round_filter_coeff(int *filter_coeff_quant, double *filter_coeff, int sqr_filter_length, int factor);
double calculate_error_coeff_provided(double **A, double *b, double *c, int size);
void add_A(double **Amerged, double ***A, int start, int stop, int size);
void add_b(double *bmerged, double **b, int start, int stop, int size);

int gns_solve_by_Cholesky_decomp(double **LHS, double *rhs, double *x, int noEq);
long long fast_filter_dist_estimate(ENC_ALF_VAR *enc_alf, double **E, double *y, int *coeff, int filter_length);
long long calc_alf_lcu_dist(ENC_CTX *ctx, BOOL skip_lcu_boundary, int comp_idx, int lcu_idx, int lcu_y_pos, int lcu_x_pos, int lcu_height,
    int lcu_width, BOOL is_above_avail, pel *pic_src, pel *pic_cmp, int stride, int format_shift);

void allocate_alf_param(ALF_PARAM **alf_par, int comp_idx
#if ALF_SHAPE
    , int num_coef
#endif
);
void free_alf_param(ALF_PARAM *alf_param, int comp_idx);

#endif
