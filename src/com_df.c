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

#include "com_def.h"
#include "com_df.h"
#include "com_tbl.h"

const u8 sm_tc_table[54] =
{
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,5,5,6,6,7,8,9,10,11,13,14,16,18,20,22,24
};

const u8 sm_beta_table[52] =
{
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,7,8,9,10,11,12,13,14,15,16,17,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64
};


typedef unsigned short byte;
#if DEBLOCK_M4647
byte ALPHA_TABLE[64] =
{
    0,  0,  0,  0,  0,  0,  0,  0,
    1,  1,  1,  1,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  2,  2,  2,
    2,  2,  2,  3,  3,  3,  3,  4,
    4,  4,  5,  5,  6,  6,  7,  7,
    8,  9,  10, 10, 11, 12, 13, 15,
    16, 17, 19, 21, 23, 25, 27, 29,
    32, 35, 38, 41, 45, 49, 54, 59
};
byte  BETA_TABLE[64] =
{
    0,  0,  0,  0,  0,  0,  0,  0,
    0,  1,  1,  1,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  2,  2,
    2,  2,  2,  2,  3,  3,  3,  3,
    4,  4,  5,  5,  5,  6,  6,  7,
    8,  8,  9,  10, 11, 12, 13, 14,
    15, 16, 17, 18, 19, 20, 21, 22,
    23, 23, 24, 24, 25, 25, 26, 27
};
#else
byte ALPHA_TABLE[64] =
{
    0, 0, 0, 0, 0, 0, 1, 1,
    1, 1, 1, 2, 2, 2, 3, 3,
    4, 4, 5, 5, 6, 7, 8, 9,
    10, 11, 12, 13, 15, 16, 18, 20,
    22, 24, 26, 28, 30, 33, 33, 35,
    35, 36, 37, 37, 39, 39, 42, 44,
    46, 48, 50, 52, 53, 54, 55, 56,
    57, 58, 59, 60, 61, 62, 63, 64
};
byte  BETA_TABLE[64] =
{
    0, 0, 0, 0, 0, 0, 1, 1,
    1, 1, 1, 1, 1, 2, 2, 2,
    2, 2, 3, 3, 3, 3, 4, 4,
    4, 4, 5, 5, 5, 5, 6, 6,
    6, 7, 7, 7, 8, 8, 8, 9,
    9, 10, 10, 11, 11, 12, 13, 14,
    15, 16, 17, 18, 19, 20, 21, 22,
    23, 23, 24, 24, 25, 25, 26, 27
};

#endif
byte CLIP_TAB_AVS[64] =
{
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 2, 2,
    2, 2, 2, 2, 2, 2, 3, 3,
    3, 3, 3, 3, 3, 4, 4, 4,
    5, 5, 5, 6, 6, 6, 7, 7,
    7, 7, 8, 8, 8, 9, 9, 9
};

void create_edge_filter_avs2(int w, int h, int ***edge_filter)
{
    //allocate memory
    for (int i = 0; i < LOOPFILTER_DIR_TYPE; i++)
    {
        edge_filter[i] = (int **)calloc((h >> LOOPFILTER_SIZE_IN_BIT), sizeof(int *));
        for (int j = 0; j < (h >> LOOPFILTER_SIZE_IN_BIT); j++)
        {
            edge_filter[i][j] = (int *)calloc((w >> LOOPFILTER_SIZE_IN_BIT), sizeof(int));
        }
    }
    //reset value
    for (int i = 0; i < LOOPFILTER_DIR_TYPE; i++)
    {
        for (int b4_y = 0; b4_y < (h >> LOOPFILTER_SIZE_IN_BIT); b4_y++)
        {
            for (int b4_x = 0; b4_x < (w >> LOOPFILTER_SIZE_IN_BIT); b4_x++)
            {
                edge_filter[i][b4_y][b4_x] = 0;
            }
        }
    }
}

void clear_edge_filter_avs2(int x, int y, int w, int h, int ***edge_filter)
{
    for (int i = 0; i < LOOPFILTER_DIR_TYPE; i++)
    {
        for (int b4_y = (y >> LOOPFILTER_SIZE_IN_BIT); b4_y < ((y + h) >> LOOPFILTER_SIZE_IN_BIT); b4_y++)
        {
            for (int b4_x = (x >> LOOPFILTER_SIZE_IN_BIT); b4_x < ((x + w) >> LOOPFILTER_SIZE_IN_BIT); b4_x++)
            {
                edge_filter[i][b4_y][b4_x] = 0;
            }
        }
    }
}

void delete_edge_filter_avs2(int ***edge_filter, int h)
{
    for (int i = 0; i < LOOPFILTER_DIR_TYPE; i++)
    {
        if( edge_filter[i] )
        {
            for( int j = 0; j < (h >> LOOPFILTER_SIZE_IN_BIT); j++ )
            {
                if( edge_filter[i][j] )
                {
                    com_mfree_fast( edge_filter[i][j] );
                    edge_filter[i][j] = NULL;
                }
            }

            com_mfree_fast( edge_filter[i] );
            edge_filter[i] = NULL;
        }
    }
}

void set_edge_filter_param_hor_avs2(COM_PIC * pic, int ***edge_filter, int x_pel, int y_pel, int cuw, int cuh, int edge_condition)
{
    int w = cuw >> LOOPFILTER_SIZE_IN_BIT;
    int b4_x_start = x_pel >> LOOPFILTER_SIZE_IN_BIT;
    int b4_y_start = y_pel >> LOOPFILTER_SIZE_IN_BIT;
    int dir = 1;
    if (y_pel > 0 && (y_pel % LOOPFILTER_GRID == 0))
    {
        if (b4_y_start == 0)
        {
            return;
        }
        for (int i = 0; i < w; i++)
        {
            if ((int)(b4_x_start + i) < ((pic->width_luma) >> LOOPFILTER_SIZE_IN_BIT)&& (int)(b4_y_start) < ((pic->height_luma) >> LOOPFILTER_SIZE_IN_BIT))
            {
                if (edge_filter[dir][b4_y_start][b4_x_start + i])
                {
                    break;
                }
                edge_filter[dir][b4_y_start][b4_x_start + i] = edge_condition;
            }
            else
            {
                break;
            }
        }
    }
}

void set_edge_filter_param_ver_avs2(COM_PIC * pic, int ***edge_filter, int x_pel, int y_pel, int cuw, int cuh, int edge_condition)
{
    int h = cuh >> LOOPFILTER_SIZE_IN_BIT;
    int b4_x_start = x_pel >> LOOPFILTER_SIZE_IN_BIT;
    int b4_y_start = y_pel >> LOOPFILTER_SIZE_IN_BIT;
    int dir = 0;
    /* vertical filtering */
    if (x_pel > 0 && (x_pel % LOOPFILTER_GRID == 0))
    {
        if (b4_x_start == 0)
        {
            return;
        }
        for (int i = 0; i < h; i++)
        {
            if ((int)(b4_y_start + i) < (pic->height_luma >> LOOPFILTER_SIZE_IN_BIT))
            {
                if (edge_filter[dir][b4_y_start + i][b4_x_start])
                {
                    break;
                }
                edge_filter[dir][b4_y_start + i][b4_x_start] = edge_condition;
            }
            else
            {
                break;
            }
        }
    }
}

void set_edge_filter_avs2(COM_INFO *info, COM_MAP *map, COM_PIC * pic, int*** edge_filter)
{
    int i, j;
    BOOL b_recurse = 1;
    int lcu_num = 0;
    /* all lcu */
    for (j = 0; j < info->pic_height_in_lcu; j++)
    {
        for (i = 0; i < info->pic_width_in_lcu; i++)
        {
            set_edge_filter_one_scu_avs2(info, map, pic, edge_filter, (i << info->log2_max_cuwh), (j << info->log2_max_cuwh), info->max_cuwh, info->max_cuwh, 0, 0, b_recurse);
            lcu_num++;
        }
    }
}

void set_edge_filter_one_scu_avs2(COM_INFO *info, COM_MAP *map, COM_PIC * pic, int ***edge_filter, int x, int y, int cuw, int cuh, int cud, int cup, BOOL b_recurse)
{
    s8 split_mode = NO_SPLIT;
    int lcu_idx;
    int edge_type = 0;

    lcu_idx = (x >> info->log2_max_cuwh) + (y >> info->log2_max_cuwh) * info->pic_width_in_lcu;
    if (b_recurse)
    {
        com_get_split_mode(&split_mode, cud, cup, cuw, cuh, info->max_cuwh, (map->map_split[lcu_idx]));
    }
    if (b_recurse && split_mode != NO_SPLIT)
    {
        COM_SPLIT_STRUCT split_struct;
        com_split_get_part_structure(split_mode, x, y, cuw, cuh, cup, cud, info->log2_max_cuwh - MIN_CU_LOG2, &split_struct);
        for (int part_num = 0; part_num < split_struct.part_count; ++part_num)
        {
            int cur_part_num = part_num;
            int sub_cuw = split_struct.width[cur_part_num];
            int sub_cuh = split_struct.height[cur_part_num];
            int x_pos = split_struct.x_pos[cur_part_num];
            int y_pos = split_struct.y_pos[cur_part_num];
            if (x_pos < info->pic_width && y_pos < info->pic_height)
            {
                set_edge_filter_one_scu_avs2(info, map, pic, edge_filter, x_pos, y_pos, sub_cuw, sub_cuh, split_struct.cud, split_struct.cup[cur_part_num], b_recurse);
            }
        }
    }
    else
    {
        // luma tb filtering due to intra_DT and PBT
        int scu_idx = PEL2SCU(y) * info->pic_width_in_scu + PEL2SCU(x);
        int pb_tb_part = map->map_pb_tb_part[scu_idx];
        int tb_part = MCU_GET_TB_PART_LUMA(pb_tb_part);

        switch (tb_part)
        {
        case SIZE_2NxhN:
            set_edge_filter_param_hor_avs2(pic, edge_filter, x, y + cuh / 4 * 1, cuw, cuh, EDGE_TYPE_LUMA);
            set_edge_filter_param_hor_avs2(pic, edge_filter, x, y + cuh / 4 * 2, cuw, cuh, EDGE_TYPE_LUMA);
            set_edge_filter_param_hor_avs2(pic, edge_filter, x, y + cuh / 4 * 3, cuw, cuh, EDGE_TYPE_LUMA);
            break;
        case SIZE_hNx2N:
            set_edge_filter_param_ver_avs2(pic, edge_filter, x + cuw / 4 * 1, y, cuw, cuh, EDGE_TYPE_LUMA);
            set_edge_filter_param_ver_avs2(pic, edge_filter, x + cuw / 4 * 2, y, cuw, cuh, EDGE_TYPE_LUMA);
            set_edge_filter_param_ver_avs2(pic, edge_filter, x + cuw / 4 * 3, y, cuw, cuh, EDGE_TYPE_LUMA);
            break;
        case SIZE_NxN:
            set_edge_filter_param_hor_avs2(pic, edge_filter, x, y + cuh / 4 * 2, cuw, cuh, EDGE_TYPE_LUMA);
            set_edge_filter_param_ver_avs2(pic, edge_filter, x + cuw / 4 * 2, y, cuw, cuh, EDGE_TYPE_LUMA);
            break;
        default:
            break;
        }

        set_edge_filter_param_hor_avs2(pic, edge_filter, x, y, cuw, cuh, EDGE_TYPE_ALL);  ///UP
        set_edge_filter_param_ver_avs2(pic, edge_filter, x, y, cuw, cuh, EDGE_TYPE_ALL);  /// LEFT
    }
}


int check_skip_filtering(COM_INFO *info, COM_MAP *map, COM_REFP refp[MAX_NUM_REF_PICS][REFP_NUM], u32 *MbP, u32 *MbQ, int dir, int block_y, int block_x)
{
#if DEBLOCK_M4647
    int t = (block_x)+(block_y)* info->pic_width_in_scu;
    int reflist;
    s8 *refiP,*refiQ;
    s16 (*mvP)[MV_D],(*mvQ)[MV_D];

    refiP = *(map->map_refi + t);
    mvP   = *(map->map_mv   + t);
    refiQ = *(map->map_refi + t + (dir ? (-info->pic_width_in_scu) : (-1)));
    mvQ   = *(map->map_mv   + t + (dir ? (-info->pic_width_in_scu) : (-1)));

    COM_PIC *p_pic0 = REFI_IS_VALID(refiP[REFP_0]) ? refp[refiP[REFP_0]][REFP_0].pic : NULL;
    COM_PIC *p_pic1 = REFI_IS_VALID(refiP[REFP_1]) ? refp[refiP[REFP_1]][REFP_1].pic : NULL;
    COM_PIC *q_pic0 = REFI_IS_VALID(refiQ[REFP_0]) ? refp[refiQ[REFP_0]][REFP_0].pic : NULL;
    COM_PIC *q_pic1 = REFI_IS_VALID(refiQ[REFP_1]) ? refp[refiQ[REFP_1]][REFP_1].pic : NULL;

    if ( (p_pic0 == NULL && p_pic1 == NULL) || (q_pic0 == NULL && q_pic1 == NULL) )
    {
        return 0;
    }
    if (MCU_GET_CBF(*MbP) || MCU_GET_CBF(*MbQ))
    {
        return 0;
    }

    if (p_pic0 == q_pic0 && p_pic1 == q_pic1)
    {
        for (reflist = REFP_0; reflist < REFP_NUM; reflist++)
        {
            if (REFI_IS_VALID(refiP[reflist]))
            {
                if ((abs(mvP[reflist][MV_X] - mvQ[reflist][MV_X]) >= 4) ||
                        (abs(mvP[reflist][MV_Y] - mvQ[reflist][MV_Y]) >= 4))
                {
                    return 0;
                }
            }
        }
        return 1;
    }
    if (p_pic0 == q_pic1 && p_pic1 == q_pic0)
    {
        for (reflist = REFP_0; reflist < REFP_NUM; reflist++)
        {
            if (REFI_IS_VALID(refiP[reflist]))
            {
                if ((abs(mvP[reflist][MV_X] - mvQ[!reflist][MV_X]) >= 4) ||
                        (abs(mvP[reflist][MV_Y] - mvQ[!reflist][MV_Y]) >= 4))
                {
                    return 0;
                }
            }
        }
        return 1;
    }
    return 0;
#else
    int skip_filtering;
    int t = (block_x)+(block_y)* info->pic_width_in_scu;
    int reflist;
    s8 *curr_refi;
    s8 *neighbor_refi;
    s16(*curr_mv)[MV_D];
    s16(*neighbor_mv)[MV_D];
    curr_refi = *(map->map_refi + t);
    curr_mv = *(map->map_mv + t);
    neighbor_refi = *(map->map_refi + t + (dir ? (-info->pic_width_in_scu) : (-1)));
    neighbor_mv = *(map->map_mv + t + (dir ? (-info->pic_width_in_scu) : (-1)));
    s16 mv0[2] = { 0, 0 };
    s16 mv1[2] = { 0, 0 };
    s8 ref0 = -1;
    s8 ref1 = -1;
    BOOL b_forward = (REFI_IS_VALID(curr_refi[REFP_0]) & REFI_IS_VALID(neighbor_refi[REFP_0]));
    BOOL b_backward = (REFI_IS_VALID(curr_refi[REFP_1]) & REFI_IS_VALID(neighbor_refi[REFP_1]));

    if ((b_forward || b_backward))
    {
        if (b_forward) //use forward
        {
            reflist = REFP_0;
        }
        else //if (b_backward) // use backward
        {
            reflist = REFP_1;
        }
        mv0[MV_X] = curr_mv[reflist][MV_X];
        mv0[MV_Y] = curr_mv[reflist][MV_Y];
        ref0 = curr_refi[reflist];
        mv1[MV_X] = neighbor_mv[reflist][MV_X];
        mv1[MV_Y] = neighbor_mv[reflist][MV_Y];
        ref1 = neighbor_refi[reflist];
        switch (info->pic_header.slice_type)
        {
        case COM_ST_B:
            if ((MCU_GET_CBF(*MbP) == 0) && (MCU_GET_CBF(*MbQ) == 0) &&
                    (abs(mv0[MV_X] - mv1[MV_X]) < 4) &&
                    (abs(mv0[MV_Y] - mv1[MV_Y]) < 4) &&
                    (ref0 == ref1) && REFI_IS_VALID(ref0))
            {
                skip_filtering = 1;
            }
            else
            {
                skip_filtering = 0;
            }
            break;
        default:
            skip_filtering = 0;
        }
    }
    else
    {
        skip_filtering = 0;
    }
    return skip_filtering;
#endif
}

void edge_loop_x(COM_INFO *info, COM_MAP *map, COM_REFP refp[MAX_NUM_REF_PICS][REFP_NUM], pel *src_ptr, int QP, int dir, int width, int chro, u32 *MbP, u32 *MbQ, int block_y, int block_x
#if DBR
    , pel *src_ptr_org, int src_stride, int enc_k, DBR_PARAM *dbr_param
#endif
)
{
#if DBK_SCC
    int df_type = info->pic_header.loop_fitler_type;
#endif
    int temp_pel, ptr_inc;
    int inc, inc2, inc3, inc4;
    int abs_delta;
    int L3, L2, L1, L0, R0, R1, R2, R3;
    int fs; //fs stands for filtering strength. The larger fs is, the stronger filter is applied.
    int alpha, beta, beta1;
    int flatness_left, flatness_right; // flatness_left and flatness_right describe how flat the curve is of one codingUnit.
    int shift1 = info->bit_depth_internal - 8;
    int skip_filtering = 0;
    int alpha_c_offset = info->pic_header.alpha_c_offset;
    int beta_offset = info->pic_header.beta_offset;

#if DBR
    int *offsets      = NULL;
    int dbr_flag      = 0;
    int thresh_index  = 0;
    int inc_offset[6], org_offset[6];
    int org_stride, org_inc, org_inc2, org_inc3, org_inc4, i;

    org_stride = dir ? 1 : src_stride;
    org_inc    = dir ? src_stride : 1;
    org_inc2 = org_inc << 1;
    org_inc3 = org_inc + org_inc2;
    org_inc4 = org_inc + org_inc3;
    org_offset[0]    = -org_inc3;
    org_offset[1]    = -org_inc2;
    org_offset[2]    = -org_inc;
    org_offset[3]    = 0;
    org_offset[4]    = org_inc;
    org_offset[5]    = org_inc2;

    if (NULL != dbr_param)
    {
        offsets      = dir ? dbr_param->horizontal_offsets      : dbr_param->vertical_offsets;
        dbr_flag     = dir ? dbr_param->dbr_horizontal_enabled  : dbr_param->dbr_vertical_enabled;
        thresh_index = dir ? dbr_param->thresh_horizontal_index : dbr_param->thresh_vertical_index;
    }
#endif

    ptr_inc = dir ? 1 : width;
    inc    = dir ? width : 1;
    inc2 = inc << 1;
    inc3 = inc + inc2;
    inc4 = inc + inc3;

#if DBR
    inc_offset[0] = -inc3;
    inc_offset[1] = -inc2;
    inc_offset[2] = -inc;
    inc_offset[3] = 0;
    inc_offset[4] = inc;
    inc_offset[5] = inc2;
#endif

    alpha = ALPHA_TABLE[COM_CLIP3(MIN_QUANT, MAX_QUANT_BASE, QP + alpha_c_offset)] << shift1;
    beta  = BETA_TABLE [COM_CLIP3(MIN_QUANT, MAX_QUANT_BASE, QP + beta_offset   )] << shift1;
    beta1 = 1 << shift1;

    for (temp_pel = 0; temp_pel < LOOPFILTER_SIZE; temp_pel++)
    {
        if (temp_pel % 4 == 0)
        {
            skip_filtering = check_skip_filtering(info, map, refp, MbP, MbQ, dir, block_y + !dir * (temp_pel >> 2), block_x + dir * (temp_pel >> 2));
        }

        L3 = src_ptr[-inc4];
        L2 = src_ptr[-inc3];
        L1 = src_ptr[-inc2];
        L0 = src_ptr[-inc];
        R0 = src_ptr[0];
        R1 = src_ptr[inc];
        R2 = src_ptr[inc2];
        R3 = src_ptr[inc3];

        abs_delta = COM_ABS(R0 - L0);
#if DEBLOCK_M4647
        if (!skip_filtering)
        {
#else
        if (!skip_filtering && (abs_delta < alpha) && (abs_delta > 1))
        {
#endif
#if DBR
            const pel before_filter[6] = { src_ptr[-inc3], src_ptr[-inc2], src_ptr[-inc], src_ptr[0], src_ptr[inc], src_ptr[inc2] };
                  pel after_filter[6]  = { src_ptr[-inc3], src_ptr[-inc2], src_ptr[-inc], src_ptr[0], src_ptr[inc], src_ptr[inc2] };
#endif
#if DBK_SCC
            if (df_type && abs_delta >= 4 * alpha)
            {
                fs = 0;
            }
            else
            {
#endif
            flatness_left = (COM_ABS(L1 - L0) < beta) ? 2 : 0;
            if (COM_ABS(L2 - L0) < beta)
            {
                flatness_left++;
            }
            flatness_right = (COM_ABS(R0 - R1) < beta) ? 2 : 0;
            if (COM_ABS(R0 - R2) < beta)
            {
                flatness_right++;
            }

            switch (flatness_left + flatness_right)
            {
            case 6:
#if DBK_SCC
                if (df_type)
                {
                    fs = (COM_ABS(R0 - R1) <= beta / 4 && COM_ABS(L0 - L1) <= beta / 4 &&
                        COM_ABS(R0 - R3) <= beta / 2 && COM_ABS(L0 - L3) <= beta / 2 &&
                        abs_delta < alpha) ? 4 : 3;
                }
                else
                {
                    fs = (COM_ABS(R0 - R1) <= beta / 4 && COM_ABS(L0 - L1) <= beta / 4 && abs_delta < alpha) ? 4 : 3;
                }
#elif DEBLOCK_M4647
                fs = (COM_ABS(R0 - R1) <= beta / 4 && COM_ABS(L0 - L1) <= beta / 4 && abs_delta < alpha) ? 4 : 3;
#else
                fs = (R1 == R0 && L0 == L1) ? 4 : 3;
#endif
                break;
            case 5:
#if DBK_SCC
                if (df_type)
                {
                    fs = (R1 == R0 && L0 == L1 && COM_ABS(L2 - R2) < alpha) ? 3 : 2;
                }
                else
                {
                    fs = (R1 == R0 && L0 == L1) ? 3 : 2;
                }
#else
                fs = (R1 == R0 && L0 == L1) ? 3 : 2;
#endif
                break;
            case 4:
                fs = (flatness_left == 2) ? 2 : 1;
                break;
            case 3:
                fs = (COM_ABS(L1 - R1) < beta) ? 1 : 0;
                break;
            default:
                fs = 0;
                break;
            }
#if DBK_SCC
            }
#endif
            if (chro)
            {
                if (fs > 0)
                {
                    fs--;
                }
#if DEBLOCK_M4647
                switch (fs)
                {
                case 3:
                    src_ptr[-inc2] = (pel)((L2 * 3 + L1 * 8 + L0 * 3 + R0 * 2 + 8) >> 4);                           //L2
                    src_ptr[ inc ] = (pel)((R2 * 3 + R1 * 8 + R0 * 3 + L0 * 2 + 8) >> 4);                           //R2
                case 2:
                case 1:
                    src_ptr[-inc] = (pel)((L1 * 3 + L0 * 10 + R0 * 3 + 8) >> 4);
                    src_ptr[   0] = (pel)((R1 * 3 + R0 * 10 + L0 * 3 + 8) >> 4);
                    break;

                default:
                    break;
                }
#else
                switch (fs)
                {
                case 3:
                    src_ptr[-inc] = (pel)((L2 + (L1 << 2) + (L0 << 2) + (L0 << 1) + (R0 << 2) + R1 + 8) >> 4);           //L0
                    src_ptr[0] = (pel)((L1 + (L0 << 2) + (R0 << 2) + (R0 << 1) + (R1 << 2) + R2 + 8) >> 4);           //R0
                    src_ptr[-inc2] = (pel)((L2 * 3 + L1 * 8 + L0 * 4 + R0 + 8) >> 4);
                    src_ptr[inc] = (pel)((R2 * 3 + R1 * 8 + R0 * 4 + L0 + 8) >> 4);
                    break;
                case 2:
                    src_ptr[-inc] = (pel)(((L1 << 1) + L1 + (L0 << 3) + (L0 << 1) + (R0 << 1) + R0 + 8) >> 4);
                    src_ptr[0] = (pel)(((L0 << 1) + L0 + (R0 << 3) + (R0 << 1) + (R1 << 1) + R1 + 8) >> 4);
                    break;
                case 1:
                    src_ptr[-inc] = (pel)((L0 * 3 + R0 + 2) >> 2);
                    src_ptr[0] = (pel)((R0 * 3 + L0 + 2) >> 2);
                    break;
                default:
                    break;
                }
#endif
            }
            else
            {
#if DBR
                if (enc_k)
                {
                    switch (fs)
                    {
                    case 4:
                        after_filter[2] = (pel)((L2 * 3 + L1 * 8 + L0 * 10 + R0 * 8 + R1 * 3 + 16) >> 5);             //L0
                        after_filter[3] = (pel)((R2 * 3 + R1 * 8 + R0 * 10 + L0 * 8 + L1 * 3 + 16) >> 5);             //R0
                        after_filter[1] = (pel)((L2 * 4 + L1 * 5 + L0 * 4 + R0 * 3 + 8) >> 4);                       //L1
                        after_filter[4] = (pel)((R2 * 4 + R1 * 5 + R0 * 4 + L0 * 3 + 8) >> 4);                       //R1
                        after_filter[0] = (pel)((L3 * 2 + L2 * 2 + L1 * 2 + L0 * 1 + R0 + 4) >> 3);                          //L2
                        after_filter[5] = (pel)((R3 * 2 + R2 * 2 + R1 * 2 + R0 * 1 + L0 + 4) >> 3);                          //R2
                        break;
                    case 3:
                        after_filter[2] = (pel)((L2 + (L1 << 2) + (L0 << 2) + (L0 << 1) + (R0 << 2) + R1 + 8) >> 4); //L0
                        after_filter[3] = (pel)((L1 + (L0 << 2) + (R0 << 2) + (R0 << 1) + (R1 << 2) + R2 + 8) >> 4); //R0
                        after_filter[1] = (pel)((L2 * 3 + L1 * 8 + L0 * 4 + R0 + 8) >> 4);                           //L2
                        after_filter[4] = (pel)((R2 * 3 + R1 * 8 + R0 * 4 + L0 + 8) >> 4);                           //R2
                        break;
                    case 2:
                        after_filter[2] = (pel)(((L1 << 1) + L1 + (L0 << 3) + (L0 << 1) + (R0 << 1) + R0 + 8) >> 4);
                        after_filter[3] = (pel)(((L0 << 1) + L0 + (R0 << 3) + (R0 << 1) + (R1 << 1) + R1 + 8) >> 4);
                        break;
                    case 1:
                        after_filter[2] = (pel)((L0 * 3 + R0 + 2) >> 2);
                        after_filter[3] = (pel)((R0 * 3 + L0 + 2) >> 2);
                        break;
                    default:
                        break;
                    }

                    fs = -1;
                }
#endif
                switch (fs)
                {
                case 4:
#if DEBLOCK_M4647
                    src_ptr[-inc] = (pel)((L2 * 3 + L1 * 8 + L0 * 10 + R0 * 8 + R1 * 3 + 16) >> 5);             //L0
                    src_ptr[0] = (pel)((R2 * 3 + R1 * 8 + R0 * 10 + L0 * 8 + L1 * 3 + 16) >> 5);             //R0
                    src_ptr[-inc2] = (pel)((L2 * 4 + L1 * 5 + L0 * 4 + R0 * 3 + 8) >> 4);                       //L1
                    src_ptr[inc] = (pel)((R2 * 4 + R1 * 5 + R0 * 4 + L0 * 3 + 8) >> 4);                       //R1
                    src_ptr[-inc3] = (pel)((L3 * 2 + L2 * 2 + L1 * 2 + L0 * 1 + R0 + 4) >> 3);                          //L2
                    src_ptr[inc2] = (pel)((R3 * 2 + R2 * 2 + R1 * 2 + R0 * 1 + L0 + 4) >> 3);                          //R2
#else
                    src_ptr[-inc] = (pel)((L0 + ((L0 + L2) << 3) + L2 + (R0 << 3) + (R2 << 2) + (R2 << 1) + 16) >> 5);             //L0
                    src_ptr[-inc2] = (pel)(((L0 << 3) - L0 + (L2 << 2) + (L2 << 1) + R0 + (R0 << 1) + 8) >> 4);           //L1
                    src_ptr[-inc3] = (pel)(((L0 << 2) + L2 + (L2 << 1) + R0 + 4) >> 3);       //L2
                    src_ptr[0] = (pel)((R0 + ((R0 + R2) << 3) + R2 + (L0 << 3) + (L2 << 2) + (L2 << 1) + 16) >> 5);             //R0
                    src_ptr[inc] = (pel)(((R0 << 3) - R0 + (R2 << 2) + (R2 << 1) + L0 + (L0 << 1) + 8) >> 4);           //R1
                    src_ptr[inc2] = (pel)(((R0 << 2) + R2 + (R2 << 1) + L0 + 4) >> 3);       //R2

#endif
                    break;
                case 3:
                    src_ptr[-inc] = (pel)((L2 + (L1 << 2) + (L0 << 2) + (L0 << 1) + (R0 << 2) + R1 + 8) >> 4); //L0
                    src_ptr[0] = (pel)((L1 + (L0 << 2) + (R0 << 2) + (R0 << 1) + (R1 << 2) + R2 + 8) >> 4); //R0
                    src_ptr[-inc2] = (pel)((L2 * 3 + L1 * 8 + L0 * 4 + R0 + 8) >> 4);                           //L2
                    src_ptr[inc] = (pel)((R2 * 3 + R1 * 8 + R0 * 4 + L0 + 8) >> 4);                           //R2
                    break;
                case 2:
                    src_ptr[-inc] = (pel)(((L1 << 1) + L1 + (L0 << 3) + (L0 << 1) + (R0 << 1) + R0 + 8) >> 4);
                    src_ptr[0] = (pel)(((L0 << 1) + L0 + (R0 << 3) + (R0 << 1) + (R1 << 1) + R1 + 8) >> 4);
                    break;
                case 1:
                    src_ptr[-inc] = (pel)((L0 * 3 + R0 + 2) >> 2);
                    src_ptr[0] = (pel)((R0 * 3 + L0 + 2) >> 2);
                    break;
                default:
                    break;
                }
#if DBR
                //calculation offsets
                if (enc_k)
                {
                    for (i = 0; i < 6; i++)
                    {
                        if (COM_ABS(before_filter[i] - after_filter[i]) > enc_k)
                        {
                            if (before_filter[i] > after_filter[i])
                            {
                                int diff = (src_ptr_org[org_offset[i]] - ((before_filter[i] + after_filter[i] + 1) >> 1));
                                offsets[0] += diff;
                                offsets[4] += diff*diff - (src_ptr_org[org_offset[i]] - after_filter[i])*(src_ptr_org[org_offset[i]] - after_filter[i]);
                                offsets[2]++;
                            }
                            else
                            {
                                int diff = (src_ptr_org[org_offset[i]] - ((before_filter[i] + after_filter[i] + 1) >> 1));
                                offsets[1] += diff;
                                offsets[5] += diff*diff - (src_ptr_org[org_offset[i]] - after_filter[i])*(src_ptr_org[org_offset[i]] - after_filter[i]);
                                offsets[3]++;
                            }
                        }
                    }
                }
                else
                {
                    if (dbr_flag && !chro)
                    {
                        for (i = 0; i < 6; i++)
                        {
                            if (COM_ABS(before_filter[i] - src_ptr[inc_offset[i]]) > thresh_index)
                            {
                                if (before_filter[i] > src_ptr[inc_offset[i]])
                                {
                                    src_ptr[inc_offset[i]] = (pel)COM_CLIP3(0, (1 << info->bit_depth_internal) - 1, ((before_filter[i] + src_ptr[inc_offset[i]] + 1) >> 1) - offsets[0]);
                                }
                                else
                                {
                                    src_ptr[inc_offset[i]] = (pel)COM_CLIP3(0, (1 << info->bit_depth_internal) - 1, ((before_filter[i] + src_ptr[inc_offset[i]] + 1) >> 1) + offsets[1]);
                                }
                            }
                        }
                    }
                }
#endif
            }
            src_ptr += ptr_inc;    // Next row or column
#if DBR
            if (src_ptr_org != NULL)
            {
                src_ptr_org += org_stride;
            }
#endif
            temp_pel += chro;
        }
        else
        {
            src_ptr += ptr_inc;
#if DBR
            if (src_ptr_org != NULL)
            {
                src_ptr_org += org_stride;
            }
#endif
            temp_pel += chro;
        }
    }
}

void deblock_mb_avs2(COM_INFO *info, COM_MAP *map, COM_PIC *pic, COM_REFP refp[MAX_NUM_REF_PICS][REFP_NUM], int*** edge_filter, int mb_y, int mb_x, int edge_dir
#if DBR
    , COM_PIC *pic_org, int enc, DBR_PARAM *dbr_param
#endif
#if RDO_DBK_LUMA_ONLY
    , int only_luma
#endif
)
{
    pel          *SrcY, *SrcU, *SrcV;
#if DBR
    pel          *src_y_org   = NULL;
    int           src_stride = 0;
#endif
    int           edge_condition;
    int           dir, QP;
    unsigned int  b4_x_start;
    unsigned int  b4_y_start;
    int           skip_filtering;
    int           x_pel = mb_x << LOOPFILTER_SIZE_IN_BIT;
    int           y_pel = mb_y << LOOPFILTER_SIZE_IN_BIT;
    int           s_l = pic->stride_luma;
    int           s_c = pic->stride_chroma;
    int           t;
    t = x_pel + y_pel * s_l;
    SrcY = pic->y + t;
#if DBR
    if (pic_org != NULL)
    {
        src_stride = pic_org->stride_luma;
        src_y_org  = pic_org->y + x_pel + y_pel * src_stride;
    }
#endif
    t = (x_pel >> 1) + (y_pel >> 1) * s_c;
    SrcU = pic->u + t;
    SrcV = pic->v + t;
    u32    *MbP, *MbQ;
    t = mb_x + mb_y * info->pic_width_in_scu;
    MbQ = map->map_scu + t; // current Mb
#if USE_SP
    u8 *MbUSPQ;
    MbUSPQ = map->map_usp + t;
    if (MSP_GET_SP_INFO(*MbUSPQ))
    {
        return;
    }
#endif
    dir = edge_dir;
    {
        edge_condition = (dir && mb_y) || (!dir && mb_x);     // can not filter beyond frame boundaries
        if (!dir && mb_x && !info->sqh.cross_patch_loop_filter)
        {
            edge_condition = (map->map_patch_idx[t] == map->map_patch_idx[t - 1]) ? edge_condition : 0;
            //  can not filter beyond slice boundaries
#if USE_SP
            if (MSP_GET_SP_INFO(*(MbUSPQ - 1)))
            {
                edge_condition = 0;
            }
#endif
        }
        if (dir && mb_y && !info->sqh.cross_patch_loop_filter)
        {
            edge_condition = (map->map_patch_idx[t] == map->map_patch_idx[t - info->pic_width_in_scu]) ? edge_condition : 0;
            //  can not filter beyond slice boundaries
#if USE_SP
            if (MSP_GET_SP_INFO(*(MbUSPQ - info->pic_width_in_scu)))
            {
                edge_condition = 0;
            }
#endif
        }
        b4_x_start = mb_x;
        b4_y_start = mb_y;
        edge_condition = (edge_filter[dir][b4_y_start][b4_x_start] &&
                         edge_condition) ? edge_filter[dir][b4_y_start][b4_x_start] : 0;
        // then  4 horizontal
        if (edge_condition)
        {
            MbP = (dir) ? (MbQ - info->pic_width_in_scu) : (MbQ - 1);         // MbP = Mb of the remote 4x4 block
            int MbQ_qp = MCU_GET_QP(*MbQ);
            int MbP_qp = MCU_GET_QP(*MbP);
            QP = (MbP_qp + MbQ_qp + 1) >> 1;
            // Average QP of the two blocks
            skip_filtering = 0;
            if (!skip_filtering)
            {
                edge_loop_x(info, map, refp, SrcY, QP - info->qp_offset_bit_depth, dir, s_l, 0, MbP, MbQ, mb_y << (LOOPFILTER_SIZE_IN_BIT - 2), mb_x << (LOOPFILTER_SIZE_IN_BIT - 2)
#if DBR
                    , src_y_org, src_stride, enc, dbr_param
#endif
                );
                if ((edge_condition == EDGE_TYPE_ALL)
#if RDO_DBK_LUMA_ONLY
                    && !only_luma
#endif
#if DBR
                    && !enc
#endif
                    )
                {
                    if (((mb_y << MIN_CU_LOG2) % (LOOPFILTER_GRID << 1) == 0 && dir) || (((mb_x << MIN_CU_LOG2) % (LOOPFILTER_GRID << 1) == 0) && (!dir)))
                    {
                        int cQPuv;
                        int c_p_QPuv = MbP_qp + info->pic_header.chroma_quant_param_delta_cb - info->qp_offset_bit_depth;
                        int c_q_QPuv = MbQ_qp + info->pic_header.chroma_quant_param_delta_cb - info->qp_offset_bit_depth;
                        c_p_QPuv = COM_CLIP( c_p_QPuv, MIN_QUANT - 16, MAX_QUANT_BASE );
                        c_q_QPuv = COM_CLIP( c_q_QPuv, MIN_QUANT - 16, MAX_QUANT_BASE );
                        if (c_p_QPuv >= 0)
                        {
                            c_p_QPuv = com_tbl_qp_chroma_adjust[COM_MIN(MAX_QUANT_BASE, c_p_QPuv)];
                        }
                        if (c_q_QPuv >= 0)
                        {
                            c_q_QPuv = com_tbl_qp_chroma_adjust[COM_MIN(MAX_QUANT_BASE, c_q_QPuv)];
                        }
                        c_p_QPuv = COM_CLIP( c_p_QPuv + info->qp_offset_bit_depth, MIN_QUANT, MAX_QUANT_BASE + info->qp_offset_bit_depth );
                        c_q_QPuv = COM_CLIP( c_q_QPuv + info->qp_offset_bit_depth, MIN_QUANT, MAX_QUANT_BASE + info->qp_offset_bit_depth );

                        cQPuv = (c_p_QPuv + c_q_QPuv + 1) >> 1;
                        edge_loop_x(info, map, refp, SrcU, cQPuv - info->qp_offset_bit_depth, dir, s_c, 1, MbP, MbQ, mb_y << (LOOPFILTER_SIZE_IN_BIT - 2), mb_x << (LOOPFILTER_SIZE_IN_BIT - 2)
#if DBR
                            , NULL, 0, enc, NULL
#endif
                        );

                        c_p_QPuv = MbP_qp + info->pic_header.chroma_quant_param_delta_cr - info->qp_offset_bit_depth;
                        c_q_QPuv = MbQ_qp + info->pic_header.chroma_quant_param_delta_cr - info->qp_offset_bit_depth;
                        c_p_QPuv = COM_CLIP( c_p_QPuv, MIN_QUANT - 16, MAX_QUANT_BASE );
                        c_q_QPuv = COM_CLIP( c_q_QPuv, MIN_QUANT - 16, MAX_QUANT_BASE );

                        if (c_p_QPuv >= 0)
                        {
                            c_p_QPuv = com_tbl_qp_chroma_adjust[COM_MIN(MAX_QUANT_BASE, c_p_QPuv)];
                        }
                        if (c_q_QPuv >= 0)
                        {
                            c_q_QPuv = com_tbl_qp_chroma_adjust[COM_MIN(MAX_QUANT_BASE, c_q_QPuv)];
                        }
                        c_p_QPuv = COM_CLIP( c_p_QPuv + info->qp_offset_bit_depth, MIN_QUANT, MAX_QUANT_BASE + info->qp_offset_bit_depth );
                        c_q_QPuv = COM_CLIP( c_q_QPuv + info->qp_offset_bit_depth, MIN_QUANT, MAX_QUANT_BASE + info->qp_offset_bit_depth );
                        cQPuv = (c_p_QPuv + c_q_QPuv + 1) >> 1;
                        edge_loop_x(info, map, refp, SrcV, cQPuv - info->qp_offset_bit_depth, dir, s_c, 1, MbP, MbQ, mb_y << (LOOPFILTER_SIZE_IN_BIT - 2), mb_x << (LOOPFILTER_SIZE_IN_BIT - 2)
#if DBR
                            , NULL, 0, enc, NULL
#endif
                        );
                    }
                }
            }
        }
        //}
    }//end loop dir
}

//deblock one CU
void deblock_block_avs2(COM_INFO *info, COM_MAP *map, COM_PIC * pic, COM_REFP refp[MAX_NUM_REF_PICS][REFP_NUM], int*** edge_filter, int pel_x, int pel_y, int cuw, int cuh )
{
#if DBR
    DBR_PARAM dbr_param = info->pic_header.ph_dbr_param;
#endif

    for (int mb_y = pel_y >> LOOPFILTER_SIZE_IN_BIT; mb_y < ((pel_y + cuh) >> LOOPFILTER_SIZE_IN_BIT); mb_y++)
    {
        for (int mb_x = pel_x >> LOOPFILTER_SIZE_IN_BIT; mb_x < ((pel_x + cuw) >> LOOPFILTER_SIZE_IN_BIT); mb_x++)
        {
            //vertical
            deblock_mb_avs2(info, map, pic, refp, edge_filter, mb_y, mb_x, 0
#if DBR
                , NULL, 0, &dbr_param
#endif
#if RDO_DBK_LUMA_ONLY
                , 1
#endif
            );
        }
    }
    for (int mb_y = pel_y >> LOOPFILTER_SIZE_IN_BIT; mb_y < ((pel_y + cuh) >> LOOPFILTER_SIZE_IN_BIT); mb_y++)
    {
        for (int mb_x = pel_x >> LOOPFILTER_SIZE_IN_BIT; mb_x < ((pel_x + cuw) >> LOOPFILTER_SIZE_IN_BIT); mb_x++)
        {
            //horizontal
            deblock_mb_avs2(info, map, pic, refp, edge_filter, mb_y, mb_x, 1
#if DBR
                , NULL, 0, &dbr_param
#endif
#if RDO_DBK_LUMA_ONLY
                , 1
#endif
            );
        }
    }
}

int printf_flag = 0;

//deblock one frame
void deblock_frame_avs2(COM_INFO *info, COM_MAP *map, COM_PIC * pic, COM_REFP refp[MAX_NUM_REF_PICS][REFP_NUM], int*** edge_filter)
{
    int mb_x, mb_y;
    int blk_nr = -1;
#if DBR
    DBR_PARAM dbr_param = info->pic_header.ph_dbr_param;
#endif

    printf_flag = 1;

    for (mb_y = 0; mb_y < (pic->height_luma >> LOOPFILTER_SIZE_IN_BIT); mb_y++)
    {
        for (mb_x = 0; mb_x < (pic->width_luma >> LOOPFILTER_SIZE_IN_BIT); mb_x++)
        {
            blk_nr++;
            //vertical
            deblock_mb_avs2(info, map, pic, refp, edge_filter, mb_y, mb_x, 0
#if DBR
                , NULL, 0, &dbr_param
#endif
#if RDO_DBK_LUMA_ONLY
                , 0
#endif
            );
        }
    }
    blk_nr = -1;
    for (mb_y = 0; mb_y < (pic->height_luma >> LOOPFILTER_SIZE_IN_BIT); mb_y++)
    {
        for (mb_x = 0; mb_x < (pic->width_luma >> LOOPFILTER_SIZE_IN_BIT); mb_x++)
        {
            blk_nr++;
            //horizontal
            deblock_mb_avs2(info, map, pic, refp, edge_filter, mb_y, mb_x, 1
#if DBR
                , NULL, 0, &dbr_param
#endif
#if RDO_DBK_LUMA_ONLY
                , 0
#endif
            );
        }
    }
    printf_flag = 0;
}