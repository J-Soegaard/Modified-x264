#include "common.h"

/*
Find set of MV candidates based on EPZS (but without "accelerated" MV candidate)
b_mvc is the size of set B while the size of set C = i_mvc - b_mvc.
*/
void x264_mb_predict_mv_ref16x16_EPZS( x264_t *h, int i_list, int i_ref, int16_t mvc[10][2], int *i_mvc, int *b_mvc)
{
    int16_t (*mvr)[2] = h->mb.mvr[i_list][i_ref];
    int i = 0, b = 0; // Counter for set B and C

    
    /* No (0,0) MV */ 
    /* mvc[i][0] = 0;
    mvc[i][1] = 0; // (0,0) MV
    i++; */ 

    /* EPZS Set B */
    if( h->fref[0][0]->i_ref[0] > 0 )
    {
        x264_frame_t *l0 = h->fref[0][0];
        int field = h->mb.i_mb_y&1;
        int curpoc = h->fdec->i_poc + h->fdec->i_delta_poc[field];
        int refpoc = h->fref[i_list][i_ref>>SLICE_MBAFF]->i_poc;
        refpoc += l0->i_delta_poc[field^(i_ref&1)];

        #define SET_TMVP( dx, dy ) \
        { \
            int mb_index = h->mb.i_mb_xy + dx + dy*h->mb.i_mb_stride; \
            int scale = (curpoc - refpoc) * l0->inv_ref_poc[MB_INTERLACED&field]; \
            mvc[i][0] = (l0->mv16x16[mb_index][0]*scale + 128) >> 8; \
            mvc[i][1] = (l0->mv16x16[mb_index][1]*scale + 128) >> 8; \
            i++; \
        }
        SET_TMVP(0,0);                          // Co-located at t-1 (Only part of set B)
        b++;

        /* EPZS Set C */ // Without "accelerated" MV predictor
        if( h->mb.i_mb_x < h->mb.i_mb_width-1 )
            SET_TMVP(1,0); // Right neighbour at t-1
        if( h->mb.i_mb_y < h->mb.i_mb_height-1 )
            SET_TMVP(0,1); // Bottom neighbour at t-1
        if( h->mb.i_mb_x > 0 )
            SET_TMVP(-1,0); // Left neighbour at t-1
        if( h->mb.i_mb_y > 0 )
            SET_TMVP(0,-1); // Top neighbour at t-1

        #undef SET_TMVP        
    }   

    #define SET_MVP(mvp) \
    { \
        CP32( mvc[i], mvp ); \
        i++; \
    }

    #define SET_IMVP(xy) if( xy >= 0 ) \
    { \
        int shift = 1 + MB_INTERLACED - h->mb.field[xy]; \
        int16_t *mvp = h->mb.mvr[i_list][i_ref<<1>>shift][xy]; \
        mvc[i][0] = mvp[0]; \
        mvc[i][1] = mvp[1]<<1>>shift; \
        i++; \
    }
    SET_MVP( mvr[h->mb.i_mb_left_xy[0]] );  // Spatial left
    SET_MVP( mvr[h->mb.i_mb_top_xy] );      // Spatial top
    SET_MVP( mvr[h->mb.i_mb_topright_xy] ); // Spatial top-right

    #undef SET_IMVP
    #undef SET_MVP

    *i_mvc = i;
    *b_mvc = b;

}
