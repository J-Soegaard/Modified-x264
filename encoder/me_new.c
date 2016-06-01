#include "common/common.h"
#include "macroblock.h"
#include "me.h"

/* presets selected from good points on the speed-vs-quality curve of several test videos
 * subpel_iters[i_subpel_refine] = { refine_hpel, refine_qpel, me_hpel, me_qpel }
 * where me_* are the number of EPZS iterations run on all candidate block types,
 * and refine_* are run only on the winner.
 * the subme=8,9 values are much higher because any amount of satd search makes
 * up its time by reducing the number of qpel-rd iterations. */
 static const uint8_t subpel_iterations[][4] =
 {{0,0,0,0},
 {1,1,0,0},
 {0,1,1,0},
 {0,2,1,0},
 {0,2,1,1},
 {0,2,1,2},
 {0,0,2,2},
 {0,0,2,2},
 {0,0,4,10},
 {0,0,4,10},
 {0,0,4,10},
 {0,0,4,10}};

/* (x-1)%6 */
 static const uint8_t mod6m1[8] = {5,0,1,2,3,4,5,0};
/* radius 2 hexagon. repeated entries are to avoid having to compute mod6 every time. */
 static const int8_t hex2[8][2] = {{-1,-2}, {-2,0}, {-1,2}, {1,2}, {2,0}, {1,-2}, {-1,-2}, {-2,0}};
 static const int8_t square1[9][2] = {{0,0}, {0,-1}, {0,1}, {-1,0}, {1,0}, {-1,-1}, {-1,1}, {1,-1}, {1,1}};

#define BITS_MVD( mx, my )\
 (p_cost_mvx[(mx)<<2] + p_cost_mvy[(my)<<2])

#define COST_MV( mx, my )\
 do\
 {\
    int cost = h->pixf.fpelcmp[i_pixel]( p_fenc, FENC_STRIDE,\
     &p_fref_w[(my)*stride+(mx)], stride )\
    + BITS_MVD(mx,my);\
    COPY3_IF_LT( bcost, cost, bmx, mx, bmy, my );\
} while(0)

#define COST_MV_HPEL( mx, my, cost )\
do\
{\
    intptr_t stride2 = 16;\
    pixel *src = h->mc.get_ref( pix, &stride2, m->p_fref, stride, mx, my, bw, bh, &m->weight[0] );\
    cost = h->pixf.fpelcmp[i_pixel]( p_fenc, FENC_STRIDE, src, stride2 )\
    + p_cost_mvx[ mx ] + p_cost_mvy[ my ];\
} while(0)

#define COST_MV_X3_DIR( m0x, m0y, m1x, m1y, m2x, m2y, costs )\
{\
    pixel *pix_base = p_fref_w + bmx + bmy*stride;\
    h->pixf.fpelcmp_x3[i_pixel]( p_fenc,\
        pix_base + (m0x) + (m0y)*stride,\
        pix_base + (m1x) + (m1y)*stride,\
        pix_base + (m2x) + (m2y)*stride,\
        stride, costs );\
    (costs)[0] += BITS_MVD( bmx+(m0x), bmy+(m0y) );\
    (costs)[1] += BITS_MVD( bmx+(m1x), bmy+(m1y) );\
    (costs)[2] += BITS_MVD( bmx+(m2x), bmy+(m2y) );\
}

#define COST_MV_X4_DIR( m0x, m0y, m1x, m1y, m2x, m2y, m3x, m3y, costs )\
{\
    pixel *pix_base = p_fref_w + bmx + bmy*stride;\
    h->pixf.fpelcmp_x4[i_pixel]( p_fenc,\
        pix_base + (m0x) + (m0y)*stride,\
        pix_base + (m1x) + (m1y)*stride,\
        pix_base + (m2x) + (m2y)*stride,\
        pix_base + (m3x) + (m3y)*stride,\
        stride, costs );\
    (costs)[0] += BITS_MVD( bmx+(m0x), bmy+(m0y) );\
    (costs)[1] += BITS_MVD( bmx+(m1x), bmy+(m1y) );\
    (costs)[2] += BITS_MVD( bmx+(m2x), bmy+(m2y) );\
    (costs)[3] += BITS_MVD( bmx+(m3x), bmy+(m3y) );\
}

#define COST_MV_X4( m0x, m0y, m1x, m1y, m2x, m2y, m3x, m3y )\
{\
    pixel *pix_base = p_fref_w + omx + omy*stride;\
    h->pixf.fpelcmp_x4[i_pixel]( p_fenc,\
        pix_base + (m0x) + (m0y)*stride,\
        pix_base + (m1x) + (m1y)*stride,\
        pix_base + (m2x) + (m2y)*stride,\
        pix_base + (m3x) + (m3y)*stride,\
        stride, costs );\
    costs[0] += BITS_MVD( omx+(m0x), omy+(m0y) );\
    costs[1] += BITS_MVD( omx+(m1x), omy+(m1y) );\
    costs[2] += BITS_MVD( omx+(m2x), omy+(m2y) );\
    costs[3] += BITS_MVD( omx+(m3x), omy+(m3y) );\
    COPY3_IF_LT( bcost, costs[0], bmx, omx+(m0x), bmy, omy+(m0y) );\
    COPY3_IF_LT( bcost, costs[1], bmx, omx+(m1x), bmy, omy+(m1y) );\
    COPY3_IF_LT( bcost, costs[2], bmx, omx+(m2x), bmy, omy+(m2y) );\
    COPY3_IF_LT( bcost, costs[3], bmx, omx+(m3x), bmy, omy+(m3y) );\
}

#define COST_MV_X3_ABS( m0x, m0y, m1x, m1y, m2x, m2y )\
{\
    h->pixf.fpelcmp_x3[i_pixel]( p_fenc,\
        p_fref_w + (m0x) + (m0y)*stride,\
        p_fref_w + (m1x) + (m1y)*stride,\
        p_fref_w + (m2x) + (m2y)*stride,\
        stride, costs );\
    costs[0] += p_cost_mvx[(m0x)<<2]; /* no cost_mvy */\
    costs[1] += p_cost_mvx[(m1x)<<2];\
    costs[2] += p_cost_mvx[(m2x)<<2];\
    COPY3_IF_LT( bcost, costs[0], bmx, m0x, bmy, m0y );\
    COPY3_IF_LT( bcost, costs[1], bmx, m1x, bmy, m1y );\
    COPY3_IF_LT( bcost, costs[2], bmx, m2x, bmy, m2y );\
}

/*  1  */
/* 101 */
/*  1  */
#define DIA1_ITER( mx, my )\
{\
    omx = mx; omy = my;\
    COST_MV_X4( 0,-1, 0,1, -1,0, 1,0 );\
}

#define CROSS( start, x_max, y_max )\
{\
    int i = start;\
    if( (x_max) <= X264_MIN(mv_x_max-omx, omx-mv_x_min) )\
        for( ; i < (x_max)-2; i+=4 )\
            COST_MV_X4( i,0, -i,0, i+2,0, -i-2,0 );\
        for( ; i < (x_max); i+=2 )\
            {\
                if( omx+i <= mv_x_max )\
                    COST_MV( omx+i, omy );\
                if( omx-i >= mv_x_min )\
                    COST_MV( omx-i, omy );\
            }\
            i = start;\
            if( (y_max) <= X264_MIN(mv_y_max-omy, omy-mv_y_min) )\
                for( ; i < (y_max)-2; i+=4 )\
                    COST_MV_X4( 0,i, 0,-i, 0,i+2, 0,-i-2 );\
                for( ; i < (y_max); i+=2 )\
                    {\
                        if( omy+i <= mv_y_max )\
                            COST_MV( omx, omy+i );\
                        if( omy-i >= mv_y_min )\
                            COST_MV( omx, omy-i );\
                    }\
                }

#define FPEL(mv) (((mv)+2)>>2) /* Convert subpel MV to fullpel with rounding... */
#define SPEL(mv) ((mv)<<2)     /* ... and the reverse. */
#define SPELx2(mv) (SPEL(mv)&0xFFFCFFFC) /* for two packed MVs */

void clip_and_remove_duplicates_in_MV_candidates( int bmx, int bmy, int16_t (*mvc)[2], int *i_mvc, int *b_mvc, int *c_mvc, int stride, int mb_count){

    int i = i_mvc[0];
    int b = b_mvc[0];
    int k,j;    
    char test;
    ALIGNED_4( int16_t new_mvc[10][2] );
    int n_i = 0;
    int n_b = 0;
    int n_c = 0;

    for( k=0 ; k<i ; k++ )
    {
        test = 1;

        if( mvc[k][1]*stride+mvc[k][0] < 0 || mvc[k][1]*stride+mvc[k][0] > mb_count){
            test = 0;
        }
        else if( (mvc[k][0] == bmx) && (mvc[k][1] == bmy) )
        {
            test = 0;
        }
        else
        {
            for( j=0 ; j<n_i ; j++ )
            {
                if( (mvc[k][0] == new_mvc[j][0]) && (mvc[k][1] == new_mvc[j][1]) )
                {
                    test = 0;
                    break;   
                }
            }
        }

        if( test )
        {
            new_mvc[n_i][0] = mvc[k][0];
            new_mvc[n_i][1] = mvc[k][1];
            n_i++;
            if( k<b )
                n_b++;
            else
                n_c++;
        }        
    }   

    mvc = new_mvc;
    *i_mvc = n_i;
    *b_mvc = n_b;
    *c_mvc = n_c;

}

void x264_me_search_ref_EPZS( x264_t *h, x264_me_t *m, int16_t (*mvc)[2], int i_mvc, int b_mvc, int *p_halfpel_thresh )
{
    const int bw = x264_pixel_size[m->i_pixel].w;   /* Blockwidth */
    const int bh = x264_pixel_size[m->i_pixel].h;   /* Blockheight */
    const int i_pixel = m->i_pixel;                 /* Block size type */
    const int stride = m->i_stride[0];           

    /* Thresholds for early stopping */
    int T1 = 256 + 15; /* +15 due to the cost of MVs */
    int T2 = T1;
    int T3 = T2;
    int cost_left = h->mb.i_mb_cost[h->mb.i_mb_left_xy[0]];
    int cost_top = h->mb.i_mb_cost[h->mb.i_mb_top_xy];
    int cost_topright = h->mb.i_mb_cost[h->mb.i_mb_topright_xy];
    int cost_prev = 100000;

    int i_me_range = h->param.analyse.i_me_range;
    int bmx, bmy, bcost = COST_MAX;
    int bpred_cost = COST_MAX;
    int c_mvc = i_mvc-b_mvc;
    pixel *p_fenc = m->p_fenc[0];
    pixel *p_fref_w = m->p_fref_w;
    ALIGNED_ARRAY_16( int, costs,[16] );
    
    int mv_x_min = h->mb.mv_limit_fpel[0][0];
    int mv_y_min = h->mb.mv_limit_fpel[0][1];
    int mv_x_max = h->mb.mv_limit_fpel[1][0];
    int mv_y_max = h->mb.mv_limit_fpel[1][1];
    /* Special version of pack to allow shortcuts in CHECK_MVRANGE */
    #define pack16to32_mask2(mx,my) ((mx<<16)|(my&0x7FFF))
    uint32_t mv_min = pack16to32_mask2( -mv_x_min, -mv_y_min );
    uint32_t mv_max = pack16to32_mask2( mv_x_max, mv_y_max )|0x8000;
    uint32_t bpred_mv = 0;
    
    #define CHECK_MVRANGE(mx,my) (!(((pack16to32_mask2(mx,my) + mv_min) | (mv_max - pack16to32_mask2(mx,my))) & 0x80004000))
    
    const uint16_t *p_cost_mvx = m->p_cost_mv - m->mvp[0];
    const uint16_t *p_cost_mvy = m->p_cost_mv - m->mvp[1];  
    
    /* Calculate and check the fullpel MVP first */
    bmx = x264_clip3( FPEL(m->mvp[0]), mv_x_min, mv_x_max );
    bmy = x264_clip3( FPEL(m->mvp[1]), mv_y_min, mv_y_max );

    /* Remove duplicates */
    clip_and_remove_duplicates_in_MV_candidates( bmx, bmy, mvc, &i_mvc, &b_mvc, &c_mvc, stride, h->mb.i_mb_count);

    /* Calculate Thresholds from EPZS paper */
    if( h->fref[0][0]->i_ref[0] > 0 )
        cost_prev = h->fref[0][0]->mv_cost[h->mb.i_mb_xy];
    if( cost_topright == 0 )
        cost_topright = 100000;
    if( cost_top == 0 )
        cost_top = 100000;
    if( cost_left == 0 )
        cost_left = 100000;
    T2 = X264_MIN( cost_prev, cost_topright );
    T2 = X264_MIN( T2, cost_top );
    T2 = X264_MIN( T2, cost_left );
    T2 = 1.2 * T2 + 128;
    T3 = T2;

    /* JSOG: Print MB number and info */
    printf("\nMB %3i x %3i, Size B: %i Size C: %i \n",h->mb.i_mb_x,h->mb.i_mb_y,b_mvc,c_mvc);
    printf("SP:");

    /* Candidate set A */
    bcost = h->pixf.fpelcmp[i_pixel]( p_fenc, FENC_STRIDE, &p_fref_w[bmy*stride+bmx], stride ) + BITS_MVD( bmx, bmy );
    printf("%4i ",bw*bh); /* JSOG: Search Positions (SP) */
    
    if( bcost < T1 )
        goto early_termination;

    /* Candidate set B */
    int i = 0;
    int mx, my, cost, best_i;
    if( b_mvc > 0 )
    {
        best_i = 0;
        for( ; i<b_mvc ; i++ ){
            mx = mvc[i][0];
            my = mvc[i][1];
            cost = h->pixf.fpelcmp[i_pixel]( p_fenc, FENC_STRIDE, &p_fref_w[my*stride+mx], stride ) + BITS_MVD( mx, my );
            printf("%4i ",bw*bh); /* JSOG: Search Positions (SP) */
            COPY2_IF_LT( bcost, cost, best_i, i );
        }

        if(best_i){
            bmx = mvc[best_i][0];
            bmy = mvc[best_i][1];
        }        
    }

    if( bcost < T2 )
        goto early_termination;

    /* Candidate set C */
    if( c_mvc > 0 )
    {        
        best_i = 0;
        for( ; i<i_mvc ; i++ ){
            mx = mvc[i][0];
            my = mvc[i][1];
            cost = h->pixf.fpelcmp[i_pixel]( p_fenc, FENC_STRIDE, &p_fref_w[my*stride+mx], stride ) + BITS_MVD( mx, my );
            printf("%4i ",bw*bh); /* JSOG: Search Positions (SP) */
            COPY2_IF_LT( bcost, cost, best_i, i );
        }

        if(best_i){
            bmx = mvc[best_i][0];
            bmy = mvc[best_i][1];
        }        
    }
    
    if( bcost < T3 )
        goto early_termination;    

    /* diamond search, radius 1 */
    bcost <<= 4;
    i = i_me_range;
    do
    {
        COST_MV_X4_DIR( 0,-1, 0,1, -1,0, 1,0, costs );
                    printf("%4i ",4*bw*bh); /* JSOG: Search Positions (SP) */
        COPY1_IF_LT( bcost, (costs[0]<<4)+1 );
        COPY1_IF_LT( bcost, (costs[1]<<4)+3 );
        COPY1_IF_LT( bcost, (costs[2]<<4)+4 );
        COPY1_IF_LT( bcost, (costs[3]<<4)+12 );
        if( !(bcost&15) )
            break;
        bmx -= (bcost<<28)>>30;
        bmy -= (bcost<<30)>>30;
        bcost &= ~15;
    } while( --i && CHECK_MVRANGE(bmx, bmy) );
    bcost >>= 4;


    /* -> qpel mv */   
    early_termination: 
    if( h->mb.i_subpel_refine < 3 )
    {
        uint32_t bmv = pack16to32_mask(bmx,bmy);
        uint32_t bmv_spel = SPELx2(bmv);
        m->cost_mv = p_cost_mvx[bmx<<2] + p_cost_mvy[bmy<<2];
        m->cost = bcost;
        M32( m->mv ) = bmv_spel;
    }
    else
    {
        uint32_t bmv = pack16to32_mask(bmx,bmy);
        uint32_t bmv_spel = SPELx2(bmv);
        M32(m->mv) = bpred_cost < bcost ? bpred_mv : bmv_spel;
        m->cost = X264_MIN( bpred_cost, bcost );
    }

/*    printf("\n Prev cost: %i ",cost_left);
    printf("\n Current cost: %i ", bcost);

    if( h->fref[0][0]->i_ref[0] > 0 )
    {
        x264_frame_t *l0 = h->fref[0][0];
        printf("\n Prev cost: %i ",l0->mv_cost[h->mb.i_mb_xy]);
    } */


}

#undef COST_MV
