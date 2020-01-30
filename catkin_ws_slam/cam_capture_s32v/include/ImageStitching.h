#ifndef _IMAGESTITCHING_H_

#define _IMAGESTITCHING_H_

#define Uint32  unsigned int
#define Uint16  unsigned short
#define Uint8   unsigned char
#define S08 char
#define S32 int
#define U08 unsigned S08
#define U16 unsigned short
#define U32 unsigned int
typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned long DWORD;
#define IMG_WIDTH         1280
#define IMG_HEIGHT        720
#define BACK_COEF				(256)
#define FORWARD		(0u)
#define REVERSE		(1u)
#define  DST_HIGHT  1248//804// 1665
#define  DST_WIDTH   804//1248// 1056
/************************************************************************/
///unsigned int *CAPTURE_MEM_FRONT_cpy[IMG_HEIGHT];
///unsigned int *CAPTURE_MEM_back_cpy[IMG_HEIGHT];
///unsigned int *CAPTURE_MEM_left_cpy[IMG_HEIGHT];
///unsigned int *CAPTURE_MEM_right_cpy[IMG_HEIGHT];
#define LUT_FSV_VIEW   1280*720
#define LUT_WT_FB      507*804// 655*1056// (329472)
#define LUT_WT_LR      356*1248//1664*464//360448
#define LUT_POS_FB     507*804//655*1056 //329472
#define LUT_POS_LR     356*1248// 1664*464//360448

//unsigned int Lut_Fsv_View[1280*720];
//unsigned int Lut_Front[LUT_POS_FB];
//unsigned int Lut_Back[LUT_POS_FB];
//unsigned int Lut_Left[LUT_POS_LR];
//unsigned int Lut_Right[LUT_POS_LR];
//unsigned long int Wt_Lut_Front[LUT_WT_FB];
//unsigned long int Wt_Lut_Back[LUT_WT_FB];
//unsigned long int Wt_Lut_Left[LUT_WT_LR];
//unsigned long int Wt_Lut_Right[LUT_WT_LR];
//unsigned char  SVM_BUFFERuyvy[DST_WIDTH * DST_HIGHT * 2];
//unsigned char  SVM_BUFFERuyvy270[DST_WIDTH * DST_HIGHT * 2];
//unsigned char  SVM_BUFFERyuyv[DST_WIDTH * DST_HIGHT * 2];


/****************************************************************************/
void stitching_block2();

void stitchPart(Uint32 **rawDataAddr, Uint16 ccdNum, Uint16 rowStart, Uint16 rowEnd, Uint16 colStart, Uint16 colEnd);

void stitchFusion(Uint32 **rawDataAddrFB, Uint32 **rawDataAddrLR, int fusionBlock, Uint16 rowStart, Uint16 rowEnd, Uint16 colStart, Uint16 colEnd);

void Resize_Interpolation_SingleView(unsigned short *resize_lut, unsigned
                                        int **remap_lut, unsigned char * dout,
                                        int Nwidth, int Nheight);
int UndistortionSVProccess(unsigned int * p_lut,
						   unsigned char* result_image,
						   unsigned char* source_image,
						   int result_height,
						   int result_width,
						   int source_height,
						   int source_width,
						   int cam_id);
#endif
