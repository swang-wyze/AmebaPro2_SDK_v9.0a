/*------------------------------------------------------------------------------
--                                                                                                                               --
--       This software is confidential and proprietary and may be used                                   --
--        only as expressly authorized by a licensing agreement from                                     --
--                                                                                                                               --
--                            Verisilicon.                                                                                    --
--                                                                                                                               --
--                   (C) COPYRIGHT 2014 VERISILICON                                                            --
--                            ALL RIGHTS RESERVED                                                                    --
--                                                                                                                               --
--                 The entire notice above must be reproduced                                                 --
--                  on all copies and should not be removed.                                                    --
--                                                                                                                               --
--------------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/wait.h>
#include "base_type.h"

//#include "error.h"
//#include "test_bench_utils.h"
//#include "get_option.h"
#include "hal_video_cmd.h"

#include "hal_video.h"

#define MAXARGS 128
#define CMDLENMAX 256


/* Command line options */
static struct option jpeg_options[] = {
	{"help", 'H', 0},

	{"output", 'o', 1},
	{"firstPic", 'a', 1},
	{"lastPic", 'b', 1},
	{"lumWidthSrc", 'w', 1},
	{"lumHeightSrc", 'h', 1},
	{"width", 'x', 1},
	{"height", 'y', 1},
	{"horOffsetSrc", 'X', 1},
	{"verOffsetSrc", 'Y', 1},
	{"restartInterval", 'R', 1},
	{"qLevel", 'q', 1},
	{"frameType", 'g', 1},
	{"colorConversion", 'v', 1},
	{"rotation", 'G', 1},
	{"codingType", 'p', 1},
	{"codingMode", 'm', 1},
	{"markerType", 't', 1},
	{"units", 'u', 1},
	{"xdensity", 'k', 1},
	{"ydensity", 'l', 1},
	{"write", 'W', 1},
	{"trigger", 'P', 1},
	{"inputAlignmentExp", 'Q', 1},
	{"input", 'i', 1},
	{"hashtype", 'A', 1},  /* hash frame data, 0--disable, 1--crc32, 2--checksum */
	//{"mirror", 'M', 1},	// Ameba_pro2 non support mirror
	{"enableConstChroma", 'd', 1},
	{"constCb", 'e', 1},
	{"constCr", 'f', 1},
	{"frameRateNum", 'n', 1},
	{"frameRateDenom", 'r', 1},
	{"qpMin", 'E', 1},
	{"qpMax", 'F', 1},
	{"rcMode", 'V', 1 },
	{"picQpDeltaRange", 'U', 1 },
	{"fixedQP", 'O', 1},
	{"voe", '0', 1},
	{NULL, 0, 0}
};

static struct option options[] = {
	{"help",    'H', 2},
	{"firstPic", 'a', 1},
	{"lastPic",  'b', 1},
	{"width", 'x', 1},
	{"height", 'y', 1},
	{"lumWidthSrc", 'w', 1},
	{"lumHeightSrc", 'h', 1},
	{"horOffsetSrc", 'X', 1},
	{"verOffsetSrc", 'Y', 1},
	{"inputFormat", 'l', 1},        /* Input image format */
	{"colorConversion", 'O', 1},    /* RGB to YCbCr conversion type */
	{"rotation", 'r', 1},           /* Input image rotation */
	{"outputRateNumer", 'f', 1},
	{"outputRateDenom", 'F', 1},
	{"inputRateNumer", 'j', 1},
	{"inputRateDenom", 'J', 1},

	/*stride*/
	{"inputAlignmentExp", '0', 1},
	{"refAlignmentExp", '0', 1},
	{"refChromaAlignmentExp",  '0',  1},
	{"input",   'i', 1},
	{"output",    'o', 1},
	{"cabacInitFlag", 'p', 1},
	{"qp_size",   'Q', 1},
	{"qpMinI", '0', 1},             /* Minimum frame header qp for I picture */
	{"qpMaxI", '0', 1},             /* Maximum frame header qp for I picture */
	{"qpMin", 'n', 1},              /* Minimum frame header qp for any picture */
	{"qpMax", 'm', 1},              /* Maximum frame header qp for any picture */
	{"qpHdr",     'q', 1},
	{"hrdConformance", 'C', 1},     /* HDR Conformance (ANNEX C) */
	{"cpbSize", 'c', 1},            /* Coded Picture Buffer Size */
	{"vbr", '0', 1},                /* Variable Bit Rate Control by qpMin */
	{"intraQpDelta", 'A', 1},       /* QP adjustment for intra frames */
	{"fixedIntraQp", 'G', 1},       /* Fixed QP for all intra frames */
	{"bFrameQpDelta", 'V', 1},	  /* QP adjustment for B frames */
	{"byteStream", 'N', 1},         /* Byte stream format (ANNEX B) */
	{"bitPerSecond", 'B', 1},
	{"picRc",   'U', 1},
	{"ctbRc",   'u', 1},
	{"picSkip", 's', 1},            /* Frame skipping */
	{"profile",       'P', 1},   	  /* profile   (ANNEX A):support main and main still picture */
	{"level",         'L', 1},      /* Level * 30  (ANNEX A) */
	{"intraPicRate",  'R', 1},      /* IDR interval */
	{"bpsAdjust", '1', 1},          /* Setting bitrate on the fly */
	{"bitrateWindow", 'g', 1},      /* bitrate window of pictures length */
	{"disableDeblocking", 'D', 1},
	{"tc_Offset", 'W', 1},
	{"beta_Offset", 'E', 1},
	{"sliceSize", 'e', 1},
	{"chromaQpOffset", 'I', 1},     /* Chroma qp index offset */
	{"enableSao", 'M', 1},          /* Enable or disable SAO */
	{"videoRange", 'k', 1},
	{"sei", 'S', 1},                /* SEI messages */
	{"codecFormat", '0', 1},         /* select videoFormat: HEVC/H264/JPEG/NV12/RGB */
	{"enableCabac", 'K', 1},        /* H.264 entropy coding mode, 0 for CAVLC, 1 for CABAC */
	{"userData", 'z', 1},           /* SEI User data file */
	{"videoStab", 'Z', 1},          /* video stabilization */

	/* Only long option can be used for all the following parameters because
	 * we have no more letters to use. All shortOpt=0 will be identified by
	 * long option. */
	{"cir", '0', 1},
	{"intraArea", '0', 1},
	{"ipcm1Area", '0', 1},
	{"ipcm2Area", '0', 1},
	{"roi1Area", '0', 1},
	{"roi2Area", '0', 1},
	{"roi1DeltaQp", '0', 1},
	{"roi2DeltaQp", '0', 1},
	{"roi1Qp", '0', 1},
	{"roi2Qp", '0', 1},
	{"roi3Area", '0', 1},
	{"roi3DeltaQp", '0', 1},
	{"roi3Qp", '0', 1},
	{"roi4Area", '0', 1},
	{"roi4DeltaQp", '0', 1},
	{"roi4Qp", '0', 1},
	{"roi5Area", '0', 1},
	{"roi5DeltaQp", '0', 1},
	{"roi5Qp", '0', 1},
	{"roi6Area", '0', 1},
	{"roi6DeltaQp", '0', 1},
	{"roi6Qp", '0', 1},
	{"roi7Area", '0', 1},
	{"roi7DeltaQp", '0', 1},
	{"roi7Qp", '0', 1},
	{"roi8Area", '0', 1},
	{"roi8DeltaQp", '0', 1},
	{"roi8Qp", '0', 1},

	{"ipcmFilterDisable", '0', 1},

	{"roiMapDeltaQpBlockUnit", '0', 1},
	{"roiMapDeltaQpEnable", '0', 1},
	{"ipcmMapEnable", '0', 1},

	{"outBufSizeMax", '0', 1},

	{"constrainIntra", '0', 1},
	{"smoothingIntra", '0', 1},
	{"scaledWidth", '0', 1},
	{"scaledHeight", '0', 1},
	{"scaledOutputFormat", '0', 1},
	{"enableDeblockOverride", '0', 1},
	{"deblockOverride", '0', 1},
	{"enableScalingList", '0', 1},
	{"compressor", '0', 1},
	{"testId", '0', 1},            /* TestID for generate vector. */
	{"gopSize", '0', 1},
	{"gopConfig", '0', 1},
	{"gopLowdelay", '0', 1},
	{"LTR", '0', 1},
	{"interlacedFrame", '0', 1},
	{"fieldOrder", '0', 1},
	{"outReconFrame", '0', 1},
	{"tier", '0', 1},

	{"gdrDuration", '0', 1},
	{"bitVarRangeI", '0', 1},
	{"bitVarRangeP", '0', 1},
	{"bitVarRangeB", '0', 1},
	{"smoothPsnrInGOP", '0', 1},
	{"staticSceneIbitPercent", '0', 1},
	{"tolCtbRcInter", '0', 1},
	{"tolCtbRcIntra", '0', 1},
	{"ctbRowQpStep", '0', 1},

	{"tolMovingBitRate", '0', 1},
	{"monitorFrames", '0', 1},
	{"roiMapDeltaQpFile", '0', 1},
	{"roiMapDeltaQpBinFile", '0', 1},
	{"ipcmMapFile", '0', 1},
	{"roiMapInfoBinFile", '0', 1},
	{"RoimapCuCtrlInfoBinFile", '0', 1},
	{"RoimapCuCtrlIndexBinFile", '0', 1},
	{"RoiCuCtrlVer", '0', 1},
	{"RoiQpDeltaVer", '0', 1 },
	/* multi stream configs */
	{"bitDepthLuma",   '0', 1},   /* luma bit depth */
	{"bitDepthChroma",   '0', 1},   /* chroma bit depth */
	{"blockRCSize",   '0', 1},   /*block rc size */
	{"rcQpDeltaRange",   '0', 1},   /* ctb rc qp delta range */
	{"rcBaseMBComplexity",   '0', 1},   /* ctb rc mb complexity base */
	{"picQpDeltaRange", '0', 1},
	{"enableOutputCuInfo", '0', 1},

	{"rdoLevel", '0', 1},
	{"enableRdoQuant", '0', 1},


	//{"mirror", '0', 1},  /* mirror */ // Ameba_pro2 non support mirror
	{"hashtype", '0', 1},  /* hash frame data, 0--disable, 1--crc32, 2--checksum */
	{"verbose", '0', 1}, /* print log verbose or not */
	/* constant chroma control */
	{"enableConstChroma", '0', 1}, /* enable constant chroma setting or not */
	{"constCb", '0', 1},           /* constant pixel value for CB */
	{"constCr", '0', 1},           /* constant pixel value for CR */

	{"sceneChange", '0', 1},
	{"enableVuiTimingInfo", '0', 1}, /* Write VUI timing info in SPS */
	{"lookaheadDepth", '0', 1},
	{"halfDsInput", '0', 1},  /* external 1/2 DS input yuv */
	{"cuInfoVersion", '0', 1},

	{"ssim", '0', 1},
	{"tile", '0', 1},
	{"skipFramePOC", '0', 1},

	{"RPSInSliceHeader", '0', 1},
	{"POCConfig", '0', 1},

	/* skip map */
	{"skipMapEnable", '0', 1},
	{"skipMapFile", '0', 1},
	{"skipMapBlockUnit", '0', 1},

	/* multi-core */
	{"parallelCoreNum",  '0', 1},   /* parallel core num */

	/* stream buffer chain */
	{"streamBufChain", '0', 1},
	/* AXI alignment */
	{"AXIAlignment", '0', 1},

	/*register dump*/
	{"dumpRegister", '0', 1},
	/* raster scan output for recon on FPGA & HW */
	{"rasterscan", '0', 1},

	/*CRF constant*/
	{"crf", '0', 1},

	{"ivf", '0', 1},
	{"codedChromaIdc", '0', 1},

	/*ME vertical search range*/
	{"MEVertRange", '0', 1},

	{"psyFactor", '0', 1},

	{"aq_mode", '0', 1},
	{"aq_strength", '0', 1},

	{"tune", '0', 1},
	{"preset", '0', 1},

	/*HW write recon to DDR or not if it's pure I-frame encoding*/
	{"writeReconToDDR", '0', 1},
	{"voe", '0', 1},
	{"dbg", '0', 1},

	{NULL,      0,   0}        /* Format of last line */
};

static void jpeg_help(void);
static void help(char *test_app);

static i32 long_option(i32 argc, char **argv, struct option *option,
					   struct parameter *parameter, char **p);
static i32 short_option(i32 argc, char **argv, struct option *option,
						struct parameter *parameter, char **p);
static i32 parse(i32 argc, char **argv, struct option *option,
				 struct parameter *parameter, char **p, u32 lenght);
static i32 enc_get_next(i32 argc, char **argv, struct parameter *parameter, char **p);

/*------------------------------------------------------------------------------
  get_option parses command line options. This function should be called
  with argc and argv values that are parameters of main(). The function
  will parse the next parameter from the command line. The function
  returns the next option character and stores the current place to
  structure argument. Structure option contain valid options
  and structure parameter contains parsed option.

  option options[] = {
    {"help",           'H', 0}, // No argument
    {"input",          'i', 1}, // Argument is compulsory
    {"output",         'o', 2}, // Argument is optional
    {NULL,              0,  0}  // Format of last line
  }

  Command line format can be
  --input filename
  --input=filename
  --inputfilename
  -i filename
  -i=filename
  -ifilename

  Input argc  Argument count as passed to main().
    argv  Argument values as passed to main().
    option  Valid options and argument requirements structure.
    parameter Option and argument return structure.

  Return  1 Unknown option.
    0 Option and argument are OK.
    -1  No more options.
    -2  Option match but argument is missing.
------------------------------------------------------------------------------*/
i32 get_option(i32 argc, char **argv, struct option *option,
			   struct parameter *parameter)
{
	char *p = NULL;
	i32 ret;

	parameter->argument = "?";
	parameter->short_opt = '?';
	parameter->enable = 0;

	if (enc_get_next(argc, argv, parameter, &p)) {
		return -1;  /* End of options */
	}

	/* Long option */
	ret = long_option(argc, argv, option, parameter, &p);
	if (ret != 1) {
		return ret;
	}

	/* Short option */
	ret = short_option(argc, argv, option, parameter, &p);
	if (ret != 1) {
		return ret;
	}

	/* This is unknow option but option anyway so argument must return */
	parameter->argument = p;

	return 1;
}

/*------------------------------------------------------------------------------
  long_option
------------------------------------------------------------------------------*/
i32 long_option(i32 argc, char **argv, struct option *option,
				struct parameter *parameter, char **p)
{
	i32 i = 0;
	u32 lenght;

	if (strncmp("--", *p, 2) != 0) {
		return 1;
	}

	while (option[i].long_opt != NULL) {
		lenght = strlen(option[i].long_opt);
		if (strncmp(option[i].long_opt, *p + 2, lenght) == 0) {
			goto match;
		}
		i++;
	}
	return 1;

match:
	lenght += 2;    /* Because option start -- */
	if (parse(argc, argv, &option[i], parameter, p, lenght) != 0) {
		return -2;
	}

	return 0;
}

/*------------------------------------------------------------------------------
  short_option
------------------------------------------------------------------------------*/
i32 short_option(i32 argc, char **argv, struct option *option,
				 struct parameter *parameter, char **p)
{
	i32 i = 0;
	char short_opt;

	if (strncmp("-", *p, 1) != 0) {
		return 1;
	}

	short_opt = *(*p + 1);
	//strncpy(&short_opt, *p + 1, 1);	// fixed warring message by Raymond
	while (option[i].long_opt != NULL) {
		if (option[i].short_opt  == short_opt) {
			goto match;
		}
		i++;
	}
	return 1;

match:
	if (parse(argc, argv, &option[i], parameter, p, 2) != 0) {
		return -2;
	}

	return 0;
}

/*------------------------------------------------------------------------------
  parse
------------------------------------------------------------------------------*/
i32 parse(i32 argc, char **argv, struct option *option,
		  struct parameter *parameter, char **p, u32 lenght)
{
	char *arg;

	parameter->short_opt = option->short_opt;
	parameter->longOpt = option->long_opt;
	arg = *p + lenght;

	/* Argument and option are together */
	if (strlen(arg) != 0) {
		/* There should be no argument */
		if (option->enable == 0) {
			return -1;
		}

		/* Remove = */
		if (strncmp("=", arg, 1) == 0) {
			arg++;
		}
		parameter->enable = 1;
		parameter->argument = arg;
		return 0;
	}

	/* Argument and option are separately */
	if (enc_get_next(argc, argv, parameter, p)) {
		/* There is no more parameters */
		if (option->enable == 1) {
			return -1;
		}
		return 0;
	}

	/* Parameter is missing if next start with "-" but next time this
	 * option is OK so we must fix parameter->cnt */
	if (strncmp("-", *p,  1) == 0) {
		parameter->cnt--;
		if (option->enable == 1) {
			return -1;
		}
		return 0;
	}

	/* There should be no argument */
	if (option->enable == 0) {
		return -1;
	}

	parameter->enable = 1;
	parameter->argument = *p;

	return 0;
}

/*------------------------------------------------------------------------------
  enc_get_next
------------------------------------------------------------------------------*/
i32 enc_get_next(i32 argc, char **argv, struct parameter *parameter, char **p)
{
	/* End of options */
	if ((parameter->cnt >= argc) || (parameter->cnt < 0)) {
		return -1;
	}
	*p = argv[parameter->cnt];
	parameter->cnt++;

	return 0;
}

/*------------------------------------------------------------------------------
------------------------------------------------------------------------------*/
int ParseDelim(char *optArg, char delim)
{
	i32 i;

	for (i = 0; i < (i32)strlen(optArg); i++)
		if (optArg[i] == delim) {
			optArg[i] = 0;
			return i;
		}

	return -1;
}

int HasDelim(char *optArg, char delim)
{
	i32 i;

	for (i = 0; i < (i32)strlen(optArg); i++)
		if (optArg[i] == delim) {
			return 1;
		}

	return 0;
}


/*------------------------------------------------------------------------------
  help
------------------------------------------------------------------------------*/
void help(char *test_app)
{
	//fprintf(stdout, "Version: %s\n", git_version);
	fprintf(stdout, "Usage:  %s [options] -i inputfile\n\n", test_app);
	fprintf(stdout,
			"Default parameters are marked inside []. More detailed descriptions of\n"
			"C-Model command line parameters can be found from C-Model Command Line API Manual.\n\n");

	fprintf(stdout,
			"  -H --help                        This help\n\n");

	fprintf(stdout,
			" Parameters affecting encoder input/output frames:\n"
			"  -i[s] --input                    Read input video sequence from file. [input.yuv]\n"
			"  -o[s] --output                   Write output HEVC/H.264 stream to file. [stream.hevc]\n"
			"  -a[n] --firstPic                 The first picture to encode in the input file. [0]\n"
			"  -b[n] --lastPic                  The last picture to encode in the input file. [100]\n"
			"        --outReconFrame            0..1 Reconstructed frame output mode. [1]\n"
			"                                     0 - Do not output reconstructed frames.\n"
			"                                     1 - Output reconstructed frames.\n"
			"  -f[n] --outputRateNumer          1..1048575 Output picture rate numerator. [30]\n"
			"  -F[n] --outputRateDenom          1..1048575 Output picture rate denominator. [1]\n"
			"                                   Encoded frame rate will be\n"
			"                                   outputRateNumer/outputRateDenom fps\n"
			"  -j[n] --inputRateNumer           1..1048575 Input picture rate numerator. [30]\n"
			"  -J[n] --inputRateDenom           1..1048575 Input picture rate denominator. [1]\n\n");

	fprintf(stdout,
			" Parameters affecting input frame and encoded frame resolutions and cropping:\n"
			"  -w[n] --lumWidthSrc              Width of source image in pixels. [176]\n"
			"  -h[n] --lumHeightSrc             Height of source image in pixels. [144]\n"
			"  -x[n] --width                    Width of encoded output image in pixels. [--lumWidthSrc]\n"
			"  -y[n] --height                   Height of encoded output image in pixels. [--lumHeightSrc]\n"
			"  -X[n] --horOffsetSrc             Output image horizontal cropping offset, must be even. [0]\n"
			"  -Y[n] --verOffsetSrc             Output image vertical cropping offset, must be even. [0]\n"
		   );

	fprintf(stdout,
			" \n Parameters affecting input frame and reference frame buffer alignment in memory:\n"
			"        --inputAlignmentExp        0, 4..12 Alignment value of input frame buffer. [4]\n"
			"                                     0 = Disable alignment\n"
			"                                     4..12 = Base address of input frame buffer and each line are aligned to 2^inputAlignmentExp.\n"
			"        --refAlignmentExp          0, 4..12 Reference frame buffer alignment [0]\n"
			"                                     0 = Disable alignment \n"
			"                                     4..12 = Base address of reference frame buffer and each line are aligned to 2^refAlignmentExp.\n"
			"        --refChromaAlignmentExp    0, 4..12 Reference frame buffer alignment for chroma [0]\n"
			"                                     0 = Disable alignment \n"
			"                                     4..12 = Base address of chroma reference frame buffer and each line are aligned to 2^refChromaAlignmentExp.\n"
		   );

	fprintf(stdout,
			"\n Parameters for pre-processing frames before encoding:\n"
			"  -l[n] --inputFormat              Input YUV format. [0]\n"
			"                                     0 - YUV420 planar CbCr (IYUV/I420)\n"
			"                                     1 - YUV420 semiplanar CbCr (NV12)\n"
			"                                     2 - YUV420 semiplanar CrCb (NV21)\n"
			"                                     3 - YUYV422 interleaved (YUYV/YUY2)\n"
			"                                     4 - UYVY422 interleaved (UYVY/Y422)\n"
			"                                     5 - RGB565 16bpp\n"
			"                                     6 - BGR565 16bpp\n"
			"                                     7 - RGB555 16bpp\n"
			"                                     8 - BGR555 16bpp\n"
			"                                     9 - RGB444 16bpp\n"
			/*
								  "                                     10 - BGR444 16bpp\n"
						          "                                     11 - RGB888 32bpp\n"
						          "                                     12 - BGR888 32bpp\n"
						          "                                     13 - RGB101010 32bpp\n"
						          "                                     14 - BGR101010 32bpp\n"
						          "                                     15 - YUV420 10 bit planar CbCr (I010)\n"
						          "                                     16 - YUV420 10 bit planar CbCr (P010)\n"
						          "                                     17 - YUV420 10 bit planar CbCr packed\n"
						          "                                     18 - YUV420 10 bit packed (Y0L2)\n"
						          "                                     19 - YUV420 customer private tile for HEVC\n"
						          "                                     20 - YUV420 customer private tile for H.264\n"
						          "                                     21 - YUV420 semiplanar CbCr customer private tile\n"
						          "                                     22 - YUV420 semiplanar CrCb customer private tile\n"
						          "                                     23 - YUV420 P010 customer private tile\n"
						          "                                     24 - YUV420 semiplanar 101010 \n"
						          "                                     26 - YUV420 8 bit tile 64x4 \n"
						          "                                     27 - YUV420 CbCr 8 bit tile 64x4 \n"
						          "                                     28 - YUV420 10 bit tile 32x4 \n"
						          "                                     29 - YUV420 10 bit tile 48x4 \n"
						          "                                     30 - YUV420 CrCb 10 bit tile 48x4 \n"
						          "                                     31 - YUV420 8 bit tile 128x2 \n"
						          "                                     32 - YUV420 CbCr 8 bit tile 128x2 \n"
						          "                                     33 - YUV420 10 bit tile 96x2 \n"
						          "                                     34 - YUV420 CrCb 10 bit tile 96x2 \n"
						          "                                     35 - YUV420 semiplanar 8 bit tile 8x8 \n"
						          "                                     36 - YUV420 semiplanar 10 bit tile 8x8 \n"
						          "  -O[n] --colorConversion          RGB to YCbCr color conversion type. [0]\n"
						          "                                     0 - ITU-R BT.601, RGB limited [16..235] (BT601_l.mat)\n"
						          "                                     1 - ITU-R BT.709, RGB limited [16..235] (BT709_l.mat)\n"
						          "                                     2 - User defined, coefficients defined in test bench.\n"
						          "                                     3 - ITU-R BT.2020\n"
						          "                                     4 - ITU-R BT.601, RGB full [0..255] (BT601_f.mat)\n"
						          "                                     5 - ITU-R BT.601, RGB limited [0..219] (BT601_219.mat)\n"
						          "                                     6 - ITU-R BT.709, RGB full [0..255] (BT709_f.mat)\n"
			*/

			// Ameba_pro2 non support mirror
			"  -r[n] --rotation                 Rotate input image. [0]\n"
			"                                     0 - disabled\n"
			"                                     1 -  90 degrees right\n"
			"                                     2 -  90 degrees left\n"
			"                                     3 - 180 degrees right\n"

			"  -Z[n] --videoStab               Enable video stabilization or scene change detection. [0]\n"
			"                                    Stabilization works by adjusting cropping offset for\n"
			"                                    every frame. Scene change detection encodes scene\n"
			"                                    changes as intra frames, it is enabled when\n"
			"                                    input resolution == encoded resolution.\n\n"

			"        --scaledWidth              0..width, width of encoder down-scaled image, 0=disable [0]\n"
			"        --scaledHeight             0..height, height of encoder down-scaled image, 0=disable [0], written in scaled.yuv\n"
			"        --scaledOutputFormat       scaler output frame format, 0=YUV422, 1=YUV420 semiplanar CbCr (NV12) [0]\n"
			"        --interlacedFrame          0=Input progressive field\n"
			"                                   1=Input frame with fields interlaced [0]\n"
			"        --fieldOrder               Interlaced field order, 0=bottom first, 1=top first [0]\n"
			"        --codedChromaIdc           coded chroma format idc.[1]\n"
			"                                     0-4:0:0\n"
			"                                     1-4:2:0\n"
		   );
	fprintf(stdout,
			"\n Parameters affecting the output stream and encoding tools:\n"
			"  -P[n] --profile                  VC8000E/HEVC encoder profile [0]\n"
			"                                     VC8000E/HEVC encoder only supports Main profile, Main Still Picture profile, and Main 10 profile.\n"
			"                                     0 - Main profile\n"
			"                                     1 - Main Still Picture profile\n"
			"                                     2 - Main 10 profile\n"
			"                                     3 - MAINREXT profile\n"
			"                                   VC8000E/H.264 encoder profile [11]\n"
			"                                     VC8000E/H.264 only supports Baseline, Main, High, High 10 profiles.\n"
			"                                     9  - Baseline profile\n"
			"                                     10 - Main profile\n"
			"                                     11 - High profile\n"
			"                                     12 - High 10 profile\n"
			"  -L[n] --level                    VC8000E/HEVC level. 180 = level 6.0*30 [180]\n"
			"                                     VC8000E/HEVC HW only supports less than or equal to level 5.1 High tier in real-time.\n"
			"                                     For levels 5.0 and 5.1, support resolutions up to 4096x2048 with 4K@60fps performance.\n"
			"                                     For levels greater than level 5.1, support in non-realtime mode.\n"
			"                                     Each level has resolution and bitrate limitations:\n"
			"                                     30  - level 1.0  QCIF      (176x144)       128 kbps\n"
			"                                     60  - level 2.0  CIF       (352x288)       1.5 Mbps\n"
			"                                     63  - level 2.1  Q720p     (640x360)       3.0 Mbps\n"
			"                                     90  - level 3.0  QHD       (960x540)       6.0 Mbps\n"
			"                                     93  - level 3.1  720p HD   (1280x720)      10.0 Mbps\n"
			"                                     120 - level 4.0  2Kx1080   (2048x1080)     12.0 Mbps   High tier 30 Mbps\n"
			"                                     123 - level 4.1  2Kx1080   (2048x1080)     20.0 Mbps   High tier 50 Mbps\n"
			"                                     150 - level 5.0  4096x2160 (4096x2160)     25.0 Mbps   High tier 100 Mbps\n"
			"                                     153 - level 5.1  4096x2160 (4096x2160)     40.0 Mbps   High tier 160 Mbps\n"
			"                                     156 - level 5.2  4096x2160 (4096x2160)     60.0 Mbps   High tier 240 Mbps\n"
			"                                     180 - level 6.0  8192x4320 (8192x4320)     60.0 Mbps   High tier 240 Mbps\n"
			"                                     183 - level 6.1  8192x4320 (8192x4320)     120.0 Mbps  High tier 480 Mbps\n"
			"                                     186 - level 6.2  8192x4320 (8192x4320)     240.0 Mbps  High tier 800 Mbps\n"
			"                                   VC8000E/H.264 level. 51 = Level 5.1 [51]\n"
			"                                     10 - H264_LEVEL_1 \n"
			"                                     99 - H264_LEVEL_1_b \n"
			"                                     11 - H264_LEVEL_1_1 \n"
			"                                     12 - H264_LEVEL_1_2 \n"
			"                                     13 - H264_LEVEL_1_3 \n"
			"                                     20 - H264_LEVEL_2 \n"
			"                                     21 - H264_LEVEL_2_1 \n"
			"                                     22 - H264_LEVEL_2_2\n"
			"                                     30 - H264_LEVEL_3 \n"
			"                                     31 - H264_LEVEL_3_1 \n"
			"                                     32 - H264_LEVEL_3_2 \n"
			"                                     40 - H264_LEVEL_4 \n"
			"                                     41 - H264_LEVEL_4_1 \n"
			"                                     42 - H264_LEVEL_4_2 \n"
			"                                     50 - H264_LEVEL_5 \n"
			"                                     51 - H264_LEVEL_5_1 \n"
			"                                     52 - H264_LEVEL_5_2 \n"
			"                                     60 - H264_LEVEL_6 \n"
			"                                     61 - H264_LEVEL_6_1 \n"
			"                                     62 - H264_LEVEL_6_2 \n"
			"        --tier                     VC8000E/HEVC encoder tier [0]\n"
			"                                     VC8000E/HEVC encoder only supports Main tier and High tier\n"
			"                                     0 - Main tier\n"
			"                                     1 - High tier\n"

			"        --bitDepthLuma             bit depth of luma samples in encoded stream [8]\n"
			"                                     8=8 bit,9=9 bit, 10=10bit\n"
			"        --bitDepthChroma           bit depth of chroma samples in encoded stream [8]\n"
			"                                     8=8 bit,9=9 bit, 10=10bit\n"

			"  -N[n] --byteStream               Stream type. [1]\n"
			"                                     0 - NAL units. NAL sizes are returned in file <nal_sizes.txt>\n"
			"                                     1 - byte stream according to HEVC Standard Annex B.\n"
			"        --ivf                      IVF encapsulation, for AV1 only. [1]\n"
			"                                     0 - OBU raw output\n"
			"                                     1 - IVF output (default)\n"

			"  -k[n] --videoRange               0..1 Video signal sample range in encoded stream. [0]\n"
			"                                     0 - Y range in [16..235]; Cb, Cr in [16..240]\n"
			"                                     1 - Y, Cb, Cr range in [0..255]\n"
		   );
	// streamBufChain AmebaPro2 non support by Raymond

	fprintf(stdout,
			"  -S[n] --sei                      Enable SEI messages (buffering period + picture timing) [0]\n"
			"                                   Writes Supplemental Enhancement Information messages\n"
			"                                   containing information about picture time stamps\n"
			"                                   and picture buffering into stream.\n");

	fprintf(stdout,
			"        --codecFormat              Select Video Codec Format  [0]\n"
			"                                   0: HEVC video format output\n"
			"                                   1: H.264 video format output\n"
			"                                   2: JPEG video format output\n"
			"                                   3: NV12 video format output\n"
			"                                   4: RGB video format output\n"
			"                                   5: NV16 video format output\n");

	fprintf(stdout,
			"  -p[n] --cabacInitFlag            0..1 Initialization value for CABAC. [0]\n"
			"  -K[n] --enableCabac              0=OFF (CAVLC), 1=ON (CABAC). [1]\n"
		   );
	fprintf(stdout,
			"  -e[n] --sliceSize                [0..height/ctu_size] slice size in number of CTU rows [0]\n"
			"                                     0 to encode each picture in one slice\n"
			"                                     [1..height/ctu_size] to encode each slice with N CTU rows\n"
		   );
	fprintf(stdout,

			"  -D[n] --disableDeblocking        0..1 Disable deblocking filter [0]\n"
			"                                     0 = Inloop deblocking filter enabled (better quality)\n"
			"                                     1 = Inloop deblocking filter disabled\n"

			"  -M[n] --enableSao                0..1 Disable or enable SAO filter [1]\n"
			"                                     0 = SAO disabled\n"
			"                                     1 = SAO enabled\n"
			/*                "                                   2 = Inloop deblocking filter disabled on slice edges\n"*/
			"  -W[n] --tc_Offset                -6..6 Controls deblocking filter alpha_c0_offset (H.264) or tc_offset (HEVC). [-2]\n"
			"  -E[n] --beta_Offset              -6..6 Deblocking filter beta_offset (H.264 and HEVC). [5]\n"
		   );

	fprintf(stdout,
			"        --enableDeblockOverride    0..1 Deblocking override enable flag [0]\n"
			"                                     0 = Disable override\n"
			"                                     1 = Enable override\n"
			"        --deblockOverride          0..1 Deblocking override flag [0]\n"
			"                                     0 = Do not override filter parameters\n"
			"                                     1 = Override filter parameters\n"
		   );

	fprintf(stdout,
			"        --tile=a:b:c               HEVC Tile setting: Tile is enabled when num_tile_columns * num_tile_rows > 1. [1:1:1]\n"
			"                                       a: num_tile_columns \n"
			"                                       b: num_tile_rows \n"
			"                                       c: loop_filter_across_tiles_enabled_flag \n"
		   );

	fprintf(stdout,
			"        --enableScalingList        0..1 Use average or default scaling list. [0]\n"
			"                                     0 = Use average scaling list.\n"
			"                                     1 = Use default scaling list.\n"
		   );

	fprintf(stdout,
			"        --RPSInSliceHeader         0..1 Encoding RPS in the slice header. [0]\n"
			"                                   0 = not encoding RPS in the slice header. \n"
			"                                   1 = encoding RPS in the slice header. \n"
		   );
	fprintf(stdout,
			"        --POCConfig=a:b:c          POC type/number of bits settings.\n"
			"                                       a: picOrderCntType. [0] \n"
			"                                       b: log2MaxPicOrderCntLsb.[16] \n"
			"                                       c: log2MaxFrameNum.[12] \n"
		   );
	fprintf(stdout,
			"      --psyFactor                0..4 Weight of psycho-visual encoding. [0]\n"
			"                                     0 = disable psy-rd.\n"
			"                                     1..4 = encode psy-rd, and set strength of psyFactor, larger favor better subjective quality.\n"
		   );


	fprintf(stdout,
			"\n Parameters affecting GOP pattern, rate control and output stream bitrate:\n"
			"  -R[n] --intraPicRate             Intra-picture rate in frames. [0]\n"
			"                                   Forces every Nth frame to be encoded as intra frame.\n"
			"                                   0 = Do not force\n"
			"  -B[n] --bitPerSecond             10000..levelMax, target bitrate for rate control, in bps. [1000000]\n"
			"        --tolMovingBitRate         0..2000, percent tolerance over target bitrate of moving bitrate [2000]\n"
			"        --monitorFrames            10..120, how many frames will be monitored for moving bitrate [frame rate]\n"
			"        --bitVarRangeI             10..10000, percent variations over the average bits per frame for I frame [10000]\n"
			"        --bitVarRangeP             10..10000, percent variations over the average bits per frame for P frame [10000]\n"
			"        --bitVarRangeB             10..10000, percent variations over the average bits per frame for B frame [10000]\n"
			"        --staticSceneIbitPercent  0...100 I frame bits percent of bitrate in static scene [80]\n"
			"  --crf[n]                         -1..51 Constant rate factor mode, working with look-ahead turned on. -1=disable. [-1]\n"
			"                                   CRF mode is to keep a certain level of quality based on crf value, working as constant QP with complexity rate control.\n"
			"                                   CRF adjusts frame level QP within range of crf constant +-3 based on frame complexity. CRF will disable VBR mode if both enabled.\n"
			"  -U[n] --picRc                    0=OFF, 1=ON Picture rate control. [0]\n"
			"                                   Calculates a new target QP for every frame.\n"
			"        --smoothPsnrInGOP          0=disable, 1=enable Smooth PSNR for frames in one GOP. [0]\n"
			"                                   Only affects gopSize > 1.\n"
			"  -u[n] --ctbRc                    0..3 CTB QP adjustment mode for Rate Control and Subjective Quality. [0]\n"
			"                                     0 = No CTB QP adjustment.\n"
			"                                     1 = CTB QP adjustment for Subjective Quality only.\n"
			"                                     4 = CTB QP adjustment for Subjective Quality only, reversed.\n"
			"        --blockRCSize              Block size in CTB QP adjustment for Subjective Quality [0]\n"
			"                                     0=64x64 ,1=32x32, 2=16x16\n"
			"        --rcQpDeltaRange           Max absolute value of CU/MB QP delta relative to frame QP in CTB RC [10]\n"
			"                                   0..15 for HwCtbRcVersion <= 1; 0..51 for HwCtbRcVersion > 1.\n"
			"        --rcBaseMBComplexity       0..31 MB complexity base in CTB QP adjustment for Subjective Quality [15]\n"
			"                                   qpOffset is larger for CU/MB with larger complexity offset from rcBaseMBComplexity.\n"
			"        --tolCtbRcInter            Tolerance of CTB Rate Control for INTER frames. <float> [0.0]\n"
			"                                   CTB RC will try to limit INTER frame bits within the range of:\n"
			"                                       [targetPicSize/(1+tolCtbRcInter), targetPicSize*(1+tolCtbRcInter)].\n"
			"                                   A negative number means no bitrate limit in CTB RC.\n"
			"        --tolCtbRcIntra            Tolerance of CTB Rate Control for INTRA frames. <float> [-1.0]\n"
			"        --ctbRowQpStep             The maximum accumulated QP adjustment step per CTB Row allowed by CTB rate control.\n"
			"                                   Default value is [4] for H.264 and [16] for HEVC.\n"
			"                                   QP_step_per_CTB = (ctbRowQpStep / Ctb_per_Row) and limited by maximum = 4.\n"
			"        --picQpDeltaRange          Min:Max. Qp_Delta range in picture RC.\n"
			"                                   Min: -10..-1 Minimum Qp_Delta in picture RC. [-2]\n"
			"                                   Max:  1..10  Maximum Qp_Delta in picture RC. [3]\n"
			"                                   This range only applies to two neighboring frames of the same coding type.\n"
			"                                   This range doesn't apply when HRD overflow occurs.\n"
			"  -C[n] --hrdConformance           0=OFF, 1=ON HRD conformance. [0]\n"
			"                                   Uses standard defined model to limit bitrate variance.\n"
			"  -c[n] --cpbSize                  HRD Coded Picture Buffer size in bits. [1000000]\n"
			"                                   Buffer size used by the HRD model.\n"
			"  -g[n] --bitrateWindow            1..300, Bitrate window length in frames [intraPicRate]\n"
			"                                   Rate control allocates bits for one window and tries to\n"
			"                                   match the target bitrate at the end of each window.\n"
			"                                   Typically window begins with an intra frame, but this\n"
			"                                   is not mandatory.\n"
			"        --gopSize                  GOP Size. [0]\n"
			"                                   0..8, 0 for adaptive GOP size; 1~7 for fixed GOP size\n"
			"        --gopConfig                User-specified GOP configuration file, which defines the GOP structure table.\n"
			"                                   FrameN Type POC QPoffset QPfactor TemporalId num_ref_pics ref_pics used_by_cur\n"
			"                                     -FrameN: Normal GOP config. The table should contain gopSize lines, named Frame1, Frame2, etc.\n"
			"                                      The frames are listed in decoding order.\n"
			"                                     -Type: Slice type, can be either P or B.\n"
			"                                     -POC: Display order of the frame within a GOP, ranging from 1 to gopSize.\n"
			"                                     -QPoffset: It is added to the QP parameter to set the final QP value used for this frame.\n"
			"                                     -QPfactor: Weight used during rate distortion optimization.\n"
			"                                     -TemporalId: temporal layer ID.\n"
			"                                     -num_ref_pics: The number of reference pictures kept for this frame,\n"
			"                                      including references for current and future pictures.\n"
			"                                     -ref_pics: A List of num_ref_pics integers,\n"
			"                                      specifying the delta_POC of the reference pictures kept, relative to the POC of the current frame or LTR's index.\n"
			"                                     -used_by_cur: A List of num_ref_pics binaries,\n"
			"                                      specifying whether the corresponding reference picture is used by current picture.\n"
			"                                     If this config file is not specified, default parameters will be used.\n"
			"                                   Frame0 Type QPoffset QPfactor TemporalId num_ref_pics ref_pics used_by_cur LTR Offset Interval\n"
			"                                     -Frame0: Special GOP config. The table should contain gopSize lines.\n"
			"                                      The frames are listed in decoding order.\n"
			"                                     -Type: Slice type. If the value is not reserved, it overrides the value in normal config for special frame. [-255]\n"
			"                                     -QPoffset: If the value is not reserved, it overrides the value in normal config for special frame. [-255]\n"
			"                                     -QPfactor: If the value is not reserved, it overrides the value in normal config for special frame. [-255]\n"
			"                                     -TemporalId: temporal layer ID. If the value is not reserved, it overrides the value in normal config for special frame. [-255]\n"
			"                                     -num_ref_pics: The number of reference pictures kept for this frame\n"
			"                                      including references for current and future pictures.\n"
			"                                     -ref_pics: A List of num_ref_pics integers,\n"
			"                                      specifying the delta_POC of the reference pictures kept, relative to the POC of the current frame or LTR's index.\n"
			"                                     -used_by_cur: A List of num_ref_pics binaries, specifying whether the corresponding reference picture is used by current picture.\n"
			"                                     -LTR: 0..VCENC_MAX_LT_REF_FRAMES. long term reference setting, 0 - common frame, 1..VCENC_MAX_LT_REF_FRAMES - long term reference's index\n"
			"                                     -Offset: If LTR = 0, POC_Frame_Use_LTR(0) - POC_LTR. POC_LTR is LTR's POC, POC_Frame_Use_LTR(0) is the first frame after LTR that uses the LTR as reference.\n"
			"                                              If LTR != 0, the offset of the first LTR frame to the first encoded frame.\n"
			"                                     -Interval: If LTR = 0, POC_Frame_Use_LTR(n+1) - POC_Frame_Use_LTR(n). The POC delta between two consecutive frames that use the same LTR as reference.\n"
			"                                                If LTR != 0, POC_LTR(n+1) - POC_LTR(n). POC_LTR is LTR's POC; the POC delta between two consecutive frames that encoded as the same LTR index.\n"
			"                                     If this config file is not specified, default parameters will be used.\n"
			"        --LTR=a:b:c[:d]            a:b:c[:d] long term reference setting\n"
			"                                       a: POC_delta of two frames which are used as LTR.\n"
			"                                       b: POC_Frame_Use_LTR(0) - POC_LTR. POC_LTR is LTR's POC, POC_Frame_Use_LTR(0) is the first frame after LTR that uses the LTR as reference.\n"
			"                                       c: POC_Frame_Use_LTR(n+1) - POC_Frame_Use_LTR(n). The POC delta between two consecutive frames that use the same LTR as reference.\n"
			"                                       d: QP delta for frame using LTR. [0]\n"
			"        --gopLowdelay              Use default low delay GOP configuration if --gopConfig is not specified, only valid for gopSize <= 4. [0]\n"
			"                                   0 = Disable gopLowdelay mode\n"
			"                                   1 = Enable gopLowdelay mode\n");

	fprintf(stdout,
			"  -s[n] --picSkip                  0=OFF, 1=ON Picture skip rate control. [0]\n"
			"                                   Allows rate control to skip frames if needed.\n"
			"  -q[n] --qpHdr                    -1..51, Initial target QP. [26]\n"
			"                                   -1 = Encoder calculates initial QP. NOTE: use -q-1\n"
			"                                   The initial QP used in the beginning of stream.\n"
			"  -n[n] --qpMin                    0..51, Minimum frame header QP for any slices. [0]\n"
			"  -m[n] --qpMax                    0..51, Maximum frame header QP for any slices. [51]\n"
			"        --qpMinI                   0..51, Minimum frame header QP, overriding qpMin for I slices. [0]\n"
			"        --qpMaxI                   0..51, Maximum frame header QP, overriding qpMax for I slices. [51]\n"
			"  -A[n] --intraQpDelta             -51..51, Intra QP delta. [-5]\n"
			"                                   QP difference between target QP and intra frame QP.\n"
			"  -G[n] --fixedIntraQp             0..51, Fixed Intra QP, 0 = disabled. [0]\n"
			"                                   Use fixed QP value for every intra frame in stream.\n"
			"  -V[n] --bFrameQpDelta            -1..51, B Frame QP Delta. [-1]\n"
			"                                   QP difference between B Frame QP and target QP.\n"
			"                                   If a GOP config file is specified by --gopConfig, it will be overridden.\n"
			"                                   -1 not take effect, 0-51 could take effect.\n"
			"  -I[n] --chromaQpOffset           -12..12 Chroma QP offset. [0]\n"
			"        --vbr                      0=OFF, 1=ON. Variable Bitrate control by qpMin. [0]\n"

		   );

	fprintf(stdout,
			"        --gdrDuration              how many frames it will take to do GDR [0]\n"
			"                                   0 : disable GDR (Gradual decoder refresh), >0: enable GDR\n"
			"                                   The starting point of GDR is the frame with type set to VCENC_INTRA_FRAME.\n"
			"                                   intraArea and roi1Area are used to implement the GDR function.\n"
		   );

	fprintf(stdout,
			"        --skipFramePOC             0..n Force encode whole frame as Skip Frame when POC=n. [0]\n"
			"                                   0: Disable. \n"
			"                                   none zero: set the frame POC to be encoded as Skip Frame. \n"
		   );

	fprintf(stdout,
			"\n Parameters affecting coding:\n"
			"  -z[s] --userData                 SEI User data file name. File is read and inserted\n"
			"                                     as an SEI message before the first frame.\n"
		   );

	fprintf(stdout,
			"        --cir                      start:interval for Cyclic Intra Refresh.\n"
			"                                   Forces CTBs in intra mode. [0:0]\n"
			"        --intraArea                left:top:right:bottom CTB coordinates\n"
			"                                       Specifies the rectangular area of CTBs to\n"
			"                                       force encoding in intra mode.\n"
			"        --ipcm1Area                left:top:right:bottom CTB coordinates\n"
			"        --ipcm2Area                left:top:right:bottom CTB coordinates\n"
			"                                       Specifies the rectangular area of CTBs to\n"
			"                                       force encoding in IPCM mode.\n"
			"        --ipcmMapEnable            Enable the IPCM Map. 0-disable, 1-enable. [0]\n"
			"        --ipcmMapFile              User-specified IPCM map file, which defines the IPCM flag for each CTB in the frame.\n"
			"                                   The block size is 64x64 for HEVC and 16x16 for H.264.\n"
			"        --roi1Area                 left:top:right:bottom CTB coordinates\n"
			"        --roi2Area                 left:top:right:bottom CTB coordinates\n"
			"        --roi3Area                 left:top:right:bottom CTB coordinates\n"
			"        --roi4Area                 left:top:right:bottom CTB coordinates\n"
			"        --roi5Area                 left:top:right:bottom CTB coordinates\n"
			"        --roi6Area                 left:top:right:bottom CTB coordinates\n"
			"        --roi7Area                 left:top:right:bottom CTB coordinates\n"
			"        --roi8Area                 left:top:right:bottom CTB coordinates\n"
			"                                       Specifies the rectangular area of CTBs as\n"
			"                                       Region Of Interest with lower QP.\n"
			"        --roi1DeltaQp              -30..0 (-51..51 if absolute ROI QP supported), QP delta value for ROI 1 CTBs. [0]\n"
			"        --roi2DeltaQp              -30..0 (-51..51 if absolute ROI QP supported), QP delta value for ROI 2 CTBs. [0]\n"
			"        --roi3DeltaQp              -30..0 (-51..51 if absolute ROI QP supported), QP delta value for ROI 3 CTBs. [0]\n"
			"        --roi4DeltaQp              -30..0 (-51..51 if absolute ROI QP supported), QP delta value for ROI 4 CTBs. [0]\n"
			"        --roi5DeltaQp              -30..0 (-51..51 if absolute ROI QP supported), QP delta value for ROI 5 CTBs. [0]\n"
			"        --roi6DeltaQp              -30..0 (-51..51 if absolute ROI QP supported), QP delta value for ROI 6 CTBs. [0]\n"
			"        --roi7DeltaQp              -30..0 (-51..51 if absolute ROI QP supported), QP delta value for ROI 7 CTBs. [0]\n"
			"        --roi8DeltaQp              -30..0 (-51..51 if absolute ROI QP supported), QP delta value for ROI 8 CTBs. [0]\n"
			"        --roi1Qp                   0..51, absolute QP value for ROI 1 CTBs. [-1]. Negative value means invalid.,\n"
			"                                     and another value is used to calculate the QP in the ROI area.\n"
			"        --roi2Qp                   0..51, absolute QP value for ROI 2 CTBs. [-1]\n"
			"        --roi3Qp                   0..51, absolute QP value for ROI 3 CTBs. [-1]\n"
			"        --roi4Qp                   0..51, absolute QP value for ROI 4 CTBs. [-1]\n"
			"        --roi5Qp                   0..51, absolute QP value for ROI 5 CTBs. [-1]\n"
			"        --roi6Qp                   0..51, absolute QP value for ROI 6 CTBs. [-1]\n"
			"        --roi7Qp                   0..51, absolute QP value for ROI 7 CTBs. [-1]\n"
			"        --roi8Qp                   0..51, absolute QP value for ROI 8 CTBs. [-1]\n"
			"                                   roi1Qp..roi8Qp are only valid when absolute ROI QP is supported. Use either roiDeltaQp or roiQp.\n"
			"        --roiMapDeltaQpBlockUnit   Set the DeltaQp block size for ROI DeltaQp map file. 0-64x64,1-32x32,2-16x16,3-8x8. [0]\n"
			"        --roiMapDeltaQpEnable      Enable the QP delta for ROI regions in the frame. 0-disable,1-enable. [0]\n"
			"        --roiMapDeltaQpFile        User-specified DeltaQp map file, which defines the QP delta for each block in the frame.\n"
			"                                   The block size is defined by --roiMapDeltaQpBlockUnit.\n"
			"                                   The QP delta range is from -30 to 0 if absolute ROI QP is not supported.\n"
			"                                   The QP delta range is from -51 to 51 if absolute ROI QP is supported.\n"
			"                                   Absolute QP range is from 0 to 51 and specified with a prefix 'a', for example, 'a26'.\n"
			"                                   To calculate block width and height, alignment of 64 for the picture width and height should be used.\n"
			"        --roiMapInfoBinFile        User-specified ROI map info binary file, which defines the QP delta or absolute ROI QP for each block in the frame.\n"
			"                                   The block size is defined by --roiMapDeltaQpBlockUnit, one byte per block.\n"
			"                                   The byte bitmap of QpDelta/Skip/IPCM is related to --RoiQpDeltaVer. QP delta range is from -51 to 51.\n"
			"                                   Absolute QP range is from 0 to 51.\n"
			"        --RoiQpDeltaVer            1..3, ROI Qp Delta map version number, valid only if --roiMapInfoBinFile is present.\n"
			"        --RoimapCuCtrlInfoBinFile  User-specified ROI map CU control info binary file, which defines the ROI CU control info for each CU in the frame.\n"
			"                                   The CU control structure size is related to --RoiCuCtrlVer.\n"
			"                                   The number of controlled CUs is aligned to 64 of picture width and height in 8x8 unit.\n"
			"        --RoiCuCtrlVer             3..7, ROI map CU control version number, valid only if --RoimapCuCtrlInfoBinFile is present.\n"
		   );

	fprintf(stdout,
			"\n Parameters setting Skip Map:\n"
			"        --skipMapEnable            0..1 Enable Skip Map Mode. 0-Disable, 1-Enable. [0]\n"
			"        --skipMapBlockUnit         Block size in Skip Map Mode. [0]\n"
			"                                   0-64x64, 1-32x32, 2-16x16. Only 64x64 and 32x32 are valid for HEVC.\n"
			"        --skipMapFile              User-specified Skip Map File, which defines the Skip forcing mode for each block in the frame.\n"
			"                                   1 indicates Skip forcing mode for a block and 0 indicates not.\n"
			"                                   To calculate block width and height, an alignment of 64 of picture width and height should be used.\n"
		   );

	fprintf(stdout,
			"\n Parameters setting constant chroma:\n"
			"        --enableConstChroma        0..1 Enable/Disable setting chroma to a constant pixel value. [0]\n"
			"                                     0 = Disable. \n"
			"                                     1 = Enable. \n"
			"        --constCb                  0..255 for 8-bit [128]; 0..1023 for 10-bit [512]. The constant pixel value for Cb. \n"
			"        --constCr                  0..255 for 8-bit [128]; 0..1023 for 10-bit [512]. The constant pixel value for Cr. \n"
		   );


	fprintf(stdout,
			"\n Parameters affecting coding and performance:\n"
			"        --preset                   0...4 for HEVC. 0..1 for H264. Trade off performance and compression efficiency\n"
			"                                     Higher value means high quality but worse performance. User need explict claim preset when use this option\n"
			"        --rdoLevel                 1..3 Programable hardware RDO Level [3]\n"
			"                                     Lower value means lower quality but better performance.\n"
			"                                     Higher value means higher quality but worse performance.\n"
			"        --enableRdoQuant           0..1 Enable/Disable RDO Quantization.\n"
			"                                     0 = Disable (Default if hardware does not support RDOQ).\n"
			"                                     1 = Enable (Default if hardware supports RDOQ).\n"
		   );

	fprintf(stdout,
			"\n Parameters affecting reporting:\n"
			"        --enableOutputCuInfo       0..2 Enable/Disable writting CU encoding information to external memory [0]\n"
			"                                     0 = Disable. \n"
			"                                     1 = Enable. \n"
			"                                     2 = Enable and also write information in file <cuInfo.txt>. \n"
			"        --hashtype                 hash type for frame data hash. [0]\n"
			"                                     0=disable, 1=crc32, 2=checksum32. \n"
		   );

	fprintf(stdout,
			"        --enableVuiTimingInfo       Write VUI timing info in SPS. [1]\n"
			"                                    0=disable. \n"
			"                                    1=enable. \n"
		   );

	fprintf(stdout,
			"\n Parameters affecting lookahead encoding:\n"
			"        --lookaheadDepth            0, 4..40 The number of look-ahead frames. [0]\n"
			"                                      0 = Disable 2-pass encoding.\n"
			"                                      4-40 = Set the number of look-ahead frames and enable 2-pass encoding.\n"
		   );
	fprintf(stdout,
			"        --halfDsInput               External provided half-downsampled input YUV file.\n"
		   );
	fprintf(stdout,
			"        --aq_mode                   Mode for Adaptive Quantization - 0:none 1:uniform AQ 2:auto variance 3:auto variance with bias to dark scenes. Default 0\n"
			"        --aq_strength               Reduces blocking and blurring in flat and textured areas (0 to 3.0). Default 1.00\n"
		   );
	fprintf(stdout,
			"        --tune                       0...2 Tune parameters for best psnr/ssim/subjective quality. [0]\n"
			"                                       0 = best psnr score: disable aq-mode/psy-rd. \n"
			"                                       1 = best ssim score: enable aq-mode, disable psy-rd. \n"
			"                                       2 = best subjective quality: enable aq-mode/psy-rd. \n"
		   );

	fprintf(stdout,
			"\n Parameters affecting Reference Frame Compressor:\n"
			"        --compressor               0..3 Enable/Disable Embedded Compression [0]\n"
			"                                     0 = Disable compression\n"
			"                                     1 = Only enable luma compression\n"
			"                                     2 = Only enable chroma compression\n"
			"                                     3 = Enable both luma and chroma compression\n"
		   );



	fprintf(stdout,
			"\n Parameters affecting motion estimation and global motion:\n"

			"        --MEVertRange              ME vertical search range. Only valid if the function is supported in this version. [0]\n"
			"                                   Should be 24 or 48 for H.264; 40 or 64 for HEVC/AV1.\n"
			"                                   0 - The maximum vertical search range of this version will be used.\n"
		   );



	fprintf(stdout,
			"\n Parameters affecting runtime log level:\n"
			"        --verbose                  0..1 Log printing mode [0]\n"
			"                                     0: Print brief information.\n"
			"                                     1: Print more detailed information.\n"
		   );


	fprintf(stdout,
			"\n Parameters dump registers to sw_reg.trc:\n"
			"        --dumpRegister             0..1 dump register enable [0]\n"
			"                                   0: no register dump.\n"
			"                                   1: print register dump.\n"
		   );

	fprintf(stdout,
			"\n Raster scan output for recon on FPGA & HW:\n"
			"        --rasterscan               0..1 raster scan enable [0]\n"
			"                                   0: disable raster scan output.\n"
			"                                   1: enable raster scan output.\n"
		   );

	fprintf(stdout,
			"\n enable/disable recon write to DDR if it's pure I-frame encoding:\n"
			"        --writeReconToDDR       0..1 recon write to DDR enable [1]\n"
			"                                   0: disable recon write to DDR.\n"
			"                                   1: enable recon write to DDR.\n"
		   );
	// formatCustomizedType AmebaPro2 non support by Raymond
	// multi-segment AmebaPro2 non support by Raymond
	// HW SSIM AmebaPro2 non support by Raymond
	// sceneChange AmebaPro2 non support by Raymond
	// GMV AmebaPro2 non support by Raymond
	// HDR10 AmebaPro2 non support by Raymond
	// external SRAM AmebaPro2 non support by Raymond
	// mmuEnable AmebaPro2 non support by Raymond
	// dec400TableInput AmebaPro2 non support by Raymond
	// parallelCoreNum AmebaPro2 non support by Raymond
	// multimode/streamcfg AmebaPro2 non support by Raymond
	// overlayEnables AmebaPro2 non support by Raymond

	fprintf(stdout,
			"\n Parameters for AXI alignment:\n"
			"  --AXIAlignment                   AXI alignment setting (in hexadecimal format). [0]\n"
			"                                     bit[31:28] AXI_burst_align_wr_common\n"
			"                                     bit[27:24] AXI_burst_align_wr_stream\n"
			"                                     bit[23:20] AXI_burst_align_wr_chroma_ref\n"
			"                                     bit[19:16] AXI_burst_align_wr_luma_ref\n"
			"                                     bit[15:12] AXI_burst_align_rd_common\n"
			"                                     bit[11: 8] AXI_burst_align_rd_prp\n"
			"                                     bit[ 7: 4] AXI_burst_align_rd_ch_ref_prefetch\n"
			"                                     bit[ 3: 0] AXI_burst_align_rd_lu_ref_prefetch\n"
		   );

	fprintf(stdout,
			"\n Temporary testing parameters for internal use:\n"
			"  --parallelCoreNum                1..4 The number of cores running in parallel at frame level. [1]\n"
			"  --smoothingIntra                 0..1 Enable or disable the strong_intra_smoothing_enabled_flag (HEVC only). 0=Normal, 1=Strong. [1]\n"
//          "  --testId                         Internal test ID. [0]\n"

			"  --roiMapDeltaQpBinFile           User-specified DeltaQp map binary file, only valid when lookaheadDepth > 0.\n"
			"                                   Every byte stands for a block unit deltaQp in raster-scan order.\n"
			"                                   The block unit size is defined by --roiMapDeltaQpBlockUnit (0-64x64,1-32x32,2-16x16,3-8x8).\n"
			"                                   Frame width and height are aligned to block unit size when setting deltaQp per block.\n"
			"                                   Frame N data is followed by Frame N+1 data immediately in display order.\n"
			"  --RoimapCuCtrlIndexBinFile       User-specified ROI map CU control index binary file, which defines where the ROI info is included in the file --RoimapCuCtrlInfoBinFile.\n"
			"                                     If --RoimapCuCtrlIndexBinFile is not specified, the ROI info about CUs in each CTB should be included in the file --RoimapCuCtrlInfoBinFile.\n"
			"                                     If --RoimapCuCtrlIndexBinFile is specified, the ROI info about CUs in each CTB should be included in the file --RoimapCuCtrlIndexBinFile.\n"
			"\n");
	fprintf(stdout,
			"    --voe                          Video off load engine enable. [1]\n"
			"                                    1: enable\n"
			"                                    0: disable\n"
			"\n");
	fprintf(stdout,
			"    --dbg                          Video off load engine debug message. [0]\n"
			"                                    1: enable\n"
			"                                    0: disable\n"
			"\n");


	exit(OK);
}

void Parameter_Preset(commandLine_s *cml)
{
	/************************************************
	  Preset: H264    Preset: HEVC      LA=40   rdoLevel    RDOQ
	  N/A                 4                          Y           3                Y
	  N/A                 3                          Y           3                N
	  N/A                 2                          Y           2                N
	  1                   1                          Y           1                N
	  0                   0                          N           1                N
	*************************************************/
	if (cml->preset >= 4) {
		cml->lookaheadDepth = 40;
		cml->rdoLevel = 3;
		cml->enableRdoQuant = 1;
	} else if (cml->preset == 3) {
		cml->lookaheadDepth = 40;
		cml->rdoLevel = 3;
		cml->enableRdoQuant = 0;
	} else if (cml->preset == 2) {
		cml->lookaheadDepth = 40;
		cml->rdoLevel = 2;
		cml->enableRdoQuant = 0;
	} else if (cml->preset == 1) {
		cml->lookaheadDepth = 40;
		cml->rdoLevel = 1;
		cml->enableRdoQuant = 0;
	} else if (cml->preset == 0) {
		cml->lookaheadDepth = 0;
		cml->rdoLevel = 1;
		cml->enableRdoQuant = 0;
	}
}

static void Parameter_Check(commandLine_s *cml)
{
//  EWLHwConfig_t asic_cfg = VCEncGetAsicConfig();
//  if(cml->lookaheadDepth > 0 && ((!asic_cfg.bFrameEnabled && cml->gopSize != 1) || asic_cfg.cuInforVersion < 1)) {
//    // lookahead needs bFrame support & cuInfo version 1
//    printf("Lookahead not supported, disable lookahead!\n");
//    cml->lookaheadDepth = 0;
//  }
	if (cml->lookaheadDepth) {
		cml->gopLowdelay = 0; /* lookahead not work well with lowdelay configurations */
		cml->roiMapDeltaQpEnable = 1;
		cml->roiMapDeltaQpBlockUnit = MAX(1, cml->roiMapDeltaQpBlockUnit);
	}

	/* Encoder initialization */

	if (cml->width == DEFAULT) {
		cml->width = cml->lumWidthSrc;
	}

	if (cml->height == DEFAULT) {
		cml->height = cml->lumHeightSrc;
	}
	printf("%d %d %d\n", IS_HEVC(cml->outputFormat), IS_H264(cml->outputFormat), cml->outputFormat);
	if (IS_HEVC(cml->outputFormat) && (cml->profile > VCENC_HEVC_MAINREXT)) {
		//printf("HEVC profile not supported, default main profile!\n");
		printf("Set H264 default MAIN profile\n");
		cml->profile = VCENC_HEVC_MAIN_PROFILE;
	}
	if (IS_H264(cml->outputFormat) &&
		(cml->profile > VCENC_H264_HIGH_10_PROFILE || cml->profile < VCENC_H264_BASE_PROFILE)) {
		//printf("H264 profile not supported, default high profile!\n");
		printf("Set H264 default HIGH profile\n");
		cml->profile = VCENC_H264_HIGH_PROFILE;
	}

	cml->byteStream = (cml->byteStream) ? VCENC_BYTE_STREAM : VCENC_NAL_UNIT_STREAM;

	return;
}


/*------------------------------------------------------------------------------
  default_parameter
------------------------------------------------------------------------------*/
void default_parameter(commandLine_s *cml)
{
	int i;
	memset(cml, 0, sizeof(commandLine_s));
	strcpy(cml->input, "input.yuv");
//    strcpy(cml->output, "stream.hevc");
	cml->CodecEnable = 0;

	cml->firstPic   = 0;
	cml->lastPic    = 0;
	cml->inputRateNumer      = 30;
	cml->inputRateDenom      = 1;
	cml->outputRateNumer      = 30;
	cml->outputRateDenom      = 1;

	cml->lumWidthSrc      = DEFAULT;
	cml->lumHeightSrc     = DEFAULT;
	cml->horOffsetSrc     = DEFAULT;
	cml->verOffsetSrc     = DEFAULT;
	cml->videoStab        = DEFAULT;
	cml->rotation         = 0;
	cml->inputFormat      = 1; 				// Modify default input format is NV12
	cml->width            = DEFAULT;
	cml->height           = DEFAULT;

	cml->intraPicRate   = 0;  // only first is IDR.

//	if (format == VCENC_VIDEO_CODEC_HEVC) {
	{
		// default HEVC
		cml->outputFormat = VCENC_VIDEO_CODEC_HEVC;
		cml->max_cu_size  = 64;
		cml->min_cu_size  = 8;
		cml->max_tr_size  = 16;
		cml->min_tr_size  = 4;
		cml->tr_depth_intra = 2;  //mfu =>0
		cml->tr_depth_inter = (cml->max_cu_size == 64) ? 4 : 3;

		cml->level = VCENC_HEVC_LEVEL_6;
		cml->profile = VCENC_HEVC_MAIN_PROFILE;	// default is HEVC MAIN profile
	}
#if 0
	else {
		cml->outputFormat = VCENC_VIDEO_CODEC_H264;
		cml->max_cu_size = 16;
		cml->min_cu_size = 8;
		cml->max_tr_size = 16;
		cml->min_tr_size = 4;
		cml->tr_depth_intra = 1;
		cml->tr_depth_inter = 2;

		cml->level = VCENC_H264_LEVEL_5_1;
		cml->profile = VCENC_H264_HIGH_PROFILE;	// default is HEVC HIGH profile

	}
#endif
	cml->tier = VCENC_HEVC_MAIN_TIER;

	cml->voe = 1;

	cml->bitPerSecond = 1024000;
	cml->bitVarRangeI = 10000;
	cml->bitVarRangeP = 10000;
	cml->bitVarRangeB = 10000;
	cml->tolMovingBitRate = 2000;
	cml->monitorFrames = DEFAULT;
	cml->tolCtbRcInter = DEFAULT;
	cml->tolCtbRcIntra = DEFAULT;
	cml->u32StaticSceneIbitPercent = 80;
	cml->intraQpDelta = DEFAULT;
	cml->bFrameQpDelta = -1;

	cml->disableDeblocking = 0;

	cml->tc_Offset = -2;
	cml->beta_Offset = 5;

	cml->qpHdr = DEFAULT;
	cml->qpMin = DEFAULT;
	cml->qpMax = DEFAULT;
	cml->qpMinI = DEFAULT;
	cml->qpMaxI = DEFAULT;
	cml->picRc = DEFAULT;
	cml->ctbRc = DEFAULT; //CTB_RC
	cml->cpbSize = 1000000;
	cml->bitrateWindow = DEFAULT;
	cml->fixedIntraQp = 0;
	cml->hrdConformance = 0;
	cml->smoothPsnrInGOP = 0;
	cml->vbr = 0;

	cml->byteStream = 1;

	cml->chromaQpOffset = 0;

	cml->enableSao = 1;

	cml->strong_intra_smoothing_enabled_flag = 0;

	cml->pcm_loop_filter_disabled_flag = 0;

	cml->intraAreaLeft = cml->intraAreaRight = cml->intraAreaTop =
							 cml->intraAreaBottom = -1;  /* Disabled */
	cml->ipcm1AreaLeft = cml->ipcm1AreaRight = cml->ipcm1AreaTop =
							 cml->ipcm1AreaBottom = -1;  /* Disabled */
	cml->ipcm2AreaLeft = cml->ipcm2AreaRight = cml->ipcm2AreaTop =
							 cml->ipcm2AreaBottom = -1;  /* Disabled */
	cml->gdrDuration = 0;

	cml->picSkip = 0;

	cml->sliceSize = 0;

	cml->enableCabac = 1;
	cml->cabacInitFlag = 0;
	cml->cirStart = 0;
	cml->cirInterval = 0;
	cml->enableDeblockOverride = 0;
	cml->deblockOverride = 0;

	cml->enableScalingList = 0;

	cml->compressor = 0;
	cml->sei = 0;
	cml->videoRange = 0;
	cml->bitDepthLuma = DEFAULT;
	cml->bitDepthChroma = DEFAULT;
	cml->blockRCSize = DEFAULT;
	cml->rcQpDeltaRange = DEFAULT;
	cml->rcBaseMBComplexity = DEFAULT;
	cml->picQpDeltaMin = DEFAULT;
	cml->picQpDeltaMax = DEFAULT;
	cml->ctbRcRowQpStep = DEFAULT;

	cml->gopSize = 0;
	cml->gopCfg = NULL;
	cml->gopLowdelay = 0;
	cml->longTermGap = 0;
	cml->longTermGapOffset = 0;
	cml->longTermQpDelta = 0;
	cml->ltrInterval = DEFAULT;

	cml->outReconFrame = 1;

	cml->roiMapDeltaQpBlockUnit = 0;
	cml->roiMapDeltaQpEnable = 0;
	cml->roiMapDeltaQpFile = NULL;
	cml->roiMapDeltaQpBinFile = NULL;
	cml->roiMapInfoBinFile        = NULL;
	cml->RoimapCuCtrlInfoBinFile  = NULL;
	cml->RoimapCuCtrlIndexBinFile = NULL;
	cml->RoiCuCtrlVer  = 0;
	cml->RoiQpDeltaVer = 1;
	cml->ipcmMapEnable = 0;
	cml->ipcmMapFile = NULL;
	cml->roi_ctrl[0].roiQp = DEFAULT;
	cml->roi_ctrl[1].roiQp = DEFAULT;
	cml->roi_ctrl[2].roiQp = DEFAULT;
	cml->roi_ctrl[3].roiQp = DEFAULT;
	cml->roi_ctrl[4].roiQp = DEFAULT;
	cml->roi_ctrl[5].roiQp = DEFAULT;
	cml->roi_ctrl[6].roiQp = DEFAULT;
	cml->roi_ctrl[7].roiQp = DEFAULT;

	cml->interlacedFrame = 0;

	/*stride*/
	cml->exp_of_input_alignment = 4;
	cml->exp_of_ref_alignment = 0;
	cml->exp_of_ref_ch_alignment = 0;


	cml->enableOutputCuInfo = 0;

	cml->rdoLevel = 3;
	cml->hashtype = 0;
	cml->verbose = 0;

	/* constant chroma control */
	cml->constChromaEn = 0;
	cml->constCb = DEFAULT;
	cml->constCr = DEFAULT;

	for (i = 0; i < MAX_SCENE_CHANGE; i ++) {
		cml->sceneChange[i] = 0;
	}

	cml->tiles_enabled_flag = 0;
	cml->num_tile_columns = 1;
	cml->num_tile_rows  = 1;
	cml->loop_filter_across_tiles_enabled_flag = 1;

	cml->skip_frame_enabled_flag = 0;
	cml->skip_frame_poc = 0;

	cml->picOrderCntType = 0;
	cml->log2MaxPicOrderCntLsb = 16;
	cml->log2MaxFrameNum = 12;

	cml->RpsInSliceHeader = 0;
	cml->ssim = 1;
	cml->vui_timing_info_enable = 1;
	cml->halfDsInput = NULL;

	/* skip mode */
	cml->skipMapEnable = 0;
	cml->skipMapFile = NULL;
	cml->skipMapBlockUnit = 0;

	/* Frame-level core parallelism option */


	/*dump register*/
	cml->dumpRegister = 0;

	cml->rasterscan = 0;
	cml->cuInfoVersion = -1;

#ifdef RECON_REF_1KB_BURST_RW
	cml->exp_of_input_alignment = 10;
	cml->exp_of_ref_alignment = 10;
	cml->exp_of_ref_ch_alignment = 10;
	cml->compressor = 2;
#endif
#ifdef RECON_REF_ALIGN64
	cml->exp_of_ref_alignment = 6;
	cml->exp_of_ref_ch_alignment = 6;
#endif

	cml->enableRdoQuant = DEFAULT;

	/*CRF constant*/
	cml->crf = -1;

	/* AXI alignment */
	cml->AXIAlignment = 0;

	/*Ivf support*/
	cml->ivf = 1;

	/*PSY factor*/
	cml->PsyFactor = 0;
	cml->codedChromaIdc = VCENC_CHROMA_IDC_420;
	cml->aq_mode = 0;
	cml->aq_strength = 1.0;

	cml->preset = DEFAULT;

	cml->writeReconToDDR = 1;
#if 1 // default is HEVC
//	if (format == VCENC_VIDEO_CODEC_JPEG) {

//	    cml->outputFormat = VCENC_VIDEO_CODEC_JPEG;
	cml->qLevel = 1;
	cml->restartInterval = 0;

	//cml->frameType = 0;
	cml->colorConversion = 0;
	cml->partialCoding = 0;
	cml->codingMode = 0;
	cml->markerType = 0;
	cml->unitsType = 0;
	cml->xdensity = 1;
	cml->ydensity = 1;

	cml->hashtype = 0;
	cml->mirror = 0;
	cml->constChromaEn = 0;
	cml->constCb = 0x80;
	cml->constCr = 0x80;

	cml->rcMode        = 1;
	cml->picQpDeltaMin = -2;
	cml->picQpDeltaMax = 3;
	cml->qpmin = 0;
	cml->qpmax = 51;
	cml->fixedQP = -1;
	cml->exp_of_input_alignment = 4;
//	}
	cml->voe_dbg    = 0;

#endif

}

/*------------------------------------------------------------------------------
  parameter
------------------------------------------------------------------------------*/
i32 parameter_enc_get(i32 argc, char **argv, commandLine_s *cml)
{
	struct parameter prm;
	i32 status = OK;
	i32 ret, i;
	char *p;
	i32 bpsAdjustCount = 0;
	bool bChromaQpOffset = false;

	prm.cnt = 1;
	while ((ret = get_option(argc, argv, options, &prm)) != -1) {
		if (ret == -2) {
			status = NOK;
		}
		p = prm.argument;
		switch (prm.short_opt) {
		case 'H':
			help(argv[0]);
			break;
		case 'i':
			if (strlen(p) < MAX_PATH) {
				strcpy(cml->input, p);
			} else {
				printf("String over range %d > %d\n", strlen(p), MAX_PATH);
			}
			break;
		case 'o':
			if (strlen(p) < MAX_PATH) {
				strcpy(cml->output, p);
			} else {
				printf("String over range %d > %d\n", strlen(p), MAX_PATH);
			}
			break;
		case 'a':
			cml->firstPic = atoi(p);
			break;
		case 'b':
			cml->lastPic = atoi(p);
			break;
		case 'x':
			cml->width = atoi(p);
			break;
		case 'y':
			cml->height = atoi(p);
			break;
		case 'z':
			cml->userData = p;
			break;
		case 'w':
			cml->lumWidthSrc = atoi(p);
			break;
		case 'h':
			cml->lumHeightSrc = atoi(p);
			break;
		case 'X':
			cml->horOffsetSrc = atoi(p);
			break;
		case 'Y':
			cml->verOffsetSrc = atoi(p);
			break;
		case 'l':
			cml->inputFormat = atoi(p);
			break;
		case 'O':
			cml->colorConversion = atoi(p);
			break;
		case 'f':
			cml->outputRateNumer = atoi(p);
			break;
		case 'F':
			cml->outputRateDenom = atoi(p);
			break;
		case 'j':
			cml->inputRateNumer = atoi(p);
			break;
		case 'J':
			cml->inputRateDenom = atoi(p);
			break;
		case 'p':
			cml->cabacInitFlag = atoi(p);
			break;
		case 'K':
			cml->enableCabac = atoi(p);
			break;

		case 'q':
			cml->qpHdr = atoi(p);
			break;
		case 'n':
			cml->qpMin = atoi(p);
			break;
		case 'm':
			cml->qpMax = atoi(p);
			break;

		case 'B':
			cml->bitPerSecond = atoi(p);
			break;

		case 'U':
			cml->picRc = atoi(p);
			break;
		case 'u':
			cml->ctbRc = atoi(p); //CTB_RC
			break;
		case 'C':
			cml->hrdConformance = atoi(p);
			break;
		case 'c':
			cml->cpbSize = atoi(p);
			break;
		case 's':
			cml->picSkip = atoi(p);
			break;
		case 'L':
			cml->level = atoi(p);
			break;
		case 'P':
			cml->profile = atoi(p);
			break;
		case 'r':
			cml->rotation = atoi(p);
			break;
		case 'R':
			cml->intraPicRate = atoi(p);
			break;
		case 'Z':
			cml->videoStab = atoi(p);
			break;
		case 'A':
			cml->intraQpDelta = atoi(p);
			break;
		case 'D':
			cml->disableDeblocking = atoi(p);
			break;
		case 'W':
			cml->tc_Offset = atoi(p);
			break;
		case 'E':
			cml->beta_Offset = atoi(p);
			break;
		case 'e':
			cml->sliceSize = atoi(p);
			break;

		case 'g':
			cml->bitrateWindow = atoi(p);
			break;
		case 'G':
			cml->fixedIntraQp = atoi(p);
			break;
		case 'N':
			cml->byteStream = atoi(p);
			break;
		case 'I':
			cml->chromaQpOffset = atoi(p);
			bChromaQpOffset = true;
			break;
		case 'M':
			cml->enableSao = atoi(p);
			break;

		case 'k':
			cml->videoRange = atoi(p);
			break;

		case 'S':
			cml->sei = atoi(p);
			break;
		case 'V':
			cml->bFrameQpDelta = atoi(p);
			break;
		case '1':
			if (bpsAdjustCount == MAX_BPS_ADJUST) {
				break;
			}
			/* Argument must be "xx:yy", replace ':' with 0 */
			if ((i = ParseDelim(p, ':')) == -1) {
				break;
			}
			/* xx is frame number */
			cml->bpsAdjustFrame[bpsAdjustCount] = atoi(p);
			/* yy is new target bitrate */
			cml->bpsAdjustBitrate[bpsAdjustCount] = atoi(p + i + 1);
			bpsAdjustCount++;
			break;

		case '0':

			/* Check long option */
			if (strcmp(prm.longOpt, "tier") == 0) {
				cml->tier = atoi(p);
			}

			if (strcmp(prm.longOpt, "roi1DeltaQp") == 0) {
				cml->roi_ctrl[0].roiDeltaQp = atoi(p);
			}

			if (strcmp(prm.longOpt, "roi2DeltaQp") == 0) {
				cml->roi_ctrl[1].roiDeltaQp = atoi(p);
			}

			if (strcmp(prm.longOpt, "roi3DeltaQp") == 0) {
				cml->roi_ctrl[2].roiDeltaQp = atoi(p);
			}

			if (strcmp(prm.longOpt, "roi4DeltaQp") == 0) {
				cml->roi_ctrl[3].roiDeltaQp = atoi(p);
			}

			if (strcmp(prm.longOpt, "roi5DeltaQp") == 0) {
				cml->roi_ctrl[4].roiDeltaQp = atoi(p);
			}

			if (strcmp(prm.longOpt, "roi6DeltaQp") == 0) {
				cml->roi_ctrl[5].roiDeltaQp = atoi(p);
			}

			if (strcmp(prm.longOpt, "roi7DeltaQp") == 0) {
				cml->roi_ctrl[6].roiDeltaQp = atoi(p);
			}

			if (strcmp(prm.longOpt, "roi8DeltaQp") == 0) {
				cml->roi_ctrl[7].roiDeltaQp = atoi(p);
			}

			if (strcmp(prm.longOpt, "roi1Qp") == 0) {
				cml->roi_ctrl[0].roiQp = atoi(p);
			}

			if (strcmp(prm.longOpt, "roi2Qp") == 0) {
				cml->roi_ctrl[1].roiQp = atoi(p);
			}

			if (strcmp(prm.longOpt, "roi3Qp") == 0) {
				cml->roi_ctrl[2].roiQp = atoi(p);
			}

			if (strcmp(prm.longOpt, "roi4Qp") == 0) {
				cml->roi_ctrl[3].roiQp = atoi(p);
			}

			if (strcmp(prm.longOpt, "roi5Qp") == 0) {
				cml->roi_ctrl[4].roiQp = atoi(p);
			}

			if (strcmp(prm.longOpt, "roi6Qp") == 0) {
				cml->roi_ctrl[5].roiQp = atoi(p);
			}

			if (strcmp(prm.longOpt, "roi7Qp") == 0) {
				cml->roi_ctrl[6].roiQp = atoi(p);
			}

			if (strcmp(prm.longOpt, "roi8Qp") == 0) {
				cml->roi_ctrl[7].roiQp = atoi(p);
			}

			if (strcmp(prm.longOpt, "roiMapDeltaQpBlockUnit") == 0) {
				cml->roiMapDeltaQpBlockUnit = atoi(p);
			}

			if (strcmp(prm.longOpt, "roiMapDeltaQpEnable") == 0) {
				cml->roiMapDeltaQpEnable = atoi(p);
			}

			if (strcmp(prm.longOpt, "ipcmMapEnable") == 0) {
				cml->ipcmMapEnable = atoi(p);
			}

			if (strcmp(prm.longOpt, "outBufSizeMax") == 0) {
				cml->outBufSizeMax = atoi(p);
			}

			if (strcmp(prm.longOpt, "codecFormat") == 0) {
				if (cml->outputFormat != atoi(p)) {
//    	      printf("codec Format not match %d %d\n",cml->outputFormat,atoi(p));
					//cml->outputFormat = atoi(p);
					//return NOK;
				}
				if (IS_H264(cml->outputFormat)) {

					cml->max_cu_size  = 16;
					cml->min_cu_size  = 8;
					cml->max_tr_size  = 16;
					cml->min_tr_size  = 4;
					cml->tr_depth_intra = 1;
					cml->tr_depth_inter = 2;

//    		cml->outputFormat = VCENC_VIDEO_CODEC_H264;
					cml->level = VCENC_H264_LEVEL_5_1;
//    		cml->profile = VCENC_H264_HIGH_PROFILE;	// default is H264 HIGH profile

				}
			}

			if (strcmp(prm.longOpt, "cir") == 0) {
				/* Argument must be "xx:yy", replace ':' with 0 */
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				/* xx is cir start */
				cml->cirStart = atoi(p);
				/* yy is cir interval */
				cml->cirInterval = atoi(p + i + 1);
			}

			if (strcmp(prm.longOpt, "intraArea") == 0) {
				/* Argument must be "xx:yy:XX:YY".
				 * xx is left coordinate, replace first ':' with 0 */
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->intraAreaLeft = atoi(p);
				/* yy is top coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->intraAreaTop = atoi(p);
				/* XX is right coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->intraAreaRight = atoi(p);
				/* YY is bottom coordinate */
				p += i + 1;
				cml->intraAreaBottom = atoi(p);
				cml->intraAreaEnable = 1;
			}

			if (strcmp(prm.longOpt, "ipcm1Area") == 0) {
				/* Argument must be "xx:yy:XX:YY".
				 * xx is left coordinate, replace first ':' with 0 */
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->ipcm1AreaLeft = atoi(p);
				/* yy is top coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->ipcm1AreaTop = atoi(p);
				/* XX is right coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->ipcm1AreaRight = atoi(p);
				/* YY is bottom coordinate */
				p += i + 1;
				cml->ipcm1AreaBottom = atoi(p);
			}

			if (strcmp(prm.longOpt, "ipcm2Area") == 0) {
				/* Argument must be "xx:yy:XX:YY".
				 * xx is left coordinate, replace first ':' with 0 */
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->ipcm2AreaLeft = atoi(p);
				/* yy is top coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->ipcm2AreaTop = atoi(p);
				/* XX is right coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->ipcm2AreaRight = atoi(p);
				/* YY is bottom coordinate */
				p += i + 1;
				cml->ipcm2AreaBottom = atoi(p);
			}

			if (strcmp(prm.longOpt, "roi1Area") == 0) {
				/* Argument must be "xx:yy:XX:YY".
				 * xx is left coordinate, replace first ':' with 0 */
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[0].roiAreaLeft = atoi(p);
				/* yy is top coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[0].roiAreaTop = atoi(p);
				/* XX is right coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[0].roiAreaRight = atoi(p);
				/* YY is bottom coordinate */
				p += i + 1;
				cml->roi_ctrl[0].roiAreaBottom = atoi(p);
				cml->roi_ctrl[0].roiAreaEnable = 1;
				cml->roi_ctrl[0].roiAreaEnable = 1;
			}

			if (strcmp(prm.longOpt, "roi2Area") == 0) {
				/* Argument must be "xx:yy:XX:YY".
				 * xx is left coordinate, replace first ':' with 0 */
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[1].roiAreaLeft = atoi(p);
				/* yy is top coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[1].roiAreaTop = atoi(p);
				/* XX is right coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[1].roiAreaRight = atoi(p);
				/* YY is bottom coordinate */
				p += i + 1;
				cml->roi_ctrl[1].roiAreaBottom = atoi(p);
				cml->roi_ctrl[1].roiAreaEnable = 1;
			}

			if (strcmp(prm.longOpt, "roi3Area") == 0) {
				/* Argument must be "xx:yy:XX:YY".
				 * xx is left coordinate, replace first ':' with 0 */
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[2].roiAreaLeft = atoi(p);
				/* yy is top coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[2].roiAreaTop = atoi(p);
				/* XX is right coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[2].roiAreaRight = atoi(p);
				/* YY is bottom coordinate */
				p += i + 1;
				cml->roi_ctrl[2].roiAreaBottom = atoi(p);
				cml->roi_ctrl[2].roiAreaEnable = 1;
			}

			if (strcmp(prm.longOpt, "roi4Area") == 0) {
				/* Argument must be "xx:yy:XX:YY".
				 * xx is left coordinate, replace first ':' with 0 */
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[3].roiAreaLeft = atoi(p);
				/* yy is top coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[3].roiAreaTop = atoi(p);
				/* XX is right coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[3].roiAreaRight = atoi(p);
				/* YY is bottom coordinate */
				p += i + 1;
				cml->roi_ctrl[3].roiAreaBottom = atoi(p);
				cml->roi_ctrl[3].roiAreaEnable = 1;
			}

			if (strcmp(prm.longOpt, "roi5Area") == 0) {
				/* Argument must be "xx:yy:XX:YY".
				 * xx is left coordinate, replace first ':' with 0 */
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[4].roiAreaLeft = atoi(p);
				/* yy is top coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[4].roiAreaTop = atoi(p);
				/* XX is right coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[4].roiAreaRight = atoi(p);
				/* YY is bottom coordinate */
				p += i + 1;
				cml->roi_ctrl[4].roiAreaBottom = atoi(p);
				cml->roi_ctrl[4].roiAreaEnable = 1;
			}

			if (strcmp(prm.longOpt, "roi6Area") == 0) {
				/* Argument must be "xx:yy:XX:YY".
				 * xx is left coordinate, replace first ':' with 0 */
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[5].roiAreaLeft = atoi(p);
				/* yy is top coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[5].roiAreaTop = atoi(p);
				/* XX is right coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[5].roiAreaRight = atoi(p);
				/* YY is bottom coordinate */
				p += i + 1;
				cml->roi_ctrl[5].roiAreaBottom = atoi(p);
				cml->roi_ctrl[5].roiAreaEnable = 1;
			}

			if (strcmp(prm.longOpt, "roi7Area") == 0) {
				/* Argument must be "xx:yy:XX:YY".
				 * xx is left coordinate, replace first ':' with 0 */
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[6].roiAreaLeft = atoi(p);
				/* yy is top coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[6].roiAreaTop = atoi(p);
				/* XX is right coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[6].roiAreaRight = atoi(p);
				/* YY is bottom coordinate */
				p += i + 1;
				cml->roi_ctrl[6].roiAreaBottom = atoi(p);
				cml->roi_ctrl[6].roiAreaEnable = 1;
			}

			if (strcmp(prm.longOpt, "roi8Area") == 0) {
				/* Argument must be "xx:yy:XX:YY".
				 * xx is left coordinate, replace first ':' with 0 */
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[7].roiAreaLeft = atoi(p);
				/* yy is top coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[7].roiAreaTop = atoi(p);
				/* XX is right coordinate */
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->roi_ctrl[7].roiAreaRight = atoi(p);
				/* YY is bottom coordinate */
				p += i + 1;
				cml->roi_ctrl[7].roiAreaBottom = atoi(p);
				cml->roi_ctrl[7].roiAreaEnable = 1;
			}

			if (strcmp(prm.longOpt, "smoothingIntra") == 0) {
				cml->strong_intra_smoothing_enabled_flag = atoi(p);
				ASSERT(cml->strong_intra_smoothing_enabled_flag < 2);
			}

			if (strcmp(prm.longOpt, "scaledWidth") == 0) {
				cml->scaledWidth = atoi(p);
			}

			if (strcmp(prm.longOpt, "scaledHeight") == 0) {
				cml->scaledHeight = atoi(p);
			}

			if (strcmp(prm.longOpt, "scaledOutputFormat") == 0) {
				cml->scaledOutputFormat = atoi(p);
			}

			if (strcmp(prm.longOpt, "enableDeblockOverride") == 0) {
				cml->enableDeblockOverride = atoi(p);
			}
			if (strcmp(prm.longOpt, "deblockOverride") == 0) {
				cml->deblockOverride = atoi(p);
			}

			if (strcmp(prm.longOpt, "enableScalingList") == 0) {
				cml->enableScalingList = atoi(p);
			}

			if (strcmp(prm.longOpt, "compressor") == 0) {
				cml->compressor = atoi(p);
			}

			if (strcmp(prm.longOpt, "interlacedFrame") == 0) {
				cml->interlacedFrame = atoi(p);
			}

			if (strcmp(prm.longOpt, "fieldOrder") == 0) {
				cml->fieldOrder = atoi(p);
			}
			if (strcmp(prm.longOpt, "gopSize") == 0) {
				cml->gopSize = atoi(p);
			}
			if (strcmp(prm.longOpt, "gopConfig") == 0) {
				cml->gopCfg = p;
			}
			if (strcmp(prm.longOpt, "gopLowdelay") == 0) {
				cml->gopLowdelay = atoi(p);
			}
			if (strcmp(prm.longOpt, "LTR") == 0) {
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->ltrInterval = atoi(p);
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->longTermGapOffset = atoi(p);
				p += i + 1;
				i = ParseDelim(p, ':');
				cml->longTermGap = atoi(p);
				if (i >= 0) {
					p += i + 1;
					cml->longTermQpDelta = atoi(p);
				}
			}

			if (strcmp(prm.longOpt, "tile") == 0) {
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->num_tile_columns = atoi(p);
				p += i + 1;

				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->num_tile_rows = atoi(p);
				p += i + 1;

				i = ParseDelim(p, ':');
				cml->loop_filter_across_tiles_enabled_flag = atoi(p);
				p += i + 1;

				cml->tiles_enabled_flag = ((cml->num_tile_columns * cml->num_tile_rows) > 1);
			}

			if (strcmp(prm.longOpt, "skipFramePOC") == 0) {
				cml->skip_frame_poc = atoi(p);
				cml->skip_frame_enabled_flag = (cml->skip_frame_poc != 0);
			}
			if (strcmp(prm.longOpt, "bFrameQpDelta") == 0) {
				cml->bFrameQpDelta = atoi(p);
			}

			if (strcmp(prm.longOpt, "outReconFrame") == 0) {
				cml->outReconFrame = atoi(p);
			}


			if (strcmp(prm.longOpt, "gdrDuration") == 0) {
				cml->gdrDuration = atoi(p);
			}

			if (strcmp(prm.longOpt, "ipcmFilterDisable") == 0) {
				cml->pcm_loop_filter_disabled_flag = atoi(p);
			}

			if (strcmp(prm.longOpt, "bitVarRangeI") == 0) {
				cml->bitVarRangeI = atoi(p);
			}

			if (strcmp(prm.longOpt, "bitVarRangeP") == 0) {
				cml->bitVarRangeP = atoi(p);
			}

			if (strcmp(prm.longOpt, "bitVarRangeB") == 0) {
				cml->bitVarRangeB = atoi(p);
			}
			if (strcmp(prm.longOpt, "tolMovingBitRate") == 0) {
				cml->tolMovingBitRate = atoi(p);
			}

			if (strcmp(prm.longOpt, "tolCtbRcInter") == 0) {
				cml->tolCtbRcInter = atof(p);
			}
			if (strcmp(prm.longOpt, "tolCtbRcIntra") == 0) {
				cml->tolCtbRcIntra = atof(p);
			}

			if (strcmp(prm.longOpt, "smoothPsnrInGOP") == 0) {
				cml->smoothPsnrInGOP = atoi(p);
			}

			if (strcmp(prm.longOpt, "staticSceneIbitPercent") == 0) {
				cml->u32StaticSceneIbitPercent = atoi(p);
			}

			if (strcmp(prm.longOpt, "monitorFrames") == 0) {
				cml->monitorFrames = atoi(p);
			}
			if (strcmp(prm.longOpt, "roiMapDeltaQpFile") == 0) {
				cml->roiMapDeltaQpFile = p;
			}
			if (strcmp(prm.longOpt, "roiMapDeltaQpBinFile") == 0) {
				cml->roiMapDeltaQpBinFile = p;
			}
			if (strcmp(prm.longOpt, "ipcmMapFile") == 0) {
				cml->ipcmMapFile = p;
			}
			if (strcmp(prm.longOpt, "roiMapInfoBinFile") == 0) {
				cml->roiMapInfoBinFile = p;
			}
			if (strcmp(prm.longOpt, "RoimapCuCtrlInfoBinFile") == 0) {
				cml->RoimapCuCtrlInfoBinFile = p;
			}
			if (strcmp(prm.longOpt, "RoimapCuCtrlIndexBinFile") == 0) {
				cml->RoimapCuCtrlIndexBinFile = p;
			}
			if (strcmp(prm.longOpt, "RoiCuCtrlVer") == 0) {
				cml->RoiCuCtrlVer = atoi(p);
			}
			if (strcmp(prm.longOpt, "RoiQpDeltaVer") == 0) {
				cml->RoiQpDeltaVer = atoi(p);
			}

			/* stride */
			if (strcmp(prm.longOpt, "inputAlignmentExp") == 0) {
				cml->exp_of_input_alignment = atoi(p);
			}

			if (strcmp(prm.longOpt, "refAlignmentExp") == 0) {
				cml->exp_of_ref_alignment = atoi(p);
			}

			if (strcmp(prm.longOpt,  "refChromaAlignmentExp") == 0) {
				cml->exp_of_ref_ch_alignment = atoi(p);
			}

			if (strcmp(prm.longOpt, "bitDepthLuma") == 0) {
				cml->bitDepthLuma = atoi(p);
			}

			if (strcmp(prm.longOpt, "bitDepthChroma") == 0) {
				cml->bitDepthChroma = atoi(p);
			}

			if (strcmp(prm.longOpt, "blockRCSize") == 0) {
				cml->blockRCSize = atoi(p);
			}

			if (strcmp(prm.longOpt, "rcQpDeltaRange") == 0) {
				cml->rcQpDeltaRange = atoi(p);
			}

			if (strcmp(prm.longOpt, "rcBaseMBComplexity") == 0) {
				cml->rcBaseMBComplexity = atoi(p);
			}

			if (strcmp(prm.longOpt, "picQpDeltaRange") == 0) {
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->picQpDeltaMin = atoi(p);
				p += i + 1;
				cml->picQpDeltaMax = atoi(p);
			}

			if (strcmp(prm.longOpt, "ctbRowQpStep") == 0) {
				cml->ctbRcRowQpStep = atoi(p);
			}

			if (strcmp(prm.longOpt, "sceneChange") == 0) {
				i32 j = 0, iSc = 0, tmp;
				while (j++ < MAX_SCENE_CHANGE) {
					if ((tmp = atoi(p))) {
						cml->sceneChange[iSc ++] = tmp;
					}

					if ((i = ParseDelim(p, ':')) == -1) {
						break;
					}
					p += i + 1;
				}
			}

			/* CU infor dumping */
			if (strcmp(prm.longOpt, "enableOutputCuInfo") == 0) {
				cml->enableOutputCuInfo = atoi(p);
			}

			if (strcmp(prm.longOpt, "rdoLevel") == 0) {
				cml->rdoLevel = atoi(p);
			}

			if (strcmp(prm.longOpt, "enableRdoQuant") == 0) {
				cml->enableRdoQuant = atoi(p);
			}

			if (strcmp(prm.longOpt, "enableVuiTimingInfo") == 0) {
				cml->vui_timing_info_enable = atoi(p);
			}
			if (strcmp(prm.longOpt, "lookaheadDepth") == 0) {
				cml->lookaheadDepth = atoi(p);
			}
			if (strcmp(prm.longOpt, "cuInfoVersion") == 0) {
				cml->cuInfoVersion = atoi(p);
			}
			if (strcmp(prm.longOpt, "halfDsInput") == 0) {
				cml->halfDsInput = p;
			}
			/* mirror */
			/*
			        if (strcmp(prm.longOpt, "mirror") == 0)
			        {
			          cml->mirror = atoi(p);
			          cml->mirror = CLIP3(0, 1, cml->mirror);
			        }
			*/
			if (strcmp(prm.longOpt, "hashtype") == 0) {
				cml->hashtype = atoi(p);
			}

			if (strcmp(prm.longOpt, "verbose") == 0) {
				cml->verbose = atoi(p);
			}

			/* constant chroma control */
			if (strcmp(prm.longOpt, "enableConstChroma") == 0) {
				cml->constChromaEn = atoi(p);
			}
			if (strcmp(prm.longOpt, "constCb") == 0) {
				cml->constCb = atoi(p);
			}
			if (strcmp(prm.longOpt, "constCr") == 0) {
				cml->constCr = atoi(p);
			}

			/* qpMin/qpMax for I picture */
			if (strcmp(prm.longOpt, "qpMinI") == 0) {
				cml->qpMinI = atoi(p);
			}
			if (strcmp(prm.longOpt, "qpMaxI") == 0) {
				cml->qpMaxI = atoi(p);
			}

			/* vbr, controlled by qpMin */
			if (strcmp(prm.longOpt, "vbr") == 0) {
				cml->vbr = atoi(p);
			}

			if (strcmp(prm.longOpt, "RPSInSliceHeader") == 0) {
				cml->RpsInSliceHeader = atoi(p);
			}

			if (strcmp(prm.longOpt, "POCConfig") == 0) {
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->picOrderCntType = atoi(p);
				p += i + 1;
				if ((i = ParseDelim(p, ':')) == -1) {
					break;
				}
				cml->log2MaxPicOrderCntLsb = atoi(p);
				p += i + 1;
				cml->log2MaxFrameNum = atoi(p);
			}

			if (strcmp(prm.longOpt, "ssim") == 0) {
				cml->ssim = atoi(p);
			}

			if (strcmp(prm.longOpt, "MEVertRange") == 0) {
				cml->MEVertRange = atoi(p);
			}

			/* skip map */
			if (strcmp(prm.longOpt, "skipMapEnable") == 0) {
				cml->skipMapEnable = atoi(p);
			}
			if (strcmp(prm.longOpt, "skipMapBlockUnit") == 0) {
				cml->skipMapBlockUnit = atoi(p);
			}
			if (strcmp(prm.longOpt, "skipMapFile") == 0) {
				cml->skipMapFile = p;
			}

			if (strcmp(prm.longOpt, "dumpRegister") == 0) {
				cml->dumpRegister = atoi(p);
			}

			if (strcmp(prm.longOpt, "rasterscan") == 0) {
				cml->rasterscan = atoi(p);
			}

			if (strcmp(prm.longOpt, "crf") == 0) {
				cml->crf = atoi(p);
			}

			if (strcmp(prm.longOpt, "AXIAlignment") == 0) {
				cml->AXIAlignment = strtoul(p, NULL, 16);
			}

			if (strcmp(prm.longOpt, "ivf") == 0) {
				cml->ivf = atoi(p);
			}

			if (strcmp(prm.longOpt, "psyFactor") == 0) {
				cml->PsyFactor = atoi(p);
			}

			if (strcmp(prm.longOpt, "codedChromaIdc") == 0) {
				cml->codedChromaIdc = (VCEncChromaIdcType)atoi(p);
			}

			if (strcmp(prm.longOpt, "aq_mode") == 0) {
				cml->aq_mode = atoi(p);
			}
			if (strcmp(prm.longOpt, "aq_strength") == 0) {
				cml->aq_strength = atof(p);
			}

			if (strcmp(prm.longOpt, "writeReconToDDR") == 0) {
				cml->writeReconToDDR = atoi(p);
			}
			if (strcmp(prm.longOpt, "voe") == 0) {
				cml->voe = atoi(p);
			}

			if (strcmp(prm.longOpt, "dbg") == 0) {
				cml->voe_dbg = atoi(p);
			}
#if 0
			if (strcmp(prm.longOpt, "TxTypeSearchEnable") == 0) {
				cml->TxTypeSearchEnable = atoi(p);
			}
#endif
			if (strcmp(prm.longOpt, "preset") == 0) {
				cml->preset = atof(p);
				Parameter_Preset(cml);
			}
			if (strcmp(prm.longOpt, "tune") == 0) {
				int tune = atoi(p);
				switch (tune) {
				case 0:
					cml->aq_mode = 0;
					cml->PsyFactor = 0;
					break;
				case 1:
					cml->aq_mode = 2;
					cml->PsyFactor = 0;
					break;
				case 2:
					cml->aq_mode = 2;
					cml->PsyFactor = 2;
					break;
				default:
					ASSERT(0 && "unknown tune mode");
					break;
				}
			}
			break;

		default:
			break;
		}
	}
	if (!bChromaQpOffset) {
		cml->chromaQpOffset = 0;
	}

	Parameter_Check(cml);
	return status;
}




int parameter_jpg_get(i32 argc, char **argv, commandLine_s *cml)
{
	i32 ret, i;
	char *optarg;
	struct parameter argument;
	int status = 0;
	argument.cnt = 1;

	{
//	    cml->outputFormat = VCENC_VIDEO_CODEC_JPEG;
		cml->qLevel = 1;
		cml->restartInterval = 0;
		cml->inputFormat = 1;
//	    cml->frameType = 0;
		cml->colorConversion = 0;
		cml->rotation = 0;
		cml->partialCoding = 0;
		cml->codingMode = 0;
		cml->markerType = 0;
		cml->unitsType = 0;
		cml->xdensity = 1;
		cml->ydensity = 1;


		cml->hashtype = 0;
		cml->mirror = 0;
		cml->constChromaEn = 0;
		cml->constCb = 0x80;
		cml->constCr = 0x80;
		cml->inputRateNumer = 30;
		cml->inputRateDenom = 1;

		cml->rcMode        = 1;
		cml->picQpDeltaMin = -2;
		cml->picQpDeltaMax = 3;
		cml->qpmin = 0;
		cml->qpmax = 51;
		cml->fixedQP = -1;
		cml->exp_of_input_alignment = 4;
		cml->voe = 1;
	}

	while ((ret = get_option(argc, argv, jpeg_options, &argument)) != -1) {
		if (ret == -2) {
			status = -1;
		}
		optarg = argument.argument;

		switch (argument.short_opt) {
		case 'H':
			jpeg_help();
			exit(0);
		case 'i':
			if (strlen(optarg) < MAX_PATH) {
				strcpy(cml->input, optarg);
			} else {
				printf("String over range %d > %d\n", strlen(optarg), MAX_PATH);
			}
			break;

		case 'o':
			if (strlen(optarg) < MAX_PATH) {
				strcpy(cml->output, optarg);
			} else {
				printf("String over range %d > %d\n", strlen(optarg), MAX_PATH);
			}
			break;


		case 'a':
			cml->firstPic = atoi(optarg);
			break;
		case 'b':
			cml->lastPic = atoi(optarg) - 1;
			if (cml->lastPic < 0) {
				cml->lastPic = 0;
			}
			break;
		case 'x':
			cml->width = atoi(optarg);
			break;
		case 'y':
			cml->height = atoi(optarg);
			break;
		case 'w':
			cml->lumWidthSrc = atoi(optarg);
			break;
		case 'h':
			cml->lumHeightSrc = atoi(optarg);
			break;
		case 'X':
			cml->horOffsetSrc = atoi(optarg);
			break;
		case 'Y':
			cml->verOffsetSrc = atoi(optarg);
			break;
		case 'R':
			cml->restartInterval = atoi(optarg);
			break;
		case 'q':
			cml->qLevel = atoi(optarg);
			break;
		case 'g':
//            cml->frameType = atoi(optarg);
			cml->inputFormat = atoi(optarg);
			break;
		case 'v':
			cml->colorConversion = atoi(optarg);
			break;
		case 'G':
			cml->rotation = atoi(optarg);
			break;
		case 'p':
			cml->partialCoding = atoi(optarg);
			break;
		case 'm':
			cml->codingMode = atoi(optarg);
			break;
		case 't':
			cml->markerType = atoi(optarg);
			break;
		case 'u':
			cml->unitsType = atoi(optarg);
			break;
		case 'k':
			cml->xdensity = atoi(optarg);
			break;
		case 'l':
			cml->ydensity = atoi(optarg);
			break;
#if 0
		case 'I':
			cml->inputThumb = optarg;
			break;
		case 'T':
			cml->thumbnail = atoi(optarg);
			break;
		case 'K':
			cml->widthThumb = atoi(optarg);
			break;
		case 'L':
			cml->heightThumb = atoi(optarg);
			break;
		case 'c':
			cml->comLength = atoi(optarg);
			break;
		case 'C':
			if (strlen(optarg) < MAX_PATH) {
				strcpy(cml->com, optarg);
			} else {
				status = -1;
			}
			break;
#endif

		case 'A':
			cml->hashtype = atoi(optarg);
			break;
		/*
		        case 'M':
		            cml->mirror = atoi(optarg);
		            break;
		*/
		case 'd':
			cml->constChromaEn = atoi(optarg);
			break;
		case 'e':
			cml->constCb = atoi(optarg);
			break;
		case 'f':
			cml->constCr = atoi(optarg);
			break;
		case 'n':
			cml->inputRateNumer = atoi(optarg);
			break;
		case 'r':
			cml->inputRateDenom = atoi(optarg);
			break;
		case 'V':
			cml->rcMode = atoi(optarg);
			if ((cml->rcMode < 0) || (cml->rcMode > 2)) {
				status = -1;
			}
			break;
		case 'E':
			cml->qpmin = atoi(optarg);
			break;
		case 'F':
			cml->qpmax = atoi(optarg);
			break;
		case 'U':
			if ((i = ParseDelim(optarg, ':')) == -1) {
				break;
			}
			cml->picQpDeltaMin = atoi(optarg);
			optarg += i + 1;
			cml->picQpDeltaMax = atoi(optarg);

			if ((cml->picQpDeltaMin < -10) || (cml->picQpDeltaMin > -1) || (cml->picQpDeltaMax < 1) || (cml->picQpDeltaMax > 10)) {
				status = -1;
			}
			break;
		case 'O':
			cml->fixedQP = atoi(optarg);
			break;
		case 'Q':
			cml->exp_of_input_alignment = atoi(optarg);
			break;

		case '0':
			/* Check long option */
			if (strcmp(argument.longOpt, "voe") == 0) {
				cml->voe = atoi(optarg);
			}

			break;

		default:
			break;
		}
	}

	return status;
}

/*------------------------------------------------------------------------------

    JPEG Help

------------------------------------------------------------------------------*/
void jpeg_help(void)
{
	fprintf(stdout, "Usage:  %s [options] -i inputfile\n", "jpeg_testenc");
	fprintf(stdout,
			"  -H    --help              Display this help.\n\n"
		   );

	fprintf(stdout,
			" Parameters affecting input frame and encoded frame resolutions and cropping:\n"
			"  -i[s] --input             Read input from file. [input.yuv]\n"
			"  -I[s] --inputThumb        Read thumbnail input from file. [thumbnail.jpg]\n"
			"  -o[s] --output            Write output to file. [stream.jpg]\n"
			"  -a[n] --firstPic          First picture of input file. [0]\n"
			"  -b[n] --lastPic           Last picture of input file. [0]\n"
			"  -w[n] --lumWidthSrc       Width of source image. [176]\n"
			"  -h[n] --lumHeightSrc      Height of source image. [144]\n"
			"  -x[n] --width             Width of output image. [--lumWidthSrc]\n"
			"  -y[n] --height            Height of output image. [--lumHeightSrc]\n"
			"  -X[n] --horOffsetSrc      Output image horizontal offset. [0]\n"
			"  -Y[n] --verOffsetSrc      Output image vertical offset. [0]\n"
		   );

	fprintf(stdout,
			"\n Parameters for pre-processing frames before encoding:\n"
			"  -g[n] --frameType         Input YUV format. [0]\n"
			"                               0 - YUV420 planar CbCr (IYUV/I420)\n"
			"                               1 - YUV420 semi-planar CbCr (NV12)\n"
			"                               2 - YUV420 semi-planar CrCb (NV21)\n"
			"                               3 - YUYV422 interleaved (YUYV/YUY2)\n"
			"                               4 - UYVY422 interleaved (UYVY/Y422)\n"
			"                               5 - RGB565 16bpp\n"
			"                               6 - BGR565 16bpp\n"
			"                               7 - RGB555 16bpp\n"
			"                               8 - BGR555 16bpp\n"
			"                               9 - RGB444 16bpp\n"
			"                               10 - BGR444 16bpp\n"
			"                               11 - RGB888 32bpp\n"
			"                               12 - BGR888 32bpp\n"
			"                               13 - RGB101010 32bpp\n"
			"                               15 - YUV420 10 bit planar CbCr (I010)\n"
			"                               16 - YUV420 10 bit planar CbCr (P010)\n"
			"                               19 - YUV420 customer private tile for HEVC\n"
			"                               20 - YUV420 customer private tile for H.264\n"
			"  -v[n] --colorConversion   RGB to YCbCr color conversion type. [0]\n"
			"                               0 - ITU-R BT.601, RGB limited [16...235] (BT601_l.mat)\n"
			"                               1 - ITU-R BT.709, RGB limited [16...235] (BT709_l.mat)\n"
			"                               2 - User defined, coefficients defined in test bench.\n"
			"                               3 - ITU-R BT.2020\n"
			"                               4 - ITU-R BT.601, RGB full [0...255] (BT601_f.mat)\n"
			"                               5 - ITU-R BT.601, RGB limited [0...219] (BT601_219.mat)\n"
			"                               6 - ITU-R BT.709, RGB full [0...255] (BT709_f.mat)\n"
			"  -G[n] --rotation          Rotate input image. [0]\n"
			"                               0 - disabled\n"
			"                               1 - 90 degrees right\n"
			"                               2 - 90 degrees left\n"
			"                               3 - 180 degrees\n"
//            "  -M[n]  --mirror           Mirror input image. [0]\n"
//            "                               0 - disabled horizontal mirror\n"
//            "                               1 - enable horizontal mirror\n"
			"  -Q[n]  --inputAlignmentExp Alignment value of input frame buffer. [4]\n"
			"                               0 = Disable alignment \n"
			"                               4..12 = Base address of input frame buffer and each line are aligned to 2^inputAlignmentExp \n"
		   );
	fprintf(stdout,
			"  -d[n] --enableConstChroma   0..1 Enable/Disable setting chroma to a constant pixel value. [0]\n"
			"                                 0 = Disable. \n"
			"                                 1 = Enable. \n"
			"  -e[n] --constCb             0..255. The constant pixel value for Cb. [128]\n"
			"  -f[n] --constCr             0..255. The constant pixel value for Cr. [128]\n"
		   );

	fprintf(stdout,
			"\n Parameters affecting the output stream and encoding tools:\n"
			"  -R[n] --restartInterval   Restart interval in MCU rows. [0]\n"
			"  -q[n] --qLevel            0..10, quantization scale. [1]\n"
			"                            10 = use testbench defined qtable\n"
			"  -p[n] --codingType        0=whole frame, 1=partial frame encoding. [0]\n"
			"  -m[n] --codingMode        0=YUV420, 1=YUV422, 2=Monochrome [0]\n"
			"  -t[n] --markerType        Quantization/Huffman table markers. [0]\n"
			"                               0 = Single marker\n"
			"                               1 = Multiple markers\n"
			"  -u[n] --units             Units type of x- and y-density. [0]\n"
			"                               0 = pixel aspect ratio\n"
			"                               1 = dots/inch\n"
			"                               2 = dots/cm\n"
			"  -k[n] --xdensity          Xdensity to APP0 header. [1]\n"
			"  -l[n] --ydensity          Ydensity to APP0 header. [1]\n"
			"        --streamBufChain    Enable two output stream buffers. [0]\n"
			"                               0 - Single output stream buffer.\n"
			"                               1 - Two output stream buffers chained together.\n"
			"                            Note the minimum allowable size of the first stream buffer is 1k bytes + thumbnail data size if any.\n"

		   );
// AmebaProII non support thumbnail
// AmebaProII non support hashtype CRC32/Checksum32


	fprintf(stdout,
			"\n");
#if 0
	fprintf(stdout,
			"\n Parameters affecting lossless encoding:\n"
			"        --lossless          0=lossy, 1~7 Enalbe lossless with prediction select mode n [0]\n"
			"        --ptrans            0..7 Point transform value for lossless encoding. [0]\n");
#ifdef TB_DEFINED_COMMENT
	fprintf(stdout,
			"\n   Using comment values defined in testbench!\n");
#else
	// AmebaProII non support Comment header data

#if 0
	fprintf(stdout,
			"  -c[n] --comLength         Comment header data length. [0]\n"
			"  -C[s] --comFile           Comment header data file. [com.txt]\n");
#endif
#endif
#endif
	fprintf(stdout,
			"\n Parameters affecting Motion JPEG, rate control, JPEG bitsPerPic:\n"
#if 0 // JPEG¡@non support MJPEG&RC
			"  -J[n] --mjpeg             Enable Motion JPEG[0]\n"
			"                               0 = Disable motion JPEG.\n"
			"                               1 = Enable motion JPEG.\n"
			"  -B[n] --bitPerSecond      Target bit per second. [0]\n"
			"                               0 - RC OFF\n"
			"                               none zero - RC ON\n"
#endif
			"  -n[n] --frameRateNum      1..1048575 Output picture rate numerator. [30]\n"
			"  -r[n] --frameRateDenom    1..1048575 Output picture rate denominator. [1]\n"
			"  -V[n] --rcMode            0..2, JPEG/MJPEG RC mode. [1]\n"
			"                               0 = single frame RC mode. \n"
			"                               1 = video RC with CBR. \n"
			"                               2 = video RC with VBR. \n"
			"  -U[n:m] --picQpDeltaRange Min:Max. Qp Delta range in picture-level rate control.\n"
			"                               Min: -10..-1 Minimum Qp_Delta in picture RC. [-2]\n"
			"                               Max:  1..10  Maximum Qp_Delta in picture RC. [3]\n"
			"                               This range only applies to two neighboring frames.\n"
			"  -E[n] --qpMin             0..51, Minimum frame qp. [0]\n"
			"  -F[n] --qpMax             0..51, Maxmum frame qp. [51]\n"
			"  -O[n] --fixedQP           -1..51, Fixed qp for every frame. [-1]\n"
			"                               -1 = disable fixed qp mode\n"
			"                               0-51 = value of fixed qp.\n"
		   );
#if 0
	fprintf(stdout,
			"\n Parameters for DEC400 compressed table(tile status):\n"
			"  --dec400TableInput            Read input DEC400 compressed table from file. [dec400CompTableinput.bin]\n"
		   );
#endif
	fprintf(stdout,
			"\nTesting parameters that are not supported for end-user:\n"
			"  -P[n] --trigger           Logic Analyzer trigger at picture <n>. [-1]\n"
			"                            -1 = Disable the trigger.\n"
			"  -D[n] --XformCustomerPrivateFormat    -1..4 Convert YUV420 to customer private format. [-1]\n"
			"                               -1 - No conversion to customer private format\n"
			"                               0 - customer private tile format for HEVC\n"
			"                               1 - customer private tile format for H.264\n"
			"                               2 - customer private YUV422_888\n"
			"                               3 - common data 8-bit tile 4x4\n"
			"                               4 - common data 10-bit tile 4x4\n"
			"\n");
#if 0
	fprintf(stdout,
			"\n Parameters for OSD overlay controls (i should be a number from 1 to 8):\n"
			"  --overlayEnables             8 bits indicate enable for 8 overlay region. [0]\n"
			"                                   1: region 1 enabled\n"
			"                                   2: region 2 enabled\n"
			"                                   3: region 1 and 2 enabled\n"
			"                                   and so on.\n"
			"  --olInputi                   input file for overlay region 1-8. [olInputi.yuv]\n"
			"                                   for example --olInput1"
			"  --olFormati                  0..1 Specify the overlay input format. [0]\n"
			"                                   0: ARGB8888\n"
			"                                   1: NV12\n"
			"  --olAlphai                   0..255 Specify a global alpha value for NV12 overlay format. [0]\n"
			"  --olWidthi                   Width of overlay region. Must be set if region enabled. [0]\n"
			"  --olHeighti                  Height of overlay region. Must be set if region enabled. [0]\n"
			"  --olXoffseti                 Horizontal offset of overlay region top left pixel. [0]\n"
			"                                   must be under 2 aligned condition. [0]\n"
			"  --olYoffseti                 Vertical offset of overlay region top left pixel. [0]\n"
			"                                   must be under 2 aligned condition. [0]\n"
			"  --olYStridei                 Luma stride in bytes. Default value is based on format.\n"
			"                                   [olWidthi * 4] if ARGB888.\n"
			"                                   [olWidthi] if NV12.\n"
			"  --olUVStridei                Chroma stride in bytes. Default value is based on luma stride.\n"
			"  --olCropXoffseti             OSD cropping top left horizontal offset. [0]\n"
			"                                   must be under 2 aligned condition. [0]\n"
			"  --olCropYoffseti             OSD cropping top left vertical offset. [0]\n"
			"                                   must be under 2 aligned condition. [0]\n"
			"  --olCropWidthi               OSD cropping width. [olWidthi]\n"
			"  --olCropHeighti              OSD cropping height. [olHeighti]\n"
			"\n");
#endif
}

