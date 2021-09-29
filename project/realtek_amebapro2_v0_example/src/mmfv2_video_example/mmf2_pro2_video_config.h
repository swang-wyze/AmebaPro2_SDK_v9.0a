#ifndef MMF2_PRO2_VIDEO_CONFIG_H
#define MMF2_PRO2_VIDEO_CONFIG_H

/*ISP CHANNEL*/
//ISP channel 0/1/2:HEVC/JPEG/NV12 4:only RGB

/*ENCODE TYPE*/
//type : 0:HEVC 1:H264 2:JPEG 3:NV12 4:RGB 5:HEVC+JPEG 6:H264+JPEG
enum encode_type {
	VIDEO_HEVC = 0,
	VIDEO_H264,
	VIDEO_JPEG,
	VIDEO_NV12,
	VIDEO_RGB,
	VIDEO_NV16,
	VIDEO_HEVC_JPEG,
	VIDEO_H264_JPEG
};

enum video_resolution {
	VIDEO_QCIF = 0,
	VIDEO_CIF,
	VIDEO_WVGA,
	VIDEO_VGA,
	VIDEO_D1,
	VIDEO_HD,
	VIDEO_FHD,
	VIDEO_3M,
	VIDEO_5M,
};

static const int video_res_w[8] = {
	176, //QCIF
	352, // CIF
	640, // WVGA
	640, // VGA
	720, //D1
	1280, // HD
	1920, // FHD
	2048, // 3M
	2592, // 5M
};

static const int video_res_h[8] = {
	144,  //QCIF
	288,  // CIF
	360,  // WVGA
	480,  // VGA
	480,  //D1
	720, // HD
	1080, // FHD
	1536, // 3M
	1944, // 5M
};

#endif /* MMF2_PRO2_VIDEO_CONFIG_H */