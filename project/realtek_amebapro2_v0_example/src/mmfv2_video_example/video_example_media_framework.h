#ifndef MMF2_VIDEO_EXAMPLE_H
#define MMF2_VIDEO_EXAMPLE_H

#include "platform_opts.h"

void video_example_media_framework(void);

void mmf2_video_example_v1_init(void);

void mmf2_video_example_v2_init(void);

void mmf2_video_example_v3_init(void);

void mmf2_video_example_v1_shapshot_init(void);

void mmf2_video_example_simo_init(void);

void mmf2_video_example_av_init(void);

void mmf2_video_example_av2_init(void);

void mmf2_video_example_av21_init(void);

void mmf2_video_example_av_mp4_init(void);

void mmf2_video_example_av_rtsp_mp4_init(void);

void mmf2_video_example_joint_test_init(void);

void mmf2_video_example_joint_test_rtsp_mp4_init(void);

void mmf2_video_example_2way_audio_pcmu_doorbell_init(void);

void mmf2_video_example_2way_audio_pcmu_init(void);

void mmf2_video_example_array_rtsp_init(void);

void mmf2_video_example_v1_param_change_init(void);

void mmf2_video_example_av_mp4_httpfs_init(void);

void mmf2_video_example_v4_rgb_init(void);

void mmf2_video_example_v1_reset_init(void);

void mmf2_video_example_p2p_av_init(void);

int video_voe_presetting(int v1_enable, int v1_resolution, int v1_bps, int v1_shapshot,
						 int v2_enable, int v2_resolution, int v2_bps,
						 int v3_enable, int v3_resolution, int v3_bps,
						 int v4_enable, int v4_resolution);


#endif /* MMF2_VIDEO_EXAMPLE_H */