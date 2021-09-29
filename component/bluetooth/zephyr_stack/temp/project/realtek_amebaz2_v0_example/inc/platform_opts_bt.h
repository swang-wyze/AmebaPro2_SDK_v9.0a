#ifndef __PLATFORM_OPTS_BT_H__
#define __PLATFORM_OPTS_BT_H__

#ifndef CONFIG_USE_ZEPHYR_BT_STACK
/*RTK STACK MACROs************************************************************/
#define CONFIG_BT			0

#define VERSION_2019		1
#define VERSION_2021		2
#define UPPER_STACK_VERSION	VERSION_2019

#if CONFIG_BT
#define CONFIG_FTL_ENABLED
#define CONFIG_BT_CONFIG			0
#define CONFIG_BT_AIRSYNC_CONFIG	0
#define CONFIG_BT_PERIPHERAL		0
#define CONFIG_BT_CENTRAL			0
#define CONFIG_BT_SCATTERNET		0
#define CONFIG_BT_BEACON			0
#define CONFIG_BT_FUZZ_TEST			0
#define CONFIG_BT_OTA_CENTRAL_CLIENT					0
#define CONFIG_BT_HARMONY_ADAPTER	0
#define CONFIG_BT_DATATRANS			0
#define CONFIG_BT_MESH_PROVISIONER	0
#define CONFIG_BT_MESH_DEVICE		0
#define CONFIG_BT_MESH_PROVISIONER_MULTIPLE_PROFILE		0
#define CONFIG_BT_MESH_DEVICE_MULTIPLE_PROFILE			0
#define CONFIG_BT_MESH_CENTRAL							0
#define CONFIG_BT_MESH_PERIPHERAL						0
#define CONFIG_BT_MESH_SCATTERNET						0
#define CONFIG_BT_MESH_TEST			0
#endif // CONFIG_BT

#if defined CONFIG_BT_SCATTERNET && CONFIG_BT_SCATTERNET
#undef CONFIG_BT_PERIPHERAL
#undef CONFIG_BT_CENTRAL
#define CONFIG_BT_PERIPHERAL		1
#define CONFIG_BT_CENTRAL			1
#endif

#if ((defined CONFIG_BT_MESH_PROVISIONER && CONFIG_BT_MESH_PROVISIONER) || \
    (defined CONFIG_BT_MESH_DEVICE && CONFIG_BT_MESH_DEVICE) || \
    (defined CONFIG_BT_MESH_PROVISIONER_MULTIPLE_PROFILE && CONFIG_BT_MESH_PROVISIONER_MULTIPLE_PROFILE) || \
    (defined CONFIG_BT_MESH_DEVICE_MULTIPLE_PROFILE && CONFIG_BT_MESH_DEVICE_MULTIPLE_PROFILE))
#define CONFIG_BT_MESH_USER_API		1
#define CONFIG_BT_MESH_IDLE_CHECK	0
#endif

#if ((defined CONFIG_BT_MESH_PROVISIONER && CONFIG_BT_MESH_PROVISIONER) && (defined CONFIG_BT_MESH_DEVICE && CONFIG_BT_MESH_DEVICE))
#error "CONFIG_BT_MESH_PROVISIONER & CONFIG_BT_MESH_DEVICE can not open at the same time"
#endif

#if ((defined CONFIG_BT_MESH_PROVISIONER_MULTIPLE_PROFILE && CONFIG_BT_MESH_PROVISIONER_MULTIPLE_PROFILE) && (defined CONFIG_BT_MESH_DEVICE_MULTIPLE_PROFILE && CONFIG_BT_MESH_DEVICE_MULTIPLE_PROFILE))
#error "CONFIG_BT_MESH_PROVISIONER_MULTIPLE_PROFILE & CONFIG_BT_MESH_DEVICE_MULTIPLE_PROFILE can not open at the same time"
#endif

#if (((defined CONFIG_BT_MESH_CENTRAL && CONFIG_BT_MESH_CENTRAL) && (defined CONFIG_BT_MESH_PERIPHERAL && CONFIG_BT_MESH_PERIPHERAL)) || \
    ((defined CONFIG_BT_MESH_CENTRAL && CONFIG_BT_MESH_CENTRAL) && (defined CONFIG_BT_MESH_SCATTERNET && CONFIG_BT_MESH_SCATTERNET)) || \
    ((defined CONFIG_BT_MESH_PERIPHERAL && CONFIG_BT_MESH_PERIPHERAL) && (defined CONFIG_BT_MESH_SCATTERNET && CONFIG_BT_MESH_SCATTERNET)) || \
    ((defined CONFIG_BT_MESH_CENTRAL && CONFIG_BT_MESH_CENTRAL) && (defined CONFIG_BT_MESH_PERIPHERAL && CONFIG_BT_MESH_PERIPHERAL) && (defined CONFIG_BT_MESH_SCATTERNET && CONFIG_BT_MESH_SCATTERNET)))
#error "Only one of CONFIG_BT_MESH_CENTRAL, CONFIG_BT_MESH_PERIPHERAL and CONFIG_BT_MESH_SCATTERNET can be enabled"
#endif

#else
/*Zephyr STACK MACROs************************************************************/
#define CONFIG_BT                    1
#if CONFIG_BT
	#define CONFIG_FTL_ENABLED
	#define CONFIG_BT_CONFIG         0
	#define CONFIG_BT_PERIPHERAL     1
	#define CONFIG_BT_CENTRAL        1
	/* TBD: Macros about samples */
#endif

#endif

#endif // __PLATFORM_OPTS_BT_H__

