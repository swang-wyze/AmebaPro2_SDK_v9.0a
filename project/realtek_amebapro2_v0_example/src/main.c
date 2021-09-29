#include "FreeRTOS.h"
#include "task.h"
#include "diag.h"
#include "hal.h"
#include <example_entry.h>
#include "log_service.h"

#if CONFIG_EXAMPLE_MEDIA_VIDEO
#include "hal_video.h"
#include "video_example_media_framework.h"

extern hal_video_adapter_t *v_adapter;
hal_gpio_adapter_t sensor_en_gpio;

#endif

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
	console_init();

#if CONFIG_EXAMPLE_MEDIA_VIDEO
	// Enable ISP PWR
	hal_gpio_init(&sensor_en_gpio, PIN_A5);
	hal_gpio_set_dir(&sensor_en_gpio, GPIO_OUT);
	hal_gpio_write(&sensor_en_gpio, 1);
#endif

	printf("pre_example_entry\r\n");
	/* pre-processor of application example */
	pre_example_entry();

	printf("wlan_network\r\n");
	wlan_network();

	printf("application example\r\n");
	/* Execute application example */
	example_entry();

#if CONFIG_EXAMPLE_MEDIA_VIDEO
	/* Execute video example */
	video_example_media_framework();
#endif

	vTaskStartScheduler();
	while (1);
}
