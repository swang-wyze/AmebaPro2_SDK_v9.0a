#include "FreeRTOS.h"
#include "task.h"
#include "diag.h"
#include "hal.h"
#include <example_entry.h>
#include "video_example_media_framework.h"

hal_gpio_adapter_t sensor_en_gpio;
void __libc_init_arrayx(void)
{
	// TODO: find why GCC remove this function from compiler build-in library
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
	console_init();

	// Enable ISP PWR
	hal_gpio_init(&sensor_en_gpio, PIN_A5);
	hal_gpio_set_dir(&sensor_en_gpio, GPIO_OUT);
	hal_gpio_write(&sensor_en_gpio, 1);

	/* pre-processor of application example */
	pre_example_entry();

	wlan_network();

	/* Execute application example */
	example_entry();

	/* Execute video example */
	video_example_media_framework();

	vTaskStartScheduler();
	while (1);
}
