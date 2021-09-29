#include <FreeRTOS.h>
#include <task.h>
#include <platform_stdlib.h>

void test_thread(void *param)
{
	ed25519_test(0);
	sha512_hkdf_test(0);
	curve25519_test(0);
	vTaskDelete(NULL);
}

void main(void)
{
	if (xTaskCreate(test_thread, ((const char *)"test_thread"), 2048, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(test_thread) failed", __FUNCTION__);
	}

	vTaskStartScheduler();
}

