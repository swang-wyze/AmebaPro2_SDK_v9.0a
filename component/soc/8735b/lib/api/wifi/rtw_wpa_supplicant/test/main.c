#include <FreeRTOS.h>
#include <task.h>
#include <platform_stdlib.h>
#include <utils/os.h>

static void wps_os_malloc_free_test(void)
{
	unsigned char *ptr1 = NULL, *ptr2 = NULL;

	printf("\r\nheap size=%d\r\n", xPortGetFreeHeapSize());
	ptr1 = os_malloc(1024);

	if (ptr1) {
		printf("\r\nheap size=%d after os_malloc(1024)=%p\r\n", xPortGetFreeHeapSize(), ptr1);
		os_free(ptr1);
		printf("\r\nheap size=%d after os_free(%p)\r\n", xPortGetFreeHeapSize(), ptr1);
	} else {
		printf("\r\nos_malloc(1024) FAILED\r\n");
	}

	ptr1 = os_zalloc(1024);

	if (ptr1) {
		printf("\r\nheap size=%d after os_zalloc(1024)=%p\r\n", xPortGetFreeHeapSize(), ptr1);
		ptr2 = os_realloc(ptr1, 1024, 2048);

		if (ptr2) {
			printf("\r\nheap size=%d after os_realloc(%p, 1024, 2048)=0x%p\r\n", xPortGetFreeHeapSize(), ptr1, ptr2);
			os_free(ptr2);
			printf("\r\nheap size=%d after os_free(%p)\r\n", xPortGetFreeHeapSize(), ptr2);
		} else {
			printf("\r\nos_realloc(%p, 1024, 2048) FAILED\r\n", ptr1);
		}
	} else {
		printf("\r\nos_malloc(1024) FAILED\r\n");
	}
}

void test_thread(void *param)
{
	init_rom_wlan_ram_map();
	wps_crypto_test(0);
	wps_os_malloc_free_test();

	vTaskDelete(NULL);
}

void main(void)
{
	if (xTaskCreate(test_thread, ((const char *)"test_thread"), 2048, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(test_thread) failed", __FUNCTION__);
	}

	vTaskStartScheduler();
}

