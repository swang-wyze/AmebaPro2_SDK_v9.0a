/**
*****************************************************************************************
*     Copyright(c) 2020, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     low_power.c
  * @brief    Source file of power saving function specific for GEL.
  * @details  User command interfaces.
  * @author   sherman_sun
  * @date     2020_11_06
  * @version  v1.0
  * *************************************************************************************
  */
#include "low_power.h"

#define APP_TASK_PRIORITY             1         //!< Task priorities
#define APP_TASK_STACK_SIZE           256 * 10  //!< Task stack size

void *bt_mesh_ps_task_handle;
extern void rltk_coex_ps_leave(void);
extern void rltk_coex_ps_enter(void);

enum ps_state
{
    ps_state_disable,
    ps_state_enable
};
uint8_t ps_state = ps_state_disable;

void bt_mesh_ps_task(void *p_param)
{
    /* avoid gcc compile warning */
    (void)p_param;
    uint8_t state = DEVICE_IDLE_CHECK_FALSE;

    while (true) {
        state = bt_mesh_idle_check();
        if (state == DEVICE_IDLE_CHECK_TRUE && ps_state == ps_state_disable) {
            rltk_coex_ps_enter();
            ps_state = ps_state_enable;
            printf("\r\n Enter powersaving mode");
        } else if (state == DEVICE_IDLE_CHECK_FALSE && ps_state == ps_state_enable) {
            rltk_coex_ps_leave();
            ps_state = ps_state_disable;
            printf("\r\n Leave powersaving mode");
        }

        plt_delay_ms(200);
    }
}

void bt_mesh_power_saving_init(void)
{
    os_task_create(&bt_mesh_ps_task_handle, "bt_mesh_idle_check_app", bt_mesh_ps_task, 0, APP_TASK_STACK_SIZE,
                   APP_TASK_PRIORITY);
}

void bt_mesh_power_saving_deinit(void)
{
    if (bt_mesh_ps_task_handle) {
		os_task_delete(bt_mesh_ps_task_handle);
	}
    //leave ps
    rltk_coex_ps_leave();
    bt_mesh_ps_task_handle = NULL;
}

