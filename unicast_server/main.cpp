/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

//#include "../src/modules/sd_card.h"

#include <zephyr/settings/settings.h>

#include "macros_common.h"
#include "openearable_common.h"
#include "streamctrl.h"

#include "../src/Battery/PowerManager.h"
#include "../src/SensorManager/SensorManager.h"
#include "../src/utils/StateIndicator.h"

#include "device_info.h"
#include "battery_service.h"
#include "button_service.h"
#include "sensor_service.h"
#include "led_service.h"

#include "SensorScheme.h"
#include "DefaultSensors.h"

#include "../src/SD_Card/SDLogger/SDLogger.h"

#include "uicr.h"

#include "streamctrl.h"

#include "bt_mgmt.h"

#include "bt_mgmt_conn_interval.h"
#include "conn_interval/conn_intvl_linear.h"

#include "../src/SPI_esp32/esp32_link.hpp"

#include <SEGGER_RTT.h>
#include "../src/bluetooth/gatt_services/spi_cmd_service.h"   // 路径按你放置位置调整
//#include "sd_card.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);
//BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
//	     "Console device is not ACM CDC UART device");

/* STEP 5.4 - Include header for USB */
#include <zephyr/usb/usb_device.h>


/* -------- ESP32 SPI periodic sender thread -------- */


/* ---------- ESP32 SPI worker thread (1Hz) ---------- */
static void esp32_spi_thread(void*, void*, void*)
{
    int init_ret = esp32_link_init();
    LOG_INF("esp32_link_init ret=%d", init_ret);

    // while (1) {
    //     uint8_t tx[4] = {0xAA, 0x55, 0x01, 0x02};
    //     uint8_t rx[4] = {0};

    //     int ret = esp32_link_xfer(tx, rx, sizeof(tx));

    //     /* 你要看的“证据”就在这里：ret=0 并且 rx 是否变为 10 20 30 40 */
    //     LOG_INF("ESP32 xfer ret=%d rx=%02X %02X %02X %02X",
    //             ret, rx[0], rx[1], rx[2], rx[3]);

    //     k_sleep(K_SECONDS(1));
    // }
}

K_THREAD_STACK_DEFINE(esp32_spi_stack, 1024);
static struct k_thread esp32_spi_thread_data;
/* -------------------------------------------------- */


/* -------------------------------------------------- */



int main(void) {
	int ret;

	LOG_DBG("nRF5340 APP core started");

	SEGGER_RTT_WriteString(0, "RTT RAW: booted\n");


	ret = power_manager.begin();
	ERR_CHK(ret);

	uint8_t standalone = uicr_standalone_get();

	LOG_INF("Standalone mode: %i", standalone);

	/*sdcard_manager.init();

	sdcard_manager.mount();*/

	/* STEP 5.5 - Enable USB */
	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		ret = usb_enable(NULL);
		if (ret) {
			LOG_ERR("Failed to enable USB");
			return 0;
		}
	}

	streamctrl_start();


	uint32_t sirk = uicr_sirk_get();

	if (sirk == 0xFFFFFFFFU) {
		state_indicator.set_pairing_state(SET_PAIRING);
	} else if (bonded_device_count > 0 && !oe_boot_state.timer_reset) {
		state_indicator.set_pairing_state(PAIRED);
	} else {
		state_indicator.set_pairing_state(BONDING);
	}

	init_sensor_manager();

	//sensor_config imu = {ID_IMU, 80, 0};
	//sensor_config imu = {ID_PPG, 400, 0};
	//sensor_config temp = {ID_OPTTEMP, 10, 0};
	// sensor_config temp = {ID_BONE_CONDUCTION, 100, 0};

	//config_sensor(&temp);

	//sensor_config ppg = {ID_PPG, 400, 0};
	//config_sensor(&ppg);

	ret = init_led_service();
	ERR_CHK(ret);

	ret = init_battery_service();
	ERR_CHK(ret);

	ret = init_button_service();
	ERR_CHK(ret);

	ret = initParseInfoService(&defaultSensorIds, defaultSensors);
	ERR_CHK(ret);

	ret = init_sensor_service();
	ERR_CHK(ret);

	ret = init_spi_cmd_service();
	ERR_CHK(ret);

	bt_mgmt_conn_interval_init(new ConnIntvlLinear(
	    4,                // linear increase step (8ms units)
	    CONFIG_BLE_ACL_CONN_INTERVAL,
	    CONFIG_BLE_ACL_CONN_INTERVAL_SLOW
	));
	
		/* --------- ESP32 SPI quick test (send A5 5A once at boot) --------- */
	/* --------- ESP32 SPI periodic sender (every 1s) --------- */
	k_thread_create(&esp32_spi_thread_data,
					esp32_spi_stack,
					K_THREAD_STACK_SIZEOF(esp32_spi_stack),
					esp32_spi_thread,
					nullptr, nullptr, nullptr,
					7, 0, K_NO_WAIT);
	k_thread_name_set(&esp32_spi_thread_data, "esp32_spi");

	/* -------------------------------------------------------- */

	/* ------------------------------------------------------------------ */

	// error test
	//long *a = nullptr;
	//*a = 10;

	return 0;
}
