/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <unistd.h>
#include <assert.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "multi_heap.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_bt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

#define FAIL() do { printf("FAILURE\n"); return; } while(0)

extern uint8_t _bt_bss_start;
extern uint8_t _bt_bss_end;
extern uint8_t _bt_controller_bss_start;
extern uint8_t _bt_controller_bss_end;

extern void ble_store_config_init(void);

static const char *tag = "MEM_RELEASE_APP";

static void
bleprph_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    uint8_t own_addr_type = 0;
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, NULL, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

static void nimble_host_on_reset(int reason)
{
    ESP_LOGI(tag, "Resetting state; reason=%d", reason);
}

static void nimble_host_on_sync(void)
{
    ESP_LOGI(tag, "NimBLE host synchronized");
    bleprph_advertise();
}

static void nimble_host_task_fn(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

static void bt_stack_init(void)
{
    esp_err_t ret = nimble_port_init();
    ESP_ERROR_CHECK(ret);

    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = nimble_host_on_reset;
    ble_hs_cfg.sync_cb = nimble_host_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Set the default device name. */
    int rc = ble_svc_gap_device_name_set(tag);
    assert(rc == 0);

    /* XXX Need to have template for store */
    ble_store_config_init();

    nimble_port_freertos_init(nimble_host_task_fn);
}

static void bt_stack_deinit(void)
{
    int rc = nimble_port_stop();
    assert(rc == 0);

    nimble_port_deinit();
    ESP_LOGI(tag, "BLE Host Task Stopped");
}

/* Initialize soft AP */
void wifi_init_softap(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    esp_netif_create_default_wifi_ap();

    wifi_config_t wifi_ap_config = {
        .ap = {
            .ssid = "test_ap",
            .ssid_len = 7,
            .channel = 10,
            .password = "test_pass",
            .max_connection = 5,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));

    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(void)
{
    esp_err_t ret = ESP_OK;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Initialize NVS — it is used to store PHY calibration data */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (1) {
        wifi_init_softap();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    /* initialize and then deinitialize bluetooth stack */
    bt_stack_init();

    vTaskDelay(pdMS_TO_TICKS(5000));

    bt_stack_deinit();

    vTaskDelay(pdMS_TO_TICKS(5000));

    bt_stack_init();

#if 0
    /* Get the size of heap located in external RAM */
    const uint32_t free_before = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    ESP_LOGI(tag, "Free size in external RAM heap: %"PRIu32, free_before);

    /* Make sure at least one of the Bluetooth BSS section that can be used as a heap */
    const uint32_t heap_size = sizeof(multi_heap_info_t);
    const uint32_t bt_bss_size = &_bt_bss_end - &_bt_bss_start;
    const uint32_t bt_ctrl_bss_size = &_bt_controller_bss_end - &_bt_controller_bss_start;

    ESP_LOGI(tag, "bt_bss_size %"PRIu32", bt_ctrl_bss_size %"PRIu32, bt_bss_size, bt_ctrl_bss_size);
    if (bt_bss_size < heap_size && bt_ctrl_bss_size < heap_size)
    {
        ESP_LOGW(tag, "Bluetooth BSS sections are too small!");
        FAIL();
    }

    /* Release the BSS sections to use them as heap */
    ret = esp_bt_mem_release(ESP_BT_MODE_BTDM);
    ESP_ERROR_CHECK(ret);

    /* Check that we have more available memory in the external RAM heap */
    const uint32_t free_after = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    ESP_LOGI(tag, "Free size in external RAM after releasing: %"PRIu32, free_after);
    if (free_after <= free_before) {
        FAIL();
    }
    ESP_LOGI(tag, "Free heap size increased by %"PRIu32" bytes", free_after - free_before);

    if (heap_caps_check_integrity_all(true)) {
        ESP_LOGI(tag, "Comprehensive heap check: SUCCESS");
    }
#endif
}
