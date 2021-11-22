#ifndef __SIMPLE_BLE_H__
#define __SIMPLE_BLE_H__

#include "nrf_log.h"

#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "ble_nus.h"

#include "nrf_power.h"
#include "nrf_bootloader_info.h"
#include "ble_dfu.h"
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"

#include "nrf_pwr_mgmt.h"

#include "bsp_btn_ble.h"
#include "app_timer.h"

void bsp_event_handler(bsp_event_t event);
void simple_ble_init(void);
void advertising_start(void);

ret_code_t ble_data_send(uint8_t* sendbuf, uint16_t llength);

#endif
