/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0 TODO
 */

#ifndef COMMON_H__
#define COMMON_H__

#define SHM_START_ADDR      (DT_IPC_SHM_BASE_ADDRESS  + 0x400)  // TODO why do they start here?
#define SHM_SIZE            0x7c00  // TODO: make sure size if below DT_MMIO_SRAM_SHM_SIZE
#define SHM_DEVICE_NAME     "sram0.shm"

#define VRING_COUNT         2
#define VRING_TX_ADDRESS    (SHM_START_ADDR + SHM_SIZE - 0x400)
#define VRING_RX_ADDRESS    (VRING_TX_ADDRESS - 0x400)
#define VRING_ALIGNMENT     4
#define VRING_SIZE          16

#define VDEV_STATUS_ADDR    DT_IPC_SHM_BASE_ADDRESS


#define IPC_TX_CHANNEL      0  // Channel used for application to notify network
#define IPC_RX_CHANNEL      1  // Channel used for network to notify application


/* LEDs for FPGA application is: 4,5,6,7 */
/* UART for FPGA application is: 0,1,2,3 */

/* LEDs for FPGA network is: 16,17,18,19 */
/* UART for FPGA network is: 20,21,22,23 */

#define NETWORK_LED0_GPIO       4
#define NETWORK_LED1_GPIO       5
#define NETWORK_LED2_GPIO       28
#define NETWORK_LED3_GPIO       29

#define NETWORK_UART_RX_GPIO    25
#define NETWORK_UART_TX_GPIO    26
#define NETWORK_UART_RTS_GPIO   27
#define NETWORK_UART_CTS_GPIO   7

#define MSG_ITERATIONS 1000

#endif
