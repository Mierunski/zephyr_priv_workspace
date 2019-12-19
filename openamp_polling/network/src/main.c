/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>

#include <openamp/open_amp.h>
#include <metal/sys.h>
#include <metal/device.h>
#include <metal/alloc.h>

#include "../../common.h"

#define PORT0  DT_GPIO_LEDS_LED_0_GPIOS_CONTROLLER
#define PORT1  DT_GPIO_LEDS_LED_1_GPIOS_CONTROLLER
#define PORT2  DT_GPIO_LEDS_LED_2_GPIOS_CONTROLLER
#define PORT3  DT_GPIO_LEDS_LED_3_GPIOS_CONTROLLER

#define LED0 DT_GPIO_LEDS_LED_0_GPIOS_PIN
#define LED1 DT_GPIO_LEDS_LED_1_GPIOS_PIN
#define LED2 DT_GPIO_LEDS_LED_2_GPIOS_PIN
#define LED3 DT_GPIO_LEDS_LED_3_GPIOS_PIN

// LEDs are 16,17,18,19
// UART is  20,21,22,23

#define SLEEP_TIME	500

// Testing OpenAMP
static metal_phys_addr_t shm_physmap[] = { SHM_START_ADDR };
static struct metal_device shm_device = {
    .name = SHM_DEVICE_NAME,
    .bus = NULL,
    .num_regions = 1,
    {
        {
            .virt       = (void *) SHM_START_ADDR,
            .physmap    = shm_physmap,
            .size       = SHM_SIZE,
            .page_shift = 0xffffffff,
            .page_mask  = 0xffffffff,
            .mem_flags  = 0,
            .ops        = { NULL },
        },
    },
    .node = { NULL },
    .irq_num = 0,
    .irq_info = NULL
};

static volatile unsigned int received_data;

static struct virtio_vring_info rvrings[2] = {
    [0] = {
        .info.align = VRING_ALIGNMENT,
    },
    [1] = {
        .info.align = VRING_ALIGNMENT,
    },
};
static struct virtio_device vdev;
static struct rpmsg_virtio_device rvdev;
static struct metal_io_region *io;
static struct virtqueue *vq[2];

static unsigned char virtio_get_status(struct virtio_device *vdev)
{
    return sys_read8(VDEV_STATUS_ADDR);
}

static u32_t virtio_get_features(struct virtio_device *vdev)
{
    return 1 << VIRTIO_RPMSG_F_NS;
}

static void virtio_notify(struct virtqueue *vq)
{
	/* We don't use interrupts, so no notification to second core */
}

struct virtio_dispatch dispatch = {
    .get_status = virtio_get_status,
    .get_features = virtio_get_features,
    .notify = virtio_notify,
};

volatile bool received = false;
int endpoint_cb(struct rpmsg_endpoint *ept, void *data,
        size_t len, u32_t src, void *priv)
{
    received_data = *((unsigned int *) data);
    received = true;

    return RPMSG_SUCCESS;
}

struct rpmsg_endpoint my_ept;
struct rpmsg_endpoint *ep = &my_ept;

static void rpmsg_service_unbind(struct rpmsg_endpoint *ept)
{
    (void)ept;
    rpmsg_destroy_ept(ep);
}

static unsigned int receive_message(void)
{

	while(!received) {
		virtqueue_notification(vq[1]);
	}
	received = false;
	return received_data;
}

static int send_message(unsigned int message)
{
    return rpmsg_send(ep, &message, sizeof(message));
}


void main(void)
{    
    int cnt = 0;
    struct device *gpio0, *gpio1, *gpio2, *gpio3;

    // Configure LEDs; LED2 and LED3 are Connected to logic analyzer
    gpio0 = device_get_binding(PORT0);
    gpio1 = device_get_binding(PORT1);
    gpio2 = device_get_binding(PORT2);
    gpio3 = device_get_binding(PORT3);

    gpio_pin_configure(gpio0, LED0, GPIO_DIR_OUT);
    gpio_pin_configure(gpio1, LED1, GPIO_DIR_OUT);
    gpio_pin_configure(gpio2, LED2, GPIO_DIR_OUT);
    gpio_pin_configure(gpio3, LED3, GPIO_DIR_OUT);

    gpio_pin_write(gpio0, LED0, 1);
    gpio_pin_write(gpio1, LED1, 1);
    gpio_pin_write(gpio2, LED2, 1);
    gpio_pin_write(gpio3, LED3, 1);


    // Testing OpenAMP
    int status = 0;
    unsigned int message = 0;
    struct metal_device *device;
    struct rpmsg_device *rdev;
    struct metal_init_params metal_params = METAL_INIT_DEFAULTS;

    status = metal_init(&metal_params);
    if (status != 0) {
        printk("metal_init: failed - error code %d\n", status);
        return;
    }

    status = metal_register_generic_device(&shm_device);
    if (status != 0) {
        printk("Couldn't register shared memory device: %d\n", status);
        return;
    }

    status = metal_device_open("generic", SHM_DEVICE_NAME, &device);
    if (status != 0) {
        printk("metal_device_open failed: %d\n", status);
        return;
    }

    io = metal_device_io_region(device, 0);
    if (io == NULL) {
        printk("metal_device_io_region failed to get region\n");
        return;
    }

    /* setup vdev */
    vq[0] = virtqueue_allocate(VRING_SIZE);
    if (vq[0] == NULL) {
        printk("virtqueue_allocate failed to alloc vq[0]\n");
        return;
    }
    vq[1] = virtqueue_allocate(VRING_SIZE);
    if (vq[1] == NULL) {
        printk("virtqueue_allocate failed to alloc vq[1]\n");
        return;
    }

    vdev.role = RPMSG_REMOTE;
    vdev.vrings_num = VRING_COUNT;
    vdev.func = &dispatch;
    rvrings[0].io = io;
    rvrings[0].info.vaddr = (void *)VRING_TX_ADDRESS;
    rvrings[0].info.num_descs = VRING_SIZE;
    rvrings[0].info.align = VRING_ALIGNMENT;
    rvrings[0].vq = vq[0];

    rvrings[1].io = io;
    rvrings[1].info.vaddr = (void *)VRING_RX_ADDRESS;
    rvrings[1].info.num_descs = VRING_SIZE;
    rvrings[1].info.align = VRING_ALIGNMENT;
    rvrings[1].vq = vq[1];

    vdev.vrings_info = &rvrings[0];

    /* setup rvdev */
    status = rpmsg_init_vdev(&rvdev, &vdev, NULL, io, NULL);
    if (status != 0) {
        printk("rpmsg_init_vdev failed %d\n", status);
        return;
    }

    rdev = rpmsg_virtio_get_rpmsg_device(&rvdev);

    status = rpmsg_create_ept(ep, rdev, "k", RPMSG_ADDR_ANY,
            RPMSG_ADDR_ANY, endpoint_cb, rpmsg_service_unbind);
    if (status != 0) {
        printk("rpmsg_create_ept failed %d\n", status);
        return;
    }

    printk("Network Core loop and receive, then send message...\n");
    while (message < MSG_ITERATIONS - 1) {
        message = receive_message();

        message++;
        status = send_message(message);
        if (status < 0) {
            printk("send_message(%d) failed with status %d\n",
                   message, status);
            goto _cleanup;
        }
    }


_cleanup:
    rpmsg_deinit_vdev(&rvdev);
    metal_finish();

    printk("OpenAMP demo on Network Core ended.\n");
    
    while(1) {
        gpio_pin_write(gpio0, LED2, cnt % 2);
        gpio_pin_write(gpio1, LED3, (cnt + 1) % 2);
        k_sleep(SLEEP_TIME);
        cnt++;
    }
}
