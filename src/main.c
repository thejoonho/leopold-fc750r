/**
 * Copyright (c) 2025 Joonho Jang
 * Copyright (c) 2025, 2024, 2018 Nordic Semiconductor ASA
 * Copyright (c) 2024 BayLibre SAS
 * Copyright (c) 2018 Laczen
 * Copyright (c) 2018 Intel Corporation
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <bluetooth/services/hids.h>
#include <errno.h>
#include <sample_usbd.h>
#include <soc.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/dis.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/mfd/npm1300.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm1300_charger.h>
#include <zephyr/fs/zms.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_hid.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/spinlock.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/usb/usbd.h>

#include "fuel_gauge.h"

////////////////////////////////////////////////////////////////////////
//                             FULL SYSTEM                            //
////////////////////////////////////////////////////////////////////////

#define WIRELESS_MODE 1
#define WIRED_MODE 0

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static atomic_t nRF_mode = WIRED_MODE;

////////////////////////////////////////////////////////////////////////
//                            Nordic ASCII ART                        //
////////////////////////////////////////////////////////////////////////

static const char img_data[] = {
#include "logo.file"
};

/** @brief print_logo
 *
 * @note Prints the Nordic Semiconductor Logo to pay my respect to the Nordic
 * Semiconductor engineers for their incredible engineering. ALSO, the terminal
 * has never looked so BEAUTIFUL!
 */
static void print_logo(void);

////////////////////////////////////////////////////////////////////////
//                           POWER MANAGEMENT                         //
////////////////////////////////////////////////////////////////////////

#define BUCK_BASE 0x04U  // Base address
#define BUCK1NORMVOUT_OFFSET 0x08U
#define BUCKSWCTRLSEL_OFFSET 0x0FU

#define GPIO_BASE 0x06U  // Base address
#define GPIOMODE0_OFFSET 0x00U
#define GPIODEN0_OFFSET 0x0FU

// fg = fuel gauge
static atomic_t fg_wait = false;
static struct k_work_delayable fg_update;

static const struct device *pmic = DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_pmic));
static const struct device *pmic_gpio =
    DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_gpio));
static const struct device *pmic_buck1 =
    DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_buck1));
static const struct device *pmic_buck2 =
    DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_buck2));
static const struct device *charger =
    DEVICE_DT_GET(DT_NODELABEL(npm1300_ek_charger));

static volatile bool vbus_connected;

/** @brief fg_update_handler
 *
 * @note When the keyboard is in WIRELESS_MODE, it'll constantly update the
 * central with the battery percentage every minute.
 */
static void fg_update_handler(struct k_work *work);

/** @brief pmic_custom_settings
 *
 * @note When the nPM1300 resets itself (e.g. VBAT is disconnected and
 * reconnected), it uses the factory default configurations. This function
 * changes those configurations shown in ./nPM_PowerUp_Config
 */
static int pmic_custom_settings(void);
static void event_callback(const struct device *dev, struct gpio_callback *cb,
                           uint32_t pins);

////////////////////////////////////////////////////////////////////////
//                        SETTINGS FLASH STORAGE                      //
////////////////////////////////////////////////////////////////////////

#define FN_MODE 1
#define MULTIMEDIA_MODE 0

#define RGB_OFF 0
#define RGB_MODE1 1
#define RGB_MODE2 2
#define RGB_MODE3 3
#define RGB_MODE4 4
#define RGB_MODE5 5
#define RGB_MODE6 6

/**
 * @note ZMS Structure
 * leopold_fc750r
 *   |_ current_central
 *   |_ central1_addr [1]
 *   |_ central2_addr [2]
 *   |_ central3_addr [3]
 *   |_ fn_mode
 *   |_ rgb_mode
 */

static int current_central = 1;
static bt_addr_le_t central1_addr = {0};
static bt_addr_le_t central2_addr = {0};
static bt_addr_le_t central3_addr = {0};

static const bt_addr_le_t clean_addr = {0};

static bool fn_mode = FN_MODE;
static int rgb_mode = RGB_OFF;

/** @brief leopold_fc750R_handle_set
 *
 * @note Initializes the current_central, central_addr, central2_addr, and
 * central3_addr with the stored value.
 *
 * @note Intializes the fn_mode and rgb_mode as well.
 */
int leopold_fc750R_handle_set(const char *name, size_t len,
                              settings_read_cb read_cb, void *cb_arg);

SETTINGS_STATIC_HANDLER_DEFINE(leopold_fc750R, "leopold_fc750R", NULL,
                               leopold_fc750R_handle_set, NULL, NULL);

////////////////////////////////////////////////////////////////////////
//                      KEYBOARD MATRIX SCANNING                      //
////////////////////////////////////////////////////////////////////////

static bool fn_pressed = false;

struct kb_event {
  uint16_t code;
  int32_t value;
};

K_MSGQ_DEFINE(kb_msgq, sizeof(struct kb_event), 2, 1);  // 2 or 4?

/** @brief key_pressed
 *
 * @note Calls the appropriate functions based on the user's key press
 */
static void key_pressed(struct input_event *evt, void *user_data);
INPUT_CALLBACK_DEFINE(NULL, key_pressed, NULL);

/** @brief central_id_ctrl
 *
 * @note Ensures that the current_central, central1_addr, central2_addr, and
 * central3_addr are properly updated in the ZMS flash storage.
 */
static void central_id_ctrl(int32_t pressed, int central_id);

////////////////////////////////////////////////////////////////////////
//                             BLE SETTINGS                           //
////////////////////////////////////////////////////////////////////////

#define PAIRING_MODE 1
#define CONNECTING_MODE 0

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define ADV_INT_20MS BT_GAP_MS_TO_ADV_INTERVAL(20)

#define BASE_USB_HID_SPEC_VERSION 0x0101

#define OUTPUT_REPORT_MAX_LEN 1
#define OUTPUT_REPORT_BIT_MASK_CAPS_LOCK 0x02
#define INPUT_REP_KEYS_REF_ID 0
#define OUTPUT_REP_KEYS_REF_ID 0
#define SCAN_CODE_POS 2
#define KEYS_MAX_LEN (INPUT_REPORT_KEYS_MAX_LEN - SCAN_CODE_POS)

#define HIDS_QUEUE_SIZE 10

/**
 * @note The configuration below is the same as BOOT mode configuration
 * This simplifies the code as the BOOT mode is the same as REPORT mode.
 * Changing this configuration would require separate implementation of
 * BOOT mode report generation.
 */
#define KEY_CTRL_CODE_MIN 224  // Control key codes - required 8 of them
#define KEY_CTRL_CODE_MAX 231  // Control key codes - required 8 of them
#define KEY_CODE_MIN 0         // Normal key codes
#define KEY_CODE_MAX 101       // Normal key codes
#define KEY_PRESS_MAX \
  6  // Max number of non-control keys pressed simultaneously

/**
 * @note Number of bytes in key report
 * 1B - control keys
 * 1B - reserved
 * rest - non-control keys
 */
#define INPUT_REPORT_KEYS_MAX_LEN (1 + 1 + KEY_PRESS_MAX)

/**
 * @note Current report map construction requires exactly 8 buttons
 */
BUILD_ASSERT((KEY_CTRL_CODE_MAX - KEY_CTRL_CODE_MIN) + 1 == 8);

/**
 * @note OUT report internal indexes. This is a position in internal report
 * table and is not related to report ID.
 */
enum { OUTPUT_REP_KEYS_IDX = 0 };
/**
 * @note INPUT report internal indexes. This is a position in internal report
 * table and is not related to report ID.
 */
enum { INPUT_REP_KEYS_IDX = 0 };

BT_HIDS_DEF(hids_obj, OUTPUT_REPORT_MAX_LEN, INPUT_REPORT_KEYS_MAX_LEN);

/**
 * @note We cannot change the peripheral's core identity address which is saved
 * in identity_addr[0]. This will not be used during advertisement as it CANNOT
 * be modified.
 */
static bt_addr_le_t identity_addr[CONFIG_BT_ID_MAX] = {0};
static int identity_count = CONFIG_BT_ID_MAX;

static atomic_t ble_adv_mode = CONNECTING_MODE;
static atomic_t adv_safe = true;

static struct conn_mode {
  struct bt_conn *conn;
  bool in_boot_mode;
} conn_mode[CONFIG_BT_HIDS_MAX_CLIENT_COUNT];

static struct keyboard_state {
  uint8_t ctrl_keys_state;
  uint8_t keys_state[KEY_PRESS_MAX];
} hid_keyboard_state;

static int64_t pairing_key_start_time = 0;
static atomic_t enable_passkey_input = false;
static atomic_t idx_passkey_input = 0;
static volatile unsigned int passkey_input[6] = {0};
static struct bt_conn *passkey_conn = NULL;

static struct k_work adv_work;
static struct k_work_delayable pm_cancel;
static struct k_work_delayable cm_cancel;

/**
 * @note Pairing advertising packets: Local name, Service Flags, Service UUID
 */
static const struct bt_data ad_pm[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

/**
 * @note Pairing scanning packets: Appearance
 */
static const struct bt_data sd_pm[] = {
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
                  (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
                  (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
};

/**
 * @note Pairing advertisement parameters: Undirected,
 * connectable, scannable, use identity address, and 100~150ms interval
 */

static const struct bt_le_adv_param adv_param_pm1 = {
    .id = 1,  // Use identity index 1
    .options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY,
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    .peer = NULL,
};

static const struct bt_le_adv_param adv_param_pm2 = {
    .id = 2,  // Use identity index 2
    .options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY,
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    .peer = NULL,
};

static const struct bt_le_adv_param adv_param_pm3 = {
    .id = 3,  // Use identity index 3
    .options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY,
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    .peer = NULL,
};

/**
 * @note Connecting advertising packets: Service Flags.
 */
static const struct bt_data ad_cm[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
};

/**
 * @note Connecting scanning packets: Local name
 */
static const struct bt_data sd_cm[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/**
 * @note Connection Mode Parameters: Undirected, connectable,
 * scannable, filter accept for scanning packets and connection, and 20ms
 * interval (20ms to follow Apple BLE Accessory Guidelines)
 */

static const struct bt_le_adv_param adv_param_cm1 = {
    .id = 1,  // Use identity index 1
    .options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_FILTER_CONN |
               BT_LE_ADV_OPT_FILTER_SCAN_REQ,
    .interval_min = ADV_INT_20MS,
    .interval_max = ADV_INT_20MS,
    .peer = NULL,
};

static const struct bt_le_adv_param adv_param_cm2 = {
    .id = 2,  // Use identity index 2
    .options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_FILTER_CONN |
               BT_LE_ADV_OPT_FILTER_SCAN_REQ,
    .interval_min = ADV_INT_20MS,
    .interval_max = ADV_INT_20MS,
    .peer = NULL,
};

static const struct bt_le_adv_param adv_param_cm3 = {
    .id = 3,  // Use identity index 3
    .options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_FILTER_CONN |
               BT_LE_ADV_OPT_FILTER_SCAN_REQ,
    .interval_min = ADV_INT_20MS,
    .interval_max = ADV_INT_20MS,
    .peer = NULL,
};

static void pm_cancel_handler(struct k_work *work);
static void cm_cancel_handler(struct k_work *work);
static void adv_work_handler(struct k_work *work);

/** @brief advertising_start
 *
 * @note The advertisement process can either be for pairing/connection. The
 * function is called either from main (during startup), recycled_cb (for safe
 * advertising), key_pressed (wake-up advertising), or central_id_ctrl.
 *
 * @note If Pairing fails, it'll not reattempt the pairing process.
 * It'll stop advertising after 10min.
 *
 * @note If Connection fails, it'll not reattempt the pairing process.
 * It'll stop advertising after 1min.
 */
static void advertising_start(void);
static void connected_cb(struct bt_conn *conn, uint8_t err);
static void disconnected_cb(struct bt_conn *conn, uint8_t reason);
static void recycled_cb(void);
static void security_changed_cb(struct bt_conn *conn, bt_security_t level,
                                enum bt_security_err err);
static void identity_resolved_cb(struct bt_conn *conn, const bt_addr_le_t *rpa,
                                 const bt_addr_le_t *identity);
struct bt_conn_cb connection_callbacks = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
    .recycled = recycled_cb,
    .security_changed = security_changed_cb,
    .identity_resolved = identity_resolved_cb,
};

void process_passkey_input(struct kb_event kb_evt);
static void auth_passkey_entry(struct bt_conn *conn);
static void auth_cancel(struct bt_conn *conn);
static struct bt_conn_auth_cb conn_auth_callbacks = {
    .passkey_entry = auth_passkey_entry,
    .cancel = auth_cancel,
};

static void pairing_complete(struct bt_conn *conn, bool bonded);
static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason);
static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    .pairing_complete = pairing_complete,
    .pairing_failed = pairing_failed,
};

static void caps_lock_handler(const struct bt_hids_rep *rep);
static void hids_outp_rep_handler(struct bt_hids_rep *rep, struct bt_conn *conn,
                                  bool write);
static void ble_hid_init(void);

/** @brief Function process keyboard state and sends it
 *
 *  @param pstate     The state to be sent
 *  @param boot_mode  Information if boot mode protocol is selected.
 *  @param conn       Connection handler
 *
 *  @return 0 on success or negative error code.
 */
static int key_report_con_send(const struct keyboard_state *state,
                               bool boot_mode, struct bt_conn *conn);

/** @brief Function process and send keyboard state to all active connections
 *
 * Function process global keyboard state and send it to all connected
 * clients.
 *
 * @return 0 on success or negative error code.
 */
static int key_report_send(void);

/** @brief Change key code to ctrl code mask
 *
 *  Function changes the key code to the mask in the control code
 *  field inside the report.
 *  Returns 0 if key code is not a control key.
 *
 *  @param key Key code
 *
 *  @return Mask of the control key or 0.
 */
static int hid_kbd_state_key_set(uint8_t key, bool mod_key);
static int hid_kbd_state_key_clear(uint8_t key, bool mod_key);

////////////////////////////////////////////////////////////////////////
//                             USB SETTINGS                           //
////////////////////////////////////////////////////////////////////////

#define DIP2_NODE DT_NODELABEL(dipsw2)
static const struct gpio_dt_spec dip2 = GPIO_DT_SPEC_GET(DIP2_NODE, gpios);

static bool usb_hid_ready = false;
static const uint8_t hid_report_desc[] = HID_KEYBOARD_REPORT_DESC();

enum kb_report_idx {
  KB_MOD_KEY = 0,   // Modifier key
  KB_RESERVED,      // Reserved
  KB_KEY_CODE1,     // Key Code
  KB_KEY_CODE2,     // Key Code
  KB_KEY_CODE3,     // Key Code
  KB_KEY_CODE4,     // Key Code
  KB_KEY_CODE5,     // Key Code
  KB_KEY_CODE6,     // Key Code
  KB_REPORT_COUNT,  // 8
};

// HID input report storage & module state
UDC_STATIC_BUF_DEFINE(report, KB_REPORT_COUNT);  // Submitting input reports
static uint32_t kb_duration;
static bool kb_ready;  // Lets the system know if it's ok to send

// Updates if the USB interaction is ready or not
static void kb_iface_ready(const struct device *dev, const bool ready);
static int kb_get_report(const struct device *dev, const uint8_t type,
                         const uint8_t id, const uint16_t len,
                         uint8_t *const buf);
static int kb_set_report(const struct device *dev, const uint8_t type,
                         const uint8_t id, const uint16_t len,
                         const uint8_t *const buf);
/* Idle duration is stored but not used to calculate idle reports. */
static void kb_set_idle(const struct device *dev, const uint8_t id,
                        const uint32_t duration);
static uint32_t kb_get_idle(const struct device *dev, const uint8_t id);
static void kb_set_protocol(const struct device *dev, const uint8_t proto);
static void kb_output_report(const struct device *dev, const uint16_t len,
                             const uint8_t *const buf);
struct hid_device_ops kb_ops = {
    .iface_ready = kb_iface_ready,
    .get_report = kb_get_report,
    .set_report = kb_set_report,
    .set_idle = kb_set_idle,
    .get_idle = kb_get_idle,
    .set_protocol = kb_set_protocol,
    .output_report = kb_output_report,
};

/* doc device msg-cb start */
static void msg_cb(struct usbd_context *const usbd_ctx,
                   const struct usbd_msg *const msg);
static int usb_hid_report_set(uint8_t key, bool mod_key);
static int usb_hid_report_clear(uint8_t key, bool mod_key);

////////////////////////////////////////////////////////////////////////
//                              RGB LEDs                              //
////////////////////////////////////////////////////////////////////////

#define STRIP_NODE DT_ALIAS(led_strip)

#if DT_NODE_HAS_PROP(DT_ALIAS(led_strip), chain_length)
#define STRIP_NUM_PIXELS DT_PROP(DT_ALIAS(led_strip), chain_length)
#else
#error Unable to determine length of LED strip
#endif

#define DELAY_TIME K_MSEC(CONFIG_SAMPLE_LED_UPDATE_DELAY)

#define RGB(_r, _g, _b) {.r = (_r), .g = (_g), .b = (_b)}

/**
 * @note The RGB values has be a sum of 15 or less for the RGB to update
 * correctly.
 */
static const struct led_rgb colors[] = {
    RGB(0x00, 0x00, 0x00), /* off */
    RGB(0x03, 0x03, 0x03), /* white */
    RGB(0x00, 0x07, 0x08), /* torquoise */

    // Rainbow
    RGB(0x00, 0x07, 0x08), /* torquoise */
    RGB(0x02, 0x04, 0x09), /* royal blue */
    RGB(0x01, 0x02, 0x0c), /* dark blue */
    RGB(0x02, 0x00, 0x0d), /* dark dark blue */
    RGB(0x04, 0x00, 0x0b), /* dark purple */
    RGB(0x04, 0x01, 0x0a), /* purple */
    RGB(0x05, 0x01, 0x09), /* purple - pink */
    RGB(0x07, 0x00, 0x08), /* pink */
    RGB(0x09, 0x00, 0x06), /* bright pink */
    RGB(0x0b, 0x00, 0x04), /* pink - red  */
    RGB(0x0d, 0x00, 0x02), /* light red  */
    RGB(0x0e, 0x00, 0x01), /* red  */
    RGB(0x0f, 0x00, 0x00), /* dark red  */
    RGB(0x0e, 0x01, 0x00), /* tangerine  */
    RGB(0x0c, 0x03, 0x00), /* organge  */
    RGB(0x0a, 0x05, 0x00), /* organge - yellow  */
    RGB(0x06, 0x09, 0x00), /* yellow - green */
    RGB(0x01, 0x0e, 0x00), /*  green  */

    // Christmas
    RGB(0x05, 0x05, 0x05), /* white */
    RGB(0x0f, 0x00, 0x00), /* red */
    RGB(0x08, 0x0f, 0x00), /* green */
};

static struct led_rgb pixels[STRIP_NUM_PIXELS];
static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

static void update_rgb_rainbow(void) {
  int rc;
  size_t cursor = 0;

  while (cursor < 87) {
    memcpy(&pixels[cursor], &colors[0], sizeof(struct led_rgb));
    cursor++;
  }

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[72 - 1], &colors[3], sizeof(struct led_rgb));
  memcpy(&pixels[71 - 1], &colors[3], sizeof(struct led_rgb));
  memcpy(&pixels[38 - 1], &colors[3], sizeof(struct led_rgb));
  memcpy(&pixels[37 - 1], &colors[3], sizeof(struct led_rgb));
  memcpy(&pixels[12 - 1], &colors[3], sizeof(struct led_rgb));
  memcpy(&pixels[11 - 1], &colors[3], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[70 - 1], &colors[4], sizeof(struct led_rgb));
  memcpy(&pixels[39 - 1], &colors[4], sizeof(struct led_rgb));
  memcpy(&pixels[36 - 1], &colors[4], sizeof(struct led_rgb));
  memcpy(&pixels[37 - 1], &colors[4], sizeof(struct led_rgb));
  memcpy(&pixels[10 - 1], &colors[4], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[73 - 1], &colors[5], sizeof(struct led_rgb));
  memcpy(&pixels[35 - 1], &colors[5], sizeof(struct led_rgb));
  memcpy(&pixels[69 - 1], &colors[5], sizeof(struct led_rgb));
  memcpy(&pixels[40 - 1], &colors[5], sizeof(struct led_rgb));
  memcpy(&pixels[14 - 1], &colors[5], sizeof(struct led_rgb));
  memcpy(&pixels[13 - 1], &colors[5], sizeof(struct led_rgb));
  memcpy(&pixels[9 - 1], &colors[5], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[74 - 1], &colors[6], sizeof(struct led_rgb));
  memcpy(&pixels[75 - 1], &colors[6], sizeof(struct led_rgb));
  memcpy(&pixels[68 - 1], &colors[6], sizeof(struct led_rgb));
  memcpy(&pixels[41 - 1], &colors[6], sizeof(struct led_rgb));
  memcpy(&pixels[34 - 1], &colors[6], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[67 - 1], &colors[7], sizeof(struct led_rgb));
  memcpy(&pixels[15 - 1], &colors[7], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[76 - 1], &colors[8], sizeof(struct led_rgb));
  memcpy(&pixels[66 - 1], &colors[8], sizeof(struct led_rgb));
  memcpy(&pixels[42 - 1], &colors[8], sizeof(struct led_rgb));
  memcpy(&pixels[33 - 1], &colors[8], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[43 - 1], &colors[9], sizeof(struct led_rgb));
  memcpy(&pixels[32 - 1], &colors[9], sizeof(struct led_rgb));
  memcpy(&pixels[16 - 1], &colors[9], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[77 - 1], &colors[10], sizeof(struct led_rgb));
  memcpy(&pixels[65 - 1], &colors[10], sizeof(struct led_rgb));
  memcpy(&pixels[44 - 1], &colors[10], sizeof(struct led_rgb));
  memcpy(&pixels[31 - 1], &colors[10], sizeof(struct led_rgb));
  memcpy(&pixels[17 - 1], &colors[10], sizeof(struct led_rgb));
  memcpy(&pixels[8 - 1], &colors[10], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[78 - 1], &colors[11], sizeof(struct led_rgb));
  memcpy(&pixels[64 - 1], &colors[11], sizeof(struct led_rgb));
  memcpy(&pixels[45 - 1], &colors[11], sizeof(struct led_rgb));
  memcpy(&pixels[30 - 1], &colors[11], sizeof(struct led_rgb));
  memcpy(&pixels[18 - 1], &colors[11], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[79 - 1], &colors[12], sizeof(struct led_rgb));
  memcpy(&pixels[63 - 1], &colors[12], sizeof(struct led_rgb));
  memcpy(&pixels[46 - 1], &colors[12], sizeof(struct led_rgb));
  memcpy(&pixels[29 - 1], &colors[12], sizeof(struct led_rgb));
  memcpy(&pixels[19 - 1], &colors[12], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[80 - 1], &colors[13], sizeof(struct led_rgb));
  memcpy(&pixels[62 - 1], &colors[13], sizeof(struct led_rgb));
  memcpy(&pixels[47 - 1], &colors[13], sizeof(struct led_rgb));
  memcpy(&pixels[28 - 1], &colors[13], sizeof(struct led_rgb));
  memcpy(&pixels[20 - 1], &colors[13], sizeof(struct led_rgb));
  memcpy(&pixels[21 - 1], &colors[13], sizeof(struct led_rgb));
  memcpy(&pixels[7 - 1], &colors[13], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[61 - 1], &colors[14], sizeof(struct led_rgb));
  memcpy(&pixels[48 - 1], &colors[14], sizeof(struct led_rgb));
  memcpy(&pixels[27 - 1], &colors[14], sizeof(struct led_rgb));
  memcpy(&pixels[22 - 1], &colors[14], sizeof(struct led_rgb));
  memcpy(&pixels[6 - 1], &colors[14], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[81 - 1], &colors[15], sizeof(struct led_rgb));
  memcpy(&pixels[60 - 1], &colors[15], sizeof(struct led_rgb));
  memcpy(&pixels[49 - 1], &colors[15], sizeof(struct led_rgb));
  memcpy(&pixels[26 - 1], &colors[15], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[82 - 1], &colors[16], sizeof(struct led_rgb));
  memcpy(&pixels[59 - 1], &colors[16], sizeof(struct led_rgb));
  memcpy(&pixels[50 - 1], &colors[16], sizeof(struct led_rgb));
  memcpy(&pixels[5 - 1], &colors[16], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[83 - 1], &colors[17], sizeof(struct led_rgb));
  memcpy(&pixels[84 - 1], &colors[17], sizeof(struct led_rgb));
  memcpy(&pixels[58 - 1], &colors[17], sizeof(struct led_rgb));
  memcpy(&pixels[51 - 1], &colors[17], sizeof(struct led_rgb));
  memcpy(&pixels[25 - 1], &colors[17], sizeof(struct led_rgb));
  memcpy(&pixels[23 - 1], &colors[17], sizeof(struct led_rgb));
  memcpy(&pixels[4 - 1], &colors[17], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[85 - 1], &colors[18], sizeof(struct led_rgb));
  memcpy(&pixels[57 - 1], &colors[18], sizeof(struct led_rgb));
  memcpy(&pixels[52 - 1], &colors[18], sizeof(struct led_rgb));
  memcpy(&pixels[3 - 1], &colors[18], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[86 - 1], &colors[19], sizeof(struct led_rgb));
  memcpy(&pixels[56 - 1], &colors[19], sizeof(struct led_rgb));
  memcpy(&pixels[53 - 1], &colors[19], sizeof(struct led_rgb));
  memcpy(&pixels[24 - 1], &colors[19], sizeof(struct led_rgb));
  memcpy(&pixels[2 - 1], &colors[19], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[87 - 1], &colors[20], sizeof(struct led_rgb));
  memcpy(&pixels[55 - 1], &colors[20], sizeof(struct led_rgb));
  memcpy(&pixels[54 - 1], &colors[20], sizeof(struct led_rgb));
  memcpy(&pixels[1 - 1], &colors[20], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);
}
static void update_rgb_christmas(void) {
  int rc;
  size_t cursor = 0;

  while (cursor < 87) {
    memcpy(&pixels[cursor], &colors[0], sizeof(struct led_rgb));
    cursor++;
  }

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }

  k_sleep(DELAY_TIME);

  memcpy(&pixels[72 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[73 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[74 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[75 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[76 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[77 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[78 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[79 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[80 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[81 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[82 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[83 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[84 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[85 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[86 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[87 - 1], &colors[22], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }
  k_sleep(DELAY_TIME);
  k_sleep(K_MSEC(1000));

  memcpy(&pixels[55 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[56 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[57 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[58 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[59 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[60 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[61 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[62 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[63 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[64 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[65 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[66 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[67 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[68 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[69 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[70 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[71 - 1], &colors[23], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }
  k_sleep(DELAY_TIME);
  k_sleep(K_MSEC(1000));

  memcpy(&pixels[38 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[39 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[40 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[41 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[42 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[43 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[44 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[45 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[46 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[47 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[48 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[49 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[50 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[51 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[52 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[53 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[54 - 1], &colors[21], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }
  k_sleep(DELAY_TIME);
  k_sleep(K_MSEC(1000));

  memcpy(&pixels[25 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[26 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[27 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[28 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[29 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[30 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[31 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[32 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[33 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[34 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[35 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[36 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[37 - 1], &colors[21], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }
  k_sleep(DELAY_TIME);
  k_sleep(K_MSEC(1000));

  memcpy(&pixels[12 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[13 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[14 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[15 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[16 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[17 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[18 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[19 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[20 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[21 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[22 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[23 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[24 - 1], &colors[21], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }
  k_sleep(DELAY_TIME);
  k_sleep(K_MSEC(1000));

  memcpy(&pixels[1 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[2 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[3 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[4 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[5 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[6 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[7 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[8 - 1], &colors[23], sizeof(struct led_rgb));
  memcpy(&pixels[9 - 1], &colors[21], sizeof(struct led_rgb));
  memcpy(&pixels[10 - 1], &colors[22], sizeof(struct led_rgb));
  memcpy(&pixels[11 - 1], &colors[21], sizeof(struct led_rgb));

  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }
  k_sleep(DELAY_TIME);
}

static void update_rgb(void) {
  int rc;
  size_t cursor = 0;
  size_t color = 0;

  /**
   * strip - connected LEDs
   * pixels[i] - the pixels index contain the color it'll have
   * colors[i] - the color index contain the color (RGB)
   */

  LOG_INF("Displaying RGB Mode %d", rgb_mode);

  // Solid Colours
  if (rgb_mode == 0) {  // Off (slap)
    color = 0;
  } else if (rgb_mode == 1) {  // White (slap)
    color = 1;
  } else if (rgb_mode == 2) {  // Turquoise (slap)
    color = 2;
  } else if (rgb_mode == 3) {  // Rainbow (left to right animation)
    update_rgb_rainbow();
    return;

  } else if (rgb_mode == 4) {  // Christmas (top to bottom animation)
    update_rgb_christmas();
    return;
  }

  while (cursor < 87) {
    memcpy(&pixels[cursor], &colors[color], sizeof(struct led_rgb));
    cursor++;
  }
  rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
  if (rc) {
    LOG_ERR("couldn't update strip: %d", rc);
  }
  k_sleep(DELAY_TIME);
}

////////////////////////////////////////////////////////////////////////
//                                 MAIN                               //
////////////////////////////////////////////////////////////////////////

int main(void) {
  // 1sec Delay for LOG to appear on Terminal
  k_sleep(K_MSEC(1000));
  int err;
  int rc;

  print_logo();

  // POWER MANAGEMENT
  printk("\n\x1b[32m\x1b[1mPower Management Setup:\x1b[39m\x1b[0m\n");

  if (!device_is_ready(pmic)) {
    LOG_ERR("PMIC device not ready.");
    return 1;
  } else {
    LOG_INF("PMIC is ready.");
  }

  rc = pmic_custom_settings();
  if (rc != 0) {
    return 1;
  }

  if (!device_is_ready(charger)) {
    LOG_ERR("Charger device not ready.");
    return 1;
  } else {
    LOG_INF("Charger is ready.");
  }

  if (fuel_gauge_init(charger) < 0) {
    LOG_ERR("Could not initialize fuel gauge.");
    return 1;
  } else {
    LOG_INF("Fuel gauge is ready.");
  }

  static struct gpio_callback event_cb;
  gpio_init_callback(
      &event_cb, event_callback,
      BIT(NPM1300_EVENT_VBUS_DETECTED) | BIT(NPM1300_EVENT_VBUS_REMOVED));

  err = mfd_npm1300_add_callback(pmic, &event_cb);
  if (err) {
    LOG_ERR("Failed to add pmic callback.\n");
    return 1;
  }

  struct sensor_value val;
  rc = sensor_attr_get(charger, SENSOR_CHAN_CURRENT, SENSOR_ATTR_UPPER_THRESH,
                       &val);
  if (rc < 0) {
    return 1;
  }

  vbus_connected = (val.val1 != 0) || (val.val2 != 0);

  if (!device_is_ready(pmic_buck1)) {
    LOG_ERR("PMIC BUCK1 device not ready.");
    return 1;
  } else {
    LOG_INF("PMIC BUCK1 is ready.");
  }

  if (!device_is_ready(pmic_buck2)) {
    LOG_ERR("PMIC BUCK2 device not ready.");
    return 1;
  } else {
    LOG_INF("PMIC BUCK2 is ready.");
  }

  if (!device_is_ready(pmic_gpio)) {
    LOG_ERR("PMIC GPIO device not ready.");
    return 1;
  } else {
    LOG_INF("PMIC GPIO is ready.");
  }

  // GPIO 0 as INPUT (Pull Down)
  err = gpio_pin_configure(pmic_gpio, 0,
                           GPIO_INPUT | GPIO_ACTIVE_LOW | GPIO_PULL_DOWN);
  if (err) {
    LOG_ERR("gpio_pin_configure=%d", err);
    return 1;
  }

  atomic_set(&fg_wait, false);
  k_work_init_delayable(&fg_update, fg_update_handler);

  LOG_INF("Power Management Setup Complete");

  // BLE SETTINGS
  printk("\n\x1b[32m\x1b[1mBLE Settings Setup:\x1b[39m\x1b[0m\n");
  atomic_set(&ble_adv_mode, CONNECTING_MODE);
  atomic_set(&adv_safe, true);
  atomic_set(&enable_passkey_input, false);
  atomic_set(&idx_passkey_input, 0);
  pairing_key_start_time = 0;
  passkey_conn = NULL;
  for (int i = 0; i < 6; i++) {
    passkey_input[i] = 0;
  }

  err = settings_subsys_init();
  if (err) {
    LOG_ERR("Failed settings subsys initialization");
    return 1;
  }

  err = bt_conn_cb_register(&connection_callbacks);
  if (err) {
    LOG_ERR("Failed to register connection callbacks");
    return 1;
  }

  err = bt_conn_auth_cb_register(&conn_auth_callbacks);
  if (err) {
    LOG_ERR("Failed to register authorization callbacks");
    return 1;
  }

  err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
  if (err) {
    LOG_ERR("Failed to register authorization info callbacks.");
    return 1;
  }

  ble_hid_init();

  err = bt_enable(NULL);
  if (err) {
    LOG_ERR("Bluetooth init failed (err %d)", err);
    return 1;
  }

  k_work_init(&adv_work, adv_work_handler);
  k_work_init_delayable(&pm_cancel, pm_cancel_handler);
  k_work_init_delayable(&cm_cancel, cm_cancel_handler);

  LOG_INF("BLE Settings Setup Complete");

  // ZMS FLASH STORAGE
  printk("\n\x1b[32m\x1b[1mZMS Flash Storage Setup:\x1b[39m\x1b[0m\n");
  current_central = 1;
  central1_addr = clean_addr;
  central2_addr = clean_addr;
  central3_addr = clean_addr;

  for (int i = 0; i < CONFIG_BT_ID_MAX; i++) {
    identity_addr[i] = clean_addr;
  }
  identity_count = CONFIG_BT_ID_MAX;

  fn_mode = FN_MODE;
  rgb_mode = RGB_OFF;

  if (IS_ENABLED(CONFIG_SETTINGS)) {
    if (settings_load() != 0) {
      LOG_ERR("Settings failed to load");
      return 1;
    }
  }

  LOG_INF("current_central after load: %d", current_central);

  char addr_str[BT_ADDR_LE_STR_LEN];

  // Retrieve Central Identity Addr
  bt_addr_le_to_str(&central1_addr, addr_str, sizeof(addr_str));
  LOG_INF("central1_addr after load: %s", addr_str);

  bt_addr_le_to_str(&central2_addr, addr_str, sizeof(addr_str));
  LOG_INF("central2_addr after load: %s", addr_str);

  bt_addr_le_to_str(&central3_addr, addr_str, sizeof(addr_str));
  LOG_INF("central3_addr after load: %s", addr_str);

  // Retrieve Peripheral (Keyboard) Identity Addr
  bt_id_get(identity_addr, &identity_count);

  // Create identity addresses for all invalid slots
  for (int i = 1; i < CONFIG_BT_ID_MAX; i++) {
    rc = bt_addr_le_cmp(&(identity_addr[i]), &clean_addr);
    if (!rc) {
      err = bt_id_create(NULL, NULL);
      if (err < 0) {
        LOG_WRN(
            "Invalid identity address found: Did not create new identity "
            "address (err %d)",
            rc);

        rc = bt_id_reset(i, NULL, NULL);
        if (rc < 0) {
          LOG_WRN("bt_id_reset did not create new identity address");
        } else {
          LOG_INF("bt_id_reset created new identity address");
        }

      } else {
        LOG_INF("Invalid identity address found: Created new identity address");
      }
    }
  }

  // Reset the identity_count
  identity_count = CONFIG_BT_ID_MAX;
  bt_id_get(identity_addr, &identity_count);

  bt_addr_le_to_str(&(identity_addr[0]), addr_str, sizeof(addr_str));
  LOG_INF("identity_addr0 after load: %s", addr_str);

  bt_addr_le_to_str(&(identity_addr[1]), addr_str, sizeof(addr_str));
  LOG_INF("identity_addr1 after load: %s", addr_str);

  bt_addr_le_to_str(&(identity_addr[2]), addr_str, sizeof(addr_str));
  LOG_INF("identity_addr2 after load: %s", addr_str);

  bt_addr_le_to_str(&(identity_addr[3]), addr_str, sizeof(addr_str));
  LOG_INF("identity_addr3 after load: %s", addr_str);

  LOG_INF("ZMS Flash Storage Setup Complete");

  // USB SETTINGS
  printk("\n\x1b[32m\x1b[1mUSB Settings Setup:\x1b[39m\x1b[0m\n");
  usb_hid_ready = false;
  struct usbd_context *sample_usbd;
  const struct device *hid_dev;

  for (int i = 0; i < KB_REPORT_COUNT; i++) {
    report[i] = 0;
  }

  hid_dev = DEVICE_DT_GET_ONE(zephyr_hid_device);
  if (!device_is_ready(hid_dev)) {
    LOG_ERR("HID Device is not ready");
    return -EIO;
  }

  rc = hid_device_register(hid_dev, hid_report_desc, sizeof(hid_report_desc),
                           &kb_ops);
  if (rc != 0) {
    LOG_ERR("Failed to register HID Device (err: %d)", rc);
    return rc;
  }

  sample_usbd = sample_usbd_init_device(msg_cb);
  if (sample_usbd == NULL) {
    LOG_ERR("Failed to initialize USB device");
    return -ENODEV;
  }

  if (!usbd_can_detect_vbus(sample_usbd)) {
    rc = usbd_enable(sample_usbd);
    if (rc) {
      LOG_ERR("Failed to enable device support");
      return rc;
    }
  }

  LOG_INF("USB Settings Setup Complete");

  // RGB LEDS SETUP
  printk("\n\x1b[32m\x1b[1mRGB LEDs Setup:\x1b[39m\x1b[0m\n");

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (device_is_ready(strip) != 1) {
    LOG_ERR("LED strip device %s is not ready", strip->name);
    return 1;
  } else {
    LOG_INF("Found LED strip device %s", strip->name);
  }

  update_rgb();
  LOG_INF("RGB LEDs Setup Complete");

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // LEOPOLD FC750R MECHANICAL SETUP
  printk("\n\x1b[32m\x1b[1mLeopold FC750R Mechanical Setup:\x1b[39m\x1b[0m\n");

  fn_pressed = false;

  if (gpio_pin_get_dt(&dip2) == 1) {
    atomic_set(&nRF_mode, WIRED_MODE);
    LOG_INF("Keyboard is in WIRED MODE");
  } else {
    atomic_set(&nRF_mode, WIRELESS_MODE);
    LOG_INF("Keyboard is in WIRELESS MODE");
  }

  LOG_INF("Leopold FC750R Mechanical Setup Complete");

  // POST-SETUP LOGS
  printk("\n\x1b[32m\x1b[1mPost-Setup Logs:\x1b[39m\x1b[0m\n");
  if (atomic_get(&nRF_mode) == WIRELESS_MODE) {
    advertising_start();
  }

  while (true) {
    struct kb_event kb_evt;
    bool mod_key = true;

    k_msgq_get(&kb_msgq, &kb_evt, K_FOREVER);

    if (atomic_get(&nRF_mode) == WIRED_MODE) {
      // Disconnect with any BLE connections
      if (conn_mode[0].conn != NULL) {
        atomic_set(&adv_safe, false);
        bt_conn_disconnect(conn_mode[0].conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        LOG_INF("Disconnecting from current BLE connection");
        return 0;
      }

      uint8_t key = input_to_hid_modifier(kb_evt.code);
      if (key == 0) {
        key = (uint8_t)input_to_hid_code(kb_evt.code);
        mod_key = false;
      }

      if (kb_evt.value == 1) {
        err = usb_hid_report_set(key, mod_key);
      } else {
        err = usb_hid_report_clear(key, mod_key);
      }

      if (err) {
        LOG_WRN("Cannot set selected key.");
        continue;
      }

      if (!kb_ready) {
        LOG_INF("USB HID device is not ready");
        continue;
      }

      if (usbd_is_suspended(sample_usbd)) {
        /* on a press of any button, send wakeup request */
        if (kb_evt.value) {
          rc = usbd_wakeup_request(sample_usbd);
          if (rc) {
            LOG_WRN("Remote wakeup error (err: %d)", rc);
          }
        }
        continue;
      }

      rc = hid_device_submit_report(hid_dev, KB_REPORT_COUNT, report);
      if (rc) {
        LOG_WRN("HID submit report error (err: %d)", rc);
      }
    }

    if (atomic_get(&nRF_mode) == WIRELESS_MODE) {
      uint8_t key = input_to_hid_modifier(kb_evt.code);
      if (key == 0) {
        key = (uint8_t)input_to_hid_code(kb_evt.code);
        mod_key = false;
      }

      if (kb_evt.value == 1) {
        err = hid_kbd_state_key_set(key, mod_key);
      } else {
        err = hid_kbd_state_key_clear(key, mod_key);
      }

      if (err) {
        LOG_WRN("Cannot set selected key.");
      }

      key_report_send();

      // Update battery every 1min
      if (!atomic_get(&fg_wait)) {
        k_work_schedule(&fg_update, K_SECONDS(60));
        atomic_set(&fg_wait, true);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////
//                        FUNCTION DEFINITIONS                        //
////////////////////////////////////////////////////////////////////////

static void print_logo(void) {
  printk("\n%s", img_data);
  k_sleep(K_MSEC(1000));
}

static void fg_update_handler(struct k_work *work) {
  uint8_t battery_level = (uint8_t)fuel_gauge_update(charger, vbus_connected);
  bt_bas_set_battery_level(battery_level);

  if (atomic_get(&nRF_mode) == WIRELESS_MODE && (conn_mode[0].conn != NULL)) {
    k_work_schedule(&fg_update, K_SECONDS(60));
  } else {
    atomic_set(&fg_wait, false);
  }
}

static int pmic_custom_settings(void) {
  int rc;
  uint8_t reg = 0;

  // Set BUCK as 1.8V
  rc = mfd_npm1300_reg_write(pmic, BUCK_BASE, BUCK1NORMVOUT_OFFSET, 0x08);
  if (rc < 0) {
    return rc;
  }
  rc = mfd_npm1300_reg_read(pmic, BUCK_BASE, BUCK1NORMVOUT_OFFSET, &reg);
  if (rc < 0) {
    return rc;
  }
  LOG_INF("BUCK1NORMVOUT = %d", reg);

  // Set BUCK1 and BUCK2 ON/OFF Control as Software
  rc = mfd_npm1300_reg_write(pmic, BUCK_BASE, BUCKSWCTRLSEL_OFFSET, 0x03);
  if (rc < 0) {
    return rc;
  }
  rc = mfd_npm1300_reg_read(pmic, BUCK_BASE, BUCKSWCTRLSEL_OFFSET, &reg);
  if (rc < 0) {
    return rc;
  }
  LOG_INF("BUCKSWCTRLSEL = %d", reg);

  // Set GPIO0 as Interrupt
  rc = mfd_npm1300_reg_write(pmic, GPIO_BASE, GPIOMODE0_OFFSET, 0x00);
  if (rc < 0) {
    return rc;
  }
  rc = mfd_npm1300_reg_read(pmic, GPIO_BASE, GPIOMODE0_OFFSET, &reg);
  if (rc < 0) {
    return rc;
  }
  LOG_INF("GPIOMODE0 = %d", reg);

  // Set GPIO0 as PULL DOWN
  rc = mfd_npm1300_reg_write(pmic, GPIO_BASE, GPIODEN0_OFFSET, 0x01);
  if (rc < 0) {
    return rc;
  }
  rc = mfd_npm1300_reg_read(pmic, GPIO_BASE, GPIODEN0_OFFSET, &reg);
  if (rc < 0) {
    return rc;
  }
  LOG_INF("GPIOPUEN0 = %d", reg);

  // Set VBUS Input Current Limiter to 1.5A
  // rc = mfd_npm1300_reg_write(pmic, VBUSIN_BASE, VBUSINILIM0_OFFSET, 0x0F);
  // if (rc < 0) {
  //   return rc;
  // }
  // rc = mfd_npm1300_reg_read(pmic, VBUSIN_BASE, VBUSINILIM0_OFFSET, &reg);
  // if (rc < 0) {
  //   return rc;
  // }
  // LOG_INF("VBUSINILIM0 = %d", reg);

  // rc = mfd_npm1300_reg_write(pmic, VBUSIN_BASE, TASKUPDATEILIMSW_OFFSET,
  // 0x01); if (rc < 0) {
  //   return rc;
  // }
  // rc = mfd_npm1300_reg_read(pmic, VBUSIN_BASE, TASKUPDATEILIMSW_OFFSET,
  // &reg); if (rc < 0) {
  //   return rc;
  // }
  // LOG_INF("TASKUPDATEILIMSW = %d", reg);

  return 0;
}

static void event_callback(const struct device *dev, struct gpio_callback *cb,
                           uint32_t pins) {
  if (pins & BIT(NPM1300_EVENT_VBUS_DETECTED)) {
    LOG_INF("Vbus connected");
    vbus_connected = true;
  }

  if (pins & BIT(NPM1300_EVENT_VBUS_REMOVED)) {
    LOG_INF("Vbus removed");
    vbus_connected = false;
  }
}

int leopold_fc750R_handle_set(const char *name, size_t len,
                              settings_read_cb read_cb, void *cb_arg) {
  int err;
  const char *next;
  size_t name_len;

  name_len = settings_name_next(name, &next);

  if (!next) {
    if (!strncmp(name, "current_central", name_len)) {
      err = read_cb(cb_arg, &current_central, sizeof(current_central));
      return 0;
    }

    if (!strncmp(name, "central1_addr", name_len)) {
      err = read_cb(cb_arg, &central1_addr, sizeof(central1_addr));
      return 0;
    }

    if (!strncmp(name, "central2_addr", name_len)) {
      err = read_cb(cb_arg, &central2_addr, sizeof(central2_addr));
      return 0;
    }

    if (!strncmp(name, "central3_addr", name_len)) {
      err = read_cb(cb_arg, &central3_addr, sizeof(central3_addr));
      return 0;
    }

    if (!strncmp(name, "fn_mode", name_len)) {
      err = read_cb(cb_arg, &fn_mode, sizeof(fn_mode));
      return 0;
    }

    if (!strncmp(name, "rgb_mode", name_len)) {
      err = read_cb(cb_arg, &rgb_mode, sizeof(rgb_mode));
      return 0;
    }
  }
  return -ENOENT;
}

static void key_pressed(struct input_event *evt, void *user_data) {
  int err;
  struct kb_event kb_evt;

  /**
   * @note evt->code != INPUT_BTN_TOUCH works instead of
   * evt->type == INPUT_EV_KEY
   */

  if (evt->code != INPUT_BTN_TOUCH && evt->sync == true) {
    kb_evt.code = evt->code;
    kb_evt.value = evt->value;

    // DIP2 WIRE/WIRELESS
    if (kb_evt.code == INPUT_BTN_2) {
      if (kb_evt.value == 1) {
        atomic_set(&nRF_mode, WIRED_MODE);
        LOG_INF("Keyboard is in WIRED MODE");

        // Stop any BLE activities
        err = bt_le_adv_stop();
        if (err) {
          LOG_WRN("Did not stop any ongoing BLE advertising");
          return;
        } else {
          LOG_INF("Stopped any ongoing BLE advertising");
        }

        if (conn_mode[0].conn != NULL) {
          bt_conn_disconnect(conn_mode[0].conn,
                             BT_HCI_ERR_REMOTE_USER_TERM_CONN);
          LOG_INF("Process of disconnecting from current connection...");
          return;
        }

      } else {
        atomic_set(&nRF_mode, WIRELESS_MODE);
        LOG_INF("Keyboard is in WIRELESS MODE");
      }
      return;
    }

    if (atomic_get(&nRF_mode) == WIRELESS_MODE) {
      // PAIRING & CONNECTION MODE
      if (kb_evt.code == INPUT_KEY_PRINT) {
        central_id_ctrl(kb_evt.value, 1);
        return;

      } else if (kb_evt.code == INPUT_KEY_SCROLLLOCK) {
        central_id_ctrl(kb_evt.value, 2);
        return;

      } else if (kb_evt.code == INPUT_KEY_PAUSE) {
        central_id_ctrl(kb_evt.value, 3);
        return;
      }

      // INITIATE CONNECTION IF NO CENTRAL IS CONNECTED
      if ((conn_mode[0].conn == NULL) && kb_evt.value &&
          (atomic_get(&nRF_mode) == WIRELESS_MODE)) {
        atomic_set(&ble_adv_mode, CONNECTING_MODE);
        advertising_start();
        return;
      }

      // BLE SECURITY LEVEL 4 PASSKEY ENTRY
      if (atomic_get(&enable_passkey_input) && (kb_evt.value == 1)) {
        process_passkey_input(kb_evt);
        return;
      }
    }

    // FN KEY PRESSED/RELEASED
    if (kb_evt.code == INPUT_BTN_0) {
      if (kb_evt.value == 1) {
        fn_pressed = true;
      } else {
        fn_pressed = false;
      }
      return;
    }

    // FN KEY COMMANDS
    if (fn_pressed) {
      // check for the fn + space first

      if ((kb_evt.code == INPUT_KEY_0) && (kb_evt.value == 1)) {
        rgb_mode = 0;
        update_rgb();
      }

      if ((kb_evt.code == INPUT_KEY_1) && (kb_evt.value == 1)) {
        rgb_mode = 1;
        update_rgb();
      }

      if ((kb_evt.code == INPUT_KEY_2) && (kb_evt.value == 1)) {
        rgb_mode = 2;
        update_rgb();
      }

      if ((kb_evt.code == INPUT_KEY_3) && (kb_evt.value == 1)) {
        rgb_mode = 3;
        update_rgb();
      }

      if ((kb_evt.code == INPUT_KEY_4) && (kb_evt.value == 1)) {
        rgb_mode = 4;
        update_rgb();
      }

      if ((kb_evt.code == INPUT_KEY_5) && (kb_evt.value == 1)) {
        rgb_mode = 4;
        update_rgb();
      }

      if ((kb_evt.code == INPUT_KEY_6) && (kb_evt.value == 1)) {
        rgb_mode = 6;
        update_rgb();
      }

      // Save the RGB Mode
      err = settings_save_one("leopold_fc750R/rgb_mode",
                              (const void *)&rgb_mode, sizeof(rgb_mode));
      if (err) {
        LOG_WRN("rgb_mode was not saved to the flash storage (err %d)", err);
      } else {
        LOG_INF("rgb_mode = %d was saved to the flash storage", rgb_mode);
      }
      return;
    }

    // REROUTE KEYS
    /**
     * if (fn_pressed = true)
     *
     * FN + Space = Switch FN_Mode
     *  |_ Save FN_MODE to ZMS
     *  |_ return
     * return
     *
     * Do the RGB Stuff
     *
     */

    // CHANGE FN KEYS/MULTIMEDIA KEYS BASED ON FN_MODE
    /**
     * if(fn_pressed = true)
     *    |_ if (FN_MODE = FN_KEYS)
     *      |_ INPUT_KEY_F1 -> INPUT_KEY_BRIGHETNESSLOW
     *      |_ etc. skip
     *  |_ if (FN_MODE = MULTIMEDEIA Keys)
     *      |_ INPUT_KEY_F1
     *      |_ etc. skip
     *
     *  if(fn_pressed = false)
     *    |_ if (FN_MODE = FN_KEYS)
     *      |_ INPUT_KEY_F1
     *      |_ etc. skip
     *  |_ if (FN_MODE = MULTIMEDEIA Keys )
     *      |_ INPUT_KEY_F1 -> INPUT_KEY_BRIGHETNESSLOW
     *      |_ etc. skip
     *
     */

    // REGISTER USER KEY PRESS
    if (!atomic_get(&enable_passkey_input)) {
      err = k_msgq_put(&kb_msgq, &kb_evt, K_NO_WAIT);
      if (err != 0) {
        printk("\nKey Press Register:\n");
        LOG_WRN("Failed to register new key input to queue (err %d)", err);
      }
    }
  }
}

static void central_id_ctrl(int32_t pressed, int central_id) {
  int err;
  int rc;
  const char *key_text = "N/A";

  if (pressed) {
    printk("\nCentral ID Control:\n");
    current_central = central_id;
    err = settings_save_one("leopold_fc750R/current_central",
                            (const void *)&current_central,
                            sizeof(current_central));
    if (err) {
      LOG_WRN("current_central was not saved to the flash storage (err %d)",
              err);
      return;
    } else {
      LOG_INF("current_central = %d was saved to the flash storage",
              central_id);
    }
    pairing_key_start_time = k_uptime_get();

  } else {
    int64_t time_pressed = k_uptime_get() - pairing_key_start_time;

    if (central_id == 1) {
      key_text = "PrtSc";
    } else if (central_id == 2) {
      key_text = "ScrLk";
    } else if (central_id == 3) {
      key_text = "Pause";
    }

    if (time_pressed >= 3000) {  // Pairing Mode
      LOG_INF("%s was pressed for %lld sec. Pairing Mode activated", key_text,
              time_pressed / 1000);
      atomic_set(&ble_adv_mode, PAIRING_MODE);

      // Case: central has never been paired before
      if (central_id == 1) {
        rc = bt_addr_le_cmp(&central1_addr, &clean_addr);
      } else if (central_id == 2) {
        rc = bt_addr_le_cmp(&central2_addr, &clean_addr);
      } else if (central_id == 3) {
        rc = bt_addr_le_cmp(&central3_addr, &clean_addr);
      }

      if (rc == 0) {
        advertising_start();
      } else {
        if (central_id == 1) {
          err = bt_unpair(1, &central1_addr);
        } else if (central_id == 2) {
          err = bt_unpair(2, &central2_addr);
        } else if (central_id == 3) {
          err = bt_unpair(3, &central3_addr);
        }

        if (err) {
          LOG_WRN("Cannot delete bond (err: %d)", err);

        } else {
          LOG_INF("Bond deleted successfully");
          if (central_id == 1) {
            central1_addr = clean_addr;
            err = settings_save_one("leopold_fc750R/central1_addr",
                                    (const void *)&central1_addr,
                                    sizeof(central1_addr));
          } else if (central_id == 2) {
            central2_addr = clean_addr;
            err = settings_save_one("leopold_fc750R/central2_addr",
                                    (const void *)&central2_addr,
                                    sizeof(central2_addr));
          } else if (central_id == 3) {
            central3_addr = clean_addr;
            err = settings_save_one("leopold_fc750R/central3_addr",
                                    (const void *)&central3_addr,
                                    sizeof(central3_addr));
          }

          if (err) {
            LOG_WRN(
                "central%d_addr was not saved to the flash storage (err %d)",
                current_central, err);

          } else {
            advertising_start();
          }
        }
      }

    } else {  // Connection Mode
      LOG_INF("%s was pressed for %lld sec. Connection Mode activated",
              key_text, time_pressed / 1000);
      atomic_set(&ble_adv_mode, CONNECTING_MODE);
      advertising_start();
    }

    pairing_key_start_time = 0;
    return;
  }
}

static void pm_cancel_handler(struct k_work *work) {
  if (conn_mode[0].conn == NULL) {
    bt_le_adv_stop();
  }
}

static void cm_cancel_handler(struct k_work *work) {
  if (conn_mode[0].conn == NULL) {
    bt_le_adv_stop();
  }
}

static void adv_work_handler(struct k_work *work) {
  int err;
  int rc;

  if (atomic_get(&ble_adv_mode) == PAIRING_MODE) {
    err = bt_le_adv_stop();
    if (err) {
      LOG_WRN("Did not stop any ongoing BLE advertising");
      return;
    } else {
      LOG_INF("Stopped any ongoing BLE advertising");
    }

    // Safe Advertising
    if (conn_mode[0].conn != NULL) {
      atomic_set(&adv_safe, false);
      bt_conn_disconnect(conn_mode[0].conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
      LOG_INF("Process of disconnecting from current connection...");
      LOG_INF("Requesting to advertise again when safe...");
      return;
    }

    // Safe Identity Changing
    err = bt_id_delete((uint8_t)(current_central));
    if (err) {
      LOG_WRN("Did not delete identity address");
    } else {
      LOG_INF("Deleted previous identity address");
    }

    // Use reset to reclaim the memory storage and create new identity address
    rc = bt_id_reset(current_central, NULL, NULL);
    if (rc < 0) {
      LOG_WRN("Did not create new identity address");
    } else {
      LOG_INF("Created new identity address");
    }

    const struct bt_le_adv_param *adv_param_pm = NULL;

    if (current_central == 1) {
      adv_param_pm = &adv_param_pm1;
    } else if (current_central == 2) {
      adv_param_pm = &adv_param_pm2;
    } else if (current_central == 3) {
      adv_param_pm = &adv_param_pm3;
    }

    err = bt_le_adv_start(adv_param_pm, ad_pm, ARRAY_SIZE(ad_pm), sd_pm,
                          ARRAY_SIZE(sd_pm));
    if (err) {
      LOG_WRN("Pairing advertising failed to start (err %d)", err);
    } else {
      LOG_INF("Pairing advertising successfully started");
    }

    k_work_schedule(&pm_cancel, K_SECONDS(600));
    return;
  }

  if (atomic_get(&ble_adv_mode) == CONNECTING_MODE) {
    err = bt_le_adv_stop();
    if (err) {
      LOG_WRN("Did not stop any ongoing BLE advertising");
      return;
    } else {
      LOG_INF("Stopped any ongoing BLE advertising");
    }

    // Check if central address is invalid
    bt_addr_le_t *peer = NULL;
    if (current_central == 1) {
      peer = &central1_addr;
    } else if (current_central == 2) {
      peer = &central2_addr;
    } else if (current_central == 3) {
      peer = &central3_addr;
    }

    rc = bt_addr_le_cmp(peer, &clean_addr);
    if (rc == 0) {
      LOG_INF("Invalid address -- Will not advertise");
      return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(peer, addr_str, sizeof(addr_str));
    LOG_INF("Attempting to connect with address %s", addr_str);

    if (conn_mode[0].conn != NULL) {
      // Check if it's already connected, if so do nothing
      const bt_addr_le_t *current_conn = bt_conn_get_dst(conn_mode[0].conn);
      rc = bt_addr_le_cmp(current_conn, peer);
      if (rc == 0) {
        LOG_INF("Already connected to desired central");
        return;
      }

      // Safe Advertising
      atomic_set(&adv_safe, false);
      bt_conn_disconnect(conn_mode[0].conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
      LOG_INF("Process of disconnecting from current connection...");
      LOG_INF("Requesting to advertise again when safe...");
      return;
    }

    err = bt_le_filter_accept_list_clear();
    if (err) {
      LOG_WRN("BLE Filter Accept List was not cleared");
      return;
    } else {
      LOG_INF("BLE Filter Accept List was cleared");
    }

    err = bt_le_filter_accept_list_add(peer);
    if (err) {
      LOG_WRN("BLE Filter Accept List did not add address %s", addr_str);
      return;
    } else {
      LOG_INF("BLE Filter Accept List added address %s", addr_str);
    }

    const struct bt_le_adv_param *adv_param_cm = NULL;

    if (current_central == 1) {
      adv_param_cm = &adv_param_cm1;
    } else if (current_central == 2) {
      adv_param_cm = &adv_param_cm2;
    } else if (current_central == 3) {
      adv_param_cm = &adv_param_cm3;
    }

    err = bt_le_adv_start(adv_param_cm, ad_cm, ARRAY_SIZE(ad_cm), sd_cm,
                          ARRAY_SIZE(sd_cm));

    if (err) {
      LOG_WRN("Connection advertising failed to start (err %d)", err);
      return;
    } else {
      LOG_INF("Connection advertising successfully started");
    }

    k_work_schedule(&cm_cancel, K_SECONDS(60));
    return;
  }
}

static void advertising_start(void) {
  printk("\nBLE Advertising:\n");
  struct k_work_sync sync1, sync2, sync3;

  // Wait until all work and delayed work queue/running are cleared
  k_work_cancel_sync(&adv_work, &sync1);
  k_work_cancel_delayable_sync(&pm_cancel, &sync2);
  k_work_cancel_delayable_sync(&cm_cancel, &sync3);

  LOG_INF("Submitting new advertising job");
  k_work_submit(&adv_work);
}

static void connected_cb(struct bt_conn *conn, uint8_t err) {
  char addr_str[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

  if (err) {
    LOG_INF("Failed to connect to %s 0x%02x %s", addr_str, err,
            bt_hci_err_to_str(err));
    return;
  }

  LOG_INF("Connected to central: %s", addr_str);

  err = bt_hids_connected(&hids_obj, conn);
  if (err) {
    LOG_WRN("Failed to notify HID service about connection");
    return;
  }

  for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
    if (!conn_mode[i].conn) {
      conn_mode[i].conn = bt_conn_ref(conn);
      conn_mode[i].in_boot_mode = false;
      break;
    }
  }

  // Request for Level 4 security
  err = bt_conn_set_security(conn, BT_SECURITY_L4);
  if (err) {
    LOG_WRN("Failed to request setting security level 4 (err %u)", err);
    return;
  }

  // Notify the battery level
  if (atomic_get(&ble_adv_mode) == CONNECTING_MODE) {
    k_work_schedule(&fg_update, K_SECONDS(0));
  }
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason) {
  int err;
  char addr_str[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

  LOG_INF("Disconnected from %s, reason 0x%02x %s", addr_str, reason,
          bt_hci_err_to_str(reason));

  err = bt_hids_disconnected(&hids_obj, conn);

  if (err) {
    LOG_WRN("Failed to notify HID service about disconnection");
  }

  for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
    if (conn_mode[i].conn == conn) {
      bt_conn_unref(conn_mode[i].conn);
      conn_mode[i].conn = NULL;
    }
  }

  // In case it was cancelled while passkey
  if (passkey_conn != NULL) {
    atomic_set(&enable_passkey_input, false);
    bt_conn_unref(passkey_conn);
    passkey_conn = NULL;
  }
}

static void recycled_cb(void) {
  LOG_INF(
      "Connection object available from previous conn. Disconnect is "
      "complete!");

  if (!atomic_get(&adv_safe)) {
    LOG_INF("Safe to advertise -- Advertising again");
    atomic_set(&adv_safe, true);
    advertising_start();
  }
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level,
                                enum bt_security_err err) {
  char addr_str[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

  if (err) {
    LOG_WRN("Security failed: %s level %u (err %u)", addr_str, level, err);
    bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
  } else {
    LOG_INF("Security changed: %s level %u", addr_str, level);
  }
}

static void identity_resolved_cb(struct bt_conn *conn, const bt_addr_le_t *rpa,
                                 const bt_addr_le_t *identity) {
  LOG_INF("Remote identity address has been resolved");
}

void process_passkey_input(struct kb_event kb_evt) {
  switch (kb_evt.code) {
    case INPUT_KEY_0:
      passkey_input[atomic_get(&idx_passkey_input)] = 0;
      atomic_inc(&idx_passkey_input);
      printk("0");
      break;

    case INPUT_KEY_1:
      passkey_input[atomic_get(&idx_passkey_input)] = 1;
      atomic_inc(&idx_passkey_input);
      printk("1");
      break;

    case INPUT_KEY_2:
      passkey_input[atomic_get(&idx_passkey_input)] = 2;
      atomic_inc(&idx_passkey_input);
      printk("2");
      break;

    case INPUT_KEY_3:
      passkey_input[atomic_get(&idx_passkey_input)] = 3;
      atomic_inc(&idx_passkey_input);
      printk("3");
      break;

    case INPUT_KEY_4:
      passkey_input[atomic_get(&idx_passkey_input)] = 4;
      atomic_inc(&idx_passkey_input);
      printk("4");
      break;

    case INPUT_KEY_5:
      passkey_input[atomic_get(&idx_passkey_input)] = 5;
      atomic_inc(&idx_passkey_input);
      printk("5");
      break;

    case INPUT_KEY_6:
      passkey_input[atomic_get(&idx_passkey_input)] = 6;
      atomic_inc(&idx_passkey_input);
      printk("6");
      break;

    case INPUT_KEY_7:
      passkey_input[atomic_get(&idx_passkey_input)] = 7;
      atomic_inc(&idx_passkey_input);
      printk("7");
      break;

    case INPUT_KEY_8:
      passkey_input[atomic_get(&idx_passkey_input)] = 8;
      atomic_inc(&idx_passkey_input);
      printk("8");
      break;

    case INPUT_KEY_9:
      passkey_input[atomic_get(&idx_passkey_input)] = 9;
      atomic_inc(&idx_passkey_input);
      printk("9");
      break;

    default:
      break;
  }

  if (atomic_get(&idx_passkey_input) == 6) {
    int rc;
    unsigned int passkey = 0;

    passkey = passkey_input[0] * 100000 + passkey_input[1] * 10000 +
              passkey_input[2] * 1000 + passkey_input[3] * 100 +
              passkey_input[4] * 10 + passkey_input[5];

    printk("\n");
    LOG_INF("Passkey to be sent: %d", passkey);

    rc = bt_conn_auth_passkey_entry(passkey_conn, passkey);
    if (rc != 0) {
      LOG_WRN("Passkey was not sent to the central");
    } else {
      LOG_INF("Passkey sent");
    }

    atomic_set(&enable_passkey_input, false);
    bt_conn_unref(passkey_conn);
    passkey_conn = NULL;
  }
}

static void auth_passkey_entry(struct bt_conn *conn) {
  LOG_INF("Central is requesting for a passkey");
  printk("Enter Passkey: ");

  atomic_set(&enable_passkey_input, true);
  atomic_set(&idx_passkey_input, 0);
  passkey_conn = bt_conn_ref(conn);
}

static void auth_cancel(struct bt_conn *conn) {
  char addr_str[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

  LOG_INF("Cancelled pairing with Central: %s", addr_str);
  // Will invoke pairing_failed function
}

static void pairing_complete(struct bt_conn *conn, bool bonded) {
  int err;
  char addr_str[BT_ADDR_LE_STR_LEN];

  LOG_INF("Pairing Complete");
  const bt_addr_le_t *new_address = bt_conn_get_dst(conn);

  if (current_central == 1) {
    central1_addr = *new_address;
    err =
        settings_save_one("leopold_fc750R/central1_addr",
                          (const void *)&central1_addr, sizeof(central1_addr));
    if (err) {
      LOG_WRN("central1_addr was not saved to the flash storage");
    } else {
      LOG_INF("central1_addr was saved to the flash storage");
    }

  } else if (current_central == 2) {
    central2_addr = *new_address;
    err =
        settings_save_one("leopold_fc750R/central2_addr",
                          (const void *)&central2_addr, sizeof(central2_addr));
    if (err) {
      LOG_WRN("central2_addr was not saved to the flash storage");
    } else {
      LOG_INF("central2_addr was saved to the flash storage");
    }

  } else if (current_central == 3) {
    central3_addr = *new_address;
    err =
        settings_save_one("leopold_fc750R/central3_addr",
                          (const void *)&central3_addr, sizeof(central3_addr));
    if (err) {
      LOG_WRN("central3_addr was not saved to the flash storage");
    } else {
      LOG_INF("central3_addr was saved to the flash storage");
    }
  }

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

  LOG_INF("Pairing completed: %s, bonded: %d", addr_str, bonded);
  atomic_set(&ble_adv_mode, CONNECTING_MODE);  // Reset to DCM

  // Notify the battery state
  k_work_schedule(&fg_update, K_SECONDS(0));

  return;
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason) {
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  LOG_INF("Pairing failed conn: %s, reason %d %s", addr, reason,
          bt_security_err_to_str(reason));
  atomic_set(&ble_adv_mode, CONNECTING_MODE);  // Reset to DCM
  return;
}

static void caps_lock_handler(const struct bt_hids_rep *rep) {
  // uint8_t report_val =
  //     ((*rep->data) & OUTPUT_REPORT_BIT_MASK_CAPS_LOCK) ? 1 : 0;
}

static void hids_outp_rep_handler(struct bt_hids_rep *rep, struct bt_conn *conn,
                                  bool write) {
  char addr[BT_ADDR_LE_STR_LEN];

  if (!write) {
    LOG_INF("Output report read");
    return;
  };

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  LOG_INF("Output report has been received %s", addr);
  caps_lock_handler(rep);
}

static void hids_boot_kb_outp_rep_handler(struct bt_hids_rep *rep,
                                          struct bt_conn *conn, bool write) {
  char addr[BT_ADDR_LE_STR_LEN];

  if (!write) {
    LOG_INF("Output report read");
    return;
  };

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  LOG_INF("Boot Keyboard Output report has been received %s\n", addr);
  caps_lock_handler(rep);
}

static void hids_pm_evt_handler(enum bt_hids_pm_evt evt, struct bt_conn *conn) {
  char addr[BT_ADDR_LE_STR_LEN];
  size_t i;

  for (i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
    if (conn_mode[i].conn == conn) {
      break;
    }
  }

  if (i >= CONFIG_BT_HIDS_MAX_CLIENT_COUNT) {
    LOG_INF("Cannot find connection handle when processing PM");
    return;
  }

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  switch (evt) {
    case BT_HIDS_PM_EVT_BOOT_MODE_ENTERED:
      LOG_INF("Boot mode entered %s\n", addr);
      conn_mode[i].in_boot_mode = true;
      break;

    case BT_HIDS_PM_EVT_REPORT_MODE_ENTERED:
      LOG_INF("Report mode entered %s\n", addr);
      conn_mode[i].in_boot_mode = false;
      break;

    default:
      break;
  }
}

static void ble_hid_init(void) {
  int err;
  struct bt_hids_init_param hids_init_obj = {0};
  struct bt_hids_inp_rep *hids_inp_rep;
  struct bt_hids_outp_feat_rep *hids_outp_rep;

  static const uint8_t report_map[] = {
      0x05, 0x01, /* Usage Page (Generic Desktop) */
      0x09, 0x06, /* Usage (Keyboard) */
      0xA1, 0x01, /* Collection (Application) */

  /* Keys */
#if INPUT_REP_KEYS_REF_ID
      0x85, INPUT_REP_KEYS_REF_ID,
#endif
      0x05, 0x07, /* Usage Page (Key Codes) */
      0x19, 0xe0, /* Usage Minimum (224) */
      0x29, 0xe7, /* Usage Maximum (231) */
      0x15, 0x00, /* Logical Minimum (0) */
      0x25, 0x01, /* Logical Maximum (1) */
      0x75, 0x01, /* Report Size (1) */
      0x95, 0x08, /* Report Count (8) */
      0x81, 0x02, /* Input (Data, Variable, Absolute) */

      0x95, 0x01, /* Report Count (1) */
      0x75, 0x08, /* Report Size (8) */
      0x81, 0x01, /* Input (Constant) reserved byte(1) */

      0x95, 0x06, /* Report Count (6) */
      0x75, 0x08, /* Report Size (8) */
      0x15, 0x00, /* Logical Minimum (0) */
      0x25, 0x65, /* Logical Maximum (101) */
      0x05, 0x07, /* Usage Page (Key codes) */
      0x19, 0x00, /* Usage Minimum (0) */
      0x29, 0x65, /* Usage Maximum (101) */
      0x81, 0x00, /* Input (Data, Array) Key array(6 bytes) */

  /* LED */
#if OUTPUT_REP_KEYS_REF_ID
      0x85, OUTPUT_REP_KEYS_REF_ID,
#endif
      0x95, 0x05, /* Report Count (5) */
      0x75, 0x01, /* Report Size (1) */
      0x05, 0x08, /* Usage Page (Page# for LEDs) */
      0x19, 0x01, /* Usage Minimum (1) */
      0x29, 0x05, /* Usage Maximum (5) */
      0x91, 0x02, /* Output (Data, Variable, Absolute), */
                  /* Led report */
      0x95, 0x01, /* Report Count (1) */
      0x75, 0x03, /* Report Size (3) */
      0x91, 0x01, /* Output (Data, Variable, Absolute), */
                  /* Led report padding */

      0xC0 /* End Collection (Application) */
  };

  hids_init_obj.rep_map.data = report_map;
  hids_init_obj.rep_map.size = sizeof(report_map);

  hids_init_obj.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
  hids_init_obj.info.b_country_code = 0x00;
  hids_init_obj.info.flags =
      (BT_HIDS_REMOTE_WAKE | BT_HIDS_NORMALLY_CONNECTABLE);

  hids_inp_rep = &hids_init_obj.inp_rep_group_init.reports[INPUT_REP_KEYS_IDX];
  hids_inp_rep->size = INPUT_REPORT_KEYS_MAX_LEN;
  hids_inp_rep->id = INPUT_REP_KEYS_REF_ID;
  hids_init_obj.inp_rep_group_init.cnt++;

  hids_outp_rep =
      &hids_init_obj.outp_rep_group_init.reports[OUTPUT_REP_KEYS_IDX];
  hids_outp_rep->size = OUTPUT_REPORT_MAX_LEN;
  hids_outp_rep->id = OUTPUT_REP_KEYS_REF_ID;
  hids_outp_rep->handler = hids_outp_rep_handler;
  hids_init_obj.outp_rep_group_init.cnt++;

  hids_init_obj.is_kb = true;
  hids_init_obj.boot_kb_outp_rep_handler = hids_boot_kb_outp_rep_handler;
  hids_init_obj.pm_evt_handler = hids_pm_evt_handler;

  err = bt_hids_init(&hids_obj, &hids_init_obj);
  __ASSERT(err == 0, "HIDS initialization failed\n");
}

static int key_report_con_send(const struct keyboard_state *state,
                               bool boot_mode, struct bt_conn *conn) {
  int err = 0;
  uint8_t data[INPUT_REPORT_KEYS_MAX_LEN];
  uint8_t *key_data;
  const uint8_t *key_state;
  size_t n;

  data[0] = state->ctrl_keys_state;
  data[1] = 0;
  key_data = &data[2];
  key_state = state->keys_state;

  for (n = 0; n < KEY_PRESS_MAX; ++n) {
    *key_data++ = *key_state++;
  }

  if (boot_mode) {
    err =
        bt_hids_boot_kb_inp_rep_send(&hids_obj, conn, data, sizeof(data), NULL);
  } else {
    err = bt_hids_inp_rep_send(&hids_obj, conn, INPUT_REP_KEYS_IDX, data,
                               sizeof(data), NULL);
  }
  return err;
}

static int key_report_send(void) {
  for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
    if (conn_mode[i].conn) {
      int err;

      err = key_report_con_send(&hid_keyboard_state, conn_mode[i].in_boot_mode,
                                conn_mode[i].conn);
      if (err) {
        LOG_WRN("Key report send (error: %d)", err);
        return err;
      }
    }
  }
  return 0;
}

static int hid_kbd_state_key_set(uint8_t key, bool mod_key) {
  if (mod_key) {
    uint8_t ctrl_mask = key;
    hid_keyboard_state.ctrl_keys_state |= ctrl_mask;
    return 0;
  }

  for (size_t i = 0; i < KEY_PRESS_MAX; ++i) {
    if (hid_keyboard_state.keys_state[i] == 0) {
      hid_keyboard_state.keys_state[i] = key;
      return 0;
    }
  }
  /* All slots busy */
  return -EBUSY;
}

static int hid_kbd_state_key_clear(uint8_t key, bool mod_key) {
  if (mod_key) {
    uint8_t ctrl_mask = key;
    hid_keyboard_state.ctrl_keys_state &= ~ctrl_mask;
    return 0;
  }
  for (size_t i = 0; i < KEY_PRESS_MAX; ++i) {
    if (hid_keyboard_state.keys_state[i] == key) {
      hid_keyboard_state.keys_state[i] = 0;
      return 0;
    }
  }
  /* Key not found */
  return -EINVAL;
}

static void kb_iface_ready(const struct device *dev, const bool ready) {
  LOG_INF("HID device %s interface is %s", dev->name,
          ready ? "ready" : "not ready");
  kb_ready = ready;
}

static int kb_get_report(const struct device *dev, const uint8_t type,
                         const uint8_t id, const uint16_t len,
                         uint8_t *const buf) {
  LOG_WRN("Get Report not implemented, Type %u ID %u", type, id);

  return 0;
}

static int kb_set_report(const struct device *dev, const uint8_t type,
                         const uint8_t id, const uint16_t len,
                         const uint8_t *const buf) {
  if (type != HID_REPORT_TYPE_OUTPUT) {
    LOG_WRN("Unsupported report type");
    return -ENOTSUP;
  }

  return 0;
}

static void kb_set_idle(const struct device *dev, const uint8_t id,
                        const uint32_t duration) {
  LOG_INF("Set Idle %u to %u", id, duration);
  kb_duration = duration;
}

static uint32_t kb_get_idle(const struct device *dev, const uint8_t id) {
  LOG_INF("Get Idle %u to %u", id, kb_duration);
  return kb_duration;
}

static void kb_set_protocol(const struct device *dev, const uint8_t proto) {
  LOG_INF("Protocol changed to %s",
          proto == 0U ? "Boot Protocol" : "Report Protocol");
}

static void kb_output_report(const struct device *dev, const uint16_t len,
                             const uint8_t *const buf) {
  LOG_HEXDUMP_DBG(buf, len, "o.r.");
  kb_set_report(dev, HID_REPORT_TYPE_OUTPUT, 0U, len, buf);
}

static void msg_cb(struct usbd_context *const usbd_ctx,
                   const struct usbd_msg *const msg) {
  LOG_INF("USBD message: %s", usbd_msg_type_string(msg->type));

  if (msg->type == USBD_MSG_CONFIGURATION) {
    LOG_INF("\tConfiguration value %d", msg->status);
  }

  if (usbd_can_detect_vbus(usbd_ctx)) {
    if (msg->type == USBD_MSG_VBUS_READY) {
      if (usbd_enable(usbd_ctx)) {
        LOG_ERR("Failed to enable device support");
      }
    }

    if (msg->type == USBD_MSG_VBUS_REMOVED) {
      if (usbd_disable(usbd_ctx)) {
        LOG_ERR("Failed to disable device support");
      }
    }
  }
}

static int usb_hid_report_set(uint8_t key, bool mod_key) {
  if (mod_key) {
    uint8_t ctrl_mask = key;
    report[KB_MOD_KEY] |= ctrl_mask;
    report[KB_RESERVED] = 0;
    return 0;
  }

  for (size_t i = KB_KEY_CODE1; i < KB_REPORT_COUNT; ++i) {
    if (report[i] == 0) {
      report[i] = key;
      report[KB_RESERVED] = 0;
      return 0;
    }
  }

  /* All slots busy */
  return -EBUSY;
}

static int usb_hid_report_clear(uint8_t key, bool mod_key) {
  if (mod_key) {
    uint8_t ctrl_mask = key;
    report[KB_MOD_KEY] &= ~ctrl_mask;
    report[KB_RESERVED] = 0;
    return 0;
  }

  for (size_t i = KB_KEY_CODE1; i < KB_REPORT_COUNT; ++i) {
    if (report[i] == key) {
      report[i] = 0;
      report[KB_RESERVED] = 0;
      return 0;
    }
  }

  /* Key not found */
  return -EINVAL;
}

/** @note god please let this compile and flash */
