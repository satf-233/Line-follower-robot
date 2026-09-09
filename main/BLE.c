/**
 * BLE.c - Bluedroid GATT 服务器，向电脑发送 MJPEG 帧
 *
 * 协议说明见 BLE.h。核心流程：
 *   1. ble_init() 初始化 Bluedroid，创建 GATT 服务并开始广播；
 *   2. 中心设备连接后，写 CCCD 开启 Notification；
 *   3. ble_send_frame() 把一帧 MJPEG 数据按 MTU 分片，逐片发送。
 */

#include "BLE.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

#include "nvs_flash.h"

static const char *TAG = "BLE";

/* ==================== 常量 ==================== */
#define PROFILE_APP_ID        0
#define DEVICE_NAME           "ESP32S3-CAM"

/* 帧分片协议头 */
#define BLE_SYNC              0x5A
#define BLE_FLAG_START        0x01
#define BLE_FLAG_END          0x02
#define BLE_HDR_SIZE          6

#define BLE_MTU_DEFAULT       23
#define BLE_MTU_MAX           517
#define BLE_SEND_BUF_SIZE     520   /* 足以容纳 MTU 517 - 3(ATT头) = 514 字节 */

/* ==================== GATT 属性索引 ==================== */
enum {
    IDX_SVC,
    IDX_CHAR_FRAME_DECL,
    IDX_CHAR_FRAME_VAL,
    IDX_CHAR_FRAME_CFG,
    IDX_CHAR_CTRL_DECL,
    IDX_CHAR_CTRL_VAL,
    IDX_NB,
};

/* 16 位 UUID (小端字节序存储) */
static const uint16_t primary_service_uuid = 0x2800;
static const uint16_t char_decl_uuid       = 0x2803;
static const uint16_t char_cccd_uuid       = 0x2902;
static const uint16_t service_uuid         = 0xFFE0;
static const uint16_t frame_char_uuid      = 0xFFE1;
static const uint16_t ctrl_char_uuid       = 0xFFE2;

static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_write  = ESP_GATT_CHAR_PROP_BIT_WRITE;
static uint16_t cccd_value = 0x0000;   /* 初始：通知关闭 */

/* 特征值属性最大长度：通知数据由 send_indicate 直接发出，不受此限制 */
#define BLE_FRAME_ATTR_MAX_LEN  512
#define BLE_CTRL_ATTR_MAX_LEN   16

static esp_gatts_attr_db_t gatt_db[IDX_NB] = {
    [IDX_SVC] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
                               sizeof(service_uuid), sizeof(service_uuid), (uint8_t *)&service_uuid}},

    [IDX_CHAR_FRAME_DECL] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_decl_uuid, ESP_GATT_PERM_READ,
                               sizeof(char_prop_notify), sizeof(char_prop_notify), (uint8_t *)&char_prop_notify}},

    [IDX_CHAR_FRAME_VAL] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&frame_char_uuid, ESP_GATT_PERM_READ,
                               BLE_FRAME_ATTR_MAX_LEN, 0, NULL}},

    [IDX_CHAR_FRAME_CFG] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_cccd_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               sizeof(cccd_value), sizeof(cccd_value), (uint8_t *)&cccd_value}},

    [IDX_CHAR_CTRL_DECL] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_decl_uuid, ESP_GATT_PERM_READ,
                               sizeof(char_prop_write), sizeof(char_prop_write), (uint8_t *)&char_prop_write}},

    [IDX_CHAR_CTRL_VAL] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&ctrl_char_uuid, ESP_GATT_PERM_WRITE,
                               BLE_CTRL_ATTR_MAX_LEN, 0, NULL}},
};

/* 广播数据 */
/* 16-bit UUID 0xFFE0 的完整 128-bit 表示。
 * ESP-IDF v5.x 要求 p_service_uuid 传 128-bit 数组、service_uuid_len 为 16 的整数倍；
 * 其中 [12] 为 16-bit 值的低字节、[13] 为高字节（见 btc128_to_bta_uuid）。 */
static const uint8_t adv_service_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xE0, 0xFF, 0x00, 0x00,
};
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp       = false,
    .include_name       = true,
    .include_txpower    = false,
    .min_interval       = 0x0006,
    .max_interval       = 0x0010,
    .appearance         = 0x00,
    .manufacturer_len   = 0,
    .p_manufacturer_data= NULL,
    .service_data_len   = 0,
    .p_service_data     = NULL,
    .service_uuid_len   = sizeof(adv_service_uuid),
    .p_service_uuid     = (uint8_t *)adv_service_uuid,
    .flag               = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* ==================== 状态 ==================== */
typedef struct {
    /**
     * @brief BLE 上下文是否已初始化
     */
    bool            initialized;
    
    /**
     * @brief BLE 连接状态标志
     */
    bool            connected;
    
    /**
     * @brief 通知（Notification）是否已使能
     */
    bool            notify_enabled;
    
    /**
     * @brief 当前连接的 MTU（Maximum Transmission Unit）大小
     */
    uint16_t        mtu;
    
    /**
     * @brief BLE 连接 ID
     */
    uint16_t        conn_id;
    
    /**
     * @brief GATT 接口句柄（GATT Interface Handle）
     */
    esp_gatt_if_t   gatts_if;
    
    /**
     * @brief 服务的句柄（Service Handle）
     */
    uint16_t        service_handle;
    
    /**
     * @brief 图像帧特征值（Frame Characteristic）的句柄
     */
    uint16_t        frame_val_handle;
    
    /**
     * @brief 图像帧特征值的 CCCD（Client Characteristic Configuration Descriptor）句柄
     */
    uint16_t        frame_cccd_handle;
    
    /**
     * @brief 控制特征值（Control Characteristic）的句柄
     */
    uint16_t        ctrl_val_handle;
    
    /**
     * @brief 最后一次执行的命令编号
     */
    int             last_command;
    
    /**
     * @brief 发送互斥锁的信号量句柄
     */
    SemaphoreHandle_t send_lock;
    
    /**
     * @brief 发送数据缓冲区
     */
    uint8_t         send_buf[BLE_SEND_BUF_SIZE];
    
} ble_ctx_t;

static ble_ctx_t g_ble = {0};

/* ==================== 前向声明 ==================== */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param);

/* ==================== 回调 ==================== */

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        if (param->adv_data_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "广播数据配置失败: 0x%x", param->adv_data_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "广播数据配置完成，开始广播...");
        {
            esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "start_advertising 返回错误: 0x%x", ret);
            }
        }
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "开始广播失败: 0x%x", param->adv_start_cmpl.status);
        } else {
            ESP_LOGI(TAG, "广播已启动: %s", DEVICE_NAME);
        }
        break;

    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        g_ble.gatts_if = gatts_if;
        ESP_LOGI(TAG, "收到 REG_EVT，配置广播数据...");
        {
            esp_err_t r = esp_ble_gap_set_device_name(DEVICE_NAME);
            if (r != ESP_OK) ESP_LOGE(TAG, "set_device_name 失败: 0x%x", r);
            r = esp_ble_gap_config_adv_data(&adv_data);
            if (r != ESP_OK) ESP_LOGE(TAG, "config_adv_data 失败: 0x%x", r);
        }
        /* 创建属性表，框架会自动填充特征声明中的句柄与 UUID */
        esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, IDX_NB, PROFILE_APP_ID);
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "属性表创建失败: 0x%x", param->add_attr_tab.status);
            break;
        }
        g_ble.service_handle    = param->add_attr_tab.handles[IDX_SVC];
        g_ble.frame_val_handle  = param->add_attr_tab.handles[IDX_CHAR_FRAME_VAL];
        g_ble.frame_cccd_handle = param->add_attr_tab.handles[IDX_CHAR_FRAME_CFG];
        g_ble.ctrl_val_handle   = param->add_attr_tab.handles[IDX_CHAR_CTRL_VAL];
        esp_ble_gatts_start_service(g_ble.service_handle);
        break;

    case ESP_GATTS_CONNECT_EVT: {
        g_ble.connected = true;
        g_ble.conn_id   = param->connect.conn_id;
        g_ble.mtu       = BLE_MTU_DEFAULT;
        ESP_LOGI(TAG, "已连接 conn_id=%d", g_ble.conn_id);

        /* 请求更快的连接参数，提高吞吐 */
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.min_int = 0x10;   /* 20ms */
        conn_params.max_int = 0x20;   /* 40ms */
        conn_params.latency = 0;
        conn_params.timeout = 400;    /* 4s */
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }

    case ESP_GATTS_DISCONNECT_EVT:
        g_ble.connected      = false;
        g_ble.notify_enabled = false;
        g_ble.conn_id        = 0;
        ESP_LOGI(TAG, "已断开，重新广播");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_MTU_EVT:
        g_ble.mtu = param->mtu.mtu;
        ESP_LOGI(TAG, "MTU 协商: %d", g_ble.mtu);
        break;

    case ESP_GATTS_WRITE_EVT: {
        if (param->write.is_prep) {
            break;
        }
        uint16_t handle = param->write.handle;
        if (handle == g_ble.frame_cccd_handle && param->write.len == 2) {
            uint16_t cccd = param->write.value[0] | (param->write.value[1] << 8);
            g_ble.notify_enabled = (cccd & 0x0001) ? true : false;
            ESP_LOGI(TAG, "通知已%s", g_ble.notify_enabled ? "开启" : "关闭");
        } else if (handle == g_ble.ctrl_val_handle && param->write.len >= 1) {
            g_ble.last_command = param->write.value[0];
            ESP_LOGI(TAG, "收到控制命令: 0x%02x", g_ble.last_command);
        }
        break;
    }

    default:
        break;
    }
}

/* ==================== 公共函数 ==================== */

int ble_init(void) {
    esp_err_t ret;

    if (g_ble.initialized) {
        return BLE_OK;
    }

    /* NVS：Bluedroid 需要 (控制器随机地址等) */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init 失败: %d", ret);
        return BLE_ERR;
    }

    /* BLE-only 模式，释放经典蓝牙占用的内存 */
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "释放经典蓝牙内存失败: %d", ret);
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "控制器初始化失败: %d (内存不足?)", ret);
        return BLE_ERR;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "控制器使能失败: %d", ret);
        return BLE_ERR;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid 初始化失败: %d", ret);
        return BLE_ERR;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid 使能失败: %d", ret);
        return BLE_ERR;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "注册 GATTS 回调失败: %d", ret);
        return BLE_ERR;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "注册 GAP 回调失败: %d", ret);
        return BLE_ERR;
    }

    g_ble.send_lock = xSemaphoreCreateMutex();
    if (!g_ble.send_lock) {
        ESP_LOGE(TAG, "创建互斥锁失败");
        return BLE_ERR;
    }

    ret = esp_ble_gatts_app_register(PROFILE_APP_ID);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "注册应用失败: %d", ret);
        return BLE_ERR;
    }

    g_ble.initialized = true;
    g_ble.last_command = -1;
    ESP_LOGI(TAG, "BLE 初始化完成");
    return BLE_OK;
}

int ble_wait_connected(uint32_t timeout_ms) {
    uint32_t elapsed = 0;
    while (!g_ble.connected) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (timeout_ms != 0) {
            elapsed += 100;
            if (elapsed >= timeout_ms) {
                return BLE_ERR;
            }
        }
    }
    return BLE_OK;
}

bool ble_is_connected(void) {
    return g_ble.connected;
}

bool ble_is_ready(void) {
    return g_ble.connected && g_ble.notify_enabled;
}

int ble_send_frame(const uint8_t *data, size_t size) {
    if (!data || size == 0) {
        return BLE_ERR_PARAM;
    }
    if (!g_ble.connected) {
        return BLE_ERR_NOT_CONN;
    }
    if (!g_ble.notify_enabled) {
        return BLE_ERR_BUSY;
    }

    if (xSemaphoreTake(g_ble.send_lock, portMAX_DELAY) != pdTRUE) {
        return BLE_ERR_BUSY;
    }

    /* 计算单包最大负载：MTU - 3(ATT头) - 6(协议头) */
    uint16_t mtu = (g_ble.mtu >= BLE_MTU_DEFAULT) ? g_ble.mtu : BLE_MTU_DEFAULT;
    if (mtu > BLE_MTU_MAX) {
        mtu = BLE_MTU_MAX;
    }
    size_t max_payload = (size_t)mtu - 3 - BLE_HDR_SIZE;
    if (max_payload > BLE_SEND_BUF_SIZE - BLE_HDR_SIZE) {
        max_payload = BLE_SEND_BUF_SIZE - BLE_HDR_SIZE;
    }
    if (max_payload == 0) {
        max_payload = 1;
    }

    int result = BLE_OK;
    size_t offset = 0;
    uint16_t seq = 0;

    while (offset < size) {
        size_t chunk = size - offset;
        if (chunk > max_payload) {
            chunk = max_payload;
        }

        uint8_t flags = 0;
        if (offset == 0) {
            flags |= BLE_FLAG_START;
        }
        if (offset + chunk == size) {
            flags |= BLE_FLAG_END;
        }

        uint8_t *pkt = g_ble.send_buf;
        pkt[0] = BLE_SYNC;
        pkt[1] = flags;
        pkt[2] = (uint8_t)(seq & 0xFF);
        pkt[3] = (uint8_t)((seq >> 8) & 0xFF);
        pkt[4] = (uint8_t)(chunk & 0xFF);
        pkt[5] = (uint8_t)((chunk >> 8) & 0xFF);
        memcpy(pkt + BLE_HDR_SIZE, data + offset, chunk);

        uint16_t pkt_len = (uint16_t)(BLE_HDR_SIZE + chunk);

        esp_err_t err = ESP_FAIL;
        int tries = 0;
        do {
            err = esp_ble_gatts_send_indicate(g_ble.gatts_if, g_ble.conn_id,
                                              g_ble.frame_val_handle, pkt_len, pkt, false);
            if (err != ESP_OK) {
                /* 控制器通知队列满（ESP_FAIL），按连接间隔(20~40ms)等待队列排空后再试 */
                tries++;
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        } while (err != ESP_OK && tries < 100);  /* 最多约 2s */

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "发送第 %u 包失败: 0x%x", seq, err);
            result = BLE_ERR;
            break;
        }

        offset += chunk;
        seq++;
        /* 给控制器留出出队时间，避免积压 */
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (result == BLE_OK) {
        ESP_LOGI(TAG, "已发送一帧: %zu 字节, %u 包 (MTU=%d)", size, seq, mtu);
    }

    xSemaphoreGive(g_ble.send_lock);
    return result;
}

int ble_get_last_command(void) {
    return g_ble.last_command;
}

uint16_t ble_get_mtu(void) {
    return g_ble.mtu;
}

void ble_deinit(void) {
    if (!g_ble.initialized) {
        return;
    }

    esp_ble_gatts_app_unregister(g_ble.gatts_if);
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);

    if (g_ble.send_lock) {
        vSemaphoreDelete(g_ble.send_lock);
        g_ble.send_lock = NULL;
    }
    memset(&g_ble, 0, sizeof(g_ble));
    ESP_LOGI(TAG, "BLE 资源已释放");
}
