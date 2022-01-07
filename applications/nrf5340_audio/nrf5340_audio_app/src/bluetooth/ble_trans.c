/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "ble_trans.h"

#include <zephyr.h>
#include <errno.h>
#include <bluetooth/iso.h>
#include <bluetooth/bluetooth.h>
#include "macros_common.h"
#include "ctrl_events.h"
#include "ble_acl_common.h"
#include "sw_codec_select.h"
#include "ble_hci_vsc.h"
#include "ble_acl_gateway.h"

#include <logging/log.h>
LOG_MODULE_DECLARE(ble, CONFIG_LOG_BLE_LEVEL);

static enum iso_transport iso_trans_type;
static enum iso_direction iso_dir;

#define BLE_ISO_PAYLOAD_SIZE_MAX ENC_MAX_FRAME_SIZE

#define BLE_ISO_MTU (BT_ISO_CHAN_SEND_RESERVE + BLE_ISO_PAYLOAD_SIZE_MAX)
#define BLE_ISO_CONN_INTERVAL_US (BLE_ISO_CONN_INTERVAL * 1250)
#define BLE_ISO_LATENCY_MS 10
#define BLE_ISO_RETRANSMITS 2

#define CIS_ISO_CHAN_COUNT CONFIG_BT_ISO_MAX_CHAN
#define BIS_ISO_CHAN_COUNT 1
#define BLE_ISO_BIG_SYNC_TIMEOUT 50

#define CIS_CONN_RETRY_TIMES 5
#define HCI_ISO_BUF_ALLOC_PER_CHAN 2

#define NET_BUF_POOL_ITERATE(i, _)                                                                 \
	NET_BUF_POOL_FIXED_DEFINE(iso_tx_pool_##i, HCI_ISO_BUF_ALLOC_PER_CHAN,                     \
				  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), 8, NULL);
#define NET_BUF_POOL_PTR_ITERATE(i, _) &iso_tx_pool_##i,

UTIL_LISTIFY(CONFIG_BT_ISO_MAX_CHAN, NET_BUF_POOL_ITERATE)

static struct net_buf_pool *iso_tx_pools[] = { UTIL_LISTIFY(CONFIG_BT_ISO_MAX_CHAN,
							    NET_BUF_POOL_PTR_ITERATE) };
static struct bt_iso_chan iso_chan[CONFIG_BT_ISO_MAX_CHAN];
static struct bt_iso_chan *iso_chan_p[CONFIG_BT_ISO_MAX_CHAN];
static atomic_t iso_tx_pool_alloc[CONFIG_BT_ISO_MAX_CHAN];

struct worker_data {
	uint8_t channel;
	uint8_t retries;
} __aligned(4);

K_MSGQ_DEFINE(kwork_msgq, sizeof(struct worker_data), CONFIG_BT_ISO_MAX_CHAN, 4);

static K_SEM_DEFINE(sem_big_cmplt, 0, 1);
static K_SEM_DEFINE(sem_big_term, 0, 1);
static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_big_info, 0, 1);
static K_SEM_DEFINE(sem_big_sync, 0, 1);

static struct k_work_delayable iso_cis_conn_work;

#define BT_LE_SCAN_CUSTOM                                                                          \
	BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_PASSIVE, BT_LE_SCAN_OPT_NONE, BT_GAP_SCAN_FAST_INTERVAL,  \
			 BT_GAP_SCAN_FAST_WINDOW)

#define BT_LE_PER_ADV_CUSTOM                                                                       \
	BT_LE_PER_ADV_PARAM(BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2,                  \
			    BT_LE_PER_ADV_OPT_NONE)

/* Interval * 1.25 to get ms */
#define BT_INTERVAL_TO_MS(interval) ((interval)*5 / 4)
#define PA_RETRY_COUNT 6

static bool per_adv_found;
static bt_addr_le_t per_addr;
static uint8_t per_sid;
static uint16_t per_interval_ms;

static struct bt_le_ext_adv *adv;
static struct bt_iso_big *big;
static struct bt_le_per_adv_sync_param sync_create_param;
static struct bt_le_per_adv_sync *iso_sync;
static struct bt_iso_big *big;
static uint32_t sem_timeout;
static uint32_t iso_rx_cb_tot;
static uint32_t iso_rx_cb_bad;

static ble_trans_iso_rx_cb_t ble_trans_iso_rx_cb;
static int iso_bis_rx_sync_delete(void);
static int iso_bis_rx_sync_get(void);
static void iso_rx_stats_handler(struct k_timer *timer);

static uint8_t iso_chan_to_idx(struct bt_iso_chan *chan)
{
	if (chan == NULL) {
		ERR_CHK_MSG(-EINVAL, "chan is NULL");
	}

	for (uint8_t i = 0; i < CONFIG_BT_ISO_MAX_CHAN; i++) {
		if (chan == iso_chan_p[i]) {
			return i;
		}
	}

	ERR_CHK_MSG(-ENXIO, "No index found for this channel");
	CODE_UNREACHABLE;
	return UINT8_MAX;
}

K_TIMER_DEFINE(iso_rx_stats_timer, iso_rx_stats_handler, NULL);

static int ble_event_send(enum ble_evt_type ble_evt)
{
	struct event_t event;

	event.event_source = EVT_SRC_PEER;
	event.link_activity = ble_evt;
	return ctrl_events_put(&event);
}

static void scan_recv(const struct bt_le_scan_recv_info *info, struct net_buf_simple *buf)
{
	int ret;

	if (!per_adv_found && info->interval) {
		per_adv_found = true;

		per_sid = info->sid;
		per_interval_ms = BT_INTERVAL_TO_MS(info->interval);
		bt_addr_le_copy(&per_addr, info->addr);

		LOG_INF("Stop scanning...");
		ret = bt_le_scan_stop();
		ERR_CHK_MSG(ret, "bt_le_scan_stop failed");

		ret = ble_event_send(BLE_EVT_CONNECTED);
		ERR_CHK_MSG(ret, "Unable to put event BLE_EVT_CONNECTED in event queue");
		k_sem_give(&sem_per_adv);
	}
}

static struct bt_le_scan_cb scan_callbacks = {
	.recv = scan_recv,
};

static void sync_cb(struct bt_le_per_adv_sync *sync, struct bt_le_per_adv_sync_synced_info *info)
{
	k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_term_info *info)
{
	int ret;

	LOG_WRN("Lost sync");
	if (iso_dir == DIR_RX) {
		ret = iso_bis_rx_sync_delete();
		if (ret) {
			LOG_WRN("iso_bis_rx_sync_delete failed, ret = %d", ret);
		}
		ret = ble_event_send(BLE_EVT_DISCONNECTED);
		ERR_CHK_MSG(ret, "Unable to put event BLE_EVT_DISCONNECTED in event queue");
	}
}

static void recv_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_recv_info *info, struct net_buf_simple *buf)
{
	/* Currently not in use */
}

static void biginfo_cb(struct bt_le_per_adv_sync *sync, const struct bt_iso_biginfo *biginfo)
{
	k_sem_give(&sem_per_big_info);
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.recv = recv_cb,
	.biginfo = biginfo_cb,
};

static uint8_t num_iso_cis_connected(void)
{
	uint8_t num_cis_connected = 0;

	for (int i = 0; i < CIS_ISO_CHAN_COUNT; i++) {
		if (iso_chan_p[i]->state == BT_ISO_CONNECTED) {
			num_cis_connected++;
		}
	}
	return num_cis_connected;
}

static void iso_rx_stats_handler(struct k_timer *timer)
{
	float bad_pcnt;

	if (iso_rx_cb_bad == 0) {
		bad_pcnt = 0;
	} else {
		bad_pcnt = 100 * (float)iso_rx_cb_bad / iso_rx_cb_tot;
	}
	LOG_WRN("BLE ISO RX. tot: %d bad %d, Percent %1.1f", iso_rx_cb_tot, iso_rx_cb_bad,
		bad_pcnt);

	iso_rx_cb_tot = 0;
	iso_rx_cb_bad = 0;
}

static void iso_rx_cb(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		      struct net_buf *buf)
{
	bool bad_frame = false;

	iso_rx_cb_tot++;

	if (ble_trans_iso_rx_cb == NULL) {
		ERR_CHK_MSG(-EPERM, "The RX callback has not been set");
	}

	if (info->flags != BT_ISO_FLAGS_VALID) {
		bad_frame = true;
		iso_rx_cb_bad++;
	}

	ble_trans_iso_rx_cb(buf->data, buf->len, bad_frame, info->ts);
}

static void iso_connected_cb(struct bt_iso_chan *chan)
{
	LOG_DBG("ISO Channel %p connected", chan);

	atomic_clear(&iso_tx_pool_alloc[iso_chan_to_idx(chan)]);

	if (iso_trans_type == TRANS_TYPE_BIS) {
		k_sem_give(&sem_big_sync);
		k_sem_give(&sem_big_cmplt);
	} else if (iso_trans_type == TRANS_TYPE_CIS) {
		int ret;

		/* Only send BLE event on first connected CIS */
		if (num_iso_cis_connected() == 1) {
			ret = ble_event_send(BLE_EVT_CONNECTED);
			ERR_CHK_MSG(ret, "Unable to put event BLE_EVT_CONNECTED in event queue");

			ret = ble_event_send(BLE_EVT_LINK_READY);
			ERR_CHK_MSG(ret, "Unable to put event BLE_EVT_LINK_READY in event queue");
		}
	} else {
		LOG_ERR("iso_trans_type error");
	}
}

static void iso_disconnected_cb(struct bt_iso_chan *chan, uint8_t reason)
{
	int ret;

	if (iso_trans_type == TRANS_TYPE_BIS) {
		k_sem_give(&sem_big_term);
		if (reason != BT_HCI_ERR_LOCALHOST_TERM_CONN) {
			LOG_DBG("Not cancelled by local host");
			if (iso_dir == DIR_RX) {
				ret = iso_bis_rx_sync_delete();
				if (ret) {
					LOG_DBG("iso_bis_rx_sync_delete failed, ret = %d", ret);
				}
				ret = ble_event_send(BLE_EVT_DISCONNECTED);
				ERR_CHK_MSG(
					ret,
					"Unable to put event BLE_EVT_DISCONNECTED in event queue");
			}
		}
	} else if (iso_trans_type == TRANS_TYPE_CIS) {
#if (NRF5340_AUDIO_DEV == NRF5340_AUDIO_DEV_HEADSET)
		LOG_DBG("ISO CIS disconnected, reason %d", reason);
		ret = ble_event_send(BLE_EVT_DISCONNECTED);
		ERR_CHK_MSG(ret, "Unable to put event BLE_EVT_DISCONNECTED in event queue");
#elif (NRF5340_AUDIO_DEV == NRF5340_AUDIO_DEV_GATEWAY)
		for (int i = 0; i < CIS_ISO_CHAN_COUNT; i++) {
			if (chan == iso_chan_p[i]) {
				LOG_DBG("ISO CIS %d disconnected, reason %d", i, reason);
				break;
			}
		}
		/* If there's no ISO CIS channel connected,
		 * trigger BLE_EVT_DISCONNECTED for stopping encoding thread
		 */
		if (num_iso_cis_connected() == 0) {
			ret = ble_event_send(BLE_EVT_DISCONNECTED);
			ERR_CHK_MSG(ret, "Unable to put event BLE_EVT_DISCONNECTED in event queue");
		}
#endif /* (NRF5340_AUDIO_DEV == NRF5340_AUDIO_DEV_HEADSET) */
	} else {
		LOG_ERR("iso_trans_type not supported");
	}
}

/* Called when ISO TX is done and the buffer
 * has been freed
 */
static void iso_sent_cb(struct bt_iso_chan *chan)
{
	atomic_dec(&iso_tx_pool_alloc[iso_chan_to_idx(chan)]);
}

static struct bt_iso_chan_ops iso_ops = {
	.recv = iso_rx_cb,
	.connected = iso_connected_cb,
	.disconnected = iso_disconnected_cb,
	.sent = iso_sent_cb,
};

static struct bt_iso_chan_io_qos iso_tx_qos = {
	.sdu = BLE_ISO_PAYLOAD_SIZE_MAX, /* bytes */
	.rtn = BLE_ISO_RETRANSMITS,
	.phy = BT_GAP_LE_PHY_2M,
};

static struct bt_iso_chan_io_qos iso_rx_qos;
static struct bt_iso_chan_qos iso_bis_qos;

static struct bt_iso_big_create_param big_create_param = {
	.num_bis = BIS_ISO_CHAN_COUNT,
	.bis_channels = iso_chan_p,
	.interval = BLE_ISO_CONN_INTERVAL_US, /* in microseconds */
	.latency = BLE_ISO_LATENCY_MS, /* milliseconds */
	.packing = BT_ISO_PACKING_SEQUENTIAL,
	.framing = BT_ISO_FRAMING_UNFRAMED,
};

static struct bt_iso_big_sync_param big_sync_param = {
	.num_bis = BIS_ISO_CHAN_COUNT,
	.bis_channels = iso_chan_p,
	.bis_bitfield = (BIT_MASK(BIS_ISO_CHAN_COUNT) << 1),
	.mse = BT_ISO_SYNC_MSE_MIN,
	.sync_timeout = BLE_ISO_BIG_SYNC_TIMEOUT,
};

static struct bt_iso_chan_qos iso_cis_qos;

static struct bt_iso_cig_param cis_create_param = {
	.cis_channels = iso_chan_p,
	.num_cis = CIS_ISO_CHAN_COUNT,
	.sca = BT_GAP_SCA_UNKNOWN,
	.packing = BT_ISO_PACKING_SEQUENTIAL,
	.framing = BT_ISO_FRAMING_UNFRAMED,
	.latency = BLE_ISO_LATENCY_MS,
	.interval = BLE_ISO_CONN_INTERVAL_US,
};

static int iso_bis_rx_sync_delete(void)
{
	int ret;

	per_adv_found = false;

	if (iso_sync == NULL) {
		LOG_DBG("Already deleted iso_sync");
	}

	LOG_DBG("Deleting Periodic Advertising Sync...");

	ret = bt_le_per_adv_sync_delete(iso_sync);
	if (ret) {
		LOG_ERR("bt_le_per_adv_sync_delete failed: %d", ret);
	}

	iso_sync = NULL;

	return ret;
}

static int iso_bis_rx_sync_get(void)
{
	int ret;

	if (iso_sync != NULL) {
		LOG_DBG("Already have sync, returning");
		return 0;
	}

	per_adv_found = false;
	LOG_DBG("Start scanning...");
	ret = bt_le_scan_start(BT_LE_SCAN_CUSTOM, NULL);
	RET_IF_ERR_MSG(ret, "ble_le_scan_start failed");

	LOG_DBG("Waiting for periodic advertising...");

	ret = k_sem_take(&sem_per_adv, K_FOREVER);
	RET_IF_ERR_MSG(ret, "k_sem_take failed");

	LOG_DBG("Creating Periodic Advertising Sync...");
	bt_addr_le_copy(&sync_create_param.addr, &per_addr);
	sync_create_param.options = 0;
	sync_create_param.sid = per_sid;
	sync_create_param.skip = 0;
	sync_create_param.timeout = (per_interval_ms * PA_RETRY_COUNT) / 10;
	sem_timeout = per_interval_ms * PA_RETRY_COUNT;
	ret = bt_le_per_adv_sync_create(&sync_create_param, &iso_sync);
	RET_IF_ERR_MSG(ret, "bt_le_per_adv_sync_create failed");

	LOG_DBG("Waiting for periodic sync");
	k_sem_reset(&sem_per_sync);
	ret = k_sem_take(&sem_per_sync, K_MSEC(sem_timeout));
	RET_IF_ERR_MSG(ret, "sem_per_sync failed");

	LOG_DBG("Periodic sync established");
	k_sem_reset(&sem_per_big_info);
	ret = k_sem_take(&sem_per_big_info, K_MSEC(sem_timeout));
	RET_IF_ERR_MSG(ret, "sem_per_big_info failed");

	ret = ble_event_send(BLE_EVT_LINK_READY);
	ERR_CHK_MSG(ret, "Unable to put event BLE_EVT_LINK_READY in event queue");

	return 0;
}

static int iso_bis_rx_init(void)
{
	int ret;

	bt_le_scan_cb_register(&scan_callbacks);
	bt_le_per_adv_sync_cb_register(&sync_callbacks);

	ret = iso_bis_rx_sync_get();
	RET_IF_ERR_MSG(ret, "Sync lost before enter streaming state");

	return 0;
}

static void iso_bis_rx_cleanup(void)
{
	int ret;

	ret = iso_bis_rx_sync_delete();
	if (ret) {
		LOG_WRN("Failed to clean sync, ret = %d", ret);
	}

	ret = bt_iso_big_terminate(big);
	if (ret) {
		LOG_WRN("Failed to terminate BIG, ret = %d", ret);
	} else {
		big = NULL;
	}
}

static int iso_bis_rx_start(void)
{
	int ret;

	ret = iso_bis_rx_sync_get();
	if (ret) {
		LOG_WRN("iso_bis_rx_sync_get failed");
		iso_bis_rx_sync_delete();
		return ret;
	}

	LOG_DBG("Trying to get sync...");
	/* Set the BIS we want to sync to */

	ret = bt_iso_big_sync(iso_sync, &big_sync_param, &big);
	if (ret) {
		LOG_ERR("bt_iso_big_sync failed");
		iso_bis_rx_cleanup();
		return ret;
	}

	ret = k_sem_take(&sem_big_sync, K_MSEC(sem_timeout));
	if (ret) {
		LOG_ERR("k_sem_take sem_big_sync failed");
		iso_bis_rx_cleanup();
		return ret;
	}

	LOG_DBG("bt_iso_big_sync established");

	ret = ble_event_send(BLE_EVT_STREAMING);
	ERR_CHK_MSG(ret, "Unable to put event BLE_EVT_STREAMING in event queue");

	return 0;
}

static int iso_bis_rx_stop(void)
{
	int ret;

	ret = bt_iso_big_terminate(big);
	RET_IF_ERR(ret);

	LOG_DBG("Waiting for BIG terminate complete...");
	ret = k_sem_take(&sem_big_term, K_FOREVER);
	RET_IF_ERR(ret);
	LOG_DBG("BIG terminate completed");

	return 0;
}

static int iso_bis_tx_init(void)
{
	int ret;

	/* Create a non-connectable non-scannable advertising set */
	ret = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN_NAME, NULL, &adv);
	RET_IF_ERR_MSG(ret, "Failed to create advertising set");

	return 0;
}

static int iso_bis_tx_start(void)
{
	int ret;

	/* Set periodic advertising parameters */
	ret = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_CUSTOM);
	RET_IF_ERR_MSG(ret, "Failed to set periodic advertising parameters");

	/* Enable Periodic Advertising */
	ret = bt_le_per_adv_start(adv);
	RET_IF_ERR_MSG(ret, "Failed to enable periodic advertising");

	/* Start extended advertising */
	ret = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	RET_IF_ERR_MSG(ret, "Failed to start extended advertising");

	ret = ble_event_send(BLE_EVT_CONNECTED);
	ERR_CHK_MSG(ret, "Unable to put event BLE_EVT_CONNECTED in event queue");

	/* Create BIG */
	ret = bt_iso_big_create(adv, &big_create_param, &big);
	RET_IF_ERR_MSG(ret, "Failed to create BIG");

	LOG_DBG("Waiting for BIG complete");
	ret = k_sem_take(&sem_big_cmplt, K_FOREVER);
	RET_IF_ERR(ret);

	ret = ble_event_send(BLE_EVT_LINK_READY);
	ERR_CHK_MSG(ret, "Unable to put event BLE_EVT_LINK_READY in event queue");

	LOG_DBG("ISO Create done");

	return 0;
}

static int iso_bis_tx_stop(void)
{
	int ret;

	ret = bt_iso_big_terminate(big);
	RET_IF_ERR(ret);

	LOG_DBG("Waiting for BIG terminate complete...");
	ret = k_sem_take(&sem_big_term, K_MSEC(sem_timeout));
	if (ret == -EAGAIN) {
		LOG_WRN("sem_big_term timeout");
		return ret;
	}

	LOG_DBG("Stop advertising");
	ret = bt_le_ext_adv_stop(adv);
	RET_IF_ERR(ret);

	LOG_DBG("Disable periodic advertising");
	ret = bt_le_per_adv_stop(adv);
	RET_IF_ERR(ret);

	return 0;
}

static int iso_bis_start(enum iso_direction dir)
{
	switch (dir) {
	case DIR_RX:
		return iso_bis_rx_start();
	case DIR_TX:
		return iso_bis_tx_start();
	default:
		LOG_WRN("Invalid direction supplied: %d", dir);
		return -EPERM;
	};
}

static void work_iso_cis_conn(struct k_work *work)
{
	int ret;
	struct bt_iso_connect_param connect_param;
	struct worker_data work_data;

	ret = k_msgq_get(&kwork_msgq, &work_data, K_NO_WAIT);
	ERR_CHK(ret);

	ret = ble_acl_gateway_conn_peer_get(work_data.channel, &connect_param.acl);
	ERR_CHK_MSG(ret, "Connection peer get error");
	connect_param.iso_chan = iso_chan_p[work_data.channel];

	ret = bt_iso_chan_connect(&connect_param, 1);
	work_data.retries++;
	if (ret) {
		if (work_data.retries < CIS_CONN_RETRY_TIMES) {
			LOG_WRN("Got connect error from ch %d Retrying. code: %d count: %d",
				work_data.channel, ret, work_data.retries);
			ret = k_msgq_put(&kwork_msgq, &work_data, K_NO_WAIT);
			ERR_CHK(ret);
			/* Delay added to prevent controller overloading */
			k_work_reschedule(&iso_cis_conn_work, K_MSEC(500));
		} else {
			LOG_ERR("Could not connect ch %d after %d retries", work_data.channel,
				work_data.retries);
			bt_conn_disconnect(connect_param.acl, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		}
	}
}

#if (NRF5340_AUDIO_DEV == NRF5340_AUDIO_DEV_HEADSET) && (CONFIG_TRANSPORT_CIS)
static int iso_accept(const struct bt_iso_accept_info *info, struct bt_iso_chan **chan)
{
	/* Since a CIS headset only will connect to one other device, the
	 * first channel will always be used
	 */
	if (iso_chan_p[0]->iso) {
		LOG_ERR("No channels available\n");
		return -ENOMEM;
	}
	*chan = iso_chan_p[0];
	return 0;
}

static struct bt_iso_server iso_server = {
	.sec_level = BT_SECURITY_L0,
	.accept = iso_accept,
};
#endif /* (NRF5340_AUDIO_DEV == NRF5340_AUDIO_DEV_HEADSET) && (CONFIG_TRANSPORT_CIS) */

static bool is_iso_buffer_full(uint8_t iso_chan_idx)
{
	/* net_buf_alloc allocates buffers for APP->NET transfer over HCI RPMsg.
	 * The iso_sent_cb is called when a buffer/buffers have been released,
	 * i.e. data transfer to the NET core has been completed.
	 * Data will be discarded if allocation becomes too high.
	 * If the NET and APP core operates in clock sync, discarding should not occur.
	 */

	if (atomic_get(&iso_tx_pool_alloc[iso_chan_idx]) >= HCI_ISO_BUF_ALLOC_PER_CHAN) {
		return true;
	}
	return false;
}

static int iso_tx(uint8_t const *const data, size_t size, uint8_t iso_chan_idx)
{
	int ret;
	static bool wrn_printed[CONFIG_BT_ISO_MAX_CHAN];
	struct net_buf *net_buffer;

	if (is_iso_buffer_full(iso_chan_idx)) {
		if (!wrn_printed[iso_chan_idx]) {
			LOG_WRN("HCI ISO TX overrun on ch %d. Single print", iso_chan_idx);
			wrn_printed[iso_chan_idx] = true;
		}
		return -ENOMEM;
	}

	wrn_printed[iso_chan_idx] = false;

	net_buffer = net_buf_alloc(iso_tx_pools[iso_chan_idx], K_NO_WAIT);
	if (net_buffer == NULL) {
		LOG_ERR("No net buf available");
		return -ENOMEM;
	}

	atomic_inc(&iso_tx_pool_alloc[iso_chan_idx]);
	/* Headroom reserved for stack use */
	net_buf_reserve(net_buffer, BT_ISO_CHAN_SEND_RESERVE);
	net_buf_add_mem(net_buffer, data, size);

	ret = bt_iso_chan_send(iso_chan_p[iso_chan_idx], net_buffer);
	if (ret < 0) {
		LOG_ERR("Unable to send ISO data: %d", ret);
		net_buf_unref(net_buffer);
		return ret;
	}
	return 0;
}

static int iso_tx_pattern(size_t size, uint8_t iso_chan_idx)
{
	int ret;
	static uint8_t test_pattern_data[CONFIG_BT_ISO_MAX_CHAN][ENC_MAX_FRAME_SIZE];
	static uint8_t test_pattern_value[CONFIG_BT_ISO_MAX_CHAN];

	if (iso_chan_idx >= CONFIG_BT_ISO_MAX_CHAN) {
		RET_IF_ERR_MSG(-EIO, "Unknown channel");
	}

	memset(test_pattern_data[iso_chan_idx], test_pattern_value[iso_chan_idx],
	       ENC_MAX_FRAME_SIZE);
	ret = iso_tx(test_pattern_data[iso_chan_idx], size, iso_chan_idx);

	if (ret == 0) {
		if (test_pattern_value[iso_chan_idx] == UINT8_MAX) {
			test_pattern_value[iso_chan_idx] = 0;
		} else {
			test_pattern_value[iso_chan_idx]++;
		}
	}

	return ret;
}

static int iso_tx_data_or_pattern(uint8_t const *const data, size_t size, uint8_t iso_chan_idx)
{
	int ret;

	if (iso_chan_p[iso_chan_idx]->state != BT_ISO_CONNECTED) {
		LOG_DBG("ISO channel %d not connected", iso_chan_idx);
		return 0;
	}

	if (IS_ENABLED(CONFIG_BLE_ISO_TEST_PATTERN)) {
		ret = iso_tx_pattern(size, iso_chan_idx);
	} else {
		ret = iso_tx(data, size, iso_chan_idx);
	}

	return ret;
}

int ble_trans_iso_lost_notify_enable(void)
{
	return ble_hci_vsc_set_op_flag(BLE_HCI_VSC_OP_ISO_LOST_NOTIFY, 1);
}

int ble_trans_iso_tx(uint8_t const *const data, size_t size, enum ble_trans_chan_type chan_type)
{
	int ret = 0;

	/* BIS defaults to channel 0 */
	if (iso_trans_type == TRANS_TYPE_BIS) {
		ret = iso_tx_data_or_pattern(data, size, BLE_TRANS_CHANNEL_LEFT);
		RET_IF_ERR(ret);
		return 0;
	}

	switch (chan_type) {
	case BLE_TRANS_CHANNEL_STEREO:
		ret = iso_tx_data_or_pattern(data, size / 2, BLE_TRANS_CHANNEL_LEFT);
		RET_IF_ERR(ret);
		ret = iso_tx_data_or_pattern(&data[size / 2], size / 2, BLE_TRANS_CHANNEL_RIGHT);
		RET_IF_ERR(ret);
		break;
	case BLE_TRANS_CHANNEL_RETURN_MONO:
		ret = iso_tx_data_or_pattern(data, size, BLE_TRANS_CHANNEL_RETURN_MONO);
		RET_IF_ERR(ret);
		break;
	default:
		return -EPERM;
	}

	return ret;
}

int ble_trans_iso_start(void)
{
	switch (iso_trans_type) {
	case TRANS_TYPE_NOT_SET:
		LOG_ERR("Transport type not set");
		return -EPERM;
	case TRANS_TYPE_BIS:
		return iso_bis_start(iso_dir);
	case TRANS_TYPE_CIS:
		/* The ISO CIS channel established automatically
		 * after ACL connected, so ISO CIS is ready for use.
		 * And once ISO CIS channel established,
		 * we maintain it unless ACL disconnected for now.
		 */
		return 0;
	default:
		return -EPERM;
	};

	return 0;
}

int ble_trans_iso_stop(void)
{
	if (iso_trans_type == TRANS_TYPE_BIS) {
		switch (iso_dir) {
		case DIR_RX:
			return iso_bis_rx_stop();
		case DIR_TX:
			return iso_bis_tx_stop();
		default:
			return -EPERM;
		};
	} else if (iso_trans_type == TRANS_TYPE_CIS) {
		return 0;
	} else {
		return -EPERM;
	}
}

int ble_trans_iso_cig_create(void)
{
	int ret;

	struct bt_iso_cig *cig;

	ret = bt_iso_cig_create(&cis_create_param, &cig);
	if (ret) {
		LOG_ERR("Failed to create CIG (%d)\n", ret);
		return ret;
	}

	return 0;
}

int ble_trans_iso_cis_connect(struct bt_conn *conn)
{
	int ret;
	struct bt_conn *conn_active;

	for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		ret = ble_acl_gateway_conn_peer_get(i, &conn_active);
		ERR_CHK_MSG(ret, "Connection peer get error");
		if (conn == conn_active) {
			struct worker_data work_data;

			work_data.channel = i;
			work_data.retries = 0;
			ret = k_msgq_put(&kwork_msgq, &work_data, K_NO_WAIT);
			RET_IF_ERR(ret);
			k_work_schedule(&iso_cis_conn_work, K_MSEC(500 * i));
		}
	}

	return 0;
}

int ble_trans_iso_bis_rx_sync_get(void)
{
	return iso_bis_rx_sync_get();
}

int ble_trans_iso_init(enum iso_transport trans_type, enum iso_direction dir,
		       ble_trans_iso_rx_cb_t rx_cb)
{
	int ret;

	if (iso_dir != DIR_NOT_SET) {
		return -EINVAL;
	}

	iso_dir = dir;

	if (iso_trans_type != TRANS_TYPE_NOT_SET) {
		return -EPERM;
	}

	for (int8_t i = 0; i < CONFIG_BT_ISO_MAX_CHAN; i++) {
		iso_chan_p[i] = &iso_chan[i];
	}

	iso_trans_type = trans_type;

	switch (iso_trans_type) {
	case TRANS_TYPE_NOT_SET:
		LOG_ERR("NOT SET");
		return -EPERM;
	case TRANS_TYPE_BIS:
		switch (iso_dir) {
		case DIR_RX:
			iso_bis_qos.rx = &iso_rx_qos;
			iso_bis_qos.tx = NULL;
			ret = iso_bis_rx_init();
			RET_IF_ERR(ret);
			break;
		case DIR_TX:
			iso_bis_qos.rx = NULL;
			iso_bis_qos.tx = &iso_tx_qos;
			ret = iso_bis_tx_init();
			RET_IF_ERR(ret);
			break;
		default:
			return -EPERM;
		};

		iso_chan_p[0]->ops = &iso_ops;
		iso_chan_p[0]->qos = &iso_bis_qos;
		break;
	case TRANS_TYPE_CIS:
		k_work_init_delayable(&iso_cis_conn_work, work_iso_cis_conn);

		for (int i = 0; i < CIS_ISO_CHAN_COUNT; i++) {
			iso_chan_p[i]->ops = &iso_ops;
			iso_chan_p[i]->qos = &iso_cis_qos;
		}
		switch (iso_dir) {
		case DIR_RX:
			/* Use the same setting between TX and RX */
			memcpy(&iso_rx_qos, &iso_tx_qos, sizeof(iso_tx_qos));
			iso_cis_qos.rx = &iso_rx_qos;
			iso_cis_qos.tx = NULL;
			break;
		case DIR_TX:
			iso_cis_qos.rx = NULL;
			iso_cis_qos.tx = &iso_tx_qos;
			break;
		case DIR_BIDIR:
			/* Use the same setting between TX and RX */
			memcpy(&iso_rx_qos, &iso_tx_qos, sizeof(iso_tx_qos));
			iso_cis_qos.rx = &iso_rx_qos;
			iso_cis_qos.tx = &iso_tx_qos;
			break;
		default:
			return -EPERM;
		};
		break;
	default:
		return -EPERM;
	};
	ble_trans_iso_rx_cb = rx_cb;

#if (NRF5340_AUDIO_DEV == NRF5340_AUDIO_DEV_HEADSET) && (CONFIG_TRANSPORT_CIS)
	ret = bt_iso_server_register(&iso_server);
	RET_IF_ERR_MSG(ret, "Unable to register ISO server");
#endif /* (NRF5340_AUDIO_DEV == NRF5340_AUDIO_DEV_HEADSET) && (CONFIG_TRANSPORT_CIS) */

	if (CONFIG_BLE_ISO_RX_STATS_S != 0) {
		k_timer_start(&iso_rx_stats_timer, K_SECONDS(0),
			      K_SECONDS(CONFIG_BLE_ISO_RX_STATS_S));
	}

	LOG_DBG("Init finished");
	return 0;
}
