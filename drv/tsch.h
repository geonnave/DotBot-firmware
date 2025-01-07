#ifndef __TSCH_H
#define __TSCH_H

/**
 * @defgroup    drv_tsch      TSCH radio driver
 * @ingroup     drv
 * @brief       Driver for Time-Slotted Channel Hopping (TSCH)
 *
 * @{
 * @file
 * @author Geovane Fedrecheski <geovane.fedrecheski@inria.fr>
 * @copyright Inria, 2024-now
 * @}
 */

#include <stdint.h>
#include <stdlib.h>
#include <nrf.h>

//=========================== defines ==========================================

#define TSCH_TIMER_DEV 2 ///< HF timer device used for the TSCH scheduler
#define TSCH_TIMER_SLOT_CHANNEL 0
#define TSCH_TIMER_INTRA_SLOT_CHANNEL 1

//=========================== variables ========================================

typedef void (*tsch_cb_t)(uint8_t *packet, uint8_t length);  ///< Function pointer to the callback function called on packet receive

typedef enum {
    TSCH_RADIO_ACTION_SLEEP = 'S',
    TSCH_RADIO_ACTION_RX = 'R',
    TSCH_RADIO_ACTION_TX = 'T',
} tsch_radio_action_t;

// ==== BEGIN TODO: move to protocol.h, but that will be part of a larger refactoring ====
typedef enum {
    TSCH_PACKET_TYPE_BEACON = 1,
    TSCH_PACKET_TYPE_JOIN_REQUEST = 2,
    TSCH_PACKET_TYPE_JOIN_RESPONSE = 3,
    TSCH_PACKET_TYPE_INFRASTRUCTURE_DATA = 8,
    TSCH_PACKET_TYPE_EXPERIMENT_DATA = 9,
} tsch_packet_type_t;

// general packet header
typedef struct {
    uint8_t version;
    tsch_packet_type_t type;
    uint64_t dst;
    uint64_t src;
} tsch_packet_header_t;

// beacon packet
typedef struct {
    uint8_t version;
    tsch_packet_type_t type;
    uint64_t src;
    uint8_t remaining_capacity;
    uint8_t active_schedule_id;
    uint64_t asn;
} tsch_beacon_packet_header_t;
// ==== END TODO: move to protocol.h, but that will be part of a larger refactoring ======

typedef struct {
    tsch_radio_action_t radio_action;
    uint8_t frequency;
    uint32_t duration_us;
    char slot_type; // FIXME: only for debugging, remove before merge
} tsch_radio_event_t;

//=========================== prototypes ==========================================

/**
 * @brief Initializes the TSCH scheme
 *
 * @param[in] callback             pointer to a function that will be called each time a packet is received.
 *
 */
void db_tsch_init(tsch_cb_t callback);

#endif
