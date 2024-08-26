/**
 * @file
 * @defgroup project_dotbot    DotBot application
 * @ingroup projects
 * @brief This is the radio-controlled DotBot app
 *
 * The remote control can be either a keyboard, a joystick or buttons on the gateway
 * itself
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 */

#include <nrf.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
// Include BSP headers
#include "board.h"
#include "board_config.h"
#include "device.h"
#include "lh2.h"
#include "protocol.h"
#include "motors.h"
#include "radio.h"
#include "rgbled_pwm.h"
#include "timer.h"
#include "log_flash.h"
#include "lakers.h"

#ifdef CRYPTO_PSA
extern void mbedtls_memory_buffer_alloc_init(uint8_t *buf, size_t len);
#endif

//=========================== defines ==========================================

#define TIMER_DEV                 (0)
#define DB_LH2_UPDATE_DELAY_MS    (100U)   ///< 100ms delay between each LH2 data refresh
#define DB_ADVERTIZEMENT_DELAY_MS (500U)   ///< 500ms delay between each advertizement packet sending
#define DB_TIMEOUT_CHECK_DELAY_MS (200U)   ///< 200ms delay between each timeout delay check
#define TIMEOUT_CHECK_DELAY_TICKS (17000)  ///< ~500 ms delay between packet received timeout checks
#define DB_LH2_COUNTER_MASK       (0x07)   ///< Maximum number of lh2 iterations without value received
#define DB_BUFFER_MAX_BYTES       (255U)   ///< Max bytes in UART receive buffer
#define DB_DIRECTION_THRESHOLD    (0.01)   ///< Threshold to update the direction
#define DB_DIRECTION_INVALID      (-1000)  ///< Invalid angle e.g out of [0, 360] range
#define DB_MAX_SPEED              (60)     ///< Max speed in autonomous control mode
#if defined(BOARD_DOTBOT_V2)
#define DB_REDUCE_SPEED_FACTOR  (0.7)  ///< Reduction factor applied to speed when close to target or error angle is too large
#define DB_REDUCE_SPEED_ANGLE   (25)   ///< Max angle amplitude where speed reduction factor is applied
#define DB_ANGULAR_SPEED_FACTOR (35)   ///< Constant applied to the normalized angle to target error
#define DB_ANGULAR_SIDE_FACTOR  (-1)   ///< Angular side factor
#else                                  // BOARD_DOTBOT_V1
#define DB_REDUCE_SPEED_FACTOR  (0.9)  ///< Reduction factor applied to speed when close to target or error angle is too large
#define DB_REDUCE_SPEED_ANGLE   (20)   ///< Max angle amplitude where speed reduction factor is applied
#define DB_ANGULAR_SPEED_FACTOR (30)   ///< Constant applied to the normalized angle to target error
#define DB_ANGULAR_SIDE_FACTOR  (1)    ///< Angular side factor
#endif
#define EDHOC_INITIATOR_INDEX 0

typedef struct {
    uint32_t                 ts_last_packet_received;            ///< Last timestamp in microseconds a control packet was received
    db_lh2_t                 lh2;                                ///< LH2 device descriptor
    uint8_t                  radio_buffer[DB_BUFFER_MAX_BYTES];  ///< Internal buffer that contains the command to send (from buttons)
    protocol_lh2_location_t  last_location;                      ///< Last computed LH2 location received
    int16_t                  direction;                          ///< Current direction of the DotBot (angle in °)
    protocol_control_mode_t  control_mode;                       ///< Remote control mode
    protocol_lh2_waypoints_t waypoints;                          ///< List of waypoints
    uint32_t                 waypoints_threshold;                ///< Distance to target waypoint threshold
    uint8_t                  next_waypoint_idx;                  ///< Index of next waypoint to reach
    bool                     update_control_loop;                ///< Whether the control loop need an update
    bool                     advertize;                          ///< Whether an advertize packet should be sent
    bool                     update_lh2;                         ///< Whether LH2 data must be processed
    uint8_t                  lh2_update_counter;                 ///< Counter used to track when lh2 data were received and to determine if an advertizement packet is needed
    uint64_t                 device_id;                          ///< Device ID of the DotBot
    db_log_dotbot_data_t     log_data;
    // edhoc stuff
    bool                     update_edhoc;                       ///< Whether EDHOC data must be processed
    bool                     gateway_authenticated;              ///< Whether the gateway has been authenticated
    EdhocMessageBuffer       edhoc_buffer;                       ///< Internal buffer to store received but not yet handled edhoc messages
    uint8_t                  prk_out[SHA256_DIGEST_LEN];
} dotbot_vars_t;

//=========================== variables ========================================

static dotbot_vars_t _dotbot_vars;

#ifdef DB_RGB_LED_PWM_RED_PORT  // Only available on DotBot v2
static const db_rgbled_pwm_conf_t rgbled_pwm_conf = {
    .pwm  = 1,
    .pins = {
        { .port = DB_RGB_LED_PWM_RED_PORT, .pin = DB_RGB_LED_PWM_RED_PIN },
        { .port = DB_RGB_LED_PWM_GREEN_PORT, .pin = DB_RGB_LED_PWM_GREEN_PIN },
        { .port = DB_RGB_LED_PWM_BLUE_PORT, .pin = DB_RGB_LED_PWM_BLUE_PIN },
    }
};
#endif

// for EDHOC
static const uint8_t CRED_I[2][100] = {
  {0xa2, 0x02, 0x50, 0x85, 0xc1, 0xec, 0x21, 0xf2, 0x6f, 0x41, 0xe7, 0xa3, 0x0a, 0x8a, 0x87, 0xbd, 0xbe, 0xf2, 0x3c, 0x08, 0xa1, 0x01, 0xa5, 0x01, 0x02, 0x02, 0x41, 0x01, 0x20, 0x01, 0x21, 0x58, 0x20, 0x52, 0x7c, 0x4d, 0x4c, 0x08, 0x9f, 0x9f, 0xe3, 0x33, 0x56, 0xaa, 0x97, 0xa1, 0xd6, 0x72, 0xda, 0x32, 0xc1, 0x60, 0x08, 0x24, 0x4f, 0xef, 0x37, 0xf0, 0x71, 0x54, 0xe0, 0x70, 0xe6, 0x6d, 0x1f, 0x22, 0x58, 0x20, 0x32, 0xe4, 0x6c, 0x45, 0xc4, 0xdd, 0xcb, 0x6d, 0x6c, 0x52, 0x4f, 0x37, 0x9d, 0x57, 0x15, 0x9d, 0x64, 0x2d, 0xd7, 0xf0, 0x27, 0x9c, 0x45, 0x50, 0xe3, 0x44, 0x48, 0xda, 0xc4, 0x19, 0x53, 0x2c},
  {0xa2, 0x02, 0x50, 0x4f, 0x2c, 0xed, 0x1b, 0x29, 0x17, 0x4d, 0xfe, 0x91, 0x55, 0x18, 0xc7, 0x23, 0x4d, 0x2c, 0xe3, 0x08, 0xa1, 0x01, 0xa5, 0x01, 0x02, 0x02, 0x41, 0x02, 0x20, 0x01, 0x21, 0x58, 0x20, 0xed, 0x47, 0xd7, 0xb6, 0xd0, 0x0c, 0x41, 0x4b, 0xa9, 0xfe, 0x1c, 0x9e, 0x6d, 0x2b, 0x07, 0x85, 0x45, 0x14, 0x36, 0x76, 0x6d, 0x5c, 0x0e, 0x65, 0xf3, 0xd7, 0xe3, 0x3b, 0x0d, 0x35, 0x4a, 0xd6, 0x22, 0x58, 0x20, 0x44, 0x3e, 0xda, 0x79, 0x2f, 0x81, 0x88, 0x44, 0xc8, 0x86, 0xbd, 0x1e, 0xc6, 0xfa, 0x0b, 0xd3, 0x61, 0xf8, 0xaa, 0xc9, 0xa8, 0xbc, 0xc2, 0x28, 0x65, 0x02, 0xaa, 0x9e, 0xb9, 0xea, 0xbb, 0xf4},
};
static const BytesP256ElemLen I[2] = {
  {0x1f, 0x7e, 0x4a, 0xe4, 0x29, 0x3a, 0x34, 0x8b, 0xf2, 0xb1, 0x36, 0x5c, 0xe0, 0x98, 0xaa, 0x49, 0xc2, 0x07, 0xbd, 0x1b, 0xa7, 0xdd, 0xde, 0xcd, 0xfa, 0xd6, 0x0c, 0xad, 0xe8, 0x2e, 0x9e, 0xf5},
  {0x3c, 0xa8, 0x54, 0xbf, 0xaa, 0x90, 0xda, 0x16, 0xe1, 0xa8, 0xfa, 0xcc, 0x0c, 0xd8, 0x34, 0x92, 0x7e, 0xc0, 0xb3, 0x19, 0x74, 0x8b, 0xb4, 0x79, 0xf1, 0x31, 0x6b, 0x8d, 0x38, 0x30, 0x74, 0xa8},
};

// for EAD authz
static const uint8_t ID_U[2][4] = {
  {0xa1, 0x04, 0x41, 0x01},
  {0xa1, 0x04, 0x41, 0x02},
};
static const size_t ID_U_LEN = sizeof(ID_U[EDHOC_INITIATOR_INDEX]) / sizeof(ID_U[EDHOC_INITIATOR_INDEX][0]);
static const BytesP256ElemLen G_W = {0xFF, 0xA4, 0xF1, 0x02, 0x13, 0x40, 0x29, 0xB3, 0xB1, 0x56, 0x89, 0x0B, 0x88, 0xC9, 0xD9, 0x61, 0x95, 0x01, 0x19, 0x65, 0x74, 0x17, 0x4D, 0xCB, 0x68, 0xA0, 0x7D, 0xB0, 0x58, 0x8E, 0x4D, 0x41};
static const uint8_t LOC_W[] = "http://localhost:18000";
static const uint8_t LOC_W_LEN = (sizeof(LOC_W) / sizeof(LOC_W[0])) - 1; // -1 to discard the \0 at the end
static const uint8_t SS = 2;

// used during execution of EDHOC
static CredentialC cred_i = {0}, fetched_cred_r = {0};
static IdCred id_cred_r = {0};
static EdhocInitiator initiator = {0};
static EdhocMessageBuffer message_1 = {0};
static uint8_t c_r = 0;
static EdhocMessageBuffer message_2 = {0};
static EdhocMessageBuffer message_3 = {0};
static uint8_t prk_out[SHA256_DIGEST_LEN] = {0};
// used during execution of authz
static EadAuthzDevice device = {0};
static EADItemC ead_1 = {0}, ead_2 = {0};
static BytesP256ElemLen authz_secret = {0};

// measuring authz
static const gpio_t ot1 = { .port = 0, .pin = 20 };
static const gpio_t ot2 = { .port = 0, .pin = 25 };

#define AUTHZ_MEASURE 2 // 0 = off, 1 = cpu edhoc+authz, 2 = radio tx/rx time

static inline void begin_measure_cpu(gpio_t *pin) {
  if (AUTHZ_MEASURE == 1) db_gpio_set(pin);
}
static inline void begin_measure_radio(gpio_t *pin) {
  if (AUTHZ_MEASURE == 2) db_gpio_set(pin);
}

static inline void end_measure_cpu(gpio_t *pin) {
  if (AUTHZ_MEASURE == 1) db_gpio_clear(pin);
}
static inline void end_measure_radio(gpio_t *pin) {
  if (AUTHZ_MEASURE == 2) db_gpio_clear(pin);
}

//=========================== prototypes =======================================

static void _timeout_check(void);
static void _advertise(void);
static void _compute_angle(const protocol_lh2_location_t *next, const protocol_lh2_location_t *origin, int16_t *angle);
static void _update_control_loop(void);
static void _update_lh2(void);

//=========================== callbacks ========================================

static void radio_callback(uint8_t *pkt, uint8_t len) {
    //(void)len;
    puts("CALLBACK called");
    if (!_dotbot_vars.gateway_authenticated && len == 254) {
        // BENCH: packet about to be received...
        //puts("BEGIN packet");
        begin_measure_radio(&ot1);
        return;
    }
    //puts("END packet");
    end_measure_radio(&ot1);

    _dotbot_vars.ts_last_packet_received = db_timer_ticks(TIMER_DEV);
    uint8_t           *ptk_ptr           = pkt;
    protocol_header_t *header            = (protocol_header_t *)ptk_ptr;
    // Check destination address matches
    if (header->dst != DB_BROADCAST_ADDRESS && header->dst != _dotbot_vars.device_id) {
        return;
    }

    // Check version is supported
    if (header->version != DB_FIRMWARE_VERSION) {
        return;
    }

    // Check application is compatible
    if (header->application != DotBot) {
        return;
    }

    // Check gateway is authenticated (only proceed if it is an edhoc message)
    if (!_dotbot_vars.gateway_authenticated && header->type != DB_PROTOCOL_EDHOC_MSG) {
        return;
    }

    uint8_t *cmd_ptr = ptk_ptr + sizeof(protocol_header_t);
    // parse received packet and update the motors' speeds
    switch (header->type) {
        case DB_PROTOCOL_CMD_MOVE_RAW:
        {
            protocol_move_raw_command_t *command = (protocol_move_raw_command_t *)cmd_ptr;
            int16_t                      left    = (int16_t)(100 * ((float)command->left_y / INT8_MAX));
            int16_t                      right   = (int16_t)(100 * ((float)command->right_y / INT8_MAX));
            db_motors_set_speed(left, right);
        } break;
        case DB_PROTOCOL_CMD_RGB_LED:
        {
            protocol_rgbled_command_t *command = (protocol_rgbled_command_t *)cmd_ptr;
            db_rgbled_pwm_set_color(command->r, command->g, command->b);
        } break;
        case DB_PROTOCOL_LH2_LOCATION:
        {
            const protocol_lh2_location_t *location = (const protocol_lh2_location_t *)cmd_ptr;
            int16_t                        angle    = -1000;
            _compute_angle(location, &_dotbot_vars.last_location, &angle);
            if (angle != DB_DIRECTION_INVALID) {
                _dotbot_vars.last_location.x = location->x;
                _dotbot_vars.last_location.y = location->y;
                _dotbot_vars.last_location.z = location->z;
                _dotbot_vars.direction       = angle;
            }
            _dotbot_vars.update_control_loop = (_dotbot_vars.control_mode == ControlAuto);
        } break;
        case DB_PROTOCOL_CONTROL_MODE:
            db_motors_set_speed(0, 0);
            break;
        case DB_PROTOCOL_LH2_WAYPOINTS:
        {
            db_motors_set_speed(0, 0);
            _dotbot_vars.control_mode        = ControlManual;
            _dotbot_vars.waypoints.length    = (uint8_t)*cmd_ptr++;
            _dotbot_vars.waypoints_threshold = (uint32_t)((uint8_t)*cmd_ptr++ * 1000);
            memcpy(&_dotbot_vars.waypoints.points, cmd_ptr, _dotbot_vars.waypoints.length * sizeof(protocol_lh2_location_t));
            _dotbot_vars.next_waypoint_idx = 0;
            if (_dotbot_vars.waypoints.length > 0) {
                _dotbot_vars.control_mode = ControlAuto;
            }
        } break;
        case DB_PROTOCOL_EDHOC_MSG:
        {
            uint8_t buffer_len = len - sizeof(protocol_header_t) - 2; // why -2?
            memcpy(_dotbot_vars.edhoc_buffer.content, cmd_ptr, buffer_len);
            _dotbot_vars.edhoc_buffer.len = buffer_len;
            _dotbot_vars.update_edhoc = true;
        } break;
        default:
            break;
    }
}

//=========================== main =============================================

int main(void) {
    db_board_init();

    // benchmarking code
    db_gpio_init(&ot1, DB_GPIO_OUT);
    db_gpio_init(&ot2, DB_GPIO_OUT);
    begin_measure_cpu(&ot2);

#ifdef ENABLE_DOTBOT_LOG_DATA
    db_log_flash_init(LOG_DATA_DOTBOT);
#endif
    db_protocol_init();
#ifdef DB_RGB_LED_PWM_RED_PORT
    db_rgbled_pwm_init(&rgbled_pwm_conf);
#endif
    db_motors_init();
    db_radio_init(&radio_callback, DB_RADIO_BLE_1MBit);
    db_radio_set_frequency(8);  // Set the RX frequency to //2408 MHz.
    db_radio_rx();              // Start receiving packets.

    // Set an invalid heading since the value is unknown on startup.
    // Control loop is stopped and advertize packets are sent
    _dotbot_vars.direction           = DB_DIRECTION_INVALID;
    _dotbot_vars.update_control_loop = false;
    _dotbot_vars.advertize           = false;
    _dotbot_vars.update_lh2          = false;
    _dotbot_vars.lh2_update_counter  = 0;
    _dotbot_vars.update_edhoc        = false; // triggered by radio

    // Retrieve the device id once at startup
    _dotbot_vars.device_id = db_device_id();

    db_timer_init(TIMER_DEV);
    db_timer_set_periodic_ms(TIMER_DEV, 0, DB_TIMEOUT_CHECK_DELAY_MS, &_timeout_check);
    db_timer_set_periodic_ms(TIMER_DEV, 1, DB_ADVERTIZEMENT_DELAY_MS, &_advertise);
    db_timer_set_periodic_ms(TIMER_DEV, 2, DB_LH2_UPDATE_DELAY_MS, &_update_lh2);
    db_lh2_init(&_dotbot_vars.lh2, &db_lh2_d, &db_lh2_e);
    db_lh2_start();
    end_measure_cpu(&ot2);

    // Memory buffer for mbedtls
    //#ifdef RUST_PSA
    char buffer[4096 * 2] = {0};
    mbedtls_memory_buffer_alloc_init(buffer, 4096 * 2);
    //#endif

    //puts("Initializing EDHOC and EAD authz");
    credential_new(&cred_i, CRED_I[EDHOC_INITIATOR_INDEX], sizeof(CRED_I[EDHOC_INITIATOR_INDEX]) / sizeof(CRED_I[EDHOC_INITIATOR_INDEX][0]));
    _dotbot_vars.gateway_authenticated = false;
    int edhoc_state = 0;
    bool begin_edhoc = false;
    bool wait_edhoc_msg2 = false;
    //printf("Dotbot initialized.\n");
    //printf("Gateway NOT authenticated.\n");
    //db_timer_delay_ms(TIMER_DEV, 1100); // make sure it's greater than LOST_DELAY in PyDotBot

    while (1) {
        __WFE();

        if (edhoc_state == 0) {
            edhoc_state = 1;
            //puts("Beginning handhsake...");
            //puts("computing authz_secret.");
            begin_measure_cpu(&ot1);
            initiator_new(&initiator);
            begin_measure_cpu(&ot2);
            authz_device_new(&device, ID_U[EDHOC_INITIATOR_INDEX], ID_U_LEN, &G_W, LOC_W, LOC_W_LEN);
            initiator_compute_ephemeral_secret(&initiator, &G_W, &authz_secret);
            authz_device_prepare_ead_1(&device, &authz_secret, SS, &ead_1);
            end_measure_cpu(&ot2);

            initiator_prepare_message_1(&initiator, NULL, &ead_1, &message_1);
            memcpy(device.wait_ead2.h_message_1, initiator.wait_m2.h_message_1, SHA256_DIGEST_LEN);
            end_measure_cpu(&ot1);

            //initiator_prepare_message_1(&initiator, NULL, NULL, &message_1);

            begin_measure_radio(&ot2);
            db_protocol_header_to_buffer(_dotbot_vars.radio_buffer, DB_BROADCAST_ADDRESS, DotBot, DB_PROTOCOL_EDHOC_MSG);
            memcpy(_dotbot_vars.radio_buffer + sizeof(protocol_header_t), message_1.content, message_1.len);
            size_t length = sizeof(protocol_header_t) + message_1.len;
            db_radio_disable();
            db_radio_tx(_dotbot_vars.radio_buffer, length);
            end_measure_radio(&ot2);
            //puts("sent msg1.");
        } else if (_dotbot_vars.update_edhoc && edhoc_state == 1) {
            _dotbot_vars.update_edhoc = false;

            memcpy(&message_2.content, &_dotbot_vars.edhoc_buffer.content, _dotbot_vars.edhoc_buffer.len);
            message_2.len = _dotbot_vars.edhoc_buffer.len;
            begin_measure_cpu(&ot1);
            int8_t res = initiator_parse_message_2(
                &initiator,
                &message_2,
                &c_r,
                &id_cred_r,
                &ead_2
            );
            if (res != 0) {
                printf("Error parse msg2: %d\n", res);
                edhoc_state = -1;
                continue;
            }
            res = credential_check_or_fetch(NULL, &id_cred_r, &fetched_cred_r);
            if (res != 0) {
                printf("Error handling credential: %d\n", res);
                return 1;
            }

            //puts("processing ead2");
            begin_measure_cpu(&ot2);
            res = authz_device_process_ead_2(&device, &ead_2, &fetched_cred_r);
            end_measure_cpu(&ot2);
            if (res != 0) {
                printf("Error process ead2 (authz): %d\n", res);
                edhoc_state = -1;
                continue;
            }

            res = initiator_verify_message_2(&initiator, &I[EDHOC_INITIATOR_INDEX], &cred_i, &fetched_cred_r);
            if (res != 0) {
                printf("Error verify msg2: %d\n", res);
                edhoc_state = -1;
                continue;
            }

            //puts("preparing msg3");
            res = initiator_prepare_message_3(&initiator, ByReference, NULL, &message_3, _dotbot_vars.prk_out);
            if (res != 0) {
                printf("Error prep msg3: %d\n", res);
                edhoc_state = -1;
                continue;
            }
            end_measure_cpu(&ot1);

            begin_measure_radio(&ot2);
            db_protocol_header_to_buffer(_dotbot_vars.radio_buffer, DB_BROADCAST_ADDRESS, DotBot, DB_PROTOCOL_EDHOC_MSG);
            uint8_t *ptr = _dotbot_vars.radio_buffer + sizeof(protocol_header_t);
            *ptr = c_r;
            memcpy(++ptr, message_3.content, message_3.len);
            size_t length = sizeof(protocol_header_t) + 1 + message_3.len;
            db_radio_disable();
            db_radio_tx(_dotbot_vars.radio_buffer, length);
            _dotbot_vars.gateway_authenticated = true;
            edhoc_state = 2;;
            end_measure_radio(&ot2);

            printf("\nDotBot <-> Gateway authenticated.\n");
            //printf("Derived key:   ");
            //for (size_t i = 0; i < SHA256_DIGEST_LEN; i++) {
            //    printf("%X ", _dotbot_vars.prk_out[i]);
            //}
            //printf("\n");

            // wait 3s and reboot
            db_timer_delay_ms(TIMER_DEV, 3000);
            NVIC_SystemReset();
        }


        if (!_dotbot_vars.gateway_authenticated) {
          continue;
        }

        bool need_advertize = false;
        // Process available lighthouse data
        db_lh2_process_location(&_dotbot_vars.lh2);

        if (_dotbot_vars.update_lh2) {
            // Check if data is ready to send
            if (_dotbot_vars.lh2.data_ready[0][0] == DB_LH2_PROCESSED_DATA_AVAILABLE && _dotbot_vars.lh2.data_ready[1][0] == DB_LH2_PROCESSED_DATA_AVAILABLE) {

                db_lh2_stop();
                // Prepare the radio buffer
                db_protocol_header_to_buffer(_dotbot_vars.radio_buffer, DB_BROADCAST_ADDRESS, DotBot, DB_PROTOCOL_DOTBOT_DATA);
                memcpy(_dotbot_vars.radio_buffer + sizeof(protocol_header_t), &_dotbot_vars.direction, sizeof(int16_t));
                // Add the LH2 sweep
                for (uint8_t lh2_sweep_index = 0; lh2_sweep_index < LH2_SWEEP_COUNT; lh2_sweep_index++) {
                    memcpy(_dotbot_vars.radio_buffer + sizeof(protocol_header_t) + sizeof(int16_t) + lh2_sweep_index * sizeof(db_lh2_raw_data_t), &_dotbot_vars.lh2.raw_data[lh2_sweep_index][0], sizeof(db_lh2_raw_data_t));
                    // Mark the data as already sent
                    _dotbot_vars.lh2.data_ready[lh2_sweep_index][0] = DB_LH2_NO_NEW_DATA;
                }
                size_t length = sizeof(protocol_header_t) + sizeof(int16_t) + sizeof(db_lh2_raw_data_t) * LH2_SWEEP_COUNT;

                // Send the radio packet
                db_radio_disable();
                db_radio_tx(_dotbot_vars.radio_buffer, length);

                db_lh2_start();
            } else {
                _dotbot_vars.lh2_update_counter = (_dotbot_vars.lh2_update_counter + 1) & DB_LH2_COUNTER_MASK;
                need_advertize                  = (_dotbot_vars.lh2_update_counter == DB_LH2_COUNTER_MASK);
            }
            _dotbot_vars.update_lh2 = false;
        }

        if (_dotbot_vars.update_control_loop) {
            _update_control_loop();
            _dotbot_vars.update_control_loop = false;
        }

        if (_dotbot_vars.advertize && need_advertize) {
            db_protocol_header_to_buffer(_dotbot_vars.radio_buffer, DB_BROADCAST_ADDRESS, DotBot, DB_PROTOCOL_ADVERTISEMENT);
            size_t length = sizeof(protocol_header_t);
            db_radio_disable();
            db_radio_tx(_dotbot_vars.radio_buffer, length);
            _dotbot_vars.advertize = false;
        }
    }
}

//=========================== private functions ================================

static void _update_control_loop(void) {
    if (_dotbot_vars.next_waypoint_idx >= _dotbot_vars.waypoints.length) {
        db_motors_set_speed(0, 0);
        return;
    }
    float dx               = ((float)_dotbot_vars.waypoints.points[_dotbot_vars.next_waypoint_idx].x - (float)_dotbot_vars.last_location.x) / 1e6;
    float dy               = ((float)_dotbot_vars.waypoints.points[_dotbot_vars.next_waypoint_idx].y - (float)_dotbot_vars.last_location.y) / 1e6;
    float distanceToTarget = sqrtf(powf(dx, 2) + powf(dy, 2));

    float speedReductionFactor = 1.0;  // No reduction by default

    if ((uint32_t)(distanceToTarget * 1e6) < _dotbot_vars.waypoints_threshold * 2) {
        speedReductionFactor = DB_REDUCE_SPEED_FACTOR;
    }

    int16_t left_speed      = 0;
    int16_t right_speed     = 0;
    int16_t angular_speed   = 0;
    int16_t angle_to_target = 0;
    int16_t error_angle     = 0;
    if ((uint32_t)(distanceToTarget * 1e6) < _dotbot_vars.waypoints_threshold) {
        // Target waypoint is reached
        _dotbot_vars.next_waypoint_idx++;
    } else if (_dotbot_vars.direction == DB_DIRECTION_INVALID) {
        // Unknown direction, just move forward a bit
        left_speed  = (int16_t)DB_MAX_SPEED * speedReductionFactor;
        right_speed = (int16_t)DB_MAX_SPEED * speedReductionFactor;
    } else {
        // compute angle to target waypoint
        _compute_angle(&_dotbot_vars.waypoints.points[_dotbot_vars.next_waypoint_idx], &_dotbot_vars.last_location, &angle_to_target);
        error_angle = angle_to_target - _dotbot_vars.direction;
        if (error_angle < -180) {
            error_angle += 360;
        } else if (error_angle > 180) {
            error_angle -= 360;
        }
        if (error_angle > DB_REDUCE_SPEED_ANGLE || error_angle < -DB_REDUCE_SPEED_ANGLE) {
            speedReductionFactor = DB_REDUCE_SPEED_FACTOR;
        }
        angular_speed = (int16_t)(((float)error_angle / 180) * DB_ANGULAR_SPEED_FACTOR);
        left_speed    = (int16_t)(((DB_MAX_SPEED * speedReductionFactor) - (angular_speed * DB_ANGULAR_SIDE_FACTOR)));
        right_speed   = (int16_t)(((DB_MAX_SPEED * speedReductionFactor) + (angular_speed * DB_ANGULAR_SIDE_FACTOR)));
        if (left_speed > DB_MAX_SPEED) {
            left_speed = DB_MAX_SPEED;
        }
        if (right_speed > DB_MAX_SPEED) {
            right_speed = DB_MAX_SPEED;
        }
    }

    db_motors_set_speed(left_speed, right_speed);

#ifdef ENABLE_DOTBOT_LOG_DATA
    // Log control loop internal data and output on flash
    _dotbot_vars.log_data.direction          = (int32_t)_dotbot_vars.direction;
    _dotbot_vars.log_data.pos_x              = _dotbot_vars.last_location.x;
    _dotbot_vars.log_data.pos_y              = _dotbot_vars.last_location.y;
    _dotbot_vars.log_data.next_waypoint_idx  = (uint16_t)_dotbot_vars.next_waypoint_idx;
    _dotbot_vars.log_data.distance_to_target = (uint32_t)(distanceToTarget * 1e6);
    _dotbot_vars.log_data.angle_to_target    = angle_to_target;
    _dotbot_vars.log_data.error_angle        = error_angle;
    _dotbot_vars.log_data.angular_speed      = angular_speed;
    _dotbot_vars.log_data.left_speed         = left_speed;
    _dotbot_vars.log_data.right_speed        = right_speed;
    db_log_flash_write(&_dotbot_vars.log_data, sizeof(db_log_dotbot_data_t));
#endif
}

static void _compute_angle(const protocol_lh2_location_t *next, const protocol_lh2_location_t *origin, int16_t *angle) {
    float dx       = ((float)next->x - (float)origin->x) / 1e6;
    float dy       = ((float)next->y - (float)origin->y) / 1e6;
    float distance = sqrtf(powf(dx, 2) + powf(dy, 2));

    if (distance < DB_DIRECTION_THRESHOLD) {
        return;
    }

    int8_t sideFactor = (dx > 0) ? -1 : 1;
    *angle            = (int16_t)(acosf(dy / distance) * 180 / M_PI) * sideFactor;
    if (*angle < 0) {
        *angle = 360 + *angle;
    }
}

static void _timeout_check(void) {
    uint32_t ticks = db_timer_ticks(TIMER_DEV);
    if (ticks > _dotbot_vars.ts_last_packet_received + TIMEOUT_CHECK_DELAY_TICKS) {
        db_motors_set_speed(0, 0);
    }
}

static void _advertise(void) {
    _dotbot_vars.advertize = true;
}

static void _update_lh2(void) {
    _dotbot_vars.update_lh2 = true;
}
