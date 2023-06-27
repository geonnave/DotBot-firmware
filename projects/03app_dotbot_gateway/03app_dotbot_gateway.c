/**
 * @file 03app_dotbot_gateway.c
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief  Application that can be used as a gateway to communicate by radio with several DotBots
 *
 * @copyright Inria, 2022
 *
 */
#include <nrf.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
// Include BSP headers
#include "board.h"
#include "board_config.h"
#include "gpio.h"
#include "hdlc.h"
#include "protocol.h"
#include "radio.h"
#include "uart.h"
#include "edhoc_rs.h"

//=========================== defines ==========================================

#define DB_BUFFER_MAX_BYTES (255U)                           ///< Max bytes in UART receive buffer
#define DB_UART_BAUDRATE    (1000000UL)                      ///< UART baudrate used by the gateway
#define DB_RADIO_QUEUE_SIZE (8U)                             ///< Size of the radio queue (must by a power of 2)
#define DB_UART_QUEUE_SIZE  ((DB_BUFFER_MAX_BYTES + 1) * 2)  ///< Size of the UART queue size (must by a power of 2)

typedef struct {
    uint8_t length;                       ///< Length of the radio packet
    uint8_t buffer[DB_BUFFER_MAX_BYTES];  ///< Buffer containing the radio packet
} gateway_radio_packet_t;

typedef struct {
    uint8_t                current;                       ///< Current position in the queue
    uint8_t                last;                          ///< Position of the last item added in the queue
    gateway_radio_packet_t packets[DB_RADIO_QUEUE_SIZE];  ///< Buffer containing the received bytes
} gateway_radio_packet_queue_t;

typedef struct {
    uint16_t current;                     ///< Current position in the queue
    uint16_t last;                        ///< Position of the last item added in the queue
    uint8_t  buffer[DB_UART_QUEUE_SIZE];  ///< Buffer containing the received bytes
} gateway_uart_queue_t;

typedef struct {
    db_hdlc_state_t              hdlc_state;                               ///< Current state of the HDLC decoding engine
    uint8_t                      hdlc_rx_buffer[DB_BUFFER_MAX_BYTES * 2];  ///< Buffer where message received on UART is stored
    uint8_t                      hdlc_tx_buffer[DB_BUFFER_MAX_BYTES * 2];  ///< Internal buffer used for sending serial HDLC frames
    uint32_t                     buttons;                                  ///< Buttons state (one byte per button)
    uint8_t                      radio_tx_buffer[DB_BUFFER_MAX_BYTES];     ///< Internal buffer that contains the command to send (from buttons)
    gateway_radio_packet_queue_t radio_queue;                              ///< Queue used to process received radio packets outside of interrupt
    gateway_uart_queue_t         uart_queue;                               ///< Queue used to process received UART bytes outside of interrupt
    bool                         handshake_done;                           ///< Whether startup handshake is done
    // edhoc stuff
    bool                         update_edhoc;                             ///< Whether EDHOC data must be processed
    EdhocMessageBuffer           edhoc_buffer;                             ///< Internal buffer to store received but not yet handled edhoc messages
    uint8_t                      prk_out[SHA256_DIGEST_LEN];
    bool                         edhoc_done;
} gateway_vars_t;

//=========================== variables ========================================

static gateway_vars_t _gw_vars;

static const uint8_t ID_CRED_I[] = "a104412b";
static const uint8_t ID_CRED_R[] = "a104410a";
static const uint8_t CRED_I[] = "A2027734322D35302D33312D46462D45462D33372D33322D333908A101A5010202412B2001215820AC75E9ECE3E50BFC8ED60399889522405C47BF16DF96660A41298CB4307F7EB62258206E5DE611388A4B8A8211334AC7D37ECB52A387D257E6DB3C2A93DF21FF3AFFC8";
static const uint8_t G_I[] = "ac75e9ece3e50bfc8ed60399889522405c47bf16df96660a41298cb4307f7eb6";
static const uint8_t CRED_R[] = "A2026008A101A5010202410A2001215820BBC34960526EA4D32E940CAD2A234148DDC21791A12AFBCBAC93622046DD44F02258204519E257236B2A0CE2023F0931F1F386CA7AFDA64FCDE0108C224C51EABF6072";
static const uint8_t R[] = "72cc4761dbd4c78f758931aa589d348d1ef874a7e303ede2f140dcf3e6aa4aac";
static const uint8_t I[] = "fb13adeb6518cee5f88417660841142e830a81fe334380a953406a1305e8706b";
static const uint8_t G_R[] = "bbc34960526ea4d32e940cad2a234148ddc21791a12afbcbac93622046dd44f0";

//=========================== callbacks ========================================

static void uart_callback(uint8_t data) {
    if (!_gw_vars.handshake_done) {
        uint8_t version = DB_FIRMWARE_VERSION;
        db_uart_write(&version, 1);
        if (data == version) {
            _gw_vars.handshake_done = true;
        }
        return;
    }
    _gw_vars.uart_queue.buffer[_gw_vars.uart_queue.last] = data;
    _gw_vars.uart_queue.last                             = (_gw_vars.uart_queue.last + 1) & (DB_UART_QUEUE_SIZE - 1);
}

static void radio_callback(uint8_t *packet, uint8_t length) {

    protocol_header_t *header  = (protocol_header_t *)packet;
    uint8_t *cmd_ptr = packet + sizeof(protocol_header_t);
    if (header->type == DB_PROTOCOL_EDHOC_MSG) {
        uint8_t buffer_len = length - sizeof(protocol_header_t);
        memcpy(_gw_vars.edhoc_buffer.content, cmd_ptr, buffer_len);
        _gw_vars.edhoc_buffer.len = buffer_len;
        _gw_vars.update_edhoc = true;
    } else {
        if (!_gw_vars.handshake_done) {
            return;
        }
        memcpy(_gw_vars.radio_queue.packets[_gw_vars.radio_queue.last].buffer, packet, length);
        _gw_vars.radio_queue.packets[_gw_vars.radio_queue.last].length = length;
        _gw_vars.radio_queue.last                                      = (_gw_vars.radio_queue.last + 1) & (DB_RADIO_QUEUE_SIZE - 1);
    }
}

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    db_board_init();
    db_protocol_init();

    // Configure Radio as transmitter
    db_radio_init(&radio_callback, DB_RADIO_BLE_1MBit);  // All RX packets received are forwarded in an HDLC frame over UART
    db_radio_set_frequency(8);                           // Set the radio frequency to 2408 MHz.
    // Initialize the gateway context
    _gw_vars.buttons             = 0x0000;
    _gw_vars.radio_queue.current = 0;
    _gw_vars.radio_queue.last    = 0;
    _gw_vars.handshake_done      = false;
    _gw_vars.edhoc_done          = false;
    db_uart_init(&db_uart_rx, &db_uart_tx, DB_UART_BAUDRATE, &uart_callback);

    db_radio_rx_enable();

    db_gpio_init(&db_btn2, DB_GPIO_IN_PU);
    db_gpio_init(&db_btn3, DB_GPIO_IN_PU);
    db_gpio_init(&db_btn4, DB_GPIO_IN_PU);
    db_gpio_init(&db_btn1, DB_GPIO_IN_PU);

    // Initialize EDHOC
    RustEdhocResponderC responder = responder_new(R, 32*2, G_I, 32*2, ID_CRED_I, 4*2, CRED_I, 107*2, ID_CRED_R, 4*2, CRED_R, 84*2);

    while (1) {
        if (_gw_vars.update_edhoc && responder.state._0 == Start) {
            _gw_vars.update_edhoc = false;
            if (responder_process_message_1(&responder, &_gw_vars.edhoc_buffer) == 0) {
                EdhocMessageBuffer message_2;
                uint8_t c_r_out;
                responder_prepare_message_2(&responder, &message_2, &c_r_out);

                db_protocol_header_to_buffer(_gw_vars.radio_tx_buffer, DB_BROADCAST_ADDRESS, DotBot, DB_PROTOCOL_EDHOC_MSG);
                memcpy(_gw_vars.radio_tx_buffer + sizeof(protocol_header_t), message_2.content, message_2.len);
                size_t length = sizeof(protocol_header_t) + message_2.len;
                db_radio_rx_disable();
                db_radio_tx(_gw_vars.radio_tx_buffer, length);
                db_radio_rx_enable();
            } else {
                responder = responder_new(R, 32*2, G_I, 32*2, ID_CRED_I, 4*2, CRED_I, 107*2, ID_CRED_R, 4*2, CRED_R, 84*2);
            }
        } else if (_gw_vars.update_edhoc && responder.state._0 == WaitMessage3) {
            _gw_vars.update_edhoc = false;
           if (responder_process_message_3(&responder, &_gw_vars.edhoc_buffer, &_gw_vars.prk_out) == 0) {
                _gw_vars.edhoc_done = true;
            }
        }

        protocol_move_raw_command_t command;
        // Read Button 1 (P0.11)
        if (!db_gpio_read(&db_btn1)) {
            command.left_y = 100;
        } else if (!db_gpio_read(&db_btn2)) {
            command.left_y = -100;
        } else {
            command.left_y = 0;
        }

        // Read Button 2 (P0.12)
        if (!db_gpio_read(&db_btn3)) {
            command.right_y = 100;
        } else if (!db_gpio_read(&db_btn4)) {
            command.right_y = -100;
        } else {
            command.right_y = 0;
        }

        if (command.left_y != 0 || command.right_y != 0) {
            db_protocol_cmd_move_raw_to_buffer(_gw_vars.radio_tx_buffer, DB_BROADCAST_ADDRESS, DotBot, &command);
            db_radio_rx_disable();
            db_radio_tx(_gw_vars.radio_tx_buffer, sizeof(protocol_header_t) + sizeof(protocol_move_raw_command_t));
            db_radio_rx_enable();
        }

        while (_gw_vars.radio_queue.current != _gw_vars.radio_queue.last) {
            size_t frame_len = db_hdlc_encode(_gw_vars.radio_queue.packets[_gw_vars.radio_queue.current].buffer, _gw_vars.radio_queue.packets[_gw_vars.radio_queue.current].length, _gw_vars.hdlc_tx_buffer);
            db_uart_write(_gw_vars.hdlc_tx_buffer, frame_len);
            _gw_vars.radio_queue.current = (_gw_vars.radio_queue.current + 1) & (DB_RADIO_QUEUE_SIZE - 1);
        }

        while (_gw_vars.uart_queue.current != _gw_vars.uart_queue.last) {
            _gw_vars.hdlc_state = db_hdlc_rx_byte(_gw_vars.uart_queue.buffer[_gw_vars.uart_queue.current]);
            switch ((uint8_t)_gw_vars.hdlc_state) {
                case DB_HDLC_STATE_IDLE:
                case DB_HDLC_STATE_RECEIVING:
                case DB_HDLC_STATE_ERROR:
                    break;
                case DB_HDLC_STATE_READY:
                {
                    size_t msg_len = db_hdlc_decode(_gw_vars.hdlc_rx_buffer);
                    if (msg_len) {
                        _gw_vars.hdlc_state = DB_HDLC_STATE_IDLE;
                        db_radio_rx_disable();
                        db_radio_tx(_gw_vars.hdlc_rx_buffer, msg_len);
                        db_radio_rx_enable();
                    }
                } break;
                default:
                    break;
            }
            _gw_vars.uart_queue.current = (_gw_vars.uart_queue.current + 1) & (DB_UART_QUEUE_SIZE - 1);
        }
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}
