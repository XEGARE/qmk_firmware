/*
Copyright 2023 @ Nuphy <https://nuphy.com/>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "user_kb.h"
#include "uart.h" // qmk uart.h
#include "ansi.h"
#include "rf_queue.h"

USART_MGR_STRUCT Usart_Mgr;
// clang-format off
#define RX_SBYTE Usart_Mgr.RXDBuf[0]
#define RX_CMD   Usart_Mgr.RXDBuf[1]
#define RX_ACK   Usart_Mgr.RXDBuf[2]
#define RX_LEN   Usart_Mgr.RXDBuf[3]
#define RX_DAT   Usart_Mgr.RXDBuf[4]
// clang-format on

#define RF_UART_RX_TIMEOUT_MS 20

static uint8_t uart_wait_cmd  = CMD_NULL;
static bool    uart_wait_done = false;

bool f_uart_ack        = 0;
bool f_rf_read_data_ok = 0;
bool f_rf_sts_sysc_ok  = 0;
bool f_rf_new_adv_ok   = 0;
bool f_rf_reset        = 0;
bool f_rf_hand_ok      = 0;
bool f_goto_sleep      = 0;
bool f_wakeup_prepare  = 0;

uint8_t  func_tab[32]     = {0};
uint8_t  sync_lost        = 0;
uint8_t  disconnect_delay = 0;
uint32_t uart_rpt_timer   = 0;

report_buffer_t report_buff_a = {0};
report_buffer_t report_buff_b = {0};

extern DEV_INFO_STRUCT dev_info;
extern host_driver_t  *m_host_driver;
extern host_driver_t   rf_host_driver;
extern uint8_t         host_mode;
extern uint8_t         rf_blink_cnt;
extern uint16_t        rf_link_show_time;
extern uint16_t        rf_linking_time;
extern uint16_t        no_act_time;
extern bool            f_send_channel;
extern bool            f_dial_sw_init_ok;

void    uart_init(uint32_t baud); // qmk uart.c
void    uart_send_report(uint8_t report_type, uint8_t *report_buf, uint8_t report_size);
void    uart_send_bytes(uint8_t *Buffer, uint32_t Length);
uint8_t get_checksum(uint8_t *buf, uint8_t len);
void    uart_receive_pro(void);
void    break_all_key(void);

/**
 * @brief Get variable uart key send repeat interval.
 */
static uint8_t get_repeat_interval(void) {
    uint8_t interval = MAX(report_buff_a.repeat, report_buff_b.repeat);
    if (interval == 0) {
        return 50;
    } else if (interval <= 4) {
        return 16;
    } else if (interval <= 7) {
        return 24;
    } else if (interval <= 10) {
        return 32;
    }
    return 50;
}

/**
 * @brief Reset report buffers.
 */
void clear_report_buffer(void) {
    if (report_buff_a.cmd) memset(&report_buff_a.cmd, 0, sizeof(report_buffer_t));
    if (report_buff_b.cmd) memset(&report_buff_b.cmd, 0, sizeof(report_buffer_t));
}

/**
 * @brief Reset report buffers and clear the queue
 */
void clear_report_buffer_and_queue(void) {
    clear_report_buffer();
    rf_queue.clear();
}

// Cache only reports actually sent, so queued transitions cannot be replayed out of order.
void rf_send_buffered_report(report_buffer_t *report) {
    if (!f_dial_sw_init_ok || dev_info.link_mode == LINK_USB || dev_info.rf_state != RF_CONNECT) return;
    uart_send_report(report->cmd, report->buffer, report->length);
    if (report->repeat < UINT8_MAX) report->repeat++;
    if (report->cmd == CMD_RPT_BYTE_KB) report_buff_a = *report;
    if (report->cmd == CMD_RPT_BIT_KB) report_buff_b = *report;
}

/** Send one queued report per task iteration; UART transmission supplies the pacing. */
void uart_send_repeat_from_queue(void) {
    report_buffer_t report;
    if (!f_dial_sw_init_ok || dev_info.link_mode == LINK_USB || dev_info.rf_state != RF_CONNECT) return;
    if (rf_queue.dequeue(&report)) rf_send_buffered_report(&report);
}

/**
 * @brief  Uart send keys report.
 * @note   Repeats the last sent key reports periodically to reduce stuck keys.
 */
void uart_send_report_repeat(void) {
    if (!f_dial_sw_init_ok || dev_info.link_mode == LINK_USB) return;

    if (dev_info.rf_state != RF_CONNECT) {
        // toss away queue after some time if disconnected to prevent sending random keys
        if (no_act_time > 100) clear_report_buffer_and_queue(); // 1 second
        return;
    }

    // queue is not empty, send from queue.
    if (!rf_queue.is_empty()) {
        uart_send_repeat_from_queue();
        return;
    }

    uint8_t interval = get_repeat_interval();
    if (timer_elapsed32(uart_rpt_timer) >= interval) {
        if (no_act_time <= 25) { // increments every 10ms
            if (report_buff_a.cmd) {
                rf_send_buffered_report(&report_buff_a);
            }

            if (report_buff_b.cmd) {
                rf_send_buffered_report(&report_buff_b);
            }
        } else { // clear the report buffer
            clear_report_buffer_and_queue();
        }
        uart_rpt_timer = timer_read32();
    }
}

/**
 * @brief  Parsing the data received from the RF module.
 */
static bool rf_protocol_receive(void) {
    uint8_t i, check_sum = 0;

    if (Usart_Mgr.RXDState == RX_Done) {
        if (Usart_Mgr.RXDLen < 3 || RX_SBYTE != UART_HEAD) return false;

        bool is_ack = Usart_Mgr.RXDLen == 3 && RX_ACK == 0xA0;
        if (!is_ack) {
            if (Usart_Mgr.RXDLen < 5 || RX_ACK == 0xA0 || RX_LEN > UART_MAX_LEN - 5 || Usart_Mgr.RXDLen != RX_LEN + 5) return false;
            if (RX_CMD == CMD_RF_STS_SYSC && RX_LEN < 5) return false;
            if (RX_CMD == CMD_READ_DATA && RX_LEN < FUNC_VALID_LEN) return false;

            for (i = 0; i < RX_LEN; i++)
                check_sum += Usart_Mgr.RXDBuf[4 + i];

            if (check_sum != Usart_Mgr.RXDBuf[4 + i]) return false;
        }

        sync_lost = 0;
        if (is_ack) f_uart_ack = 1;
        Usart_Mgr.RXCmd = RX_CMD;
        if (RX_CMD == uart_wait_cmd) uart_wait_done = true;

        switch (RX_CMD) {
            case CMD_HAND: {
                f_rf_hand_ok = 1;
                break;
            }

            case CMD_24G_SUSPEND: {
                f_goto_sleep = 1;
                break;
            }

            case CMD_NEW_ADV: {
                f_rf_new_adv_ok = 1;
                break;
            }

            case CMD_RF_STS_SYSC: {
                static uint8_t error_cnt = 0;
                if (is_ack) break; // ACK has no link/battery payload.

                if (dev_info.link_mode == Usart_Mgr.RXDBuf[4]) {
                    error_cnt = 0;

                    dev_info.rf_state = Usart_Mgr.RXDBuf[5];

                    if ((dev_info.rf_state == RF_CONNECT) && ((Usart_Mgr.RXDBuf[6] & 0xf8) == 0)) {
                        dev_info.rf_led = Usart_Mgr.RXDBuf[6];
                    }

                    dev_info.rf_charge = Usart_Mgr.RXDBuf[7];
                    uint8_t bat_pct    = Usart_Mgr.RXDBuf[8];
                    if (dev_info.rf_charge & 0x01) bat_pct = 100;
                    if (bat_pct > 0 && bat_pct <= 100) {
                        update_bat_pct_rgb(bat_pct);
                    }
                } else {
                    if (dev_info.rf_state != RF_INVAILD) {
                        if (error_cnt >= 5) {
                            error_cnt      = 0;
                            f_send_channel = 1;
                        } else {
                            error_cnt++;
                        }
                    }
                }

                f_rf_sts_sysc_ok = 1;
                break;
            }

            case CMD_READ_DATA: {
                if (is_ack) break;
                memcpy(func_tab, &Usart_Mgr.RXDBuf[4], 32);

                if (func_tab[4] <= LINK_USB) {
                    dev_info.link_mode = func_tab[4];
                }

                if (func_tab[5] < LINK_USB) {
                    dev_info.rf_channel = func_tab[5];
                }

                if ((func_tab[6] <= LINK_BT_3) && (func_tab[6] >= LINK_BT_1)) {
                    dev_info.ble_channel = func_tab[6];
                }

                f_rf_read_data_ok = 1;
                break;
            }
        }

        return true;
    }
    return false;
}

/**
 * @brief  Uart send cmd.
 * @param  cmd: cmd.
 * @param  wait_ack: wait time for ack after sending.
 * @param  delayms: delay before sending.
 */
uint8_t uart_send_cmd(uint8_t cmd, uint8_t wait_ack, uint8_t delayms) {
    wait_ms(delayms);

    memset(&Usart_Mgr.TXDBuf[0], 0, UART_MAX_LEN);

    Usart_Mgr.TXDBuf[0] = UART_HEAD;
    Usart_Mgr.TXDBuf[1] = cmd;
    Usart_Mgr.TXDBuf[2] = 0x00;

    switch (cmd) {
        case CMD_SLEEP: {
            Usart_Mgr.TXDBuf[3] = 1;
            Usart_Mgr.TXDBuf[4] = 0;
            Usart_Mgr.TXDBuf[5] = 0;
            break;
        }

        case CMD_HAND: {
            Usart_Mgr.TXDBuf[3] = 1;
            Usart_Mgr.TXDBuf[4] = 0;
            Usart_Mgr.TXDBuf[5] = 0;
            break;
        }

        case CMD_RF_STS_SYSC: {
            Usart_Mgr.TXDBuf[3] = 1;
            Usart_Mgr.TXDBuf[4] = dev_info.link_mode;
            Usart_Mgr.TXDBuf[5] = dev_info.link_mode;
            break;
        }

        case CMD_SET_LINK: {
            dev_info.rf_state   = RF_LINKING;
            Usart_Mgr.TXDBuf[3] = 1;
            Usart_Mgr.TXDBuf[4] = dev_info.link_mode;
            Usart_Mgr.TXDBuf[5] = dev_info.link_mode;

            rf_linking_time  = 0;
            disconnect_delay = 0xff;
            break;
        }

        case CMD_NEW_ADV: {
            dev_info.rf_state   = RF_PAIRING;
            Usart_Mgr.TXDBuf[3] = 2;
            Usart_Mgr.TXDBuf[4] = dev_info.link_mode;
            Usart_Mgr.TXDBuf[5] = 1;
            Usart_Mgr.TXDBuf[6] = dev_info.link_mode + 1;

            rf_linking_time  = 0;
            disconnect_delay = 0xff;
            f_rf_new_adv_ok  = 0;
            break;
        }

        case CMD_CLR_DEVICE: {
            Usart_Mgr.TXDBuf[3] = 1;
            Usart_Mgr.TXDBuf[4] = 0;
            Usart_Mgr.TXDBuf[5] = 0;
            break;
        }

        case CMD_SET_CONFIG: {
            Usart_Mgr.TXDBuf[3] = 1;
            Usart_Mgr.TXDBuf[4] = POWER_DOWN_DELAY;
            Usart_Mgr.TXDBuf[5] = POWER_DOWN_DELAY;
            break;
        }

        case CMD_SET_NAME: {
            Usart_Mgr.TXDBuf[3]  = 17;
            Usart_Mgr.TXDBuf[4]  = 1;
            Usart_Mgr.TXDBuf[5]  = 15;
            Usart_Mgr.TXDBuf[6]  = 'N';
            Usart_Mgr.TXDBuf[7]  = 'u';
            Usart_Mgr.TXDBuf[8]  = 'P';
            Usart_Mgr.TXDBuf[9]  = 'h';
            Usart_Mgr.TXDBuf[10] = 'y';
            Usart_Mgr.TXDBuf[11] = ' ';
            Usart_Mgr.TXDBuf[12] = 'A';
            Usart_Mgr.TXDBuf[13] = 'i';
            Usart_Mgr.TXDBuf[14] = 'r';
            Usart_Mgr.TXDBuf[15] = '7';
            Usart_Mgr.TXDBuf[16] = '5';
            Usart_Mgr.TXDBuf[17] = ' ';
            Usart_Mgr.TXDBuf[18] = 'V';
            Usart_Mgr.TXDBuf[19] = '2';
            Usart_Mgr.TXDBuf[20] = '-';
            Usart_Mgr.TXDBuf[21] = get_checksum(Usart_Mgr.TXDBuf + 4, Usart_Mgr.TXDBuf[3]);
            break;
        }

        case CMD_READ_DATA: {
            Usart_Mgr.TXDBuf[3] = 2;
            Usart_Mgr.TXDBuf[4] = 0x00;
            Usart_Mgr.TXDBuf[5] = FUNC_VALID_LEN;
            Usart_Mgr.TXDBuf[6] = FUNC_VALID_LEN;
            break;
        }

        case CMD_RF_DFU: {
            Usart_Mgr.TXDBuf[3] = 1;
            Usart_Mgr.TXDBuf[4] = 0;
            Usart_Mgr.TXDBuf[5] = 0;
            break;
        }

        default:
            break;
    }

    f_uart_ack      = 0;
    Usart_Mgr.RXCmd = CMD_NULL;
    uart_wait_cmd   = cmd;
    uart_wait_done  = false;
    uart_send_bytes(Usart_Mgr.TXDBuf, Usart_Mgr.TXDBuf[3] + 5);

    uint8_t result = TX_OK;
    if (wait_ack) {
        result = TX_TIMEOUT;
        while (wait_ack--) {
            wait_ms(1);
            uart_receive_pro();
            if (uart_wait_done) {
                result = TX_OK;
                break;
            }
        }
    }

    uart_wait_cmd = CMD_NULL;
    return result;
}

/**
 * @brief RF module state sync.
 */
void dev_sts_sync(void) {
    static uint32_t interval_timer  = 0;
    static uint8_t  link_state_temp = RF_DISCONNECT;

    if (timer_elapsed32(interval_timer) < 200) return;
    interval_timer = timer_read32();

    if (f_rf_reset) {
        f_rf_reset = 0;
        wait_ms(100);
        gpio_write_pin_low(NRF_RESET_PIN);
        wait_ms(50);
        gpio_write_pin_high(NRF_RESET_PIN);
        wait_ms(50);
    } else if (f_send_channel) {
        f_send_channel = 0;
        uart_send_cmd(CMD_SET_LINK, 10, 10);
    }

    if (dev_info.link_mode == LINK_USB) {
        if (host_mode != HOST_USB_TYPE) {
            host_mode = HOST_USB_TYPE;
            break_all_key();
            host_set_driver(m_host_driver);
        }
        rf_blink_cnt = 0;
    } else {
        if (host_mode != HOST_RF_TYPE) {
            host_mode = HOST_RF_TYPE;
            break_all_key();
            host_set_driver(&rf_host_driver);
        }

        if (dev_info.rf_state != RF_CONNECT) {
            if (disconnect_delay >= 10) {
                rf_blink_cnt      = 3;
                rf_link_show_time = 0;
                link_state_temp   = dev_info.rf_state;
            } else {
                disconnect_delay++;
            }
        } else if (dev_info.rf_state == RF_CONNECT) {
            rf_linking_time  = 0;
            disconnect_delay = 0;
            rf_blink_cnt     = 0;

            if (link_state_temp != RF_CONNECT) {
                link_state_temp   = RF_CONNECT;
                rf_link_show_time = 0;
            }
        }
    }

    uart_send_cmd(CMD_RF_STS_SYSC, 1, 0);

    if (dev_info.link_mode != LINK_USB) {
        if (++sync_lost >= 5) {
            sync_lost  = 0;
            f_rf_reset = 1;
        }
    }
}

/**
 * @brief Uart send bytes.
 * @param Buffer data buf
 * @param Length data length
 */
void uart_send_bytes(uint8_t *Buffer, uint32_t Length) {
    if (Length > UART_MAX_LEN) return;
    gpio_write_pin_low(NRF_WAKEUP_PIN);
    wait_us(50);
    uart_transmit(Buffer, (uint16_t)Length);
    wait_us(50 + Length * 32);
    gpio_write_pin_high(NRF_WAKEUP_PIN);
    wait_us(200);
    Usart_Mgr.TXLastCmdTm = timer_read32();
}

/**
 * @brief get checksum.
 * @param buf data buf
 * @param len data length
 */
uint8_t get_checksum(uint8_t *buf, uint8_t len) {
    uint8_t i;
    uint8_t checksum = 0;

    for (i = 0; i < len; i++)
        checksum += *buf++;

    checksum ^= UART_HEAD;

    return checksum;
}

/**
 * @brief Uart send report.
 * @param report_type  report_type
 * @param report_buf  report_buf
 * @param report_size  report_size
 */
void uart_send_report(uint8_t report_type, uint8_t *report_buf, uint8_t report_size) {
    if (f_dial_sw_init_ok == 0) return;
    if (dev_info.link_mode == LINK_USB) return;
    if (dev_info.rf_state != RF_CONNECT) return;
    if (report_size > UART_MAX_LEN - 5) return;

    Usart_Mgr.TXDBuf[0] = UART_HEAD;
    Usart_Mgr.TXDBuf[1] = report_type;
    Usart_Mgr.TXDBuf[2] = 0x01;
    Usart_Mgr.TXDBuf[3] = report_size;

    memcpy(&Usart_Mgr.TXDBuf[4], report_buf, report_size);
    Usart_Mgr.TXDBuf[4 + report_size] = get_checksum(&Usart_Mgr.TXDBuf[4], report_size);

    // Keyboard packets are absolute states. Repeating mouse deltas or media events
    // can perform the action more than once, so send those only once.
    uint8_t copies = (report_type == CMD_RPT_BYTE_KB || report_type == CMD_RPT_BIT_KB) ? 3 : 1;
    while (copies--)
        uart_send_bytes(&Usart_Mgr.TXDBuf[0], report_size + 5);

    uart_rpt_timer = timer_read32(); // reset uart repeat timer.
}

static void rf_discard_rx_bytes(uint8_t count) {
    Usart_Mgr.RXDLen -= count;
    memmove(Usart_Mgr.RXDBuf, Usart_Mgr.RXDBuf + count, Usart_Mgr.RXDLen);
    Usart_Mgr.RXDState = Usart_Mgr.RXDLen ? RX_Receiving : RX_Idle;
}

static void rf_process_rx_buffer(void) {
    while (Usart_Mgr.RXDLen) {
        if (RX_SBYTE != UART_HEAD) {
            rf_discard_rx_bytes(1);
            continue;
        }
        if (Usart_Mgr.RXDLen < 3) return;

        uint8_t frame_len = 3;
        if (RX_ACK != 0xA0) {
            if (Usart_Mgr.RXDLen < 4) return;
            if (RX_LEN > UART_MAX_LEN - 5 || (RX_CMD == CMD_RF_STS_SYSC && RX_LEN < 5) || (RX_CMD == CMD_READ_DATA && RX_LEN < FUNC_VALID_LEN)) {
                rf_discard_rx_bytes(1);
                continue;
            }
            frame_len = RX_LEN + 5;
        }
        if (Usart_Mgr.RXDLen < frame_len) return;

        uint8_t buffered_len = Usart_Mgr.RXDLen;
        Usart_Mgr.RXDLen     = frame_len;
        Usart_Mgr.RXDState   = RX_Done;
        bool valid           = rf_protocol_receive();
        Usart_Mgr.RXDLen     = buffered_len;
        // On corrupt frames, search for the next header without losing a buffered reply.
        rf_discard_rx_bytes(valid ? frame_len : 1);
    }
}

/** Receive complete frames without waiting; retain incomplete frames across task calls. */
void uart_receive_pro(void) {
    static uint32_t rx_byte_timer = 0;

    if (Usart_Mgr.RXDLen && timer_elapsed32(rx_byte_timer) >= RF_UART_RX_TIMEOUT_MS) {
        rf_discard_rx_bytes(1);
        rf_process_rx_buffer();
        // Do not carry an abandoned partial frame into the next transaction.
        Usart_Mgr.RXDLen   = 0;
        Usart_Mgr.RXDState = RX_Idle;
    }

    while (uart_available()) {
        uint8_t byte = uart_read();
        if (!Usart_Mgr.RXDLen && byte != UART_HEAD) continue;
        if (Usart_Mgr.RXDLen == UART_MAX_LEN) rf_discard_rx_bytes(1);
        Usart_Mgr.RXDBuf[Usart_Mgr.RXDLen++] = byte;
        Usart_Mgr.RXDState                   = RX_Receiving;
        rx_byte_timer                        = timer_read32();
        rf_process_rx_buffer();
    }
}

/**
 * @brief  RF uart initial.
 */
void rf_uart_init(void) {
    /* set uart buad as 460800 */
    uart_init(460800);

    /* Enable parity check */
    USART1->CR1 &= ~((uint32_t)USART_CR1_UE);
    USART1->CR1 |= USART_CR1_M0 | USART_CR1_PCE;
    USART1->CR1 |= USART_CR1_UE;

    /* set Rx and Tx pin pull up */
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0);
}

/**
 * @brief RF module initial.
 */
void rf_device_init(void) {
    uint8_t timeout = 10;

    f_rf_hand_ok = 0;
    while (timeout--) {
        uart_send_cmd(CMD_HAND, 0, 20);
        wait_ms(5);
        uart_receive_pro();
        if (f_rf_hand_ok) break;
    }

    timeout           = 10;
    f_rf_read_data_ok = 0;
    while (timeout--) {
        uart_send_cmd(CMD_READ_DATA, 0, 20);
        wait_ms(5);
        uart_receive_pro();
        if (f_rf_read_data_ok) break;
    }

    timeout          = 10;
    f_rf_sts_sysc_ok = 0;
    while (timeout--) {
        uart_send_cmd(CMD_RF_STS_SYSC, 0, 20);
        wait_ms(5);
        uart_receive_pro();
        if (f_rf_sts_sysc_ok) break;
    }

    uart_send_cmd(CMD_SET_NAME, 10, 20);
}
