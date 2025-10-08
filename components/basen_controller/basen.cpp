#include "basen.h"
#include "esphome/core/log.h"
#include <algorithm>
#include "esphome/core/helpers.h"

namespace esphome {
namespace basen {

static const char *const TAG = "basen";

constexpr uint8_t SOI = 0x7E;
constexpr uint8_t EOI = 0x0D;

constexpr uint8_t COMMAND_INFO         = 0x01;
constexpr uint8_t COMMAND_BMS_VERSION  = 0x33;
constexpr uint8_t COMMAND_BARCODE      = 0x42;
constexpr uint16_t COMMAND_PARAMETERS  = 0xE043;
constexpr uint16_t COMMAND_ALARM_PARAMETERS  = 0x43;

// For ease of use, generate this bit mask from status bit mask
// # Bit 0     General alarm                                0000 0000 0000 0001         0x0001 
// # Bit 1     Battery high voltage alarm                   0000 0000 0000 0010         0x0002
// # Bit 2     Battery low voltage alarm                    0000 0000 0000 0100         0x0004
// # Bit 3     Battery high temperature alarm               0000 0000 0000 1000         0x0008
// # Bit 4     Battery low temperature alarm                0000 0000 0001 0000         0x0010
// # Bit 5     Battery high temperature charge alarm        0000 0000 0010 0000         0x0020
// # Bit 6     Battery low temperature charge alarm         0000 0000 0100 0000         0x0040
// # Bit 7     Battery high discharge current alarm         0000 0000 1000 0000         0x0080
// # Bit 8     Battery high charge current alarm            0000 0001 0000 0000         0x0100
// # Bit 9     Contactor alarm                              0000 0010 0000 0000         0x0200
// # Bit 10    Short circuit alarm                          0000 0100 0000 0000         0x0400
// # Bit 11    BMS internal alarm                           0000 1000 0000 0000         0x0800
// # Bit 12    Cell imbalance alarm                         0001 0000 0000 0000         0x1000
// # Bit 13    Reserved                                     0010 0000 0000 0000         0x2000
// # Bit 14    Reserved                                     0100 0000 0000 0000         0x4000
// # Bit 15    Reserved                                     1000 0000 0000 0000         0x8000

// Status bit masks, messages and conversion
static struct {
  uint8_t     offset;     // Byte offset
  uint8_t     bit;        // Bit
  uint8_t     type;       // Type: 1 - Alarm, 2 - Fault
  uint16_t    error_mask; // 16-bit error mask described above
  const char  *message; // Message text
} status_messages[] = {
  {0, 0x08, 2, 0x0004, "Cell voltage low fault"},
  {0, 0x10, 2, 0x0200, "Voltage line break"},
  {0, 0x20, 2, 0x0801, "Charge MOS fault"},
  {0, 0x40, 2, 0x0801, "Dischare MOS fault"},
  {0, 0x80, 2, 0x0800, "Voltage sensor fault"},
  {1, 0x01, 2, 0x0800, "NTC disconnection"},
  {1, 0x02, 2, 0x0800, "ADC MOD fault"},
  {1, 0x04, 2, 0x0200, "Reverse battery"},
  {1, 0x08, 2, 0x0008, "Hot failure"},
  {1, 0x10, 1, 0x0001, "Battery locked"},
  {1, 0x20, 1, 0x0001, "Communication timeout"},
  {1, 0x40, 1, 0x0001, "SN code failure"},
  {1, 0x80, 2, 0x0801, "Secondary trip protection"},
  {2, 0x01, 2, 0x0008, "DISC_OV_TEMP_Protection"},
  {2, 0x02, 2, 0x0010, "DISC_UN_TEMP_Protection"},
  {2, 0x10, 2, 0x0001, "CHG_MOS_Off"},
  {2, 0x20, 2, 0x0001, "DISC_MOS_Off"},
  {2, 0x40, 1, 0x0800, "536_COM_Timeout"},
  {3, 0x01, 0, 0x0000, "Charging"},
  {3, 0x02, 0, 0x0000, "Discharging"},
  {3, 0x04, 2, 0x0400, "Short Circuit Protection"},
  {3, 0x08, 2, 0x0080, "Overcurrent Protection"},
  {3, 0x10, 2, 0x0002, "Overvoltage Protection"}, // Cell/battery details stored in another place
  {3, 0x40, 2, 0x0020, "CHG_OV_TEMP_Protection"},
  {3, 0x80, 2, 0x0040, "CHG_UN_TEMP_Protection"},
  {4, 0x01, 2, 0x0010, "Ambient_Low_TEMP_Protection"},
  {4, 0x02, 2, 0x0008, "ENV_High_TEMP_Protection"},
  {4, 0x10, 2, 0x0001, "SOC Low protect"},
  {5, 0x01, 0, 0x0001, "Manual_CHG_MOS_Open"},
  {5, 0x02, 0, 0x0001, "Manual_CHG_MOS_Off"},
  {5, 0x04, 0, 0x0001, "Manual_DISC_MOS_Open"},
  {5, 0x08, 0, 0x0001, "Manual_DISC_MOS_Off"},
  {5, 0x10, 0, 0x0000, "Heating pad"},
  {5, 0x20, 2, 0x0008, "MOSFET_OV_TEMP_Protection"},
  {5, 0x40, 2, 0x0010, "MOSFET_LO_TEMP_Protection"},
  {5, 0x80, 2, 0x0010, "CHG_Open_TEMP_Too_Low"},
  {7, 0x01, 1, 0x0001, "SOC_Low_Alarm2"},
  {7, 0x02, 1, 0x0001, "Vibration Alarm"},
  {7, 0x04, 0, 0x0001, "Waiting to recharge during a break"},
  {7, 0x08, 2, 0x0001, "Aerosol fault"},
  {7, 0x10, 1, 0x0001, "SOH Low Alarm"},
  {7, 0x20, 2, 0x0800, "NTC short circuit"},
  {7, 0x40, 1, 0x0001, "Temp_Diff Alarm"},
  {7, 0x80, 1, 0x0001, "System Lock"},
  {8, 0x01, 1, 0x0008, "AMB_Over_TEMP_Alarm"},
  {8, 0x02, 1, 0x0010, "AMB_Low_TEMP_Alarm"},
  {8, 0x04, 1, 0x0008, "MOS_Over_TEMP_Alarm"},
  {8, 0x08, 1, 0x0001, "SOC_Low_Alarm"},
  {8, 0x10, 1, 0x1000, "Vol_DIF_Alarm"},
  {8, 0x20, 1, 0x0008, "BAT_DISC_Over_TEMP_Alarm"},
  {8, 0x40, 1, 0x0010, "BAT_DISC_Low_TEMP_Alarm"},
  {9, 0x01, 1, 0x0002, "Cell_Over_VOL_Alarm"},
  {9, 0x02, 1, 0x0004, "Cell_Low_VOL_Alarm"},
  {9, 0x04, 1, 0x0002, "Pack_Over_VOL_Alarm"},
  {9, 0x08, 1, 0x0004, "Pack_Low_VOL_Alarm"},
  {9, 0x10, 1, 0x0100, "CHG_Over_CUR_Alarm"},
  {9, 0x20, 1, 0x0080, "DISC_Over_CURR_Alarm"},
  {9, 0x40, 1, 0x0020, "BAT_CHG_Over_TEMP_Alarm"},
  {9, 0x80, 1, 0x0040, "BAT_CHG_Low_TEMP_Alarm"},
};

// Conversion operations for parameters
static BasenBMS::PARAM_OPERATION protect_param_op[] = {
  {2, 1000}, // CELL_OV_Start (V)
  {0, 1000}, // CELL_OV_Delay (Delay)
  {2, 1000}, // CELL_OV_Stop (V)
  {2, 1000}, // CELL_UV_Start (V)
  {0, 1000}, // CELL_UV_Delay (Delay)
  {2, 1000}, // CELL_UV_Stop (V)
  {2, 100},  // PACK_OV_Start (V)
  {0, 1000}, // PACK_OV_Delay (Delay)
  {2, 100},  // PACK_OV_Stop (V)
  {2, 100},  // PACK_UV_Start (V)
  {0, 1000}, // PACK_UV_Delay (Delay)
  {2, 100},  // PACK_UV_Stop (V)
  {2, 100},  // Const_Pack_V (V)
  {2, 1000}, // Const_Current
  {2, 100},  // CHG_OC1_Start
  {0, 1000}, // CHG_OC1_Delay (Delay)
  {2, 100},  // DISC_OC1_Start
  {0, 1000}, // DISC_OC1_Delay (Delay)
  {2, 100},  // CHG_OC2_Start
  {0, 1000}, // CHG_OC2_Delay (Delay)
  {2, 100},  // DISC_OC2_Start
  {0, 1000}, // DISC_OC2_Delay (Delay)
  {4, 50},   // CHG_OT_START (T)
  {0, 1000}, // CHG_OT_Delay (Delay)
  {4, 50},   // CHG_OT_STOP (T)
  {4, 50},   // DISC_OT_START (T)
  {0, 1000}, // DISC_OT_Delay (T, Delay)
  {4, 50},   // DISC_OT_STOP (T)
  {4, 50},   // CHG_UT_START (T)
  {0, 1000}, // CHG_UT_Delay (T, Delay)
  {4, 50},   // CHG_UT_STOP (T)
  {4, 50},   // DISC_UT_START (T)
  {0, 1000}, // DISC_UT_Delay (T, Delay)
  {4, 50},   // DISC_UT_STOP (T)
  {4, 50},   // MOS_OT_START (T)
  {0, 1000}, // MOS_OT_Delay (T, Delay)
  {4, 50},   // MOS_OT_STOP (T)
  {4, 50},   // ENV_OT_START (T)
  {0, 1000}, // ENV_OT_Delay (T, Delay)
  {4, 50},   // ENV_OT_STOP (T)
  {4, 50},   // ENV_UT_START (T)
  {0, 1000}, // ENV_UT_Delay (T, Delay)
  {4, 50},   // ENV_UT_STOP (T)
  {2, 1000}, // Balance_Start_Vol (V)
  {2, 1000}, // Balance_Start_Diff
  {2, 1000}, // Sleep_Cell_Volt (V)
  {2, 100},  // Shorts_Delay (Delay)
  {0, 60},   // Standby_Time (T)
  {2, 60},   // UV_OFF_Time (T)
  {0, 0},    // LC_Style
  {0, 60},   // Sleep_Time (T)
};

// Conversion operations for alarm parameters
static BasenBMS::PARAM_OPERATION alarm_param_op[] = {
  {2, 1000}, // CELL_OV_Start (V)
  {0, 1000}, // CELL_OV_Delay (Delay)
  {2, 1000}, // CELL_OV_Stop (V)
  {2, 1000}, // CELL_UV_Start (V)
  {0, 1000}, // CELL_UV_Delay (Delay)
  {2, 1000}, // CELL_UV_Stop (V)
  {2, 100},  // PACK_OV_Start (V)
  {0, 1000}, // PACK_OV_Delay (Delay)
  {2, 100},  // PACK_OV_Stop (V)
  {2, 100},  // PACK_UV_Start (V)
  {0, 1000}, // PACK_UV_Delay (Delay)
  {2, 100},  // PACK_UV_Stop (V)
  {2, 100},  // CHG_OC_Start (A)
  {0, 1000}, // CHG_OC_Delay (Delay)
  {2, 100},  // CHG_OC_Stop (A)
  {2, 100},  // DISC_OC_Start (A)
  {0, 1000}, // DISC_OC_Delay (Delay)
  {2, 100},  // DISC_OC_Stop (A)
  {4, 50},   // CHG_OT_START (T)
  {0, 1000}, // CHG_OT_Delay (Delay)
  {4, 50},   // CHG_OT_STOP (T)
  {4, 50},   // CHG_UT_START (T)
  {0, 1000}, // CHG_UT_Delay (Delay)
  {4, 50},   // CHG_UT_STOP (T)
  {4, 50},   // DISC_OT_START (T)
  {0, 1000}, // DISC_OT_Delay (Delay)
  {4, 50},   // DISC_OT_STOP (T)
  {4, 45},   // DISC_UT_START (T)
  {0, 1000}, // DISC_UT_Delay (Delay)
  {4, 45},   // DISC_UT_STOP (T)
  {4, 50},   // MOS_OT_START (T)
  {0, 1000}, // MOS_OT_Delay (Delay)
  {4, 50},   // MOS_OT_STOP (T)
  {0, 1000}, // Capacity_Low_Start (%)
  {0, 1000}, // Capacity_Low_Stop (%)
  {2, 1000}, // Vol_Diff_Start (V)
  {2, 1000}, // Vol_Diff_Stop (V)
  {4, 50},   // ENV_OT_START (T)
  {0, 1000}, // ENV_OT_Delay (Delay)
  {4, 50},   // ENV_OT_STOP (T)
  {4, 50},   // ENV_UT_START (T)
  {0, 1000}, // ENV_UT_Delay (Delay)
  {4, 50},   // ENV_UT_STOP (T)
};


/**
 * @brief Logs the configuration details of the BasenController and its devices.
 *
 * This function outputs information about each device managed by the controller,
 * including its address and timeout count, using the ESP_LOGI logging macro.
 * It also checks the UART settings with a baud rate of 9600.
 */
void BasenController::dump_config() {
  ESP_LOGI(TAG, "Basen:");
  LOG_PIN(" Flow Control Pin: ", this->flow_control_pin_);

  for (auto &device : this->devices_) {
    ESP_LOGI(TAG, " Device %02X:", device->address_);
    ESP_LOGI(TAG, "  Timeout count: %d", device->timeout_count_);
  }

  check_uart_settings(9600);
}


/**
 * @brief Sets the state of the BasenController.
 *
 * This method updates the internal state of the controller if the provided state is valid
 * and different from the current state. If the state is invalid (greater than STATE_RX_DATA),
 * a warning is logged and the state is not changed.
 *
 * @param state The new state to set. Must be less than or equal to STATE_RX_DATA.
 */
void BasenController::set_state(uint8_t state)
{
  if (state > STATE_RX_DATA) {
    ESP_LOGW(TAG, "Invalid state %d", state);
    return;
  }

  if (this->state_ == state)
    return;

  // Change state
  this->state_ = state;
}


/**
 * @brief Empties the RX buffer by reading and discarding all available bytes.
 *
 * This function continuously reads bytes from the RX buffer until it is empty
 * or a maximum of 255 bytes have been read. It returns true if any bytes were
 * read and discarded, otherwise returns false.
 *
 * @return true if any bytes were removed from the RX buffer, false otherwise.
 */
bool BasenController::empty_rx (void) {
  uint8_t data;
  uint8_t count = 255;
  bool result = false;
  while (available() && --count) {
    read_byte(&data);
    //ESP_LOGD(TAG, "Emptying RX buffer: %02X", data);
    result = true;
  }

  return result;
}


/**
 * @brief Calculates a checksum for a given data buffer.
 *
 * This function computes a checksum by iterating over the input data array,
 * performing an XOR operation on each byte to accumulate `num1`, and summing
 * all bytes to accumulate `num2`. The final checksum is obtained by XOR'ing
 * `num1` and `num2`, and masking the result to 8 bits.
 *
 * @param data Pointer to the input data buffer.
 * @param len Number of bytes in the data buffer.
 * @return uint8_t The calculated 8-bit checksum.
 */
uint8_t BasenController::checksum (const uint8_t *data, uint8_t len) {
  uint8_t num1 = 0;
  uint16_t num2 = 0;

  for (uint8_t i = 0; i < len; i++) {
      num1 ^= data[i];
      num2 += data[i];
  }

  return (uint8_t)((num1 ^ num2) & 0xFF);
}

/**
 * @brief Sends a command to the specified BasenBMS device.
 *
 * Constructs a command packet with the given command and BMS address,
 * calculates the checksum, and transmits the packet. The function also
 * updates internal state and timestamps for command transmission.
 *
 * @param BMS Pointer to the BasenBMS device to which the command will be sent.
 * @param command The 16-bit command code to send.
 */
void BasenController::send_command(BasenBMS *BMS, const uint16_t command) {
  uint8_t data[6] = {SOI, BMS->address_, (uint8_t)(command & 0xFF), (uint8_t)(command >> 8), 0xFE, EOI};

  // Calculate checksum
  // Examples: (Checksum byte is before EOI 0x0d)
  // 7e 01 01 00 fe 0d
  // 7e 01 42 00 fc 0d
  // 7e 01 dc 03 06 00 00 c2 0d
  // 7e 01 33 00 fe 0d
  data[4] = this->checksum(data, 4);

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  write_array(data, sizeof(data));
  
  if (this->flow_control_pin_ != nullptr) {
    this->flush();
    this->flow_control_pin_->digital_write(false);
  }

  memcpy(this->header_, data, sizeof(header_));

  ESP_LOGV(TAG, "Sending command: %02X %02X %02X %02X %02X %02X", data[0], data[1], data[2], data[3], data[4], data[5]);

  uint32_t now = millis();
  this->last_command_ = now;
  BMS->last_transmission_ = now;

  set_state (STATE_COMMAND_TX);
}

/**
 * @brief Checks if the current device communication has timed out.
 *
 * This method compares the elapsed time since the last transmission to the configured timeout value.
 * If a timeout is detected, it logs a warning, increments the device's timeout counter, updates the
 * connected binary sensor to indicate disconnection, marks the device state as done, resets the current
 * device pointer, and sets the controller state to bus check.
 */
void BasenController::check_timeout ()
{
  // Check for timeout while waiting for data
  uint32_t now = millis();
  uint32_t elapsed = now - current_->last_transmission_;
  if (elapsed > timeout_) {
    // Timeout while waiting for data.
    ESP_LOGW(TAG, "Timeout (%d ms) while waiting for data (%d < %d) for address %02X", elapsed, this->frame_size_, this->rx_required_, current_->address_);
    current_->timeout_count_++;

    if (retry_++ > 0) {
      // Second try, mark as offline and move to next device
      current_->set_connected(false);

      current_->state_ = BasenBMS::BMS_STATE_DONE;  // Mark as finished
      current_ = NULL;  // Reset current device
      retry_ = 0;
    }
    set_state (STATE_BUS_CHECK);
  }
}

/**
 * @brief Updates the current device by sending appropriate commands based on its state.
 *
 * This method checks the current device and sends commands to retrieve information
 * such as BMS version, barcode, parameters, or general info, depending on what data
 * is missing or outdated. It ensures a minimum delay between consecutive commands
 * to the device. The sequence of commands is as follows:
 *   - If the BMS version is not available, request it.
 *   - Else, if the barcode is not available, request it.
 *   - Else, if the parameters have not been received, request them.
 *   - Else, if more than 1 second has passed since the last transmission, request general info.
 *
 * The function returns immediately if there is no current device or if the minimum
 * command delay has not elapsed since the last command.
 */
void BasenController::update_device ()
{
  if (current_ == NULL)
    return;

  // Keep minimum delay between commands
  uint32_t now = millis();
  if ((now - this->last_command_) < this->command_delay_) {
    // Wait for the next command
    return;
  }

  if (!current_->bms_version_received_) {
    // Get BMS version
    this->send_command(current_, COMMAND_BMS_VERSION);
  } else if (!current_->barcode_received_) {
    // Get barcode
    this->send_command(current_, COMMAND_BARCODE);
  } else if (!current_->params_received_) {
    // Get parameters
    this->send_command(current_, COMMAND_PARAMETERS);
  } else if (current_->params_received_ == 1) {
    // Get alarm parameters
    this->send_command(current_, COMMAND_ALARM_PARAMETERS);
  } else if ((now-current_->last_transmission_) > 1000) {
    // Get info
    this->send_command(current_, COMMAND_INFO);
  }
}

/**
 * @brief Main UART communication loop for the BasenController.
 *
 * This function manages the UART state machine for communicating with devices on the bus.
 * It checks the current state and performs actions such as checking for bus idleness,
 * sending commands, waiting for responses, receiving and validating data frames, and
 * handling errors or timeouts.
 *
 * The function processes the following states:
 * - STATE_BUS_CHECK: Verifies if the bus is idle and clears the RX buffer if necessary.
 * - STATE_IDLE: Initiates device update by sending commands.
 * - STATE_COMMAND_TX: Prepares to receive a response after sending a command.
 * - STATE_RX_HEADER: Receives and validates the header of the incoming data frame.
 * - STATE_RX_DATA: Receives the data payload and validates the frame.
 *
 * The function also handles frame buffer management, checks for buffer overflows,
 * validates frame headers, checks data length, verifies checksums, and processes
 * received data or error codes.
 *
 * State transitions are managed internally, and appropriate logging is performed
 * for debugging and error reporting.
 */
void BasenController::uart_loop() {
  // Check state
  switch (this->state_) {
    case STATE_BUS_CHECK:
      // Check if there are no transmissions on the bus
      if (millis() - last_bus_check_ >= 3000) {
        // Check and empty RX buffer
        bool ret = empty_rx();
        this->last_bus_check_ = millis();

        if (ret) {
          ESP_LOGW(TAG, "Bus is not idle...");
          // Check again in 3 seconds
          return;
        }
        ESP_LOGD(TAG, "Bus is idle");
        set_state (STATE_IDLE);
      }
      return;
    case STATE_IDLE:
      // Start sending commands
      this->update_device();
      return;
    case STATE_COMMAND_TX:
      // Command was sent, wait for response
      set_state (STATE_RX_HEADER);
      // Minimum 6 bytes response
      this->rx_required_ = 6;
      this->frame_size_ = 0;
      return;
    case STATE_RX_HEADER:
    case STATE_RX_DATA:
      if (current_ == NULL) {
        ESP_LOGW(TAG, "No device updating, cannot receive data");
        set_state (STATE_BUS_CHECK);
        return;
      }
      break;
  }

  // Check if we have received the data
  int available_bytes = available();
  if (available_bytes) {
    // Check if we have enough space in the frame buffer
    if (this->frame_size_ + available_bytes > sizeof(this->frame_)) {
      ESP_LOGW(TAG, "Frame buffer overflow, clearing RX buffer");
      set_state (STATE_BUS_CHECK);
      return;
    }

    // Copy available data to the frame buffer
    read_array(this->frame_ + this->frame_size_, available_bytes);
    this->frame_size_ += available_bytes;
  }

  // Check if we have enough data
  if (this->frame_size_ < this->rx_required_) {
    // No, check if we have a timeout
    check_timeout();
    return;
  }

  if (state_ == STATE_RX_HEADER) {
    ESP_LOGV(TAG, "Received header: %02X %02X %02X %02X", frame_[0], frame_[1], frame_[2], frame_[3]);

    if (frame_[0] != SOI || frame_[1] != this->header_[1] || frame_[2] != this->header_[2]) {
        ESP_LOGW(TAG, "Header does not match expected values: %02X %02X %02X != %02X %02X %02X", 
                 frame_[0], frame_[1], frame_[2], this->header_[0], this->header_[1], this->header_[2]);
        set_state (STATE_BUS_CHECK);
        return;
    }

    // Check length
    uint8_t length = frame_[3];

    if (length == 0x00) {
      // Command echo, verify EOI
      if (frame_[5] != EOI) {
        ESP_LOGW(TAG, "Invalid EOI");
        set_state (STATE_BUS_CHECK);
      }

      return;
    }

    if (length > 0xF0) {
      // Invalid length
      ESP_LOGW(TAG, "Invalid length");
      set_state (STATE_BUS_CHECK);
      return;
    }

    // Check if we have enough data
    this->rx_required_ = 4 + length + 2;  // Header + data + checksum + EOI

    set_state (STATE_RX_DATA);

    // Return here to keep loop time short
    return;
  }

  if (this->rx_required_ == 7) {
    // Only one data byte, most likely an error code:
    // 00H Normal
    // 01H VER Error
    // 02H CHKSUM Error
    // 03H LCHKSUM Error
    // 04H CID2 Invalid
    // 05H Command Format Error
    // 06H Invalid data
    ESP_LOGW(TAG, "Received error: %02X %02X %02X %02%", frame_[1], frame_[2], frame_[3], frame_[4]);
    set_state (STATE_BUS_CHECK);
    return;
  }

  // ESP_LOGD(TAG, "Received data: ");
  // for (size_t i = 0; i < this->data_required_; i++) {
  //   ESP_LOGD(TAG, "%02X ", data[i]);
  // }
  // ESP_LOGD(TAG, "");

  // Check EOI
  if (frame_[frame_size_ - 1] != EOI) {
    ESP_LOGW(TAG, "Invalid EOI byte: %02X", frame_[frame_size_ - 1]);
    set_state (STATE_BUS_CHECK);
    return;
  }

  uint8_t *data = frame_ + sizeof(header_);
  uint8_t data_size = frame_[3];

  // Verify checksum
  uint8_t checksum = this->checksum(frame_, sizeof(header_) + data_size);
  
  if (checksum != data[data_size]) {
    ESP_LOGW(TAG, "Checksum mismatch: %02X != %02X", checksum, data[data_size]);
    set_state (STATE_BUS_CHECK);
    return;
  }
  
  // Handle data
  if (current_->handle_data (header_, data, data_size)) {
    // Got all data for current BMS
    current_ = NULL;
  }

  set_state (STATE_IDLE);
}

/**
 * @brief Queues a BasenBMS device for updating.
 *
 * If there is no device currently being updated, this function sets the provided
 * device as the current device and marks its state as updating. If a device is
 * already being updated, the function returns immediately without making changes.
 *
 * @param device Pointer to the BasenBMS device to be queued for updating.
 */
void BasenController::queue_device(BasenBMS *device) {
  // Check if we already have a device updating
  if (current_ != NULL)
    return;

  // No, set current device
  device->state_ = BasenBMS::BMS_STATE_UPDATING;
  current_ = device;
  retry_ = 0;
  ESP_LOGV(TAG, "Active device with address %02X", device->address_);
}

/**
 * @brief Main loop function for the BasenController.
 *
 * This function manages the state and operation of all connected BasenBMS devices.
 * - Enables the controller after a 10-second delay from startup.
 * - If not enabled, resets the current device and sets the controller state to STATE_BUS_CHECK.
 * - Iterates through all managed devices and:
 *   - Queues devices that require an update.
 *   - Publishes data for devices ready to publish, then exits early to keep loop time low.
 * - Resets devices that have completed their operation back to the idle state.
 * - Calls the UART loop handler at the end of each cycle.
 */
void BasenController::loop() {
  if (millis() >= 1000*10)
    enable_ = true;  // Enable after 10 seconds

  // Check if enabled
  if (!enable_) {
    if (current_ != NULL) {
      // Reset current device
      current_->state_= BasenBMS::BMS_STATE_IDLE;
      current_ = NULL;
    }

    for (auto &device : this->devices_) {
      device->set_connected(false);
      // Retrieve version, barcode, parameters again when enabled
      device->bms_version_received_ = false;
      device->barcode_received_ = false;
      device->params_received_ = 0;
    }

    set_state (STATE_BUS_CHECK);
    return;
  }

  uint8_t found = 0;

  // Check which devices need an update
  for (auto &device : this->devices_) {
    switch (device->state_) {
      default:
        // Device is idle, nothing to do
        break;
      case BasenBMS::BMS_STATE_WANT_UPDATE:
        // Update this device
        queue_device(device);
        found++;
        break;
      case BasenBMS::BMS_STATE_PUBLISH:
        device->publish();
        // Keep the loop time low
        return;
    }
  }

  if (!found) {
    // Remove finished devices
    for (auto &device : this->devices_) {
      if (device->state_ == BasenBMS::BMS_STATE_DONE)
        device->state_ = BasenBMS::BMS_STATE_IDLE;
    }
  }

  // Call the UART loop
  uart_loop();
}

/**
 * @brief Triggers an update request for the BasenBMS.
 *
 * If the current state is BMS_STATE_IDLE, this method sets the state to BMS_STATE_WANT_UPDATE,
 * indicating that an update is desired. This function is typically called to initiate a new
 * update cycle for the BMS (Battery Management System).
 */
void BasenBMS::update()
{
  if (this->state_ == BMS_STATE_IDLE) {
    this->state_ = BMS_STATE_WANT_UPDATE;  // Set update flag
  }
}

/**
 * @brief Publishes the current status of the Basen BMS.
 *
 * This function performs two main tasks:
 * 1. It formats the internal status bitmask as a colon-separated hexadecimal string
 *    and publishes it via the `status_bitmask_sensor_`.
 * 2. It constructs a human-readable status message by checking each defined status message
 *    against the status bitmask. If any status bits are set, their corresponding messages
 *    are concatenated and published via the `status_sensor_`. If no status bits are set,
 *    "Idle" is published as the status.
 */
void BasenBMS::publish_status()
{
  // Create a string containing all bytes
  char status_bitmask[40] = {0};

  for (uint8_t i = 0; i < sizeof (this->status_bitmask_); i++) {
    sprintf (status_bitmask + i * 3, "%02X:", this->status_bitmask_[i]);
  }

  status_bitmask[sizeof (this->status_bitmask_)*3 - 1] = '\0';
  if (this->status_bitmask_sensor_)
    this->status_bitmask_sensor_->publish_state(status_bitmask);

  // Create status message and alarm/fault status
  std::string status_message;
  uint16_t error_bitmask = 0;
  bool alarm = false;
  bool fault = false;

  for (uint8_t i = 0; i < sizeof(status_messages) / sizeof(status_messages[0]); i++) {
    if (this->status_bitmask_[status_messages[i].offset] & status_messages[i].bit) {
      // Check for alarm
      if (status_messages[i].type == 1)
        alarm = true;

      // Check for fault
      if (status_messages[i].type == 2)
        fault = true;

      // Create error bitmask
      error_bitmask |= status_messages[i].error_mask;

      // Create status message
      if (!status_message.empty())
        status_message += ", ";
      status_message += status_messages[i].message;
    }
  }

  if (status_message.empty()) {
    status_message = "Idle";
  }

  if (this->alarm_binary_sensor_) {
    this->alarm_binary_sensor_->publish_state(alarm);
  }
  if (this->fault_binary_sensor_) {
    this->fault_binary_sensor_->publish_state(fault);
  }
  if (this->charging_enabled_binary_sensor_) {
    // Check if charge MosFET is off: Byte 2, Bit 4 (0x10)
    this->charging_enabled_binary_sensor_->publish_state(!(this->status_bitmask_[2] & 0x10));
  }
  if (this->discharging_enabled_binary_sensor_) {
    // Check if discharge MosFET is off: Byte 2, Bit 5 (0x20)
    this->discharging_enabled_binary_sensor_->publish_state(!(this->status_bitmask_[2] & 0x20));
  }
  if (this->error_bitmask_sensor_)
    this->error_bitmask_sensor_->publish_state(error_bitmask);

  if (this->status_sensor_)
    this->status_sensor_->publish_state(status_message);

  if (this->cell_balancing_bitmask_sensor_) {
    std::string balancing;
    for (uint8_t i = 0; i < 16; i++) {
      if (this->cell_balancing_ & (1 << i))
        balancing += '1';
      else
        balancing += '0';
    }

    this->cell_balancing_bitmask_sensor_->publish_state(balancing);
  }
}

/**
 * @brief Publishes sensor data in blocks to reduce loop time.
 *
 * This method checks if the current state is BMS_STATE_PUBLISH. If so, it publishes
 * sensor readings in multiple steps (blocks) based on the value of publish_count_.
 * Each case in the switch statement publishes a group of related sensor values and
 * increments publish_count_ to move to the next block on the next call.
 * When all blocks have been published, publish_count_ is reset and the state is set to BMS_STATE_DONE.
 *
 * The publishing is divided as follows:
 *   - Block 0: Voltage, current, power, capacity, state of charge (SOC), state of health (SOH), and cycles.
 *   - Block 1: Average, minimum, and maximum cell voltages, their indices, and cell voltage delta.
 *   - Block 2: Temperatures (cells, MOSFET, ambient) and status.
 *
 * This approach helps to distribute the workload across multiple loop iterations, reducing the time spent in each loop.
 */
void BasenBMS::publish()
{
  if (this->state_ != BMS_STATE_PUBLISH)
    return;

  // Publish sensors in blocks to reduce loop time
  switch (this->publish_count_) {
    case 0:
      if (voltage_sensor_)
        voltage_sensor_->publish_state(this->voltage_);
      if (current_sensor_)
        current_sensor_->publish_state(this->current_);
      if (power_sensor_)
        power_sensor_->publish_state(this->voltage_ * this->current_);
      if (capacity_sensor_)
        capacity_sensor_->publish_state(this->capacity_);
      if (soc_sensor_)
        soc_sensor_->publish_state(this->soc_);
      if (soh_sensor_)
        soh_sensor_->publish_state(this->soh_);
      if (cycles_sensor_)
        cycles_sensor_->publish_state(this->cycles_);
      this->publish_count_++;
      break;
    case 1:
      if (avg_cell_voltage_sensor_)
        avg_cell_voltage_sensor_->publish_state(this->cell_avg_voltage_);
      if (min_cell_voltage_sensor_)
        min_cell_voltage_sensor_->publish_state(this->cell_min_voltage_);
      if (max_cell_voltage_sensor_)
        max_cell_voltage_sensor_->publish_state(this->cell_max_voltage_);
      if (min_cell_index_sensor_)
        min_cell_index_sensor_->publish_state(this->cell_min_index_);
      if (max_cell_index_sensor_)
        max_cell_index_sensor_->publish_state(this->cell_max_index_);
      if (delta_cell_voltage_sensor_)
        delta_cell_voltage_sensor_->publish_state(this->cell_max_voltage_ - this->cell_min_voltage_);
      this->publish_count_++;
      break;
    case 2:
      for (uint8_t i = 0; i < 8; i++) {
        if (cell_voltage_sensor_[i])
          cell_voltage_sensor_[i]->publish_state(this->cell_voltage_[i]);
      }
      this->publish_count_++;
      break;
    case 3:
      for (uint8_t i = 8; i < 16; i++) {
        if (cell_voltage_sensor_[i])
          cell_voltage_sensor_[i]->publish_state(this->cell_voltage_[i]);
      }
      this->publish_count_++;
      break;
    case 4:
      if (temperature_sensor_[0])
        temperature_sensor_[0]->publish_state(this->temperature_[0]);
      if (temperature_sensor_[1])
        temperature_sensor_[1]->publish_state(this->temperature_[1]);
      if (temperature_sensor_[2])
        temperature_sensor_[2]->publish_state(this->temperature_[2]);
      if (temperature_sensor_[3])
        temperature_sensor_[3]->publish_state(this->temperature_[3]);
      if (temperature_sensor_[4])
        temperature_sensor_[4]->publish_state(this->temperature_mos_);
      if (temperature_sensor_[5])
        temperature_sensor_[5]->publish_state(this->temperature_ambient_);
      publish_status();
      this->publish_count_++;
      break;
    default:
      this->publish_count_ = 0;
      this->state_ = BMS_STATE_DONE;
      break;
  }
}

/**
 * @brief Handles the parsing and processing of information data received from the Basen controller.
 *
 * This function processes a data packet containing various information such as cell voltages,
 * current, state of charge (SoC), capacity, temperatures, status bitmask, cycles, total voltage,
 * and state of health (SoH). The function validates the packet length and version, then iterates
 * through the data fields, updating the corresponding member variables of the controller.
 *
 * @param data Pointer to the received data buffer.
 * @param length Length of the received data buffer.
 *
 * The expected data format is:
 * - Byte 0: Version (must be 0x01)
 * - Bytes 1..n: Data fields, each consisting of:
 *   - Type (1 byte)
 *   - Count (1 byte)
 *   - Data (variable length, depending on type and count)
 *
 * Logs warnings if the data is invalid or if any field is out of expected range.
 */
void BasenBMS::handle_info (const uint8_t *data, uint8_t length) {
  // Handle info data
  if (length != 0x7C) {
    ESP_LOGW(TAG, "Invalid length for COMMAND_INFO: %d", length);
    return;
  }
  if (data[0] != 0x01) {
    ESP_LOGW(TAG, "Invalid version for COMMAND_INFO: %02X", data[0]);
    return;
  }

  uint8_t *position = const_cast<uint8_t *>(data) + 1;  // Skip version byte
  length--;

  uint8_t processed = 0;
  // Process cell voltages
  processed = handle_cell_voltages(position, length);

  if (processed == 0) {
    return;
  }

  position += processed;
  length -= processed;

  while (length > 0) {
    if (length < 2) {
      ESP_LOGW(TAG, "Invalid length for next position: %d", length);
      return;
    }

    uint8_t type = position[0];  // Data type
    uint8_t count = position[1]; // Number of elements
    // Size in bytes of one element
    uint8_t size = 2;
    if (type > 0x0A)
        size = 4;

    // Get first data element
    uint16_t data16 = 0;
    if (count && length >= 4) {
      data16 = (position[2] << 8) | position[3];
    }
  
    switch (position[0]) {
      case 0x02:  // Current
        this->current_ = (data16 / -100.0f) + 300;  // Convert to A
        break;
      case 0x03:  // SoC
        this->soc_ = data16 / 100.0f;  // Publish SoC in percentage
        break;
      case 0x04:  // Capacity
        this->capacity_ = data16 / 100;  // Publish capacity in Ah
        break;
      case 0x05:  // Temperature
        if (count > 6) {
          ESP_LOGW(TAG, "Invalid temperature count: %d", count);
          break;
        }
        for (uint8_t i = 0; i < count; i++) {
          uint8_t type = position[2 + 2*i];  // Type of temperature sensor
          int8_t value = position[2 + 2*i + 1] - 50;  // Value of temperature sensor
          if (type == 0x40) {
            // MOS temperature
            this->temperature_mos_ = value;
          } else if (type == 0x20) {
            // Ambient temperature
            this->temperature_ambient_ = value;
          } else {
            // Normal temperature sensor
            if (i < 4)
              this->temperature_[i] = value;
          }
        }
        break;
      case 0x06:  // Status bitmask
        // Create a string containing all bytes
        if (!count || (count > 5))
          break;
        memcpy (this->status_bitmask_, position + 2, count * size);
        break;
      case 0x07:  // Cycles
        this->cycles_ = data16;
        break;
      case 0x08:  // Total voltage
        this->voltage_ = data16 / 100.0f;
        break;
      case 0x09:  // SoH
        this->soh_ = data16 / 100.0f;
        break;
      default:        
        ESP_LOGV(TAG, "Data type %02X with count %d", type, count);
    }

    // Advance to next data type
    uint16_t data_size = count * size + 2;  // 2 bytes for type and count
    if (length < data_size) {
      ESP_LOGW(TAG, "Not enough data for type %02X with count %d", type, count);
      break;
    }

    length -= data_size;
    position += data_size;
  }
}

/**
 * @brief Handles incoming data packets for the BasenController.
 *
 * This function processes data received from the BMS (Battery Management System)
 * based on the command specified in the header. It updates the relevant sensors
 * or handles parameters accordingly. If the command is not recognized, a warning
 * is logged.
 *
 * @param header Pointer to the header of the sent packet. The third byte (header[2]) specifies the command.
 * @param data Pointer to the data payload associated with the command.
 * @param length Length of the data payload.
 * 
 * @return true  - Done
 *         false - Waiting for next data
 */
bool BasenBMS::handle_data (const uint8_t *header, const uint8_t *data, uint8_t length) {
  // Handle data based on the command
  switch (header[2]) {
    case COMMAND_BMS_VERSION:
      // BMS Version
      if (this->bms_version_text_sensor_) {
        this->bms_version_text_sensor_->publish_state(std::string(reinterpret_cast<const char *>(data), length));
        ESP_LOGD(TAG, "Address: %d BMS Version: %s", this->address_, this->bms_version_text_sensor_->get_state().c_str());
      }
      this->bms_version_received_ = true;  // Mark as received
      break;
    case COMMAND_BARCODE:
      // Barcode
      if (this->barcode_text_sensor_) {
        this->barcode_text_sensor_->publish_state(std::string(reinterpret_cast<const char *>(data), length));
        ESP_LOGD(TAG, "Address: %d Barcode: %s", this->address_, this->barcode_text_sensor_->get_state().c_str());
      }
      this->barcode_received_ = true;  // Mark as received
      break;
    case (uint8_t)COMMAND_PARAMETERS:
      if (header[3] == (uint8_t)(COMMAND_PARAMETERS >> 8)) {
        ESP_LOGV(TAG, "Received protect parameters");
        handle_parameters(data, length, this->params_protect_, BASEN_BMS_PROTECT_PARAMETERS, protect_param_op, this->param_sensor_);
      } else {
        ESP_LOGV(TAG, "Received alarm parameters");
        handle_parameters(data, length, this->params_alarm_, BASEN_BMS_ALARM_PARAMETERS, alarm_param_op, this->param_alarm_sensor_);
      }
      break;
    case COMMAND_INFO:
      handle_info(data, length);
      // All data has been updated, mark the device as updated and connected
      this->state_ = BasenBMS::BMS_STATE_PUBLISH;
      set_connected (true);
      return true;
    default:
      ESP_LOGW(TAG, "Unknown command: %02X", header[2]);
      return true;
  }

  return false;
}

/**
 * @brief Handles and processes cell voltage data from a given byte array.
 *
 * This function parses the provided data buffer to extract individual cell voltages,
 * calculates the average, minimum, and maximum cell voltages, and updates the current_
 * object with these values and their corresponding cell indices. It also performs
 * validation on the input data to ensure correctness.
 *
 * @param data   Pointer to the byte array containing cell voltage data.
 *               The first byte indicates the number of cells, followed by
 *               two bytes per cell representing the voltage in millivolts (big-endian).
 * @param length Length of the data buffer in bytes.
 * @return The number of bytes processed from the data buffer, or 0 if an error occurred.
 */
uint8_t BasenBMS::handle_cell_voltages (const uint8_t *data, uint8_t length) {
  if (length < 1) {
    ESP_LOGW(TAG, "Invalid length for cell voltages: %d", length);
    return 0;
  }
  length--;
  uint8_t num_cells = data[0];  // First byte is the number of cells

  if (num_cells == 0) {
    ESP_LOGW(TAG, "No cells found in data");
    return 0;
  }

  if ((num_cells*2) > length) {
    ESP_LOGW(TAG, "Number of cells (%d) exceeds data length (%d)", num_cells, length);
    return 0;
  }

  float cell_avg_voltage = 0.0f;
  float cell_min_voltage = 1000.0f;
  float cell_max_voltage = 0.0f;
  uint8_t min_cell_index = 0;
  uint8_t max_cell_index = 0;
  for (uint8_t i = 0; i < num_cells; i++) {
    // Each cell voltage is represented by two bytes, high bit indicates balancing (I assume...)
    uint16_t cell_voltage = (data[i*2 + 1] << 8) | data[i*2 + 2];
    bool balancing = cell_voltage & 0x8000;
    // Convert to V
    float voltage_V = (cell_voltage & 0xFFF) / 1000.0f;

    if (i < 16) {
      this->cell_voltage_[i] = voltage_V;
      if (balancing)
        this->cell_balancing_ |= (1 << i);
      else
        this->cell_balancing_ &= ~(1 << i);      
    }
    ESP_LOGV(TAG, "Cell %d voltage: %.3f V", i + 1, voltage_V);

    // Update average, min and max voltages
    cell_avg_voltage += voltage_V;
    if (voltage_V < cell_min_voltage) {
      cell_min_voltage = voltage_V;
      min_cell_index = i + 1;  // Cell index starts at 1
    }
    if (voltage_V > cell_max_voltage) {
      cell_max_voltage = voltage_V;
      max_cell_index = i + 1;  // Cell index starts at 1
    }

    // Publish cell voltage
  }

  cell_avg_voltage /= num_cells;  // Calculate average voltage

  // Publish calculated values
  this->cell_avg_voltage_ = cell_avg_voltage;
  this->cell_min_voltage_ = cell_min_voltage;
  this->cell_max_voltage_ = cell_max_voltage;
  this->cell_min_index_ = min_cell_index;
  this->cell_max_index_ = max_cell_index;

  // Return the number of bytes processed
  return 1 + num_cells * 2;
}

/**
 * @brief Handles and parses parameter data received from the BMS.
 *
 * This function processes a buffer containing parameter data, extracts each parameter
 * based on its type and size, and stores the values in the `params_protect_` array.
 * It then applies a corresponding operation (multiply, divide, add, subtract, or none)
 * to each parameter value as specified in the `param_operation` array.
 * Finally, it publishes the processed parameter values to their associated sensors.
 *
 * The function performs several checks:
 * - Ensures the data length is valid.
 * - Verifies that each parameter entry has a supported size (2 bytes).
 * - Checks that the parameter type index is within bounds.
 * - Confirms that the size of the `param_operation` array matches the size of `params_protect_`.
 *
 * @param data   Pointer to the buffer containing parameter data.
 * @param length Length of the data buffer.
 */
void BasenBMS::handle_parameters (const uint8_t *data, const uint8_t length, uint16_t *params, uint8_t params_count, PARAM_OPERATION *ops, sensor::Sensor **sensors) {
  params_received_++;  // Mark parameters as received

  if (length < 4) {
    ESP_LOGW(TAG, "Invalid length for COMMAND_PARAMETERS: %d", length);
    return;
  }

  uint8_t position = 0;
  while (position < length) {
    uint8_t type = data[position++];
    uint8_t size = data[position++];

    if (size != 2) {
      ESP_LOGW(TAG, "Unsupported size for type %02X: %d", type, size);
      return;
    }

    if (type >= params_count) {
      ESP_LOGW(TAG, "Invalid type %02X", type);
      return;
    }

    params[type] = (data[position] << 8) | data[position + 1];

    position += size;
  }

  // Publish parameters
  for (size_t i = 0; i < params_count; i++) {
    // Convert parameter
    float value = params[i];
    switch (ops[i].Op) {
      case 1: // Multiply
        value *= ops[i].Value;
        break;
      case 2: // Divide
        value /= ops[i].Value;
        break;
      case 3: // Add
        value += ops[i].Value;
        break;
      case 4: // Subtract
        value -= ops[i].Value;
        break;
      default:
        break; // No operation
    }

    if (sensors[i] != nullptr) {
      sensors[i]->publish_state(value);
    }
  }
}

void BasenBMS::dump_config() {

}

}  // namespace basen
}  // namespace esphome
