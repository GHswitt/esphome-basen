#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace basen {

class BasenBMS;  // Forward declaration

#define BASEN_BMS_PROTECT_PARAMETERS  0x33
#define BASEN_BMS_ALARM_PARAMETERS    0x2F

class BasenController : public uart::UARTDevice, public Component {
 public:
  void set_throttle(uint32_t throttle) { this->throttle_ = throttle; }

  void register_bms(BasenBMS *bms) {
    this->devices_.push_back(bms);
  }

  void setup () override {
    // Setup UART
    this->parent_->set_baud_rate(9600);
    this->parent_->set_rx_buffer_size(512);
    this->parent_->set_stop_bits(1);
    this->parent_->set_data_bits(8);
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  }
  void dump_config() override;
  void loop() override;

  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  void set_state(uint8_t state);
  bool empty_rx (void);
  uint8_t checksum (const uint8_t *data, uint8_t len);
  void send_command(BasenBMS *BMS, const uint16_t command);

  void queue_device(BasenBMS *bms);
  void update_device();
  void check_timeout();
  void uart_loop();

  // Controller state machine
  enum {
    STATE_BUS_CHECK = 0,
    STATE_IDLE,
    STATE_COMMAND_TX,
    STATE_RX_HEADER,
    STATE_RX_DATA
  };

  uint8_t state_{STATE_BUS_CHECK};

  // Sent header
  uint8_t header_[4]{0};
  // Buffer for received data
  uint8_t frame_[256]{0};
  // Length of the received data  
  uint8_t frame_size_{0};
  // Number of required bytes to receive
  uint8_t rx_required_{0};

  bool enable_{false};
  uint32_t last_bus_check_{0};
  uint32_t last_command_{0};
  uint32_t throttle_{0};

  uint16_t command_delay_{100};  // Delay between commands in ms
  uint16_t timeout_{2000};       // Timeout for command response in ms
  uint8_t  retry_{0};            // Retry counter

  std::vector<BasenBMS *> devices_;
  BasenBMS *current_{nullptr};  // Pointer to the currently processing BMS
};


  // // These are ordered by the type field!
  // typedef struct {
  //   uint16_t CELL_OV_Start;                // 0x00
  //   uint16_t CELL_OV_Delay;                // 0x01
  //   uint16_t CELL_OV_Stop;                 // 0x02
  //   uint16_t CELL_UV_Start;                // 0x03
  //   uint16_t CELL_UV_Delay;                // 0x04
  //   uint16_t CELL_UV_Stop;                 // 0x05
  //   uint16_t PACK_OV_Start;                // 0x06
  //   uint16_t PACK_OV_Delay;                // 0x07
  //   uint16_t PACK_OV_Stop;                 // 0x08
  //   uint16_t PACK_UV_Start;                // 0x09
  //   uint16_t PACK_UV_Delay;                // 0x0A
  //   uint16_t PACK_UV_Stop;                 // 0x0B
  //   uint16_t Const_Pack_V;                 // 0x0C
  //   uint16_t Const_Current;                // 0x0D
  //   uint16_t CHG_OC1_Start;                // 0x0E
  //   uint16_t CHG_OC1_Delay;                // 0x0F
  //   uint16_t DISC_OC1_Start;               // 0x10
  //   uint16_t DISC_OC1_Delay;               // 0x11
  //   uint16_t CHG_OC2_Start;                // 0x12
  //   uint16_t CHG_OC2_Delay;                // 0x13
  //   uint16_t DISC_OC2_Start;               // 0x14
  //   uint16_t DISC_OC2_Delay;               // 0x15
  //   uint16_t CHG_OT_START;                 // 0x16
  //   uint16_t CHG_OT_Delay;                 // 0x17
  //   uint16_t CHG_OT_STOP;                  // 0x18
  //   uint16_t DISC_OT_START;                // 0x19
  //   uint16_t DISC_OT_Delay;                // 0x1A
  //   uint16_t DISC_OT_STOP;                 // 0x1B
  //   uint16_t CHG_UT_START;                 // 0x1C
  //   uint16_t CHG_UT_Delay;                 // 0x1D
  //   uint16_t CHG_UT_STOP;                  // 0x1E
  //   uint16_t DISC_UT_START;                // 0x1F
  //   uint16_t DISC_UT_Delay;                // 0x20
  //   uint16_t DISC_UT_STOP;                 // 0x21
  //   uint16_t MOS_OT_START;                 // 0x22
  //   uint16_t MOS_OT_Delay;                 // 0x23
  //   uint16_t MOS_OT_STOP;                  // 0x24
  //   uint16_t ENV_OT_START;                 // 0x25
  //   uint16_t ENV_OT_Delay;                 // 0x26
  //   uint16_t ENV_OT_STOP;                  // 0x27
  //   uint16_t ENV_UT_START;                 // 0x28
  //   uint16_t ENV_UT_Delay;                 // 0x29
  //   uint16_t ENV_UT_STOP;                  // 0x2A
  //   uint16_t Balance_Start_Vol;            // 0x2B
  //   uint16_t Balance_Start_Diff;           // 0x2C
  //   uint16_t Sleep_Cell_Volt;              // 0x2D
  //   uint16_t Shorts_Delay;                 // 0x2E
  //   uint16_t Standby_Time;                 // 0x2F
  //   uint16_t UV_OFF_Time;                  // 0x30
  //   uint16_t LC_Style;                     // 0x31
  //   uint16_t Sleep_Time;                   // 0x32
  // } PARAMETERS_PROTECT;

class BasenBMS : public PollingComponent {
 public:
  void update();
  void dump_config() override;

  void set_parent(BasenController *parent) { parent_ = parent; parent->register_bms(this); }
  void set_address(uint8_t address) { address_ = address; }

  // Sensors
  void set_bms_version_text_sensor(text_sensor::TextSensor *bms_version_text_sensor) {
    bms_version_text_sensor_ = bms_version_text_sensor;
  }
  void set_barcode_text_sensor(text_sensor::TextSensor *barcode_text_sensor) { barcode_text_sensor_ = barcode_text_sensor; }
  void set_status_bitmask_text_sensor(text_sensor::TextSensor *sensor) {
    status_bitmask_sensor_ = sensor;
  }
  void set_status_text_sensor(text_sensor::TextSensor *sensor) {
    status_sensor_ = sensor;
  }
  void set_cell_balancing_bitmask_text_sensor(text_sensor::TextSensor *sensor) {
    cell_balancing_bitmask_sensor_ = sensor;
  }
  void set_error_bitmask_sensor(sensor::Sensor *sensor) {
    error_bitmask_sensor_ = sensor;
  }

  void set_connected_binary_sensor(binary_sensor::BinarySensor *binary_sensor) {
    connected_binary_sensor_ = binary_sensor;
  }
  void set_alarm_binary_sensor(binary_sensor::BinarySensor *binary_sensor) {
    alarm_binary_sensor_ = binary_sensor;
  }
  void set_fault_binary_sensor(binary_sensor::BinarySensor *binary_sensor) {
    fault_binary_sensor_ = binary_sensor;
  }

  void set_voltage_sensor(sensor::Sensor *sensor) {
    voltage_sensor_ = sensor;
  }
  void set_current_sensor(sensor::Sensor *sensor) {
    current_sensor_ = sensor;
  }
  void set_power_sensor(sensor::Sensor *sensor) {
    power_sensor_ = sensor;
  }
  void set_capacity_sensor(sensor::Sensor *sensor) {
    capacity_sensor_ = sensor;
  }
  void set_state_of_charge_sensor(sensor::Sensor *sensor) {
    soc_sensor_ = sensor;
  }
  void set_state_of_health_sensor(sensor::Sensor *sensor) {
    soh_sensor_ = sensor;
  }
  void set_cycles_sensor(sensor::Sensor *sensor) {
    cycles_sensor_ = sensor;
  }
  void set_temperature1_sensor(sensor::Sensor *sensor) {
    temperature_sensor_[0] = sensor;
  }
  void set_temperature2_sensor(sensor::Sensor *sensor) {
    temperature_sensor_[1] = sensor;
  }
  void set_temperature3_sensor(sensor::Sensor *sensor) {
    temperature_sensor_[2] = sensor;
  }
  void set_temperature4_sensor(sensor::Sensor *sensor) {
    temperature_sensor_[3] = sensor;
  }
  void set_temperature_mos_sensor(sensor::Sensor *sensor) {
    temperature_sensor_[4] = sensor;
  }
  void set_temperature_ambient_sensor(sensor::Sensor *sensor) {
    temperature_sensor_[5] = sensor;
  }
  void set_avg_cell_voltage_sensor(sensor::Sensor *sensor) {
    avg_cell_voltage_sensor_ = sensor;
  }
  void set_min_cell_voltage_sensor(sensor::Sensor *sensor) {
    min_cell_voltage_sensor_ = sensor;
  }
  void set_max_cell_voltage_sensor(sensor::Sensor *sensor) {
    max_cell_voltage_sensor_ = sensor;
  }
  void set_min_cell_index_sensor(sensor::Sensor *sensor) {
    min_cell_index_sensor_ = sensor;
  }
  void set_max_cell_index_sensor(sensor::Sensor *sensor) {
    max_cell_index_sensor_ = sensor;
  }
  void set_delta_cell_voltage_sensor(sensor::Sensor *sensor) {
    delta_cell_voltage_sensor_ = sensor;
  }
  void set_cell_voltage_sensor(sensor::Sensor *sensor, uint8_t index) {
    if (index >= sizeof(cell_voltage_)/sizeof(cell_voltage_[0])) {
      ESP_LOGE("BasenBMS", "Invalid cell sensor index: %d", index);
      return;
    }
    cell_voltage_sensor_[index] = sensor;
  }

  void set_param_sensor(sensor::Sensor *sensor, uint8_t index) {
    if (index >= sizeof(param_sensor_)/sizeof(param_sensor_[0])) {
      ESP_LOGE("BasenBMS", "Invalid parameter sensor index: %d", index);
      return;
    }
    param_sensor_[index] = sensor;
  }

  void set_alarm_param_sensor(sensor::Sensor *sensor, uint8_t index) {
    if (index >= sizeof(param_alarm_sensor_)/sizeof(param_alarm_sensor_[0])) {
      ESP_LOGE("BasenBMS", "Invalid alarm parameter sensor index: %d", index);
      return;
    }
    param_alarm_sensor_[index] = sensor;
  }

  typedef struct {
    uint8_t Op;   // 0 = None, 1 = Multiply, 2 = Divide, 3 = Add, 4 = Subtract
    float Value;  // Value to apply the operation with
  } PARAM_OPERATION;

 protected:
  friend BasenController;
  uint8_t address_{1};
  uint16_t timeout_count_{0};

  enum {
    BMS_STATE_IDLE = 0,
    BMS_STATE_WANT_UPDATE,
    BMS_STATE_UPDATING,
    BMS_STATE_PUBLISH,
    BMS_STATE_DONE
  };
  uint8_t state_{0};
  uint32_t last_transmission_{0};

  // Sensors
  bool bms_version_received_{false};
  bool barcode_received_{false};
  text_sensor::TextSensor *bms_version_text_sensor_{nullptr};
  text_sensor::TextSensor *barcode_text_sensor_{nullptr};
  binary_sensor::BinarySensor *connected_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *alarm_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *fault_binary_sensor_{nullptr};

  // Sensors for BMS data
  float voltage_{0.0f};
  float current_{0.0f};
  uint16_t capacity_{0};
  float soc_{0.0f};
  float soh_{0.0f};
  uint16_t cycles_{0};
  uint16_t cell_balancing_{0};
  float cell_voltage_[16]{0.0f};
  float cell_avg_voltage_{0.0f};
  float cell_min_voltage_{0.0f};
  float cell_max_voltage_{0.0f};
  uint8_t cell_min_index_{0};
  uint8_t cell_max_index_{0};
  int8_t temperature_[4]{0, 0, 0, 0}; // Temperature sensors (4 sensors)
  int8_t temperature_mos_{0};         // MOSFET temperature
  int8_t temperature_ambient_{0};     // Ambient temperature
  uint8_t status_bitmask_[10]{0};     // Status bitmask for BMS

  // Protect parameters
  uint16_t params_protect_[BASEN_BMS_PROTECT_PARAMETERS];  // Protect parameters
  uint16_t params_alarm_[BASEN_BMS_ALARM_PARAMETERS];      // Alarm parameters
  uint8_t params_received_{0};  // Flags to check if parameters have been received

  void publish(void);
  void publish_status();

  bool handle_data (const uint8_t *header, const uint8_t *data, uint8_t length);
  void handle_info(const uint8_t *data, uint8_t length);
  uint8_t handle_cell_voltages(const uint8_t *data, uint8_t length);

  void handle_parameters (const uint8_t *data, const uint8_t length, uint16_t *params, uint8_t params_count, PARAM_OPERATION *ops, sensor::Sensor **sensors);
  
  void set_connected (bool connected) {
    if (this->connected_binary_sensor_ == nullptr)
      return;

    if (connected) { 
      if (!this->connected_binary_sensor_->state)
        this->connected_binary_sensor_->publish_state(true);
    } else {
      this->connected_binary_sensor_->publish_state(false);
    }
  }

private:
  BasenController *parent_;

  uint8_t publish_count_{0};  // Counter for publish calls

  // Sensors
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *capacity_sensor_{nullptr};
  sensor::Sensor *soc_sensor_{nullptr};
  sensor::Sensor *soh_sensor_{nullptr};
  sensor::Sensor *cycles_sensor_{nullptr};
  sensor::Sensor *cell_voltage_sensor_[16]{nullptr};
  sensor::Sensor *avg_cell_voltage_sensor_{nullptr};
  sensor::Sensor *min_cell_voltage_sensor_{nullptr};
  sensor::Sensor *max_cell_voltage_sensor_{nullptr};
  sensor::Sensor *min_cell_index_sensor_{nullptr};
  sensor::Sensor *max_cell_index_sensor_{nullptr};
  sensor::Sensor *delta_cell_voltage_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_[6]{nullptr};
  sensor::Sensor *error_bitmask_sensor_{nullptr};
  text_sensor::TextSensor *status_bitmask_sensor_{nullptr};
  text_sensor::TextSensor *status_sensor_{nullptr};
  text_sensor::TextSensor *cell_balancing_bitmask_sensor_{nullptr};

  sensor::Sensor *param_sensor_[BASEN_BMS_PROTECT_PARAMETERS]{nullptr};  // Array for protect parameter sensors
  sensor::Sensor *param_alarm_sensor_[BASEN_BMS_ALARM_PARAMETERS]{nullptr};  // Array for alarm parameter sensors
  
};

}  // namespace basen
}  // namespace esphome
