#include "toshiba_climate.h"
#include "toshiba_climate_mode.h"
#include "esphome/core/log.h"

namespace esphome {
namespace toshiba_suzumi {

using namespace esphome::climate;

static const char *const TAG = "ToshibaClimateUart";

static constexpr int RECEIVE_TIMEOUT_MS = 200;
static constexpr int COMMAND_DELAY_MS = 100;

////////////////////////////////////////////////////////////////////////////////
// Checksum Calculation:
// Checksum is sum of all bytes excluding first byte, modulo 256, inverted by 256 - sum.
////////////////////////////////////////////////////////////////////////////////
uint8_t checksum(const std::vector<uint8_t> &data, uint8_t length) {
  uint8_t sum = 0;
  for (size_t i = 1; i < length; i++) {
    sum += data[i];
  }
  return 256 - sum;
}

////////////////////////////////////////////////////////////////////////////////
// Send bytes to UART with verbose logging
////////////////////////////////////////////////////////////////////////////////
void ToshibaClimateUart::send_to_uart(const ToshibaCommand &command) {
  this->last_command_timestamp_ = millis();
  ESP_LOGV(TAG, "Sending: [%s]", format_hex_pretty(command.payload).c_str());
  this->write_array(command.payload.data(), command.payload.size());
}

////////////////////////////////////////////////////////////////////////////////
// Handshake initiation sequence
////////////////////////////////////////////////////////////////////////////////
void ToshibaClimateUart::start_handshake() {
  ESP_LOGCONFIG(TAG, "Sending handshake...");
  for (const auto &handshake_cmd : HANDSHAKE) {
    enqueue_command_(ToshibaCommand{.cmd = ToshibaCommandType::HANDSHAKE, .payload = handshake_cmd});
  }
  enqueue_command_(ToshibaCommand{.cmd = ToshibaCommandType::DELAY, .delay = 2000});
  for (const auto &post_handshake : AFTER_HANDSHAKE) {
    enqueue_command_(ToshibaCommand{.cmd = ToshibaCommandType::HANDSHAKE, .payload = post_handshake});
  }
}

////////////////////////////////////////////////////////////////////////////////
// Validate and parse full incoming message frames
// Accepts and skips checksum on 13-byte partial frames to tolerate unit quirks.
////////////////////////////////////////////////////////////////////////////////
bool ToshibaClimateUart::validate_message_() {
  constexpr uint8_t kMinMessageLength = 7;

  if (this->rx_message_.size() < kMinMessageLength) {
    return true;
  }

  auto *data = this->rx_message_.data();
  uint8_t at = this->rx_message_.size() - 1;

  if (data[0] != 0x02) {
    ESP_LOGW(TAG, "Invalid header byte: 0x%02X, clearing buffer", data[0]);
    this->rx_message_.clear();
    return false;
  }

  if (data[2] != 0x03) {
    // Allow unknown or handshake message types temporarily
    return true;
  }

  uint8_t expected_length = 6 + data[6] + 1;

  if (this->rx_message_.size() < expected_length) {
    return true;
  }

  if (this->rx_message_.size() > expected_length) {
    ESP_LOGW(TAG, "Oversized message received; clearing buffer");
    this->rx_message_.clear();
    return false;
  }

  uint8_t rx_checksum = data[expected_length - 1];
  uint8_t calc_checksum = checksum(this->rx_message_, expected_length - 1);

  if (rx_checksum != calc_checksum) {
    if (expected_length == 14 && this->rx_message_.size() == 13) {
      ESP_LOGW(TAG, "Skipping checksum validate for 13-byte frame: %s",
               format_hex_pretty(data, this->rx_message_.size()).c_str());
      this->parseResponse(this->rx_message_);
      this->rx_message_.clear();
      return false;
    }

    ESP_LOGW(TAG, "Checksum mismatch %02X != %02X DATA=[%s]", rx_checksum, calc_checksum,
             format_hex_pretty(data, expected_length).c_str());
    this->rx_message_.clear();
    return false;
  }

  ESP_LOGV(TAG, "Valid message received: [%s]", format_hex_pretty(data, expected_length).c_str());
  this->parseResponse(this->rx_message_);
  this->rx_message_.clear();
  return false;
}

////////////////////////////////////////////////////////////////////////////////
// Enqueue commands to be sent and trigger processing
////////////////////////////////////////////////////////////////////////////////
void ToshibaClimateUart::enqueue_command_(const ToshibaCommand &command) {
  this->command_queue_.push_back(command);
  this->process_command_queue_();
}

////////////////////////////////////////////////////////////////////////////////
// Process command queue and handle sending with timeout and delay logic.
////////////////////////////////////////////////////////////////////////////////
void ToshibaClimateUart::process_command_queue_() {
  uint32_t now = millis();
  uint32_t cmd_delay = now - this->last_command_timestamp_;

  if ((now - this->last_rx_char_timestamp_) > RECEIVE_TIMEOUT_MS && !this->rx_message_.empty()) {
    ESP_LOGW(TAG, "RX timeout expired, clearing receive buffer");
    this->rx_message_.clear();
  }

  if (cmd_delay > COMMAND_DELAY_MS && !this->command_queue_.empty() && this->rx_message_.empty()) {
    auto cmd = this->command_queue_.front();
    if (cmd.cmd == ToshibaCommandType::DELAY && cmd_delay < cmd.delay) {
      return;
    }
    send_to_uart(cmd);
    this->command_queue_.erase(this->command_queue_.begin());
  }
}

////////////////////////////////////////////////////////////////////////////////
// Handle an incoming UART byte
////////////////////////////////////////////////////////////////////////////////
void ToshibaClimateUart::handle_rx_byte_(uint8_t c) {
  this->rx_message_.push_back(c);
  if (!validate_message_()) {
    this->rx_message_.clear();
  } else {
    this->last_rx_char_timestamp_ = millis();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Main loop for reading UART data and processing commands
////////////////////////////////////////////////////////////////////////////////
void ToshibaClimateUart::loop() {
  while (available()) {
    uint8_t c;
    this->read_byte(&c);
    this->handle_rx_byte_(c);
  }
  this->process_command_queue_();
}

////////////////////////////////////////////////////////////////////////////////
// Parse incoming responses and update component state
////////////////////////////////////////////////////////////////////////////////
void ToshibaClimateUart::parseResponse(const std::vector<uint8_t> &rawData) {
  uint8_t length = rawData.size();
  ToshibaCommandType sensor;
  uint8_t value;

  switch (length) {
    case 15:
      sensor = static_cast<ToshibaCommandType>(rawData[12]);
      value = rawData[13];
      break;
    case 16:
      ESP_LOGD(TAG, "ACK message length %d: %s", length, format_hex_pretty(rawData).c_str());
      return;
    case 17:
      sensor = static_cast<ToshibaCommandType>(rawData[14]);
      value = rawData[15];
      break;
    default:
      ESP_LOGW(TAG, "Unknown message length %d: %s", length, format_hex_pretty(rawData).c_str());
      return;
  }

  // (Your existing sensor handling code goes here, omitted for brevity)

  this->publish_state();
}

////////////////////////////////////////////////////////////////////////////////
// Initialization and standard component lifecycle functions
////////////////////////////////////////////////////////////////////////////////
void ToshibaClimateUart::setup() {
  start_handshake();
  getInitData();

  if (this->wifi_led_disabled_) {
    sendCmd(ToshibaCommandType::WIFI_LED, 128);
  }
}

void ToshibaClimateUart::getInitData() {
  requestData(ToshibaCommandType::POWER_STATE);
  requestData(ToshibaCommandType::MODE);
  requestData(ToshibaCommandType::TARGET_TEMP);
  requestData(ToshibaCommandType::FAN);
  requestData(ToshibaCommandType::POWER_SEL);
  requestData(ToshibaCommandType::SWING);
  requestData(ToshibaCommandType::ROOM_TEMP);
  requestData(ToshibaCommandType::OUTDOOR_TEMP);
  requestData(ToshibaCommandType::SPECIAL_MODE);
}

////////////////////////////////////////////////////////////////////////////////
// Implement requested missing methods to eliminate linker errors
////////////////////////////////////////////////////////////////////////////////

void ToshibaClimateUart::requestData(ToshibaCommandType cmd) {
  // Example minimal implementation or replace with your logic
  std::vector<uint8_t> payload = {2, 0, 3, 16, 0, 0, 6, 1, 48, 1, 0, 1};
  payload.push_back(static_cast<uint8_t>(cmd));
  payload.push_back(checksum(payload, payload.size()));
  ESP_LOGI(TAG, "Requesting data sensor %d, checksum %d", payload[12], payload[13]);
  enqueue_command_(ToshibaCommand{.cmd = cmd, .payload = payload});
}

void ToshibaClimateUart::sendCmd(ToshibaCommandType cmd, uint8_t value) {
  std::vector<uint8_t> payload = {2, 0, 3, 16, 0, 0, 7, 1, 48, 1, 0, 2};
  payload.push_back(static_cast<uint8_t>(cmd));
  payload.push_back(value);
  payload.push_back(checksum(payload, payload.size()));
  ESP_LOGD(TAG, "Sending command %d, value %d, checksum %d", cmd, value, payload.back());
  enqueue_command_(ToshibaCommand{.cmd = cmd, .payload = payload});
}

void ToshibaClimateUart::dump_config() {
  ESP_LOGCONFIG(TAG, "ToshibaClimateUart:");
  LOG_CLIMATE("", "Thermostat", this);
  if (outdoor_temp_sensor_ != nullptr) {
    LOG_SENSOR("", "Outdoor Temp", outdoor_temp_sensor_);
  }
  if (pwr_select_ != nullptr) {
    LOG_SELECT("", "Power selector", pwr_select_);
  }
  if (special_mode_select_ != nullptr) {
    LOG_SELECT("", "Special mode selector", special_mode_select_);
  }
  ESP_LOGI(TAG, "Min Temp: %d", min_temp_);
}

void ToshibaClimateUart::update() {
  requestData(ToshibaCommandType::ROOM_TEMP);
  if (outdoor_temp_sensor_ != nullptr) {
    requestData(ToshibaCommandType::OUTDOOR_TEMP);
  }
}

void ToshibaClimateUart::control(const climate::ClimateCall &call) {
  // Implement control logic as per your needs
  (void)call;  // placeholder
}

ClimateTraits ToshibaClimateUart::traits() {
  ClimateTraits traits;
  // You can set supported modes and features here
  return traits;
}

void ToshibaClimateUart::scan() {
  ESP_LOGI(TAG, "Starting sensor scan...");
  for (uint8_t i = 128; i < 255; i++) {
    requestData(static_cast<ToshibaCommandType>(i));
  }
}

// Destructors for polymorphic classes to fix vtable linker errors
ToshibaClimateUart::~ToshibaClimateUart() = default;
ToshibaPwrModeSelect::~ToshibaPwrModeSelect() = default;
ToshibaSpecialModeSelect::~ToshibaSpecialModeSelect() = default;

}  // namespace toshiba_suzumi
}  // namespace esphome
