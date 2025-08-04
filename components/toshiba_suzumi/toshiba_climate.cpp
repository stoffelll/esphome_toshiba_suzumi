#include "toshiba_climate.h"
#include "toshiba_climate_mode.h"
#include "esphome/core/log.h"

namespace esphome {
namespace toshiba_suzumi {

using namespace esphome::climate;

static const char* TAG = "toshiba_climate";

static constexpr int RECEIVE_TIMEOUT_MS = 200;
static constexpr int COMMAND_DELAY_MS = 100;

////////////////////////////////////////////////////////////////////////////////
// Checksum Calculation:
// Checksum is sum of all bytes excluding first byte (header), mod 256, subtracted from 256
////////////////////////////////////////////////////////////////////////////////
uint8_t checksum(const std::vector<uint8_t> &data, uint8_t length) {
  uint8_t sum = 0;
  for (size_t i = 1; i < length; i++) {
    sum += data[i];
  }
  return 256 - sum;
}

////////////////////////////////////////////////////////////////////////////////
// Send bytes via UART with logging
////////////////////////////////////////////////////////////////////////////////
void ToshibaClimateUart::send_to_uart(const ToshibaCommand &command) {
  this->last_command_timestamp_ = millis();
  ESP_LOGV(TAG, "Sending: [%s]", format_hex_pretty(command.payload).c_str());
  this->write_array(command.payload.data(), command.payload.size());
}

////////////////////////////////////////////////////////////////////////////////
// Send initial handshake commands to AC unit
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
// Validate received message with length and checksum checks
// Support fallback skip for 13-byte partial frames
////////////////////////////////////////////////////////////////////////////////
bool ToshibaClimateUart::validate_message_() {
  constexpr uint8_t kMinMessageLength = 7;

  if (this->rx_message_.size() < kMinMessageLength) {
    return true;
  }

  auto *data = this->rx_message_.data();
  uint8_t at = this->rx_message_.size() - 1;

  if (data[0] != 0x02) {
    ESP_LOGW(TAG, "Message header invalid (0x%02X), clearing buffer", data[0]);
    this->rx_message_.clear();
    return false;
  }

  if (data[2] != 0x03) {
    // Unknown command type or handshake, accept for now
    return true;
  }

  uint8_t expected_length = 6 + data[6] + 1;

  if (this->rx_message_.size() < expected_length) {
    return true;
  }

  if (this->rx_message_.size() > expected_length) {
    ESP_LOGW(TAG, "Received oversized message, clearing buffer");
    this->rx_message_.clear();
    return false;
  }

  uint8_t rx_checksum = data[expected_length - 1];
  uint8_t calc_checksum = checksum(this->rx_message_, expected_length - 1);

  if (rx_checksum != calc_checksum) {
    if (expected_length == 14 && this->rx_message_.size() == 13) {
      ESP_LOGW(TAG, "Skipping checksum validation on 13-byte frame: %s",
               format_hex_pretty(data, this->rx_message_.size()).c_str());
      this->parseResponse(this->rx_message_);
      this->rx_message_.clear();
      return false;
    }

    ESP_LOGW(TAG, "Checksum invalid: received 0x%02X != calculated 0x%02X DATA=[%s]",
             rx_checksum, calc_checksum, format_hex_pretty(data, expected_length).c_str());
    this->rx_message_.clear();
    return false;
  }

  ESP_LOGV(TAG, "Valid message received: [%s]", format_hex_pretty(data, expected_length).c_str());
  this->parseResponse(this->rx_message_);
  this->rx_message_.clear();
  return false;
}

////////////////////////////////////////////////////////////////////////////////
// Add commands to queue and start processing
////////////////////////////////////////////////////////////////////////////////
void ToshibaClimateUart::enqueue_command_(const ToshibaCommand &command) {
  this->command_queue_.push_back(command);
  this->process_command_queue_();
}

////////////////////////////////////////////////////////////////////////////////
// Handle command queue, sending commands after delays and timeouts
////////////////////////////////////////////////////////////////////////////////
void ToshibaClimateUart::process_command_queue_() {
  uint32_t now = millis();
  uint32_t cmd_delay = now - this->last_command_timestamp_;

  if ((now - this->last_rx_char_timestamp_) > RECEIVE_TIMEOUT_MS && !this->rx_message_.empty()) {
    ESP_LOGW(TAG, "RX timeout expired, clearing partial buffer");
    this->rx_message_.clear();
  }

  if (cmd_delay > COMMAND_DELAY_MS && !this->command_queue_.empty() && this->rx_message_.empty()) {
    auto new_command = this->command_queue_.front();

    if (new_command.cmd == ToshibaCommandType::DELAY && cmd_delay < new_command.delay) {
      return;
    }

    this->send_to_uart(new_command);
    this->command_queue_.erase(this->command_queue_.begin());
  }
}

////////////////////////////////////////////////////////////////////////////////
// Handle each received byte from UART
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
// Main loop processing UART and command queue
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
// Parse valid response messages and update component state
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
      ESP_LOGD(TAG, "Received ACK with length %d: %s", length, format_hex_pretty(rawData).c_str());
      return;
    case 17:
      sensor = static_cast<ToshibaCommandType>(rawData[14]);
      value = rawData[15];
      break;
    default:
      ESP_LOGW(TAG, "Unknown message length %d: %s", length, format_hex_pretty(rawData).c_str());
      return;
  }

  // Update climate or sensor state based on sensor type
  // ... (Keep your existing handling here)

  this->rx_message_.clear();
  this->publish_state();
}

////////////////////////////////////////////////////////////////////////////////
// Other helper functions like getInitData, control, update, dump_config, etc.
// Use your existing implementations or improve similarly

}  // namespace toshiba_suzumi
}  // namespace esphome
