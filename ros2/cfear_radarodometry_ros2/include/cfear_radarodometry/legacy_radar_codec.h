#pragma once

// Decoder for the "legacy_bytes" wire format: messages::msg::RadarFftDataMessage
// and messages::msg::RadarConfigurationMessage, as published by the
// leggedrobotics navtech_radar_ros driver fork (Colossus_publisher). Every
// scalar field on those messages is transmitted as a byte array (uint8[])
// rather than a plain typed field - see ros2/messages/msg/*.msg. This is a
// DIFFERENT wire format from navtech_msgs (the official Navtech IA SDK
// messages) handled elsewhere in this node; select it with
// radar.message_format: legacy_bytes.
//
// Field widths and byte order below were derived by reading the driver's
// actual encoder (Colossus_publisher::fft_data_handler /
// configuration_data_handler and Navtech::Networking::to_uint*_network, in
// the iasdk_navtech_radar SDK the driver vendors), NOT from the .msg comments
// alone - those comments are wrong for two fields (see bin_size below), and
// the driver is not on the network for this repo to test against.
// Re-verify against `ros2 topic echo` on the real driver before trusting this
// on a new radar.

#include <cstdint>
#include <cstring>
#include <vector>

namespace CFEAR_Radarodometry {

// Decode a big-endian (network order) unsigned integer of arbitrary width.
// Returns 0 if the array is too short (caller should treat that as "field
// absent" - the driver always sends full-width arrays in practice).
template <typename T>
inline T DecodeBigEndian(const std::vector<uint8_t>& bytes) {
  static_assert(std::is_unsigned<T>::value, "DecodeBigEndian requires an unsigned integer type");
  if (bytes.size() < sizeof(T))
    return T{0};
  T value = 0;
  for (size_t i = 0; i < sizeof(T); ++i)
    value = static_cast<T>((value << 8) | bytes[i]);
  return value;
}

// bin_size is the one field the driver encodes WITHOUT a byte-order swap: the
// call site does `to_uint64_host(config->bin_size())` (net_conversion.cpp),
// not `to_uint64_network(...)` used by every other field - almost certainly a
// driver bug, since the .msg comment claims "network order" like the rest.
// `to_uint64_host` just bit-copies a double into a uint64_t with no htonl/
// htons swap, so on the wire this is a NATIVE-endian (little-endian on
// x86_64/aarch64, i.e. every machine this stack runs on) 8-byte double.
//
// Also note config->bin_size() itself returns the SENSOR's raw uint16 register
// value (implicitly widened to double before the bit-copy above), not metres
// - e.g. 438, not 0.0438. Apply radar.legacy_bin_size_scale (default 1e-4,
// i.e. bin_size is in units of 1/10000 m) to recover metres; this scale is an
// assumption carried over from public Navtech driver examples, not confirmed
// against this specific RAS-3 unit - the node logs the raw decoded value plus
// the resulting range_res once at startup so it can be sanity-checked.
inline double DecodeBinSizeRawNativeEndian(const std::vector<uint8_t>& bytes) {
  if (bytes.size() < sizeof(double))
    return 0.0;
  double value = 0.0;
  std::memcpy(&value, bytes.data(), sizeof(double));
  return value;
}

}  // namespace CFEAR_Radarodometry
