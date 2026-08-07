#include "hardware/imu/imu_decode.h"

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

int main() {
    uint32_t width = 0;
    uint32_t height = 0;
    std::cin.read(reinterpret_cast<char*>(&width), sizeof(width));
    std::cin.read(reinterpret_cast<char*>(&height), sizeof(height));
    if (!std::cin || width == 0 || height == 0) {
        return 2;
    }
    std::vector<uint8_t> luma(static_cast<size_t>(width) * height);
    std::cin.read(reinterpret_cast<char*>(luma.data()),
                  static_cast<std::streamsize>(luma.size()));
    if (std::cin.gcount() != static_cast<std::streamsize>(luma.size())) {
        return 3;
    }
    uint8_t decoded[384] = {};
    const uint32_t count = imu_read_luma_vertical(
        luma.data(), static_cast<int>(width), static_cast<int>(height),
        static_cast<int>(width), decoded);
    for (uint32_t index = 0; index < count; ++index) {
        std::cout << std::hex << std::setfill('0') << std::setw(2)
                  << static_cast<unsigned>(decoded[index]);
    }
    return 0;
}
