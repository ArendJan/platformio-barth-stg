#include <Arduino.h>
#include "stm32g4xx_hal_fdcan.h"
#include <cstdint>
#include <cstddef>

// Provides a simple interface for using the CAN bus. Most of the implementation was taken from the stg-850
// implementation and the code provided in https://barth-elektronik.com/downloads/9045-0038-A.zip

namespace can {

enum Baudrate : int {
    Rate1m = 1,
    Rate500k = 2,
    Rate250k = 4,
    Rate125k = 8,
    Rate100k = 10,
    Rate50k = 20,
};

using ReceiveCallback = void (*)(uint32_t id, uint8_t const* data, size_t size);

class FDCAN {
public:
    FDCAN_HandleTypeDef can_handle;
    FDCAN(FDCAN_HandleTypeDef can_h)
      : can_handle(can_h)
    {
    }
    // FDCAN_HandleTypeDef hfdcan2;

    void init(Baudrate baudrate);
    void set_baudrate(Baudrate baudrate);

    void send(uint32_t id, uint8_t const* data, size_t size);

    template <size_t N>
    void send(uint32_t id, uint8_t const (&data)[N])
    {
        send(id, data, N);
    }

    void on_receive(ReceiveCallback callback);
};
}  // namespace can
