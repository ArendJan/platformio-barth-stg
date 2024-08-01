#include <Arduino.h>
#include "stm32g4xx_hal_fdcan.h"
#include <cstdint>
#include <cstddef>

// Provides a simple interface for using the CAN bus. Most of the implementation was taken from the stg-850
// implementation and the code provided in https://barth-elektronik.com/downloads/9045-0038-A.zip

// Modified from 👆 for the gt900


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
    FDCAN(FDCAN_GlobalTypeDef* can_t, Baudrate baudrate = Baudrate::Rate500k, bool auto_init = true);
    void init();
    void set_baudrate(Baudrate baudrate);

    void send(uint32_t id, uint8_t const* data, size_t size); // only normal can for now

    template <size_t N>
    void send(uint32_t id, uint8_t const (&data)[N])
    {
        send(id, data, N);
    }
    // void can_gpio_init();
    // void can_init_filter();
    void on_receive(ReceiveCallback callback);
    void receive(FDCAN_RxHeaderTypeDef& header, uint8_t data[64]);

    ReceiveCallback receive_callback = nullptr;
    int bus_num = 0;
};

// just start the CAN bus, remove here and in the cpp file if not needed
extern FDCAN fdcan1;
extern FDCAN fdcan2;


// helper functions:
constexpr uint32_t DLC_TO_NUM(size_t DLC);
void MX_FDCAN1_Init(FDCAN_HandleTypeDef& hfdcan1);
void MX_FDCAN2_Init(FDCAN_HandleTypeDef& hfdcan2);
