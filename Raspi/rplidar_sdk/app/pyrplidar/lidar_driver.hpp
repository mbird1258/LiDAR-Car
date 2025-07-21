#pragma once
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <string>
#include <tuple>

namespace sl {
    class RPLidar {
    public:
        RPLidar();
        ~RPLidar();
        void connect(const std::string& port);
        void start_scan();
        void stop_scan();
        std::vector<std::vector<std::tuple<uint8_t, float, float>>> get_scan();

    private:
        static constexpr size_t MAX_SCAN_BUFFER_SIZE = 3000;
        void scan_loop();
        static std::atomic<bool> ctrl_c_pressed;
        static void ctrlc(int);
        IChannel* _channel = nullptr;
        std::atomic<bool> _running;
        std::thread _scan_thread;
        std::queue<std::vector<std::tuple<uint8_t, float, float>>> _scan_buffer;
        std::mutex _buffer_mutex;
        ILidarDriver* drv = nullptr;
        std::mutex _driver_mutex;
    };
}