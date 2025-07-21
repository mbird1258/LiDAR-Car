#include "lidar_driver.hpp"
#include <memory>
#include <chrono>
#include <stdexcept>

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace sl;

RPLidar::RPLidar() : _channel(nullptr),
      _running(false),
      drv(nullptr) {
    signal(SIGINT, ctrlc);
}

RPLidar::~RPLidar() {
    stop_scan();
    
    std::lock_guard<std::mutex> lock(_driver_mutex);
    if (drv) {
        delete drv;
        drv = nullptr;
    }
    if (_channel) {
        delete _channel;
        _channel = nullptr;
    }
}

std::atomic<bool> RPLidar::ctrl_c_pressed(false);

void RPLidar::ctrlc(int) {
    ctrl_c_pressed.store(true, std::memory_order_seq_cst);
}

bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

void RPLidar::connect(const std::string& port) {
    const char * opt_channel_param_first = port.c_str();
    sl_u32 opt_channel_param_second = 460800;
    sl_result op_result;
    std::lock_guard<std::mutex> lock(_driver_mutex);
    drv = *createLidarDriver();

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
    if (SL_IS_OK((drv)->connect(_channel))) {
        op_result = drv->getDeviceInfo(devinfo);

        if (SL_IS_OK(op_result)) 
        {
            connectSuccess = true;
        }
        else{
            delete drv;
            drv = nullptr;
        }
    }

    if (!connectSuccess) {
        delete _channel;
        _channel = nullptr;
        throw std::runtime_error("Failed to connect to LiDAR");
    }

    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);

    if (!checkSLAMTECLIDARHealth(drv)) {
        delete drv;
        drv = nullptr;
        delete _channel;
        _channel = nullptr;
        throw std::runtime_error("Failed to connect to LiDAR");
    }
}

void RPLidar::start_scan() {
    std::lock_guard<std::mutex> lock(_driver_mutex);
    if (!drv) throw std::runtime_error("LiDAR not connected");
    if (_running) return;

    _running = true;
    _scan_thread = std::thread(&RPLidar::scan_loop, this);
}

void RPLidar::stop_scan() {
    _running = false;
    if (_scan_thread.joinable() && _scan_thread.get_id() != std::this_thread::get_id()) {
        _scan_thread.join();
    }

    std::lock_guard<std::mutex> lock(_driver_mutex);
    if (drv) {
        drv->stop();
        delay(200);
        drv->setMotorSpeed(0);
    }
}

void RPLidar::scan_loop() {
    {
        std::lock_guard<std::mutex> lock(_driver_mutex);
        if (!drv) return;
        drv->setMotorSpeed();
        drv->startScan(0, 1);
    }

    while (_running) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);
        
        {
            std::lock_guard<std::mutex> lock(_driver_mutex);
            if (!drv) break;
            sl_result op_result = drv->grabScanDataHq(nodes, count);

            if (SL_IS_OK(op_result)) {
                drv->ascendScanData(nodes, count);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));

                if (ctrl_c_pressed){
                    break;
                }

                continue;
            }
        }

        std::vector<std::tuple<uint8_t, float, float>> scan;
        
        for (int pos = 0; pos < (int)count; ++pos) {
            scan.emplace_back(
                nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT,
                (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
                nodes[pos].dist_mm_q2 / 4.0f
            );
        }
        
        {
            std::lock_guard<std::mutex> lock(_buffer_mutex);
            _scan_buffer.push(scan);

            while (_scan_buffer.size() > MAX_SCAN_BUFFER_SIZE) {
                _scan_buffer.pop();
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (ctrl_c_pressed){ 
            break;
        }
    }
}

std::vector<std::vector<std::tuple<uint8_t, float, float>>> RPLidar::get_scan() {
    std::lock_guard<std::mutex> lock(_buffer_mutex);
    if (_scan_buffer.empty()) return {};
    
    std::vector<std::vector<std::tuple<uint8_t, float, float>>> all_scans;
    all_scans.reserve(_scan_buffer.size());
    
    while (!_scan_buffer.empty()) {
        all_scans.push_back(std::move(_scan_buffer.front()));
        _scan_buffer.pop();
    }
    return all_scans;
    
    // auto scan = _scan_buffer.front();
    // _scan_buffer.pop();
    // return scan;
}
