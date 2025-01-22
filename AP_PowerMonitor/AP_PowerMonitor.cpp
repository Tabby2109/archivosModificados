// AP_PowerMonitor.cpp
#include "AP_PowerMonitor.h"

#define START_FRAME 0xAA
#define END_FRAME 0x55

extern const AP_HAL::HAL& hal;

AP_PowerMonitor* AP_PowerMonitor::_singleton;

AP_PowerMonitor::AP_PowerMonitor() :
    _uart(nullptr),
    _buffer_pos(0),
    _initialized(false)
{
    _singleton = this;
    memset(&_metrics, 0, sizeof(_metrics));
}

void AP_PowerMonitor::init()
{
    // Obtener instancia de SERIAL3
    _uart = hal.serial(2); // SERIAL3 es el índice 2
    if (_uart == nullptr) {
        return;
    }

    // Configurar UART - mismo baudrate que la STM32
    _uart->begin(115200);
    _initialized = true;
}

void AP_PowerMonitor::update()
{
    if (!_initialized || _uart == nullptr) {
        return;
    }

    // Leer datos disponibles
    while (_uart->available()) {
        if (_buffer_pos >= UART_BUFFER_SIZE) {
            _buffer_pos = 0;
        }

        _buffer[_buffer_pos] = _uart->read();
        _buffer_pos++;
        
        process_incoming_data();
    }
}

void AP_PowerMonitor::process_incoming_data()
{
    for (uint16_t i = 0; i < _buffer_pos - 1; i++) {
        if (_buffer[i] == START_FRAME) {
            uint16_t remaining = _buffer_pos - i;
            if (remaining >= sizeof(PowerMetrics) + 2) {
                if (_buffer[i + sizeof(PowerMetrics) + 1] == END_FRAME) {
                    if (parse_message(&_buffer[i], sizeof(PowerMetrics) + 2)) {
                        // Enviar datos via MAVLink
                        gcs().send_text(MAV_SEVERITY_INFO, "Power: V=%.1fV I=%.1fA Eff=%.1f%%",
                                      (double)_metrics.v_in,
                                      (double)_metrics.i_in,
                                      (double)_metrics.efficiency);

                        // Mover datos restantes al inicio del buffer
                        uint16_t remaining_data = _buffer_pos - (i + sizeof(PowerMetrics) + 2);
                        if (remaining_data > 0) {
                            memmove(_buffer, &_buffer[i + sizeof(PowerMetrics) + 2], remaining_data);
                        }
                        _buffer_pos = remaining_data;
                        return;
                    }
                }
            }
        }
    }

    if (_buffer_pos > UART_BUFFER_SIZE - 10) {
        _buffer_pos = 0;
    }
}

bool AP_PowerMonitor::parse_message(const uint8_t* buffer, uint16_t length)
{
    if (buffer[0] != START_FRAME || buffer[length-1] != END_FRAME) {
        return false;
    }

    memcpy(&_metrics, buffer + 1, sizeof(PowerMetrics));
    return true;
}