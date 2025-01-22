// AP_PowerMonitor.h
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

class AP_PowerMonitor {
public:
    AP_PowerMonitor();
    
    // inicializar
    void init(void);
    
    // actualizar (llamar en el loop principal)
    void update(void);
    
    // singleton
    static AP_PowerMonitor *get_singleton(void) {
        return _singleton;
    }

private:
    static AP_PowerMonitor *_singleton;
    
    // UART que usaremos (por defecto SERIAL3)
    AP_HAL::UARTDriver *_uart;

    struct PowerMetrics {
        float v_in;
        float i_in;
        float p_in;
        float v_out;
        float i_out;
        float p_out;
        float efficiency;
        float temp;
        float pf;
    } _metrics;

    // Buffer y estado
    static const uint16_t UART_BUFFER_SIZE = 128;
    uint8_t _buffer[UART_BUFFER_SIZE];
    uint16_t _buffer_pos;
    bool _initialized;
    
    void process_incoming_data(void);
    bool parse_message(const uint8_t* buffer, uint16_t length);
};