#ifndef LIBETCAM_H
#define LIBETCAM_H

#include <array>
#include <cstdint>

namespace libetcam
{
    const uint8_t ku_THRUSTER_NUMBER =              8U;
    const uint8_t ku_THROTTLE_COMMAND_SIZE =        16U;
    const uint8_t ku_THROTTLE_PACKET_SIZE =         ku_THROTTLE_COMMAND_SIZE + 1U;
    const uint8_t ku_KISS_TELEMETRY_REDUCED_SIZE =  9U;
    const uint8_t ku_TELEMETRY_SIZE =               ku_THRUSTER_NUMBER * ku_KISS_TELEMETRY_REDUCED_SIZE;

    class TelemetrySync
    {
        public:
            TelemetrySync( void );
            bool b_telemetry_sync( const uint8_t ku_recieve_byte );

        private:
            static const uint8_t    ku_SYNC_BYTE_ =      0xAAU;
            static const uint8_t    ku_SYNC_LENGTH_ =    8U;

            uint8_t                 u_sync_count_;
    };

    struct TelemetryStruct
    {
        uint8_t     u_temperature;
        float       f_voltage;
        float       f_current;
        uint16_t    us_power_consumption;
        uint32_t    us_erpm;
    };

    std::array< uint8_t, ku_THROTTLE_PACKET_SIZE > au_throttle_pack( const std::array< uint16_t, ku_THRUSTER_NUMBER >&kaus_throttle );
    std::array< struct TelemetryStruct, ku_THRUSTER_NUMBER > ax_telemetry_parser( const std::array< uint8_t, ku_TELEMETRY_SIZE >&kau_telemetry );
}

#endif
