#ifndef _I_SERIAL_H
#define _I_SERIAL_H

#include <vector>
#include "stm32f4xx_hal.h"

namespace gnss_radar
{
    namespace serial
    {
        class ISerial
        {
        public:
            virtual uint32_t available() = 0;
            virtual int32_t write(std::vector<uint8_t> data) = 0;
            virtual std::vector<uint8_t> read() = 0;
            virtual ~ISerial() = default;

            static const int32_t SERIAL_FAILURE = -1;
            static const int32_t WRITE_TIMEOUT_FAILURE = -2;
        //	static const int WRITE_WAIT_FAILURE = -3;
            // static const int READ_FAILURE = -4;
        };
        
    } /* serial */

} /* gnss_radar */

#endif /* _I_SERIAL_H */