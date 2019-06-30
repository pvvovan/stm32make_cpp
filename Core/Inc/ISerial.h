#ifndef _I_SERIAL_H
#define _I_SERIAL_H

#include <vector>
#include "stm32f4xx_hal.h"
#include <variant>

namespace gnss_radar
{
    namespace serial
    {
        class ISerial
        {
        public:
        enum class Error {
                OK,
                SerialFailure,
                WriteTimeout,
                WriteFailure,
                ReadFailure
            };
            
            virtual uint32_t available() const noexcept = 0;
            virtual std::variant<uint32_t, Error> write(std::vector<uint8_t> data) = 0;
            virtual std::variant<std::vector<uint8_t>, Error> read() = 0;
            virtual ~ISerial() = default;
        };
        
    } /* serial */

} /* gnss_radar */

#endif /* _I_SERIAL_H */
