#ifndef _UART_H
#define _UART_H

#include "ISerial.h"
#include <memory>

namespace gnss_radar
{
    namespace serial
    {
        class uart : public ISerial
        {
        private:
        const uint32_t rxBufferSize;
        const uint32_t txBufferSize;
        std::unique_ptr<uint8_t[]> rxBuffer;
        std::unique_ptr<uint8_t[]> txBuffer;
        UART_HandleTypeDef& m_huart;
        uint8_t* m_tail;
        uint32_t m_PreviousTxSize;

        public:
            uart(uint32_t rxSize, uint32_t txSize, UART_HandleTypeDef& huart);
            uint32_t available() const noexcept override;
            std::variant<uint32_t, Error> write(std::vector<uint8_t> data) override;
            std::variant<std::vector<uint8_t>, Error> read() override;
            ~uart() override = default;

            uart(uart&& other);
            uart& operator=(uart&& other);

            uart(uart const&) = delete;
            uart& operator=(uart const&) = delete;
        };
        
    } /* serial */

} /* gnss_radar */

#endif /* _UART_H */
