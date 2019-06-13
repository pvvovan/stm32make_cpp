#include "uart.h"

gnss_radar::serial::uart::uart(uint32_t rxSize, uint32_t txSize, 
                                        UART_HandleTypeDef& huart):
    rxBufferSize {rxSize},
    txBufferSize {txSize},
    m_huart {huart},
    m_PreviousTxSize {0}
{
    rxBuffer = std::make_unique<uint8_t[]>(rxBufferSize);
    txBuffer = std::make_unique<uint8_t[]>(txBufferSize);
    HAL_UART_Receive_DMA(&m_huart, rxBuffer.get(), rxBufferSize);
    m_tail = rxBuffer.get();
}

gnss_radar::serial::uart::uart(gnss_radar::serial::uart&& other):
    rxBufferSize {other.rxBufferSize},
    txBufferSize {other.txBufferSize},
    rxBuffer {std::move(other.rxBuffer)},
    txBuffer {std::move(other.txBuffer)},
    m_huart {other.m_huart},
    m_tail {other.m_tail},
    m_PreviousTxSize {other.m_PreviousTxSize}
{
}

gnss_radar::serial::uart& gnss_radar::serial::uart::operator=(gnss_radar::serial::uart&& other)
{
    if (this != &other) {
        *this = std::move(other);
    }
    return *this;
}

uint32_t gnss_radar::serial::uart::available()
{
    uint8_t const* head = rxBuffer.get() + rxBufferSize - __HAL_DMA_GET_COUNTER(m_huart.hdmarx);
	if (head >= m_tail)
	{
		return head - m_tail;
	}
	else
	{
		return head - m_tail + rxBufferSize;
	}
}

int32_t gnss_radar::serial::uart::write(std::vector<uint8_t> data)
{
    if (data.empty())
        return 0;

    if (m_huart.gState == HAL_UART_STATE_ERROR)
	{
		return SERIAL_FAILURE; // transmission error
	}

    uint32_t size = data.size();
	if (m_huart.gState != HAL_UART_STATE_READY)
	{
		uint32_t tickStart = HAL_GetTick();
		uint32_t Timeout = (uint32_t)((m_PreviousTxSize + size) / (m_huart.Init.BaudRate / 15.0f / 1000));
		while(m_huart.gState != HAL_UART_STATE_READY)
		{
			if ((HAL_GetTick() - tickStart) > Timeout)
			{
				return WRITE_TIMEOUT_FAILURE; // Timeout error
			}
		}
	}

	if (size > txBufferSize)
	{
		size = txBufferSize;
	}
    for(uint32_t i = 0; i < size; ++i) {
        txBuffer[i] = data[i];
    }

	HAL_StatusTypeDef result = HAL_UART_Transmit_DMA(&m_huart, txBuffer.get(), size);
	if (result == HAL_OK)
	{
		m_PreviousTxSize = size;
		return size;
	}
	else
	{
		return SERIAL_FAILURE; // transmission error
	}
}

std::vector<uint8_t> gnss_radar::serial::uart::read()
{
    if (m_huart.gState == HAL_UART_STATE_ERROR)
	{
		return std::vector<uint8_t>{};
	}
	uint32_t size = available();
    std::vector<uint8_t> buffer(size);
	uint8_t const* head = rxBuffer.get() + rxBufferSize - __HAL_DMA_GET_COUNTER(m_huart.hdmarx);
	for(uint32_t i = 0; i < size; ++i)
	{
		if (head != m_tail)
		{
			uint8_t c = *m_tail++;
			if (m_tail >= txBuffer.get() + rxBufferSize)
			{
				m_tail -= rxBufferSize;
			}
			buffer[i] = c;
		}
		else
		{
			return std::vector<uint8_t>{};
		}
	}
	return buffer;
}