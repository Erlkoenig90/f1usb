/*
 * Copyright (c) 2017, Niklas Gürtler
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/*
 * Die hier definierte VCP-Klasse überträgt Daten zwischen UART und einem virtuellen USB-SerialPort.
 */

#ifndef SRC_VCP_HH_
#define SRC_VCP_HH_

#include <cstdint>
#include <cstddef>
#include <stm32f1xx.h>
#include "doublebuffer.hh"
#include "usb.hh"

/// Speichert per USB angeforderte Konfigurationsdaten für den USART.
struct LineCoding {
	/// Baudrate
	uint32_t dwDTERate;
	/// Anzahl Stop-Bits
	uint8_t bCharFormat;
	/// Prüfbit
	uint8_t bParityType;
	// Anzahl Bits pro USART-Frame
	uint8_t bDataBits;
};

class VCP;

// Damit die VCP-Klasse 2 Endpoints verwalten kann, werden hier für beide Typen von Endpoint je eine Klasse definiert.

/// Wird für den Management-Endpoint des VCP genutzt. Tut außer der Konfiguration nichts.
class VCP_MgmtEP : public EPBuffer {
	public:
		using EPBuffer::EPBuffer;

		constexpr VCP_MgmtEP (uint8_t addr) : EPBuffer (addr, addr, EP_TYPE::INTERRUPT, nullptr, 0, nullptr, 0) {}

		virtual void onReceive (bool setup, size_t rxBytes) override;
		virtual void onTransmit () override;
};

/**
 * Die VCP-Klasse verbindet einen USART mit dem USB. An den Konstruktor müssen Nummer des USART, Puffer für die Endpoints,
 * DMA-Channel-Nummern (ab 0 zählend), Endpoint-Nummern und Pins übergeben werden. Die Klasse enthält Callbacks aus ISRs
 * und aus der USB-Peripherie.
 */
class VCP : private EPBuffer {
	public:
		constexpr VCP (uint8_t iUsart, UsbMem* epBufferRX, UsbMem* epBufferTX, uint16_t epBufferRXLength, uint16_t epBufferTXLength,
						uint8_t iDMA_RX, uint8_t iDMA_TX, uint8_t iMgmtEP, uint8_t iDataEP, Pin pinTX, Pin pinDTR, Pin pinRTS,
						uint8_t* intBuffer, size_t intBufferSize)
					: EPBuffer (iDataEP, iDataEP, EP_TYPE::BULK, epBufferRX, epBufferRXLength, epBufferTX, epBufferTXLength),
					m_usb2uartBuffer (intBuffer, intBufferSize / 2), m_uart2usbBuffer (intBuffer + intBufferSize / 2, intBufferSize / 2),
					m_lineCoding { 9600, 0, 0, 8 }, m_mgmtEP (iMgmtEP),
					m_pinTX (pinTX), m_pinDTR (pinDTR), m_pinRTS (pinRTS),
					m_txUSBBytes (0), m_rxUSARTBytes (0), m_txUSARTBytes (0), m_USART_Prescaler (469),
					m_USART (iUsart), m_iDMA_RX (iDMA_RX), m_iDMA_TX (iDMA_TX),
					m_usbReceiving (false), m_usbTransmitting (false), m_usartReceiving (false), m_usartTransmitting (false) {}

		// Verbiete Kopieren/Zuweisen

		VCP (const VCP&) = delete;
		VCP (VCP&&) = delete;
		VCP& operator = (const VCP&) = delete;
		VCP& operator = (VCP&&) = delete;

		void init ();
		void onDMA_RX_Finish ();
		void onDMA_TX_Finish ();
		void onUSART_IRQ ();
		void confUSART ();
		bool setLineCoding (const LineCoding& coding);
		/// Fragt aktuelle Serial-Parameter ab.
		inline const LineCoding& getLineCoding () { return m_lineCoding; }
		/// Fragt den empfangenden DMA-Channel ab.
		inline uint8_t getDMA_RX () { return m_iDMA_RX; }
		/// Fragt den sendenden DMA-Channel ab.
		inline uint8_t getDMA_TX () { return m_iDMA_TX; }
		void setLineState (bool DTR, bool RTS);

#if (__cplusplus >= 201402L)
		/// Fragt die enthaltene EPBuffer-Instanz für den Management-Endpoint ab.
		constexpr EPBuffer* getMgmtEP () { return &m_mgmtEP; }
		/// Fragt die enthaltene EPBuffer-Instanz für den Management-Endpoint ab.
		constexpr EPBuffer* getDataEP () { return this; }
#else
		/// Fragt die enthaltene EPBuffer-Instanz für den Management-Endpoint ab.
		EPBuffer* getMgmtEP () { return &m_mgmtEP; }
		/// Fragt die enthaltene EPBuffer-Instanz für den Management-Endpoint ab.
		EPBuffer* getDataEP () { return this; }
#endif
	private:
		virtual void onReset () override;
		virtual void onReceive (bool setup, size_t rxBytes) override;
		virtual void onTransmit () override;

		void setupDMA (uint8_t channel, uint8_t* memBuf, uint16_t length, bool TX);
		void setupDMA (bool force = false);
		void clearDMAInt (uint8_t channel);
		void prepareUSBreceive ();
		void prepareUSBtransmit ();
		static constexpr USART_TypeDef* s_usarts [3] = { USART1, USART2, USART3 };
		/// Gibt den dazugehörigen USART_TypeDef zurück.
		usb_always_inline USART_TypeDef* usart () const { return s_usarts [m_USART]; }

		/// Die beiden Doppelpuffer für beide Richtungen.
		DoubleBuffer m_usb2uartBuffer, m_uart2usbBuffer;
		/// Aktuelle Serial-Parameter.
		LineCoding m_lineCoding;
		/// Der Management-Endpoint.
		VCP_MgmtEP m_mgmtEP;

		/// Der Sende-Pin des USART (der Empfangs-Pin wird nicht benötigt, weil der nicht konfiguriert werden muss)
		Pin m_pinTX;
		/// Flusskontroll-Pins für RS-232
		Pin m_pinDTR, m_pinRTS;
		/// Anzahl der derzeit per USB abzusendenen Bytes
		size_t m_txUSBBytes;
		/// Anzahl der derzeit per USART zu empfangenden / sendenden Bytes
		uint16_t m_rxUSARTBytes, m_txUSARTBytes;
		/// Der per setLineCoding berechnete Prescaler für den USART.
		uint16_t m_USART_Prescaler;
		/// Die Nummer des zugehörigen USART (0-2)
		const uint8_t m_USART;
		/// DMA-Channel für den USART (ab 0 zählend)
		const uint8_t m_iDMA_RX, m_iDMA_TX;
		/// Ob gerade ein empfangender/sendender Transfer auf USB/USART im Gange ist.
		bool m_usbReceiving, m_usbTransmitting, m_usartReceiving, m_usartTransmitting;
};


#endif /* SRC_VCP_HH_ */
