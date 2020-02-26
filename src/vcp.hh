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
#include "usb.hh"

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
		constexpr VCP (UsbMem* epBufferRX, UsbMem* epBufferTX, size_t blockSize, uint8_t* buffer,
						uint8_t iMgmtEP, uint8_t iDataEP)
					: EPBuffer (iDataEP, iDataEP, EP_TYPE::BULK, epBufferRX, blockSize, epBufferTX, blockSize),
					m_buffer (buffer), m_blockSize (blockSize),
					m_mgmtEP (iMgmtEP),
					m_receiving (false) {}

		// Verbiete Kopieren/Zuweisen

		VCP (const VCP&) = delete;
		VCP (VCP&&) = delete;
		VCP& operator = (const VCP&) = delete;
		VCP& operator = (VCP&&) = delete;

		void init ();

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
		void prepareUSBreceive ();
		void prepareUSBtransmit (size_t length);

		uint8_t* m_buffer;
		size_t m_blockSize;

		/// Der Management-Endpoint.
		VCP_MgmtEP m_mgmtEP;

		/// Ob gerade ein empfangender auf USB im Gange ist.
		bool m_receiving;
};


#endif /* SRC_VCP_HH_ */
