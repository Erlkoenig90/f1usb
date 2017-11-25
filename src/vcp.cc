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

#include <algorithm>
#include <cmath>
#include "vcp.hh"
#include "util.hh"
#include "usbutils.hh"
#include "main.hh"

/// Da die USARTS keine kontinuierlichen Adressen habe, speichere sie in einem Array
constexpr USART_TypeDef* VCP::s_usarts [3];

/// Gibt die Register eines DMA-Channels (startet bei 0) zurück.
static inline DMA_Channel_TypeDef* getDMA (uint8_t iChannel) {
	return reinterpret_cast<DMA_Channel_TypeDef*> (DMA1_BASE + 0x08 + 20 * iChannel);
}

/// Indirekt aufgerufen durch USBPhys; setzt alles zurück.
void VCP::onReset () {
	EPBuffer::onReset ();

	// Setze alle Daten zurück

	m_usb2uartBuffer.reset ();
	m_uart2usbBuffer.reset ();
	m_lineCoding = { 9600, 0, 0, 8 };
	m_rxUSARTBytes = 0;
	m_txUSARTBytes = 0;
	m_txUSBBytes = 0;
	m_USART_Prescaler = 469;
	m_usbReceiving = false;
	m_usbTransmitting = false;
	m_usartReceiving = false;
	m_usartTransmitting = false;
	confUSART ();

	setupDMA (true);

	prepareUSBreceive ();
}

/// Wird einmal von der main() aufgerufen, initialisiert die Peripherie.
void VCP::init () {
	// Aktiviere Interrupts
	NVIC_EnableIRQ (static_cast<IRQn_Type> (DMA1_Channel1_IRQn + m_iDMA_RX));
	NVIC_EnableIRQ (static_cast<IRQn_Type> (DMA1_Channel1_IRQn + m_iDMA_TX));
	NVIC_EnableIRQ (static_cast<IRQn_Type> (USART1_IRQn + m_USART));
	// Aktiviere Peripherietakte für GPIO-Pins
	m_pinRTS.initClock ();
	m_pinDTR.initClock ();
	m_pinTX.initClock ();

	// Konfiguriere GPIO-Pins
	m_pinRTS.configureOutput ();
	m_pinDTR.configureOutput ();
	m_pinTX.configureAFOut ();
}

/// Berechnet und prüft die angegebene Codierung des Serial-Ports; diese wird erst in confUSART übernommen.
bool VCP::setLineCoding (const LineCoding& coding) {
	// Prüfe Konsistenz der Parameter
	if (coding.bDataBits != 8)
		return false;
	if (coding.bParityType > 2)
		return false;
	if (coding.bCharFormat > 2)
		return false;

	// Basistakt für Peripherie
	const uint32_t PCLK = m_USART == 0 ? 72000000 : 36000000;
	// Prüfe Bereich des Baudraten-Parameters
	if (coding.dwDTERate < ((PCLK+0xFFFE) / 0xFFFF) || coding.dwDTERate > PCLK)
		return false;

	// Berechne Prescaler (rundende Integer-Division: round(a/b) ~= (a + (b/2)) / b)
	uint32_t iPresc = (PCLK + (coding.dwDTERate / 2)) / coding.dwDTERate;
	// Prüfe Gültigkeit des Prescalers
	if (iPresc >= 0xFFFF || iPresc == 0) return false;

	// Berechne Abweichung der Baudrate
	uint32_t iActual = coding.dwDTERate * iPresc;
	// Prüfe ob Abweichung < 3 %
	bool iOk = (distance (iActual,PCLK)*100/3) < iActual;
	// Wenn nicht, kann der USART diese Baudrate nicht zuverlässig nutzen
	if (!iOk) return false;

	// Speichere Daten
	m_USART_Prescaler = static_cast<uint16_t> (iPresc);
	m_lineCoding = coding;

	return true;
}

/// Setzt den Zustand der DTR & RTS-Leitungen.
void VCP::setLineState (bool DTR, bool RTS) {
	// Leite weiter an Pin-Klasse
	m_pinDTR.set (DTR);
	m_pinRTS.set (RTS);
}

/// Konfiguriert den USART auf die gespeicherten Serial-Einstellungen. DMA-Transfers werden zurückgesetzt.
void VCP::confUSART () {
	// Berechne Parity-Flags
	uint16_t parity = 0;
	if (m_lineCoding.bParityType == 1)
		parity = USART_CR1_PCE | USART_CR1_PS;
	else if (m_lineCoding.bParityType == 2)
		parity = USART_CR1_PCE;

	// Berechne Flags für Stop-Bits
	uint16_t stop = 0;
	if (m_lineCoding.bCharFormat == 1) // 1.5 Stop Bits
		stop = USART_CR2_STOP_1 | USART_CR2_STOP_0;
	else if (m_lineCoding.bCharFormat == 2) // 2 Stop Bits
		stop = USART_CR2_STOP_1;

	// USART anhalten
	usart ()->CR1 = 0;

	// DMA neu initialisieren
	setupDMA (true);

	// USART neu initialisieren
	usart ()->BRR = m_USART_Prescaler;
	usart ()->CR2 = stop;
	usart ()->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
	usart ()->CR1 = parity | USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE;
}

/// Konfiguriert den angegebenen DMA-Channel zum Senden oder Empfangen.
void VCP::setupDMA (uint8_t channel, uint8_t* memBuf, uint16_t length, bool TX) {
	DMA_Channel_TypeDef* pDMA = getDMA (channel);

	// Channel abschalten
	pDMA->CCR = 0;
	// Interrupts löschen
	clearDMAInt (channel);

	// Falls keine Bytes übertragen werden, lasse den Channel aus.
	if (length) {
		// Konfiguriere Channel
		pDMA->CNDTR = length;
		pDMA->CPAR = reinterpret_cast<uint32_t> (&usart ()->DR);
		pDMA->CMAR = reinterpret_cast<uint32_t> (memBuf);
		// Starte Übertragung
		pDMA->CCR = DMA_CCR_MINC | DMA_CCR_TCIE | DMA_CCR_EN | (TX ? DMA_CCR_DIR : 0);
	}
}

/// Konfiguriert beide DMA-Channel falls sie nichts zu tun haben oder force==true ist.
void VCP::setupDMA (bool force) {
	if (force || !m_usartReceiving) {
		// Anzahl empfangbarer Bytes
		uint16_t wa = static_cast<uint16_t> (std::min<size_t> (0xFFFF, m_uart2usbBuffer.writeAvailable ()));
		m_rxUSARTBytes = wa;
		m_usartReceiving = wa;
		// Konfiguriere DMA. Falls "wa"=0 ist, wird Channel nur abgeschaltet
		setupDMA (m_iDMA_RX, m_uart2usbBuffer.write(), wa, false);
	}

	if (force || !m_usartTransmitting) {
		// Anzahl sendbarer Bytes
		uint16_t ra = static_cast<uint16_t> (std::min<size_t> (0xFFFF, m_usb2uartBuffer.readAvailable ()));
		m_txUSARTBytes = ra;
		m_usartTransmitting = ra;
		// Konfiguriere DMA. Falls "ra"=0 ist, wird Channel nur abgeschaltet
		setupDMA (m_iDMA_TX, m_usb2uartBuffer.read (), ra, true);
	}
}

/// Löscht die Interrupts des angegebenen DMA-Channel im NVIC und DMA selbst
void VCP::clearDMAInt (uint8_t channel) {
	// Interrupts löschen
	DMA1->IFCR = 0xF << (channel * 4);
	// Auch im NVIC löschen
	NVIC_ClearPendingIRQ(static_cast<IRQn_Type> (DMA1_Channel1_IRQn+channel));
}

/**
 * Sollte von der ISR aufgerufen werden, wenn der empfangende DMA-Transfer fertig ist.
 * Übernimmt die empfangenen Bytes und bereitet neuen Transfer vor.
 */
void VCP::onDMA_RX_Finish () {
	DMA_Channel_TypeDef* pDMA = getDMA (m_iDMA_RX);
	// DMA Channel abschalten
	pDMA->CCR = 0;
	// Interrupts löschen
	clearDMAInt (m_iDMA_RX);
	// Wenn vorzeitig aufgerufen, ist dmaRemaining < m_rxUSARTBytes
	uint16_t dmaRemaining = static_cast<uint16_t> (pDMA->CNDTR);
	if (dmaRemaining >= m_rxUSARTBytes) return;
	// Empfangene Bytes übernehmen
	m_uart2usbBuffer.writeFinish (m_rxUSARTBytes - dmaRemaining);
	m_usartReceiving = false;
	// Tausche Puffer, auch wenn Puffer noch nicht voll. Dadurch werden "kurze" Byte-Folgen
	// auch übernommen, aber die Effizienz auf USB-Seite sinkt.
	m_uart2usbBuffer.swap ();

	// Nächsten Transfer vorbereiten
	setupDMA ();
	prepareUSBtransmit ();
}

/**
 * Sollte von der ISR aufgerufen werden, wenn der sendende DMA-Transfer fertig ist.
 * Löscht die gesendeten Bytes und bereitet neuen Transfer vor.
 */
void VCP::onDMA_TX_Finish () {
	DMA_Channel_TypeDef* pDMA = getDMA (m_iDMA_TX);
	// DMA Channel abschalten
	pDMA->CCR = 0;
	// Wenn vorzeitig aufgerufen, ist dmaRemaining < m_txUSARTBytes
	uint16_t dmaRemaining = static_cast<uint16_t> (pDMA->CNDTR);
	if (dmaRemaining >= m_txUSARTBytes) return;

	uint16_t sent = static_cast<uint16_t> (m_txUSARTBytes - dmaRemaining);
	// Interrupts löschen
	clearDMAInt (m_iDMA_TX);
	myassert (m_usb2uartBuffer.readAvailable() >= sent);
	// Gesendete Bytes löschen (Puffer frei machen)
	m_usb2uartBuffer.readFinish (sent);
	m_usartTransmitting = false;

	// Nächsten Transfer vorbereiten
	setupDMA ();
	prepareUSBreceive ();
}

/// Wird via VCP_DataEP::onReceive aufgerufen, wenn per USB Daten für den VCP angekommen sind.
void VCP::onReceive (bool, size_t rxBytes) {
	size_t wa = m_usb2uartBuffer.writeAvailable ();
	myassert (rxBytes <= wa);

	// Frage Puffer ab
	size_t l = static_cast<uint16_t> (std::min<size_t> (wa, rxBytes));
	uint8_t* buf = m_usb2uartBuffer.write ();

	// Hole Daten aus USB-Puffer in Doppelpuffer
	getDataEP ()->getReceivedData (buf, l);
	// Schreibvorgang abschließen
	m_usb2uartBuffer.writeFinish (l);
	m_usbReceiving = false;

	// Tausche Puffer, auch wenn Puffer noch nicht voll. Dadurch werden "kurze" Byte-Folgen
	// auch übernommen, aber die Effizienz auf USB-Seite sinkt.
	m_usb2uartBuffer.swap ();
	prepareUSBreceive ();
	setupDMA ();
}

/// Wird via VCP_DataEP::onTransmit aufgerufen, wenn per USB Daten vom VCP gesendet wurden.
void VCP::onTransmit () {
	myassert (m_uart2usbBuffer.readAvailable () >= m_txUSBBytes);
	// Gebe Puffer frei.
	m_uart2usbBuffer.readFinish (m_txUSBBytes);
	m_usbTransmitting = false;

	// Nächsten Transfer vorbereiten
	prepareUSBtransmit ();
	setupDMA ();
}

/// Startet den Empfang von Daten per USB.
void VCP::prepareUSBreceive () {
	if (m_usbReceiving) return;

	size_t wa = m_usb2uartBuffer.writeAvailable ();
	// Stelle sicher, dass nicht weniger als dataEpMaxPacketSize Bytes empfangen werden;
	// sonst sendet der Host zu viele Daten die dann von der Peripherie verweigert würden.
	if (wa >= dataEpMaxPacketSize) {
		// Empfange so viel wie möglich.
		size_t count = static_cast<uint16_t> (std::min<size_t> (wa, getDataEP ()->getRxBufLength ()));
		getDataEP ()->receivePacket (count);
		m_usbReceiving = true;
	} else
		m_usbReceiving = false;
}

void VCP::prepareUSBtransmit () {
	if (m_usbTransmitting) return;
	size_t ra = m_uart2usbBuffer.readAvailable ();
	// Beim Senden dürfen auch kurze Pakete auftreten.
	if (ra > 0) {
		// Sende so viel wie möglich.
		size_t count = std::min<size_t> (ra, getDataEP ()->getTxBufLength ());
		m_txUSBBytes = count;
		m_usbTransmitting = true;
		getDataEP ()->transmitPacket (m_uart2usbBuffer.read (), count);
	} else
		m_usbTransmitting = false;
}

/**
 * Sollte von der ISR des USART aufgerufen werden. Wenn ein "IDLE"-Frame erkannt wurde, d.h.
 * eine Lücke im USART-Datenstrom, wird der DMA-Transfer vorzeitig abgebrochen und die bisher
 * empfangenen Daten direkt abgesendet. So werden auch kurze Byte-Folgen abgesendet.
 * Alternativ könnte auch ein richtiger Timeout genutzt werden.
 */
void VCP::onUSART_IRQ () {
	// Prüfe auf IDLE-Interrupt
	if (usart ()->SR & USART_SR_IDLE) {
		// SR und DR müssen gelesen werden, um Interrupt zu quittieren
		static_cast<void> (usart ()->DR);
		// Wenn überhaupt schon etwas empfangen wurde...
		if (m_usartReceiving && getDMA (m_iDMA_RX)->CNDTR < m_rxUSARTBytes) {
			// Simuliere DMA-Interrupt
			onDMA_RX_Finish ();
		}
	}
}

/// Da der Management-EP nicht genutzt wird, passiert hier nichts.
void VCP_MgmtEP::onReceive (bool, size_t) {}

/// Da der Management-EP nicht genutzt wird, passiert hier nichts.
void VCP_MgmtEP::onTransmit () {}
