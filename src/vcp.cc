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

/// Indirekt aufgerufen durch USBPhys; setzt alles zurück.
void VCP::onReset () {
	EPBuffer::onReset ();

	prepareUSBreceive ();
}

/// Wird via VCP_DataEP::onReceive aufgerufen, wenn per USB Daten für den VCP angekommen sind.
void VCP::onReceive (bool, size_t rxBytes) {
	// Hole Daten aus USB-Puffer in Doppelpuffer
	getDataEP ()->getReceivedData (m_buffer, rxBytes);

	prepareUSBtransmit (rxBytes);
}

/// Wird via VCP_DataEP::onTransmit aufgerufen, wenn per USB Daten vom VCP gesendet wurden.
void VCP::onTransmit () {
	// Nächsten Transfer vorbereiten
	prepareUSBreceive ();
}

/// Startet den Empfang von Daten per USB.
void VCP::prepareUSBreceive () {
	getDataEP ()->receivePacket (m_blockSize);
	m_receiving = true;
}

void VCP::prepareUSBtransmit (size_t length) {
	m_receiving = false;
	getDataEP ()->transmitPacket (m_buffer, length);
}

/// Da der Management-EP nicht genutzt wird, passiert hier nichts.
void VCP_MgmtEP::onReceive (bool, size_t) {}

/// Da der Management-EP nicht genutzt wird, passiert hier nichts.
void VCP_MgmtEP::onTransmit () {}
