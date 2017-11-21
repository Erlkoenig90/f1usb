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


#ifndef MAIN_HH_
#define MAIN_HH_

#include <array>
#include "util.hh"
#include "usb.hh"

/**
 * Die maximale Paketgröße für den Endpoint, welcher für die Nutzdaten genutzt wird.
 * Wird in die Deskriptoren eingebaut und zur Reservierung der Puffer benutzt.
 */
static constexpr uint16_t dataEpMaxPacketSize = 64;

static constexpr Pin LED1 (0, 5), LED2 (0, 1);

/// "Dummy"-Endpoint, welcher ankommende Daten umdreht.
class MirrorEP : public EPBuffer {
	public:
		constexpr MirrorEP (UsbMem* epBuffer, size_t length) : EPBuffer (1, 1, EP_TYPE::BULK, epBuffer, length, epBuffer, length), m_buffer {} {}
	protected:
		virtual void onReceive (bool setup, size_t rxBytes);
		virtual void onTransmit ();
		virtual void onReset ();
	private:
		uint8_t m_buffer [dataEpMaxPacketSize];
};

#endif /* MAIN_HH_ */
