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

#include <stm32f1xx.h>
#include "util.hh"

/// Wird aufgerufen, wenn myassert() fehlschlägt. Bewirkt einen Breakpoint im Debugger.
void assert_failed () {
	while (1) {
		__disable_irq ();
		__BKPT ();
	}
}

/// Initialisiert den Peripherie-Takt des GPIO-Ports.
void Pin::initClock () const {
	RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN << m_iPort);
}

/// Konfiguriert den Pin als Output.
void Pin::configureOutput () const {
	setMode (3);
}

/// Konfiguriert den Pin als Alternate Function Output.
void Pin::configureAFOut () const {
	setMode (0xB);
}

/// Schreibt den angegebenen "mode" in das CRL/CRH Register der GPIO-Peripherie.
void Pin::setMode (uint8_t mode) const {
	unsigned int bit = (m_iPin % 8)*4;
	CRx () = (CRx () & ~(uint32_t { 0xF } << bit)) | (uint32_t { mode } << bit);
}

/// Setzt den Pin auf den angegebenen Zustand.
void Pin::set (bool value) const {
	port ()->BSRR = (1 << (m_iPin + 16 * !value));
}

/// Gibt einen Zeiger auf das GPIO_TypeDef des zugehörigen Ports zurück.
GPIO_TypeDef* Pin::port () const {
	return reinterpret_cast<GPIO_TypeDef*> (0x40010800 + 0x400 * m_iPort);
}

/// Gibt einen Zeiger auf das CRL/CRH-Register für diesen Pin zurück.
__IO uint32_t& Pin::CRx () const {
	return m_iPin >= 8 ? port ()->CRH : port ()->CRL;
}

bool Pin::getOutput () const {
	return port ()->ODR & (1 << m_iPin);
}
