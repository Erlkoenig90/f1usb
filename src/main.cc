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

#include <cmath>
#include <stm32f1xx.h>
#include "util.hh"
#include "clockconf.h"
#include "usb.hh"
#include "main.hh"

static TIM_TypeDef* const timPulse = TIM2;
extern USBPhys usbPhys;

/* Allokation von Puffern zum Senden/Empfangen auf Endpoints im USB-Pufferspeicher, insg. 512 Bytes
 * Die Variablen werden über "USB_MEM" via Linker-Script im Speicher des USB-Controllers abgelegt.
 * Die USB-Peripherie liest/schreibt die übertragenen Daten direkt in diese Variablen.
 */

/**
 * Puffer für Endpoint 0. Control Endpoints dürfen nur 8,16,32 oder 64 Bytes pro Paket übertragen.
 * Wird zum Senden & Empfangen genutzt ("half-duplex")
 */
alignas(4) static UsbAlloc<64> EP0_BUF	USB_MEM;

/// Der Default Control Endpoint 0 ist Pflicht für alle USB-Geräte.
static DefaultControlEP controlEP (usbPhys, 0, EP0_BUF.data, EP0_BUF.size);

/// Zentrale Instanz für USB-Zugriff. Übergebe 2 Endpoints.
USBPhys usbPhys (std::array<EPBuffer*, 8> {{ &controlEP, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr }});

void initializePeriphalClocks () {
	// Aktiviere GPIO-Module für die genutzten Pins
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;
	// Aktiviere Timer-Takt
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
}

void enablePulseTimer () {
	timPulse->CR1 = TIM_CR1_OPM | TIM_CR1_URS | TIM_CR1_ARPE;
	timPulse->DIER = TIM_DIER_UIE;

	timPulse->PSC = 71; // => 1 MHz
	timPulse->CCR2 = 1;
	timPulse->ARR = 2;

	timPulse->CCMR1 = TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2PE;
	timPulse->CCER = TIM_CCER_CC2E;

	timPulse->EGR = TIM_EGR_UG;
	timPulse->CR1 = TIM_CR1_OPM; // | TIM_CR1_CEN;

	NVIC_EnableIRQ (TIM2_IRQn);
}

void outputPulse (std::uint16_t pulse) {
	timPulse->ARR = pulse;
	timPulse->CCR2 = 1;

	timPulse->CR1 = TIM_CR1_OPM | TIM_CR1_URS | TIM_CR1_ARPE;
	timPulse->EGR = TIM_EGR_UG;
	timPulse->CR1 = TIM_CR1_OPM | TIM_CR1_CEN;
}

extern "C" void TIM2_IRQHandler () {
	if (timPulse->SR & TIM_SR_UIF) {
		timPulse->SR = ~TIM_SR_UIF;

		controlEP.onPulseDone ();
	}
}

/**
 * Globaler Interrupt für die USB-Peripherie. Da CAN und USB ohnehin nicht gleichzeitig genutzt werden können,
 * gilt diese ISR auch für die CAN-Peripherie.
 */
extern "C" void USB_LP_CAN1_RX0_IRQHandler () {
	usbPhys.irq ();
}

int main () {
	// Noch keine Interrupts bearbeiten
	__disable_irq ();
	// Systemtakt auf 72MHz einschalten
	configureSysClock ();

	// Peripherie-Takte aktivieren
	initializePeriphalClocks ();

	LED2.configureAFOut ();
//	LED2.configureOutput ();

	enablePulseTimer ();

	// USB-Peripherie aktivieren
	usbPhys.init ();

	// Verbindung zu Host aufbauen
	usbPhys.connect ();

	// Behandlung von Interrupts aktivieren
	__enable_irq ();

	// Warte in Endlosschleife auf Interrupts
	while (1) {
		// Die WFI-Anweisung lässt den Prozessorkern schlafen, bis ein Interrupt auftritt, um Energie zu sparen.
		// Einige JTAG-Programmieradapter (ST-Link) können ggf. keine Verbindung aufbauen, solange der Controller
		// hier "hängt". Abhilfe kann die Option "Connect under Reset" bringen, oder diese Zeile einfach auszukommentieren
		// auf Kosten höherer Leistungsaufnahme.
		__WFI ();
	}
}
