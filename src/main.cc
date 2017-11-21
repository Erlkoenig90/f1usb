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
#include "vcp.hh"

extern USBPhys usbPhys;

/* Allokation von Puffern zum Senden/Empfangen auf Endpoints im USB-Pufferspeicher, insg. 512 Bytes
 * Die Variablen werden über "USB_MEM" via Linker-Script im Speicher des USB-Controllers abgelegt.
 * Die USB-Peripherie liest/schreibt die übertragenen Daten direkt in diese Variablen.
 */

/**
 * Puffer für Endpoint 0. Control Endpoints dürfen nur 8,16,32 oder 64 Bytes pro Paket übertragen.
 * Wird zum Senden & Empfangen genutzt ("half-duplex")
 */
alignas(4) static UsbAlloc<64>	EP0_BUF	USB_MEM;

/// Puffer für 3x VCP-Daten. Die per dataEpMaxPacketSize gesetzte Größe wird auch für die Deskriptoren genutzt.
alignas(4) static UsbAlloc<dataEpMaxPacketSize>	VCP_RX_BUF [3]	USB_MEM;
alignas(4) static UsbAlloc<dataEpMaxPacketSize>	VCP_TX_BUF [3]	USB_MEM;

/// Der tatsächliche Speicher für die Doppelpuffer der VCP's. Wird hier angelegt zur Initialisierung per ".bss"-Section.
static uint8_t intBuffers [3][4096];

/*
 * Pinbelegung der drei USARTS. In den Klammern stehen die Zuordnungen für das Olimexino-STM32:
 * VCP0: TX=PA9(D7), RX=PA10(D8), DTR=PA1(D3), RTS=PA5(D13)
 * VCP1: TX=PA2(D1), RX=PA3(D0), DTR=PA6(D12), RTS=PA7(D11)
 * VCP2: TX=PB10(D29), RX=PB11(D30), DTR=PC0(D15), RTS=PC1(D16)
 *
 * PC12 schaltet (per low) den 1.5kOhm Widerstand ein.
 */

/// Anlegen der drei Verwaltungsobjekte für die drei VCPs.
VCP vcp [3] = {
	{ 0, VCP_RX_BUF [0].data, VCP_TX_BUF [0].data, VCP_RX_BUF [0].size, VCP_TX_BUF [0].size, 4, 3, 1, 2, { 0, 9 }, { 0, 1 }, { 0, 5 }, intBuffers[0], sizeof(intBuffers[0]) },
	{ 1, VCP_RX_BUF [1].data, VCP_TX_BUF [1].data, VCP_RX_BUF [1].size, VCP_TX_BUF [1].size, 5, 6, 3, 4, { 0, 2 }, { 0, 6 }, { 0, 7 }, intBuffers[1], sizeof(intBuffers[1]) },
	{ 2, VCP_RX_BUF [2].data, VCP_TX_BUF [2].data, VCP_RX_BUF [2].size, VCP_TX_BUF [2].size, 2, 1, 5, 6, { 1, 10 }, { 2, 0 }, { 2, 1 }, intBuffers[2], sizeof(intBuffers[2]) }
};

/// Der Default Control Endpoint 0 ist Pflicht für alle USB-Geräte.
static DefaultControlEP controlEP (usbPhys, 0, EP0_BUF.data, EP0_BUF.size);

/// Zentrale Instanz für USB-Zugriff. Übergebe  alle 7 Endpoints.
USBPhys usbPhys (std::array<EPBuffer*, 8> {{ &controlEP,
											vcp [0].getMgmtEP (), vcp [0].getDataEP (),
											vcp [1].getMgmtEP (), vcp [1].getDataEP (),
											vcp [2].getMgmtEP (), vcp [2].getDataEP (),
											nullptr }});

void initializePeriphalClocks () {
	// Aktiviere GPIO-Module für die genutzten Pins und USARTs
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN | RCC_APB1ENR_USART3EN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;

	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
}

/*
 * Definitionen der Interrupts. Die Aufrufe werden an die entsprechenden Klassen weitergeleitet. In der "isr"-Funktion
 * kann der Aufruf mit Timing-Messungen umgeben werden.
 */

// VCP 0

extern "C" void DMA1_Channel4_IRQHandler () {
	vcp [0].onDMA_TX_Finish ();
}

extern "C" void DMA1_Channel5_IRQHandler () {
	vcp [0].onDMA_RX_Finish ();
}

extern "C" void USART1_IRQHandler () {
	vcp [0].onUSART_IRQ ();
}

// VCP 1

extern "C" void DMA1_Channel6_IRQHandler () {
	vcp [1].onDMA_RX_Finish ();
}

extern "C" void DMA1_Channel7_IRQHandler () {
	vcp [1].onDMA_TX_Finish ();
}

extern "C" void USART2_IRQHandler () {
	vcp [1].onUSART_IRQ ();
}

// VCP 2

extern "C" void DMA1_Channel3_IRQHandler () {
	vcp [2].onDMA_RX_Finish ();
}

extern "C" void DMA1_Channel2_IRQHandler () {
	vcp [2].onDMA_TX_Finish ();
}

extern "C" void USART3_IRQHandler () {
	vcp [2].onUSART_IRQ ();
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

	// Alle VCPs initialisieren
	for (VCP& p : vcp) p.init ();

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
