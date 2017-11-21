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
 * In dieser Datei befinden sich die Implementierungen von Hilfsfunktionen für den Zugriff auf die USB-Peripherie.
 */

#include <cstdint>
#include <cstddef>
#include <stm32f1xx.h>
#include "usbutils.hh"

/**
 * Die Buffer Descriptor Table wird zusammen mit dem Pufferspeicher für die Endpoints im USB-RAM
 * abgelegt. Die genaue Stelle ist beliebig und wird im Register USB_BTABLE angegeben.
 */
alignas(8) EP_BufDesc BufDescTable [8] USB_MEM;

/**
 * Da die einzelnen Bits der EPnR-Register auf verschiedene Arten zu setzen sind und beim Schreiben
 * genau darauf geachtet werden muss dass nicht versehentlich die falschen Bits geschrieben werden,
 * kapselt diese Funktion den Schreibzugriff. Der Parameter EP gibt den Index des Registers an. Alle
 * Bits, die geschrieben werden sollen (unabhängig vom Wert) müssen in "mask" auf 1 gesetzt werden;
 * ist mask=0, wird gar nichts geschrieben. Die tatsächlich zu schreibenden Werte werden in "data"
 * angegeben. Bits welche in "mask" 0 sind, werden in "data" ignoriert. In "old" wird der vorherige
 * Zustand des Registers übergeben, falls er zuvor ohnehin schon abgefragt wurde. Ist das nicht
 * der Fall, kann die überladene Funktion ohne diesen Parameter genutzt werden.
 */
void setEPnR (uint8_t EP, uint16_t mask, uint16_t data, uint16_t old) {
	myassert (EP < numEP);

	// Diese Bits werden beim Schreiben von 0 gelöscht und bleiben bei 1 unverändert.
	constexpr uint16_t rc_w0 = USB_EP_CTR_RX_Msk | USB_EP_CTR_TX_Msk;
	// Diese Bits werden beim Schreiben von 1 umgeschaltet, und bleiben bei 0 unverändert.
	constexpr uint16_t toggle = USB_EP_DTOG_RX_Msk | USB_EPRX_STAT_Msk | USB_EP_DTOG_TX_Msk | USB_EPTX_STAT_Msk;
	// Diese Bits verhalten sich "normal", d.h. der geschriebene Wert wird direkt übernommen.
	constexpr uint16_t rw = USB_EP_T_FIELD_Msk | USB_EP_KIND_Msk | USB_EPADDR_FIELD;

	// Prüfe zu löschende Bits
	uint16_t wr0 = static_cast<uint16_t> (rc_w0 & (~mask | data));
	// Bei Bits mit Umschalte-Verhalten muss der alte Zustand beachtet und per XOR verarbeitet werden
	uint16_t wr1 = (mask & toggle) & (old ^ data);
	// Bei "normalen" Bits wird der alte Zustand beibehalten oder auf Wunsch überschrieben.
	uint16_t wr2 = rw & ((old & ~mask) | data);

	// Kombiniere alle drei Schreibmethoden.
	EPnR[EP].data = static_cast<uint16_t> (wr0 | wr1 | wr2);
}

/**
 * Diese Überladung kann genutzt werden, wenn der aktuelle Zustand von EPnR noch nicht abgefragt wurde;
 * in diesem Fall fragt die Funktion den Zustand ab und übergibt ihn an den "old"-Parameter.
 */
void setEPnR (uint8_t EP, uint16_t mask, uint16_t data) {
	setEPnR (EP, mask, data, EPnR [EP].data);
}
