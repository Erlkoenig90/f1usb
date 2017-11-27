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
 * In dieser Datei befinden sich die Deklarationen einiger Hilfsfunktionen- und Datentypen für den Zugriff auf die USB-Peripherie.
 */

#ifndef USBUTILS_HH_
#define USBUTILS_HH_

#include <cstddef>
#include <cstdint>

#include "util.hh"

/// Der Controller hat nur 8 EPnR-Register
static constexpr uint8_t numEP = 8;

/// Muss bei Variablen genutzt werden, welche sich im USB-Pufferspeicher befinden (Buffer Descriptor Table und Endpoint-Puffer).
#define USB_MEM __attribute__((section(".usbbuf")))

/// Die Instanzen hiervon werden im USB-RAM abgelegt und geben Anfang&Größe der Empfangs/Sende-Puffer an.
struct EP_BufDesc {
	// Anfang des Sendepuffers
	uint16_t txBufferAddr;
	uint16_t Padding1;
	// Größe des Sendepuffers in Bytes
	uint16_t txBufferCount;
	uint16_t Padding2;
	// Anfang des Empfangspuffers
	uint16_t rxBufferAddr;
	uint16_t Padding3;
	// Größe und Füllstand des Empfangspuffers; Spezialformat siehe Reference Manual
	uint16_t rxBufferCount;
	uint16_t Padding4;
};
// Die einzelnen Instanzen von EP_BufDesc müssen 16 Bytes groß sein sodass sie im Array an Adressen liegen die Vielfache von 16 sind.
static_assert(sizeof(EP_BufDesc) == 16, "");

/**
 * Da der USB-RAM in Form von 16bit-Worten organisiert ist, zwischen denen jeweils eine 16bit-Lücke ist,
 * definiert UsbMem ein 16bit-Wort mit Lücke, s.d. ein Array hiervon genau auf den Speicher passt.
 */
class UsbMem {
	public:
		uint16_t data;
	private:
		char padding [2];
};
// Die einzelnen Instanzen von UsbMem müssen 4 Bytes groß sein sodass sie im Array an Adressen liegen die Vielfache von 4 sind.
static_assert (sizeof(UsbMem) == 4, "");

/**
 * Enthält ein Array namens "data" aus UsbMem, wobei N/2 als Größe angenommen wird. Kann genutzt werden,
 * um Pufferspeicher im USB-RAM für Endpoints zu definieren, indem Bytes statt 16bit-Wörtern als Größe angegeben werden können.
 * Dabei wird automatisch geprüft, ob die angeforderte Größe von der Hardware unterstützt wird.
 */
template <size_t N>
struct UsbAlloc {
	static_assert (((N <= 62) && (N%2 == 0)) || ((N <= 512) && (N % 32 == 0)), "Invalid reception buffer size requested");
	static constexpr size_t size = N;

	/// Das eigentliche Daten-Array
	UsbMem data [N/2];
	/// Bietet Zugriff auf 16bit-Word "i".
	usb_always_inline uint16_t& operator [] (size_t i) {
		return data [i].data;
	}
};


/**
 * Da auf die Register-Definitionen EP0R, EP1R, ... EP8R aus der stm32f1xx.h nicht per zur Laufzeit bekanntem Index zugegriffen werden kann,
 * wird hier ein Array aus 16bit-Worten definiert, dessen Einträge eben den EPnR-Registern entsprechen. Der Typ UsbMem sorgt für
 * die erforderlichen 4-Byte-Schritte der Adresse. Somit entspricht z.B. EPnR[2] dem Register USB_EP2R.
 */
static __IO UsbMem (&EPnR) [numEP] = *reinterpret_cast<__IO UsbMem (*)[numEP]> (USB_BASE);
extern EP_BufDesc BufDescTable [numEP];

/**
 * Die Daten im USB-Pufferspeicher befinden sich aus Prozessorsicht an der Adresse 0x40006000 und folgend, mit jeweils einer 16bit-Lücke
 * zwischen zwei 16bit-Worten. Aus Sicht der Peripherie beginnt der Speicher bei 0, ohne Lücken. Da in der Buffer Descriptor Table sowie in
 * USB_BTABLE die Adresse aus Peripherie-Sicht angegeben werden muss, wird hier eine Umrechnung von "Prozessor"-Adresse zur "Peripherie"-
 * Adresse vorgenommen: Wenn der Parameter "addr" ein normaler Zeiger auf eine per USB_MEM definierte Variable ist, wird eine 16bit-Zahl
 * zurückgegeben welche als Adresse in USB_BTABLE oder EP_BufDesc genutzt werden kann.
 */
template <typename T>
usb_always_inline uint16_t mapAddr (T* addr) {
	// Ziehe Adresse von Anfangsadresse ab und teile durch 2 um Lücken herauszurechnen.
	return static_cast<uint16_t> ((reinterpret_cast<uintptr_t> (addr) - 0x40006000) / 2);
}
/**
 * Löscht die in "mask" (als Veroderung der USB_ISTR_xxx -Konstanten) angegeben Interrupts im USB_ISTR-Register.
 */
usb_always_inline void clearInterrupt (uint16_t mask) {
	USB->ISTR = (USB_ISTR_PMAOVR | USB_ISTR_ERR | USB_ISTR_WKUP | USB_ISTR_SUSP | USB_ISTR_RESET | USB_ISTR_SOF | USB_ISTR_ESOF) & ~mask;
}

void setEPnR (uint8_t EP, uint16_t mask, uint16_t data, uint16_t old);
void setEPnR (uint8_t EP, uint16_t mask, uint16_t data);

#endif /* USBUTILS_HH_ */
