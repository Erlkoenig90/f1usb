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
 * Diese Datei enthält Definitionen und Klassen für die USB-Protokollebenen, insb. für Control Transfers.
 */

#ifndef USB_HH_
#define USB_HH_

#include <cstddef>
#include <cstdint>
#include <array>
#include "util.hh"
#include "usbutils.hh"

/// Auflistung der Standard-Anfragen auf dem Default Control Endpoint 0.
enum class ST_REQ : uint8_t { GET_STATUS = 0, CLEAR_FEATURE = 1, SET_FEATURE = 3, SET_ADDRESS = 5, GET_DESCRIPTOR = 6,	SET_DESCRIPTOR = 7,
	GET_CONFIGURATION = 8, SET_CONFIGURATION = 9, GET_INTERFACE = 10, SET_INTERFACE = 11, SYNCH_FRAME = 12, GET_OS_STRING_DESC = 3 };

/// Auflistung einiger USB-CDC-Anfragen
enum class CDC_REQ : uint8_t { SEND_ENCAPSULATED_COMMAND = 0, GET_ENCAPSULATED_RESPONSE = 1, SET_LINE_CODING = 0x20, GET_LINE_CODING = 0x21, SET_CONTROL_LINE_STATE = 0x22 };

/// Auflistung der Standard-Features.
enum class FEATURE : uint8_t { DEVICE_REMOTE_WAKEUP = 1, ENDPOINT_HALT = 0, TEST_MODE = 2 };

/// Zustände des Zustandsautomaten für Control Endpoints (Klasse ControlEP)
enum class CTRL_STATE : uint8_t { SETUP, DATA_OUT, DATA_IN, DATA_IN_LAST, STATUS_IN, STATUS_OUT };

/// Auflistung der Endpoint-Typen, für den EPBuffer-Konstruktor. Die Werte entsprechen denen des EPnR-Registers
enum class EP_TYPE : uint8_t { BULK = 0, CONTROL = 1, ISOCHRONOUS = 2, INTERRUPT = 3 };

// Erlaube direktes Vergleichen der Kommandos mit Integer-Typ

usb_always_inline constexpr bool operator == (uint8_t a, ST_REQ b) { return a == static_cast<uint8_t> (b); }
usb_always_inline constexpr bool operator == (uint8_t a, CDC_REQ b) { return a == static_cast<uint8_t> (b); }

class EPBuffer;
class DefaultControlEP;

/**
 * Repräsentiert zusammen mit EPBuffer die USB-Peripherie. Bietet einige globale Operationen auf der USB-Peripherie.
 * An den Konstruktor wird ein Array mit Zeigern auf einzelne EPBuffer-Instanzen übergeben; USBPhys ruft darin die
 * entsprechenden Callbacks auf (onReset, onReceive, onTransmit).
 */
class USBPhys {
	friend class DefaultControlEP;
	public:
		constexpr USBPhys (std::array<EPBuffer*, numEP> epBuffers) : m_epBuffers (epBuffers) {}

		// Verbiete Kopieren/Zuweisen

		USBPhys (const USBPhys&) = delete;
		USBPhys (USBPhys&&) = delete;
		USBPhys& operator = (const USBPhys&) = delete;
		USBPhys& operator = (USBPhys&&) = delete;

		void init ();
		void connect ();
		void disconnect ();

		void irq ();
	private:
		void resetDataToggle ();
		void setAddress (uint8_t addr);
		void initializeDevice ();

		/// Merkt die Zeiger auf die einzelnen EP-Puffer.
		const std::array<EPBuffer*, numEP> m_epBuffers;
};

/**
 * Repräsentiert einen Endpoint Puffer der Peripherie. Ein solcher entspricht 0-2 Endpoints auf
 * dem Bus (IN/OUT mit der jeweiligen Adresse). Beide Endpoints haben den gleichen Typ. Puffer
 * Nr. "n" besteht aus dem EPnR Register und BufDescTable[n] - diese Klasse bietet im Endeffekt
 * ein API für den Zugriff auf diese beiden.
 *
 * Die Reihenfolge der Puffer entspricht nicht notwendigerweise der Reihenfolge der Endpoints
 * auf dem Bus (aus Host-Sicht); Puffer 0 kann z.B: Endpoint 0x7 (OUT) und 0x87 (IN) verarbeiten
 * während Puffer 4 Endpoint 0x0 (OUT) und 0x80 (IN) behandelt. Im Konstruktor wird in iBuffer
 * der Index des Puffers (0-7) und die Adresse des Endpoints (0-15) angegeben. Die IN-Adresse
 * wird durch die Peripherie daraus automatisch durch "addr|0x80" gebildet.
 *
 * In rxBuffer und txBuffer sollen Zeiger auf die Puffer im USB-Speicher angegeben werden, d.h.
 * auf globale mit USB_MEM markierte Variablen. Werden niemals Daten in beide Richtungen
 * gleichzeitig übertragen (d.h. kein Aufruf an transmitPacket zwischen receivePacket & onReceive,
 * und kein Aufruf an receivePacket zwischen transmitPacket & onTransmit) können beide Zeiger
 * gleich sein ("half duplex").
 */
class EPBuffer {
	friend class USBPhys;
	public:
		constexpr EPBuffer (uint8_t iBuffer, uint8_t addr, EP_TYPE type, UsbMem* rxBuffer, size_t rxBufLength, UsbMem* txBuffer, size_t txBufLength)
			: m_rxBuffer (rxBuffer), m_txBuffer (txBuffer), m_rxBufLength (rxBufLength), m_txBufLength (txBufLength),
			  m_iBuffer (iBuffer), m_address (addr), m_type (type) {}

		// Verbiete Kopieren/Zuweisen

		EPBuffer (const EPBuffer&) = delete;
		EPBuffer (EPBuffer&&) = delete;
		EPBuffer& operator = (const EPBuffer&) = delete;
		EPBuffer& operator = (EPBuffer&&) = delete;

		void transmitPacket (const uint8_t* data, size_t length);
		void transmitStall ();
		void receivePacket (size_t length);
		void getReceivedData (uint8_t* buffer, size_t length);

		/// Fragt Größe des enthaltenen Empfangspuffers ab, in Bytes.
		size_t getRxBufLength () const { return m_rxBufLength; }
		/// Fragt Größe des enthaltenen Sendepuffers ab, in Bytes.
		size_t getTxBufLength () const { return m_txBufLength; }
	protected:
		virtual void onReset ();
		/**
		 * Wird von USBPhys aufgerufen, wenn Daten auf diesem Puffer empfangen wurden.
		 * "setup" gibt an, ob es ein SETUP-Transfer war, und rxBytes die Anzahl
		 * empfangener Bytes.
		 */
		virtual void onReceive (bool setup, size_t rxBytes) = 0;
		/// Wird von USBPhys aufgerufen, wenn Daten aus diesem Puffer abgesendet wurden.
		virtual void onTransmit () = 0;
	private:
		/// Speichert Empfangs-bzw. Sendepuffer.
		UsbMem* const m_rxBuffer, *const m_txBuffer;
		/// Speichert Länge der beiden Puffer.
		const size_t m_rxBufLength, m_txBufLength;
		/// Speichert Index des Puffers, d.h. Nummer des EPnR-Registers und des BufDescTable-Eintrags.
		const uint8_t m_iBuffer;
		/// Speichert Bus-Adresse der diesem Puffer zugewiesenen Endpoints.
		const uint8_t m_address;
		/// Speichert den Typ der Endpoints.
		const EP_TYPE m_type;
};

/**
 * Implementiert das Protokoll für Control-Transfers, z.B. für den Default Control Endpoint 0. Control-Transfers
 * bestehen aus den Schritten "Setup", "Data" (optional), und "Status". Bei "Setup" wird ein initiales Datenpaket
 * vom Host gesendet, und der Callback onSetupStage() aufgerufen. Dieser kann dann per dataInStage/dataOutStage
 * die "Data" Stage initiieren, oder per "statusStage" direkt die Status stage. Die "Data" stage wird durch die
 * Callbacks onDataIn bzw. onDataOut quittiert, wo statusStage aufgerufen werden muss. Die "Status" stage wird
 * durch onStatusStage quittiert, und es wird automatisch zur nächsten "Setup" stage gewechselt.
 * Die Implementierung ermöglicht die Übertragung größerer Datenblöcke, welche nicht in ein Datenpaket passen.
 * Da Control Endpoints "half-duplex" arbeiten, ist nur ein Puffer nötig.
 */
class ControlEP : public EPBuffer {
	public:
		constexpr ControlEP (uint8_t iBuffer, uint8_t addr, UsbMem* buffer, size_t bufLength)
			: EPBuffer (iBuffer, addr, EP_TYPE::CONTROL, buffer, bufLength, buffer, bufLength), m_data (nullptr), m_count (0), m_remaining (0), m_state (CTRL_STATE::SETUP) {}

		// Verbiete Kopieren/Zuweisen

		ControlEP (const ControlEP&) = delete;
		ControlEP (ControlEP&&) = delete;
		ControlEP& operator = (const ControlEP&) = delete;
		ControlEP& operator = (ControlEP&&) = delete;

		void dataInStage (const uint8_t* data, size_t length);
		void dataOutStage (uint8_t* data, size_t length);
		void statusStage (bool success);

	protected:
		/**
		 * Wird aufgerufen, nachdem vom Host ein "Setup" Paket empfangen wurde. Sollte dataInStage,
		 * dataOutStage oder statusStage aufrufen.
		 */
		virtual void onSetupStage () = 0;
		/**
		 * Wird aufgerufen, wenn in der Data Stage alle Daten vom Host empfangen wurden. "rxBytes"
		 * gibt die Anzahl der empfangenen Bytes an. Sollte statusStage aufrufen.
		 */
		virtual void onDataOut (size_t rxBytes) = 0;
		/**
		 * Wird aufgerufen, wenn in der Data Stage alle Daten an den Host gesendet wurden.
		 * Da bei "In" transfers kein Erfolg signalisiert wird, sollte hier
		 * statusStage NICHT aufgerufen werden. Kann daher leer gelassen werden.
		 */
		virtual void onDataIn () = 0;
		/**
		 * Wird aufgerufen, wenn ein leeres Datenpaket zur Signalisierung des Erfolgs
		 * abgesendet wurde (bei Out-Transfers oder solchen ohne Data Stage - in=false),
		 * oder ein leeres Paket Empfangen wurde (bei In-Transfers - in=true).
		 */
		virtual void onStatusStage (bool in) = 0;

		virtual void onReset () override;
		virtual void onReceive (bool setup, size_t rxBytes) override final;
		virtual void onTransmit () override final;
	private:
		void receiveControlPacket ();
		/// Datenpuffer für zu sendende/empfangende Daten.
		uint8_t* m_data;
		/// Gesamtzahl zu übertragender Bytes
		size_t m_count;
		/// Verbleibende Anzahl zu übertragender Bytes
		size_t m_remaining;
		/// Aktueller Zustand des Zustandsautomaten
		CTRL_STATE m_state;
};

/**
 * Nutzt das Protokoll für Control Endpoints, um auf Standard-Anfragen auf dem Default Control
 * Endpoint zu reagieren. Nutzt immer die Endpoint-Adresse 0. Hat kein externes API; die Verarbeitung
 * eigener oder klassenspezifischer Anfragen kann hier ergänzt werden.
 */
class DefaultControlEP : public ControlEP {
	public:
		constexpr DefaultControlEP (USBPhys& phys, uint8_t iBuffer, UsbMem* buffer, size_t bufLength)
			: ControlEP (iBuffer, 0, buffer, bufLength), m_phys (phys), m_wValue (0), m_wIndex (0), m_wLength (0),
			  m_setAddress (0), m_bmRequestType (0), m_bRequest (0) {}

		// Verbiete Kopieren/Zuweisen

		DefaultControlEP (const DefaultControlEP&) = delete;
		DefaultControlEP (DefaultControlEP&&) = delete;
		DefaultControlEP& operator = (const DefaultControlEP&) = delete;
		DefaultControlEP& operator = (DefaultControlEP&&) = delete;
	protected:
		virtual void onReset () override final;
		virtual void onSetupStage () override final;
		virtual void onDataOut (size_t rxBytes) override final;
		virtual void onDataIn () override final;
		virtual void onStatusStage (bool in) override final;
	private:
		/// Zugehörige USBPhys-Instanz.
		USBPhys& m_phys;
		/// Wird in onSetupStage mit den Daten aus der Anfrage gefüllt, speichert die Daten für die weiteren Callbacks.
		uint16_t m_wValue, m_wIndex, m_wLength;

		/**
		 * Wenn der Host eine Adresse zuweist, wird sie zunächst in dieser Variablen gespeichert. In der
		 * Status stage wird die Adresse in die Peripherie geschrieben und die Variable wieder auf 0 gesetzt.
		 * Laut Spezifikation darf die Adresse nicht sofort übernommen werden, sondern erst nach der Bestätigung.
		 */
		uint8_t m_setAddress;

		/// Wird in onSetupStage mit den Daten aus der Anfrage gefüllt, speichert die Daten für die weiteren Callbacks.
		uint8_t m_bmRequestType, m_bRequest;
};

#endif /* USB_HH_ */
