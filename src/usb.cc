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
 * In dieser Datei befindet sich der wesentliche Teil der USB-Peripherieansteuerung und die Behandlung von Standard-Anfragen
 * und Datenübertragungen.
 */

#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <stm32f1xx.h>

#include "usb.hh"
#include "usb_desc.hh"
#include "main.hh"

/// Globaler Interrupt für die USB-Peripherie. Sollte von der ISR direkt aufgerufen werden.
void USBPhys::irq () {
	uint16_t ISTR;

	// Nur diese Interrupts werden verarbeitet
	const uint16_t interrupts = USB_ISTR_RESET | USB_ISTR_CTR;

	// Bearbeite in einer Schleife so lange aufgetretene Ereignisse, bis die USB Peripherie keine weiteren signalisiert.
	while (((ISTR = USB->ISTR) & interrupts) != 0) {
		// Ein "RESET" tritt auf beim Ausbleiben von Paketen vom Host. Dies ist zu Beginn jeder Verbindung der Fall,
		// bis der Host das Device erkennt.
		if (ISTR & USB_ISTR_RESET) {
			// Lösche Interrupt
			clearInterrupt (USB_ISTR_RESET);
			// Bringe Hard-und Software in definierten Ausgangszustand
			initializeDevice ();
		}
		// CTR signalisiert die Beendigung eines korrekten Transfers
		if (ISTR & USB_ISTR_CTR) {
			// Richtung des letzten Transfers. false bedeutet "IN" transfer (Device->Host), true bedeutet "IN" oder "OUT"
			bool dir = ISTR & USB_ISTR_DIR; // 0=TX, 1=RX/TX
			// Die Nummer des EPnR-Registers, welches zu diesem Transfer gehört.
			uint8_t EP = (ISTR & USB_ISTR_EP_ID) >> USB_ISTR_EP_ID_Pos;
			// Stelle sicher, dass das EPnR Register existiert
			myassert (EP < numEP);
			// Frage Zustand dieses Endpoints ab
			uint16_t s = EPnR [EP].data;

			// Lösche im EPnR-Register die RX/TX-Flags, falls sie gesetzt sind. Falls die Hardware zwischen Abfragen und Löschen
			// eines der Bits setzt, wird dies nicht gelöscht und im nächsten Schleifendurchlauf behandelt.
			setEPnR (EP, s & (USB_EP0R_CTR_RX_Msk | USB_EP0R_CTR_TX_Msk), 0, s);

			// Prüfe ob es ein empfangender Transfer (OUT/SETUP) war.
			if (dir && (s & USB_EP0R_CTR_RX_Msk) != 0) {
				// Prüfe auf SETUP-Transfer (OUT und SETUP verhalten sich identisch bis auf dieses Bit)
				bool setup = s & USB_EP0R_SETUP_Msk;

				// Frage Anzahl der empfangenen Bytes im Buffer ab.
				uint16_t rxBytes = BufDescTable[EP].rxBufferCount & USB_COUNT0_RX_0_COUNT0_RX_0;

				if (m_epBuffers [EP])
					// Leite an EPBuffer-Instanz weiter.
					m_epBuffers [EP]->onReceive (setup, rxBytes);
				else
					// Ist der Endpoint nicht genutzt, löse Fehler im Debugger auf; im Normalbetrieb soll gar nichts passieren
					// statt eines Absturzes, der durch die explizite Abfrage vermieden wird.
					myassert (false);
			}
			// Prüfe ob es ein sendender Transfer (IN) war.
			if ((s & USB_EP0R_CTR_TX_Msk) != 0) {
				if (m_epBuffers [EP])
					// Leite an EPBuffer-Instanz weiter.
					m_epBuffers [EP]->onTransmit ();
				else
					// Ist der Endpoint nicht genutzt, löse Fehler im Debugger auf; im Normalbetrieb soll gar nichts passieren
					// statt eines Absturzes, der durch die explizite Abfrage vermieden wird.
					myassert (false);
			}
		}
	}
}

/**
 * Diese Funktion wartet mindestens 1µs (tSTARTUP), implementiert mit einer simplen "NOP"-Schleife. Diese wird je
 * nach Compiler tatsächlich länger brauchen, was aber normalerweise nicht schlimm ist, da sie nur 1x beim
 * Hochfahren aufgerufen wird. Falls nötig, kann die Funktion ggf. durch Inline-Assembler oder Nutzung des
 * ARM Zyklenzählers so umgebaut werden, dass sie auch nicht länger als 1µs braucht.
 */
static void delay () {
	// Zähle mindestens 72 Takte (1µs bei 72MHz)
	for (int i = 0; i < 72; ++i) __NOP ();
}

/// Diese Funktion aktiviert die für USB-Übertragung nötige Peripherie, baut aber noch keine Verbindung auf.
void USBPhys::init () {
	// Die USB-Pins sind in GPIOA, aber der GPIOA-Peripherietakt muss nicht aktiviert werden, USB funktioniert direkt
	// Aktiviere USB-Takt
	RCC->APB1ENR |= RCC_APB1ENR_USBEN_Msk;

	// Konfiguriere Pin für 1.5kOhm-Widerstand, und schalte Pin auf high, s.d. Widerstand aus ist
	// TODO: Ggf. anderen Port/Pin wählen
	GPIOC->CRH = 0x44474444;
	GPIOC->BSRR = GPIO_BSRR_BS12;

	// Schalte USB Interrupt ein
	NVIC_EnableIRQ (USB_LP_CAN1_RX0_IRQn);

	// Lasse USB-Peripherie noch im Stromspar-Modus
	USB->CNTR = USB_CNTR_FRES | USB_CNTR_PDWN;
}

/// Diese Funktion startet die Verbindung zum Host. Zuvor muss init aufgerufen werden.
void USBPhys::connect () {
	// Schalte Transceiver ein, lasse Logik aus
	USB->CNTR = USB_CNTR_FRES;

	// Warte auf Hochfahren der analogen Schaltung (tSTARTUP)
	delay ();

	// Schalte USB ein, aktiviere Interrupts
	USB->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM;

	// Lösche alle Interrupts außer Reset-Interrupt
	USB->ISTR = USB_ISTR_RESET_Msk;
	NVIC_ClearPendingIRQ (USB_LP_CAN1_RX0_IRQn);

	// TODO: Ggf. anderen Port/Pin wählen
	// Schalte 1.5kOhm-Widerstand ein, s.d. Host das Gerät erkennt.
	GPIOC->BSRR = GPIO_BSRR_BR12;
}

/// Versetzt die USB-Peripherie wieder in Stromspar-Modus (wie nach init()), trennt Verbindung zum Host.
void USBPhys::disconnect () {
	// Versetze Peripherie in Stromspar-Modus
	USB->CNTR = USB_CNTR_FRES | USB_CNTR_PDWN;

	// Schalte Pin auf high, s.d. Widerstand aus ist. Host erkennt Trennen der Verbindung.
	// TODO: Ggf. anderen Port/Pin wählen
	GPIOC->BSRR = GPIO_BSRR_BS12;

	// Lösche alle Interrupts
	USB->ISTR = 0;
	NVIC_ClearPendingIRQ (USB_LP_CAN1_RX0_IRQn);
}

/**
 * Diese Funktion wird aufgerufen, wenn der Host das Device zurücksetzt ("RESET"), ggf. auch zusätzlich beim Hochfahren.
 * Sie sollte Hard-und Software auf einen definierten Anfangszustand setzen, damit der Treiber auf Host-Seite von einem
 * sauberen Zustand ausgehen kann.
 */
void USBPhys::initializeDevice () {
	myassert ((mapAddr (BufDescTable) & 7) == 0);
	// Mache der Peripherie die Buffer Descriptor Table bekannt
	USB->BTABLE = mapAddr (BufDescTable);
	// Neu verbundene Geräte haben Adresse 0. Speichere diese im USB-Register.
	USB->DADDR = USB_DADDR_EF | (0 << USB_DADDR_ADD_Pos);

	// Initialisiere alle Endpoints.
	for (EPBuffer* ep : m_epBuffers)
		if (ep)
			ep->onReset ();
}

/**
 * Benutzt DTOG_RX/TX im EPnR-Register, um alle Bulk- und Interrupt -Endpoints auf "DATA0" zurückzusetzen.
 * Dies wird aufgerufen, wenn ein SET_CONFIGURATION Befehl empfangen wurde. Control Endpoints starten immer
 * mit DATA0.
 */
void USBPhys::resetDataToggle () {
	for (uint_fast8_t iEP = 0; iEP < numEP; ++iEP) {
		uint16_t s = EPnR [iEP].data;

		// Prüfe Typ des Endpoints (Bulk/Interrupt)
		if ((s & USB_EP_T_FIELD_Msk) == USB_EP_BULK || (s & USB_EP_T_FIELD_Msk) == USB_EP_INTERRUPT) {
			// Prüfe ob Senden/Empfangen aktiviert. Diese Information ließe sich auch über m_epBuffers
			// gewinnen, aber so ist es einfacher.
			bool rx = (s & USB_EPRX_STAT) != USB_EP_RX_DIS;
			bool tx = (s & USB_EPTX_STAT) != USB_EP_TX_DIS;
			if (rx && tx)
				// Setze beide Richtungen zurück
				setEPnR (static_cast<uint8_t> (iEP), USB_EP_DTOG_RX_Msk | USB_EP_DTOG_TX_Msk, 0, s);
			else if (rx)
				// Nur Empfangen zurücksetzen
				setEPnR (static_cast<uint8_t> (iEP), USB_EP_DTOG_RX_Msk, 0, s);
			else if (tx)
				// Nur Senden zurücksetzen
				setEPnR (static_cast<uint8_t> (iEP), USB_EP_DTOG_TX_Msk, 0, s);
		}
	}
}

/**
 * Sollte beim Empfang von SET_ADDERSS aufgerufen werden. Macht die neue Adresse der Peripherie bekannt.
 */
void USBPhys::setAddress (uint8_t addr) {
	USB->DADDR = static_cast<uint16_t> (USB_DADDR_EF | addr);
}

/**
 * Bereitet das Senden eines einzelnen Datenpakets vor. Kopiert die Daten in den USB-Puffer und konfiguriert
 * die Peripherie zum Senden. Nach Abschluss des Transfers wird onTransmit aufgerufen. "length" muss <=
 * der im Konstruktor angegebenen Puffergröße sein.
 */
void EPBuffer::transmitPacket (const uint8_t* data, size_t length) {
	if (length) {
		myassert (length <= m_txBufLength && length <= 0xFFFF);
		// Baue je zwei Bytes zu einem uint16_t zusammen und schreibe diesen in den USB-Pufferspeicher.
		for (uint_fast16_t i = 0; i < length / 2; ++i) {
			uint_fast16_t a = static_cast<uint8_t> (data [i*2]);
			uint_fast16_t b = static_cast<uint8_t> (data [i*2+1]);
			m_txBuffer [i].data = static_cast<uint16_t> (a | (b << 8));
		}
		// Falls noch ein Byte übrig geblieben ist, schreibe dies einzeln.
		if (length % 2) {
			m_txBuffer [length/2].data = static_cast<uint8_t> (data [length-1]);
		}
		// Gebe Puffer-Größe&Anfang an Peripherie
		BufDescTable [m_iBuffer].txBufferAddr = mapAddr (m_txBuffer);
	} else {
		// Leeres Paket
		BufDescTable [m_iBuffer].txBufferAddr = 0;
	}
	BufDescTable [m_iBuffer].txBufferCount = static_cast<uint16_t> (length);
	// Erlaube Absenden der Daten.
	setEPnR (m_iBuffer, USB_EPTX_STAT_Msk, USB_EP_TX_VALID);
}

/**
 * Zur Nutzung im Fehlerfall, konfiguriert den Endpoint so, dass beim Datenempfang ("OUT") immer
 * STALL zurücksendet. Wird bei Control Endpoints zur Ablehnung einer Anfrage genutzt.
 */
void EPBuffer::transmitStall () {
	setEPnR (m_iBuffer, USB_EPTX_STAT_Msk, USB_EP_TX_STALL);
}

/**
 * Bereitet den Empfang eines einzelnen Datenpakets vor. Konfiguriert die Peripherie zum Empfang.
 * Nach Abschluss des Transfers wird onReceive aufgerufen. "length" muss entweder <= 62 und Vielfaches
 * von 2 sein, oder <= 512 und Vielfaches von 32. Ist length 0, wird nur ein leeres Datenpaket
 * akzeptiert (eingestellt über EP_KIND).
 */
void EPBuffer::receivePacket (size_t length) {
	if (length) {
		myassert (( ((length <= 62) && (length % 2) == 0)
				||	((length <= 512) && (length % 32 == 0)))
				&&	length <= m_rxBufLength);

		// Im rxBufferCount Register erfolgt die Angabe zu sendender Bytes entweder in 2-Byte oder 32-Byte
		// Schritten. Bestimme automatisch die nötige Größe.
		const uint16_t BL_SIZE = !(length <= 62);
		const uint16_t NUM_BLOCK = BL_SIZE ? static_cast<uint16_t> ((length/32)-1) : static_cast<uint16_t> (length/2);

		// Konfiguriere Puffer
		BufDescTable [m_iBuffer].rxBufferAddr = mapAddr (m_rxBuffer);
		BufDescTable [m_iBuffer].rxBufferCount = static_cast<uint16_t> ((BL_SIZE << 15) | (NUM_BLOCK << 10));

		// Setze EP_KIND zurück, da Paket größer 0 ist.
		setEPnR (m_iBuffer, USB_EPRX_STAT_Msk | USB_EP_KIND_Msk, USB_EP_RX_VALID);
	} else {
		BufDescTable [m_iBuffer].rxBufferCount = 0;
		// Setze EP_KIND um nur 0-Pakete zu akzeptieren
		setEPnR (m_iBuffer, USB_EPRX_STAT_Msk | USB_EP_KIND_Msk, USB_EP_RX_VALID | USB_EP_KIND);
	}
}

/**
 * Aufzurufen im onReceive-Callback; kopiert die empfangenen Daten aus dem USB-Puffer in normalen
 * Speicher. length sollte <= dem rxBytes Parameter von onReceive sein.
 */
void EPBuffer::getReceivedData (uint8_t* buffer, size_t length) {
	myassert (length <= 0xFFFF);
	// Iteriere 16bit-Worte im USB-Pufferspeicher, spalte sie in 2 Bytes auf, und speichere sie im Ziel.
	for (uint_fast16_t i = 0; i < length/2; ++i) {
		uint16_t x = m_rxBuffer [i].data;
		buffer [i*2] = static_cast<uint8_t> (x & 0xFF);
		buffer [i*2+1] = static_cast<uint8_t> ((x>>8) & 0xFF);
	}
	// Wenn Größe ungerade, übertrage verbleibendes Byte
	if (length % 2) {
		buffer [length-1] = static_cast<uint8_t> (m_rxBuffer [length/2].data & 0xFF);
	}
}

/**
 * Wird von USBPhys aufgerufen, wenn USB-Peripherie zurückgesetzt wird. Konfiguriert den Endpoint neu,
 * sendet/empfängt aber noch nichts. Kann überschrieben werden, muss aber immer auch mit
 * aufgerufen werden.
 */
void EPBuffer::onReset () {
	setEPnR (m_iBuffer, USB_EPRX_STAT_Msk | USB_EP_T_FIELD_Msk | USB_EPADDR_FIELD | USB_EP_KIND | USB_EPTX_STAT_Msk,
						static_cast<uint16_t> (USB_EP_RX_NAK | USB_EP_TX_NAK
						| (static_cast<uint16_t> (m_type) << USB_EP_T_FIELD_Pos)
						| (uint16_t { m_address } << USB_EPADDR_FIELD_Pos)));
}

/// Setzt den Protokoll-Zustandsautomaten zurück.
void ControlEP::onReset () {
	EPBuffer::onReset ();

	m_data = nullptr; m_count = 0; m_remaining = 0;
	m_state = CTRL_STATE::SETUP;

	receiveControlPacket ();
}

/// Enthält den empfangenden Teil des Zustandsautomaten für Control Transfers.
void ControlEP::onReceive (bool setup, size_t rxBytes) {
	if (setup) {
		// SETUP-Pakete sollten zwar nur auftreten wenn wir im "SETUP" Zustand sind (prüfe
		// über assert), zur Erhöhung der Robustheit setzen wir den Zustandsautomat aber einfach zurück.
		myassert (m_state == CTRL_STATE::SETUP);
		m_data = nullptr; m_count = 0; m_remaining = 0;
		m_state = CTRL_STATE::SETUP;
		onSetupStage ();
	} else {
		switch (m_state) {
			case CTRL_STATE::STATUS_IN:
				// Wir haben ein 0-Byte-Paket als Bestätigung abgesendeter Daten empfangen

				// Gehe zurück zum Anfang
				m_state = CTRL_STATE::SETUP;
				// Empfangspuffer erneut bereit machen
				receiveControlPacket ();
				// Benachrichtige höhere Protokollebene
				onStatusStage (true);
				break;
			case CTRL_STATE::DATA_OUT:
				// Es wurde Teil eines Datenblocks empfangen
				{
					// Wie viele Bytes tatsächlich noch gespeichert werden können
					// (normalerweise sollten beide immer gleich sein)
					size_t usableBytes = std::min (rxBytes, m_remaining);
					if (usableBytes) {
						// Speichere empfangene Daten
						getReceivedData (m_data, usableBytes);
						m_remaining -= usableBytes;
						m_data += usableBytes;
					}
					// Leeres bzw. nicht-volles Datenpaket terminiert Sequenz
					if (rxBytes <= getRxBufLength ()) {
						onDataOut (m_count - m_remaining);
					} else {
						// Noch nicht fertig - berechne nächsten Puffer, bereite Empfang vor
						receiveControlPacket ();
					}
				}
				break;
			default:
				myassert (false);
		}
	}
}

/// Enthält den sendenden Teil des Zustandsautomaten für Control Transfers.
void ControlEP::onTransmit () {
	switch (m_state) {
		case CTRL_STATE::DATA_IN:
			// Datenpaket abgesendet
			size_t length;
			if (m_remaining == 0) {
				// Vorheriger Transfer hat die letzten Daten gesendet, welche genau lang genug
				// für ein Paket waren. Sende daher ein 0-Paket, um Ende zu signalisieren
				length = 0;
				m_state = CTRL_STATE::DATA_IN_LAST;
			} else if (m_remaining < getRxBufLength ()) {
				// Daten sind kürzer als Paketgröße. Sende Rest der Daten als letztes Paket;
				// Host erkennt dadurch Ende des Blocks
				length = m_remaining;
				m_state = CTRL_STATE::DATA_IN_LAST;
			} else if (m_remaining == getRxBufLength ()) {
				// Verbleibende Daten passen genau in ein Paket. Sende ab, und danach ein 0-Paket
				length = m_remaining;
				m_remaining = 0;
			} else {
				// Es sind mehr Daten zu senden, als in ein Paket passen. Sende ab und durchlaufe
				// diese Abfrage erneut
				length = getRxBufLength ();
				m_remaining -= length;
			}
			transmitPacket (m_data, length);
			// Merke Position im Puffer
			m_data += length;
			break;
		case CTRL_STATE::DATA_IN_LAST:
			// Letztes Paket abgesendet. Warte auf Bestätigung.
			m_state = CTRL_STATE::STATUS_IN;
			// Empfange ein 0-Paket
			receivePacket (0);
			// Benachrichtige nächste Protokollebene über erfolgreiches Absenden
			onDataIn ();
			break;
		case CTRL_STATE::STATUS_OUT:
			// Es wurde ein 0-Paket abgesendet.
			// Beginne Zustandsautomaten von vorne
			m_state = CTRL_STATE::SETUP;
			// Empfangspuffer erneut bereit machen
			receiveControlPacket ();

			// Benachrichtige nächste Protokollebene über Übertragung des Status
			onStatusStage (false);
			break;
		default:
			myassert (false);
	}
}

/// Bereitet Empfang eines Pakets auf Control Endpoint vor.
void ControlEP::receiveControlPacket () {
	// Pakete von Control Endpoints dürfen max. 64 Bytes groß sein; empfange so viel wie möglich.
	receivePacket (std::min<size_t> (getRxBufLength (), 64));
}

/// Aufzurufen vom onSetupStage()-Callback aus. Bereitet das Absenden eines Datenblocks vor.
void ControlEP::dataInStage (const uint8_t* data, size_t length) {
	size_t pkLength;
	size_t bufSize = getTxBufLength ();
	if (length < bufSize) {
		// Muss nur 1 Paket absenden
		m_state = CTRL_STATE::DATA_IN_LAST;
		m_remaining = 0;
		m_data = nullptr;
		pkLength = length;
	} else if (length == bufSize) {
		// Muss 1 Paket und ein 0-Paket absenden um Ende zu markieren
		m_state = CTRL_STATE::DATA_IN;
		m_remaining = 0;
		m_data = nullptr;
		pkLength = length;
	} else {
		// Muss mehrere Pakete senden
		m_state = CTRL_STATE::DATA_IN;
		m_remaining = length - bufSize;
		// const_cast ist zwar unsauber aber nicht falsch, denn beim Senden wird auf m_data
		// nie geschrieben, und so kann m_data sowohl zum Senden als auch zum Empfangen genutzt werden.
		m_data = const_cast<uint8_t*> (data) + bufSize;
		pkLength = bufSize;
	}
	// Sende Daten
	transmitPacket (data, pkLength);
}

/// Aufzurufen vom onSetupStage()-Callback aus. Bereitet das Empfangen eines Datenblocks vor.
void ControlEP::dataOutStage (uint8_t* data, size_t length) {
	// Merke Daten
	m_state = CTRL_STATE::DATA_OUT;
	m_count = length;
	m_remaining = length;
	m_data = data;
	// Bereite Empfang vor
	receiveControlPacket ();
}

/**
 * Aufzurufen von einem der Callbacks aus. Signalisiert dem Host über leeres Paket oder STALL Erfolg
 * bzw. Misserfolg einer Operation.
 */
void ControlEP::statusStage (bool success) {
	if (success) {
		// Sende 0-Paket, warte Absenden ab
		m_state = CTRL_STATE::STATUS_OUT;
		transmitPacket (nullptr, 0);
	} else {
		// Gehe direkt zum Beginn des Zustandsautomaten
		m_state = CTRL_STATE::SETUP;
		// Sende STALL
		transmitStall ();
		// Empfangspuffer erneut bereit machen
		receiveControlPacket ();
	}
}

/// Setzt das Protokoll zurück.
void DefaultControlEP::onReset () {
	ControlEP::onReset ();
	// Mehr als diese Variable ist hier nicht zurückzusetzen
	m_setAddress = 0;
}

/// Verarbeite Standard-Anfragen auf dem Default Control Endpoint 0.
void DefaultControlEP::onSetupStage () {
	// Frage empfangene Daten ab
	uint8_t pkBuffer [8];
	getReceivedData (pkBuffer, 8);
	// Baue aus Rohdaten die einzelnen Zahlen zusammen und speichere sie in Member-Variablen, um auch
	// in den Data Stage Callbacks darauf zugreifen zu können.
	m_bmRequestType = pkBuffer [0];
	m_bRequest = pkBuffer [1];
	m_wValue = static_cast<uint16_t> (pkBuffer [2] | (uint16_t {pkBuffer [3]} << 8));
	m_wIndex = static_cast<uint16_t> (pkBuffer [4] | (uint16_t {pkBuffer [5]} << 8));
	m_wLength = static_cast<uint16_t> (pkBuffer [6] | (uint16_t {pkBuffer [7]} << 8));

	if (	// Eine Standard-USB-Anfrage eines Deskriptors
			(m_bmRequestType == 0x80 && m_bRequest == ST_REQ::GET_DESCRIPTOR)
			// Oder eine Microsoft-spezifische Abfrage eines OS String Deskriptors
	||		(m_bmRequestType == 0xC0 && m_bRequest == ST_REQ::GET_OS_STRING_DESC)
	) {
		// Bei Standard-Anfragen ist der Typ des Deskriptors vorgegeben, ansonsten immer den OS String Deskriptor nehmen
		D_TYPE descType = m_bmRequestType == 0xC0 ? D_TYPE::OS_DESCRIPTOR : static_cast<D_TYPE> (m_wValue >> 8);
		// Es gibt nur 1 OS String Deskriptor; bei anderen nutze den gewünschten Index
		uint8_t descIndex = m_bmRequestType == 0xC0 ? 0 : static_cast<uint8_t> (m_wValue & 0xFF);

		// Durchsuche Deskriptor-Tabelle
		const Descriptor* d = getUsbDescriptor (descType, descIndex);
		if (!d) {
			// Kein passender Deskriptor - sende Fehler
			statusStage (false);
		} else {
			// Sende nur max. so viel wie gefordert. Falls Deskriptor länger ist, wird der Host eine erneute Anfrage des
			// ganzen Deskriptors stellen, dessen Gesamtlänge immer ganz zu Beginn steht und somit nach der 1. Anfrage
			// bekannt ist.
			uint16_t length = std::min<uint16_t> (m_wLength, d->length);

			// Sende Deskriptor, ggf. in mehreren Paketen
			dataInStage (d->data, length);
		}
	} else if (m_bmRequestType == 0 && m_bRequest == ST_REQ::SET_ADDRESS) {
		// Zuweisung einer USB-Adresse.

		// Merke Adresse; diese wird erst nach Absenden der Bestätigung gesetzt
		m_setAddress = static_cast<uint8_t> (m_wValue & 0x7F);

		// Sende Bestätigung
		statusStage (true);
	} else if (m_bmRequestType == 0 && m_bRequest == ST_REQ::SET_CONFIGURATION) {
		// Dieses Device unterstützt nur 1 Konfiguration. Simuliere das Setzen, indem nur die eine akzeptiert wird
		// und dann nichts unternommen wird.

		uint8_t conf = static_cast<uint8_t> (m_wValue & 0xFF);
		if (conf == 0) {
			// Wird Konfiguration 0 gesetzt, soll sich das Gerät reinitialisieren, als hätte es soeben eine initiale Adresse bekommen.
			// In diesem einfachen Beispiel ist nichts zu tun.

			// Sende Bestätigung.
			statusStage (true);
		} else if (conf == 1) {
			// Da nur 1 Konfiguration vorhanden, kein tatsächliches Umschalten nötig

			// Bei IN/OUT transfers wird abwechselnd mit DATA0/DATA1 Befehlen übertragen, um Fehler zu erkennen.
			// Nach dem Setzen einer Konfiguration soll immer mit DATA0 weiter gemacht werden.

			// Setze hier daher alle nicht-Control-Endpoints auf "DATA0".
			m_phys.resetDataToggle ();

			// Sende Bestätigung
			statusStage (true);
		} else {
			// Nicht unterstützte Konfiguration
			statusStage (false);
		}
	} else if (m_bmRequestType <= 2 && m_bRequest == ST_REQ::CLEAR_FEATURE) {
		// Features werden nicht unterstützt
		statusStage (false);
	} else if (m_bmRequestType <= 2 && m_bRequest == ST_REQ::SET_FEATURE) {
		// Features werden nicht unterstützt
		statusStage (false);
	} else if (m_bmRequestType >= 0x80 && m_bmRequestType <= 0x82 && m_bRequest == ST_REQ::GET_STATUS) {
		// Antworte immer mit 0 - Gerät ist nicht self-powered, unterstützt kein Remote Wakeup, und kein Halt Feature.
		uint8_t buffer [2] = { 0, 0 };
		// 2 Bytes passen immer in 1 Paket, daher kann der Puffer lokal sein
		dataInStage (buffer, 2);
	} else if (m_bmRequestType == 0x81 && m_bRequest == ST_REQ::GET_INTERFACE) {
		if (m_wValue == 0 && m_wIndex == 0 && m_wLength == 1) {
			// Gerät hat nur 1 Interface. Sende immer 0 zurück.
			uint8_t buffer [1] = { 0 };
			// 1 Bytes passt immer in 1 Paket, daher kann der Puffer lokal sein
			dataInStage (buffer, 1);
		} else {
			// Nicht unterstützte Operation
			statusStage (false);
		}
	} else if (m_bmRequestType == 1 && m_bRequest == ST_REQ::SET_INTERFACE) {
		// Dieses einfache Device unterstützt nur 1 Interface. Simuliere daher das Umschalten
		if (m_wValue == 0 && m_wIndex == 0 && m_wLength == 0) {
			// Das einzige Interface wurde aktiviert. Bestätige.
			statusStage (true);
		} else {
			// Nicht unterstützte Operation
			statusStage (false);
		}

	// Ab hier folgen Geräte/Klassen-spezifische Anfragen
	} else if (m_bmRequestType == 0x40 && m_bRequest == 1) {
		outputPulse (m_wValue);
		statusStage (true);
	} else if (m_bmRequestType == 0x40 && m_bRequest == 2) {
		outputPulse (m_wValue);
	} else {
		// Unbekannte Anfragen abweisen
		statusStage (false);
	}
}

void DefaultControlEP::onPulseDone () {
	if (m_bRequest == 2)
		statusStage (true);
}

void DefaultControlEP::onDataOut (size_t) {
}

void DefaultControlEP::onDataIn () {
	// Bei den implementierten Operationen gibt es hier nichts zu tun
}

void DefaultControlEP::onStatusStage (bool in) {
	// Haben wir gerade die Bestätigung für SET_ADDRESS gesendet?
	if (m_setAddress && !in) {
		// Jetzt erst die Adresse übernehmen (von Spezifikation vorgegeben)
		m_phys.setAddress (m_setAddress);
		// Aber nur diesmal
		m_setAddress = 0;
	}
}
