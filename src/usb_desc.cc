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
 * In dieser Datei werden die konkreten Deskriptoren für das Device definiert unter Nutzung der Hilfsfunktionen
 * in usb_desc_helper.hh. Die Deskriptoren werden als einzelne globale konstante Variablen definiert und landen
 * somit als Konstanten im Flash. In der Tabelle "descriptors" werden Pointer auf die Deskriptoren zusammen mit Länge,
 * Typ und Index aufgelistet, damit sie zur Laufzeit über die Funktion getUsbDescriptor gefunden werden können.
 */

#include "usb_desc.hh"
#include "main.hh"

// Die hier im Device Deskriptor angegebene Device Class markiert das Gerät als Composite Device.
static constexpr auto deviceDescriptor = EncodeDescriptors::USB20::device (
			0x200,		// bcdUSB
			0xFF,		// bDeviceClass
			0xFF,		// bDeviceSubClass
			0xFF,		// bDeviceProtocol
			64,			// bMaxPacketSize0
			0xDEAD,		// idVendor		TODO - anpassen
			0xBEEF,		// idProduct	TODO - anpassen
			0x0100,		// bcdDevice
			1,			// iManufacturer, entspricht dem Index des strManufacturer-Deskriptors
			2,			// iProduct, entspricht dem Index des strProduct-Deskriptors
			3,			// iSerialNumber, entspricht dem Index des strSerial-Deskriptors
			1			// bNumConfigurations
		);

/*
 * Bei Endpoints darf wMaxPacketSize maximal so groß sein wie die Größe des jeweiligen in usb.cc definierten Puffers.
 * bEndpointAddress ist die Adresse des endpoints auf dem Bus; ein Endpoint kann einem beliebigen EPnR
 * Register zugewiesen werden.
 */


static constexpr auto confDescriptor = EncodeDescriptors::USB20::configuration (
			1,			// bNumInterfaces
			1,			// bConfigurationValue
			0,			// iConfiguration
			0x80,		// bmAttributes
			250,		// bMaxPower (500mA)

			EncodeDescriptors::USB20::interface (
				0,		// bInterfaceNumber
				0,		// bAlternateSetting
				2,		// bNumEndpoints
				0xFF,	// bInterfaceClass
				0xFF,	// bInterfaceSubClass
				0xFF,	// bInterfaceProtocol
				0		// iInterface
			),
			/*
			 * wMaxPacketSize darf maximal so groß sein wie die Größe des jeweiligen in usb.cc definierten Puffers.
			 * bEndpointAddress ist die Adresse des endpoints auf dem Bus; ein Endpoint kann einem beliebigen EPnR
			 * Register zugewiesen werden. @TODO - hier gewünschte Endpoints konfigurieren.
			 */
			EncodeDescriptors::USB20::endpoint (
				1,		// bEndpointAddress
				2,		// bmAttributes
				dataEpMaxPacketSize,		// wMaxPacketSize
				10		// bInterval
			),
			EncodeDescriptors::USB20::endpoint (
				0x81,	// bEndpointAddress
				2,		// bmAttributes
				dataEpMaxPacketSize,		// wMaxPacketSize
				10		// bInterval
		)
);

/**
 * Dieser Microsoft Compat Id Deskriptor identifiziert das Gerät als "WinUsb Device", s.d. automatisch der
 * WinUsb-Treiber geladen wird, welcher Anwendungen direkten Zugriff auf das Gerät ermöglicht, z.B. via libusb.
 * Dies hat keine Auswirkungen unter anderen Betriebssysstemen.
 */
static constexpr auto compatIdDescriptor = EncodeDescriptors::MS_OS_Desc::compatId (
			0x100,					// bcdVersion
			4,						// wIndex
			EncodeDescriptors::MS_OS_Desc::compatIdFunction (
				0,									// bFirstInterfaceNumber
				Util::encodeString ("WINUSB\0\0"),	// compatibleID
				std::array<Util::EncChar, 8> {}				// subCompatibleID
			)
);

// Diese Strings identifizieren das Gerät für den Benutzer. Sie müssen als UTF-16 kodiert werden (kleines u)

static constexpr auto strManufacturer = EncodeDescriptors::USB20::string (u"ACME Corp.");
static constexpr auto strProduct = EncodeDescriptors::USB20::string (u"Fluxkompensator");
static constexpr auto strSerial = EncodeDescriptors::USB20::string (u"42-1337-47/11");
static constexpr auto strFunction = EncodeDescriptors::USB20::string (u"STM32 Serial Port");

/**
 * Der Microsoft OS String Deskriptor ist erforderlich, damit der Compat Id Deskriptor überhaupt abgefragt wird.
 * Er muss den Index 0xEE haben. Das Byte mit Wert 3 ist beliebig, und wird vom PC für bRequest genutzt um
 * den compatIdDescriptor abzufragen. Da 1 und 2 schon zur LED-Steuerung belegt sind, wird hier 3 verwendet.
 */
static constexpr auto strOsStringDesc = EncodeDescriptors::USB20::string(u"MSFT100\u0003");

/// Da die Strings Deutsch sind, wird hier nur Deutsch als Sprache angegeben.
static constexpr auto langTable = EncodeDescriptors::USB20::languageTable (0x0407 /* German (Standard) */);

/**
 * Alle oben angegebenen Deskriptoren müssen in dieser Tabelle eingetragen werden, damit sie zur Laufzeit
 * gefunden werden können. Die Länge des Deskriptors wird im Konstruktor von "Descriptor" inferiert.
 */
static constexpr Descriptor descriptors [] = { { deviceDescriptor, D_TYPE::DEVICE, 0 },
									{ confDescriptor, D_TYPE::CONFIGURATION, 0 },
									{ langTable, D_TYPE::STRING, 0 },
									{ strManufacturer, D_TYPE::STRING, 1 },
									{ strProduct, D_TYPE::STRING, 2 },
									{ strSerial, D_TYPE::STRING, 3 },
									{ strFunction, D_TYPE::STRING, 4 },
									{ strOsStringDesc, D_TYPE::STRING, 0xEE },
									{ compatIdDescriptor, D_TYPE::OS_DESCRIPTOR, 0 }
};

/**
 * Diese Funktion sucht in der Tabelle nach einem Deskriptor vom angegebenen Typ und Index. Wird keiner
 * gefunden, wird nullptr zurückgegeben, ansonsten ein Pointer auf die entsprechende "Descriptor"-Instanz.
 */
const Descriptor* getUsbDescriptor (D_TYPE type, uint8_t index) {
	// Durchsuche Deskriptor-Tabelle
	for (auto& d : descriptors) {
		if (d.type == type && d.index == index)
			return &d;
	}
	return nullptr;
}
