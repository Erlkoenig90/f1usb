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
 * Diese Datei definiert Hilfsfunktionen, welche eine einfache Erstellung von USB-Deskriptoren ermöglichen. Die hier
 * definierten Funktionen können mit den gewünschten Werten aufgerufen werden, und das Ergebnis an
 * globale "static constexpr" Variablen zugewiesen werden, sodass es als Read-Only-Daten im Flash landet. Die Funktionen
 * fügen automatisch Länge und Typ hinzu, sodass deren Angabe beim Aufruf nicht nötig ist. Bei den Deskriptoren zur
 * Konfiguration und zur Microsoft Compatibility ID folgen im Speicher unmittelbar darauf eine variable Anzahl weiterer
 * Deskriptoren - diese werden an die jeweiligen Funktionen "configuration" bzw. "compatId" als Argumente variabler
 * Anzahl nach den einfachen Zahlen-Argumenten übergeben, nachdem sie zuvor über die jeweilige Funktion erzeugt wurden.
 * Die Gesamtlänge wird jeweils automatisch berechnet.
 *
 * Für die genauen Bedeutungen der einzelnen Werte siehe die USB Spezifikation (usb_20.pdf, S. 260ff.).
 */

#ifndef USB_DESC_HELPER_HH_
#define USB_DESC_HELPER_HH_

#include "encode.hh"

/**
 * Dieses enum definiert die einzelnen Typen von Deskriptoren. Der Zahlwert wird per USB-Anfrage gesendet, um bestimmte
 * Deskriptoren abzufragen. Der Typ OS_DESCRIPTOR wird nicht von der Spezifikation vorgegeben und wird auch nicht per USB
 * übertragen, sondern hier nur genutzt um den Microsoft OS-String-Deskriptor zu identifizieren und wiederzufinden.
 */
enum class D_TYPE : uint8_t {	DEVICE = 1, CONFIGURATION = 2, STRING = 3, INTERFACE = 4, ENDPOINT = 5, DEVICE_QUALIFIER = 6,
								OTHER_SPEED_CONFIGURATION = 7, INTERFACE_POWER = 8, OS_DESCRIPTOR = 100 };

namespace EncodeDescriptors {

	namespace USB20 {

		/// Der Device Deskriptor beschreibt die allgemeinen Eigenschaften des Geräts.
		usb_always_inline constexpr std::array<Util::EncChar, 18> device (uint16_t bcdUSB, uint8_t bDeviceClass, uint8_t bDeviceSubClass, uint8_t bDeviceProtocol, uint8_t bMaxPacketSize0,
				uint16_t idVendor, uint16_t idProduct, uint16_t bcdDevice, uint8_t iManufacturer, uint8_t iProduct,	uint8_t iSerialNumber, uint8_t bNumConfigurations) {
			return Util::encodeMulti (
					uint8_t { 18 },
					D_TYPE::DEVICE,
					bcdUSB,
					bDeviceClass,
					bDeviceSubClass,
					bDeviceProtocol,
					bMaxPacketSize0,
					idVendor,
					idProduct,
					bcdDevice,
					iManufacturer,
					iProduct,
					iSerialNumber,
					bNumConfigurations);

		}

		/**
		 * String-Deskriptoren werden teilweise bei Host-Geräten angezeigt und auch für den Microsoft OS String genutzt.
		 * Der Parameter sollte ein UTF-16 String Literal sein, was durch Angabe eines kleinen(!) "u" vor dem Literal erreicht
		 * wird, z.B. u"MeinGerät". Daher sind auch beliebige Sonderzeichen erlaubt. Das terminierende 0-Zeichen wird abgeschnitten.
		 */
		template <typename T, size_t N>
		usb_always_inline constexpr std::array<Util::EncChar, 2 + sizeof(T)*(N-1)> string (const T (&str) [N]) {
			return Util::encodeMulti (
					static_cast<uint8_t> (2 + sizeof(T)*(N-1)),
					D_TYPE::STRING,
					Util::encodeString (str)
			);
		}

		/**
		 * Die Sprachtabelle gibt eine Liste der vom Gerät unterstützten Sprachen an. Der Host kann bei Strings die gewünschte
		 * Sprache anfordern. Die Parameter sollten eine Folge von 16bit-Zahlen sein, welche den Sprach-IDs entsprechen
		 * (siehe http://www.usb.org/developers/docs/USB_LANGIDs.pdf ).
		 */
		template <typename... T>
		usb_always_inline constexpr std::array<Util::EncChar, 2 + 2*sizeof...(T)> languageTable (T... langs) {
			return Util::encodeMulti (
					static_cast<uint8_t> (2 + 2*sizeof...(T)),
					D_TYPE::STRING,
					static_cast<uint16_t> (langs)...
			);
		}

		/**
		 * Im Konfigurations-Deskriptor werden Informationen zur einer Betriebsart des Device gesammelt. Besonders interessant
		 * ist hier die Angabe der maximalen Leistungsaufnahme (in 2mA-Schritten). Nach den Integer-Parametern kann eine
		 * beliebig lange Folge an Interface- und Endpointdeskriptoren und solchen die Klassen/Geräte-spezifisch sind folgen.
		 */
		template <typename... T>
		usb_always_inline constexpr std::array<Util::EncChar, Util::staticSum<size_t> (9, Util::arrSize<T> ()...)> configuration (uint8_t bNumInterfaces, uint8_t bConfigurationValue, uint8_t iConfiguration, uint8_t bmAttributes, uint8_t bMaxPower, T&&... sub) {
			return Util::concatArrays<Util::EncChar> (
					Util::encodeMulti (
						uint8_t { 9 },
						D_TYPE::CONFIGURATION,
						static_cast<uint16_t> (Util::staticSum<size_t> (9, Util::arrSize<T> ()...)),
						bNumInterfaces,
						bConfigurationValue,
						iConfiguration,
						bmAttributes,
						bMaxPower
					),
					std::forward<T> (sub)...
				);
		}

		/// Informationen zu einem unterstützten Interface
		usb_always_inline constexpr std::array<Util::EncChar, 9> interface (uint8_t bInterfaceNumber, uint8_t bAlternateSetting, uint8_t bNumEndpoints, uint8_t bInterfaceClass, uint8_t bInterfaceSubClass,
				uint8_t bInterfaceProtocol, uint8_t iInterface) {
			return Util::encodeMulti (
					uint8_t { 9 },
					D_TYPE::INTERFACE,
					bInterfaceNumber,
					bAlternateSetting,
					bNumEndpoints,
					bInterfaceClass,
					bInterfaceSubClass,
					bInterfaceProtocol,
					iInterface
			);
		}

		/**
		 * Informationen zu einem unterstützten Endpoint. Hiermit müssen alle genutzten Endpoints außer Endpoint 0 definiert
		 * werden, wobei insbesondere die maximale Paketgröße und der Typ des Endpoints wichtig sind.
		 */
		usb_always_inline constexpr std::array<Util::EncChar, 7> endpoint (uint8_t bEndpointAddress, uint8_t bmAttributes, uint16_t wMaxPacketSize, uint8_t bInterval) {
			return Util::encodeMulti (
				uint8_t { 7 },
				D_TYPE::ENDPOINT,
				bEndpointAddress,
				bmAttributes,
				wMaxPacketSize,
				bInterval);
		}
	}

	namespace IAD {
		/**
		 * Gibt einen Interface Association Descriptor zurück, welcher dem OS mitteilt, welche Interfaces zusammen zu einer Funktion gehören. Dies ist für
		 * Composite Devices unter Windows nötig.
		 */
		usb_always_inline constexpr std::array<Util::EncChar, 8> interfaceAssociation (uint8_t bFirstInterface, uint8_t bInterfaceCount, uint8_t bFunctionClass, uint8_t bFunctionSubClass, uint8_t bFunctionProtocol, uint8_t iFunction) {
			return Util::encodeMulti (
				uint8_t { 0x08 },			// bLength
				uint8_t { 0x0B },			// bDescriptorType
				bFirstInterface,
				bInterfaceCount,
				bFunctionClass,
				bFunctionSubClass,
				bFunctionProtocol,
				iFunction
			);
		}
	}

	namespace MS_OS_Desc {
		/**
		 * Der Microsoft Compat ID Deskriptor definiert eine Folge an Funktionen. Nach den Integer-Parametern kann eine beliebig
		 * lange Folge an Compat ID Function Deskriptoren folgen, d.h. aus Rückgabewerten von compatIdFunction.
		 */
		template <typename... T>
		usb_always_inline constexpr std::array<Util::EncChar, Util::staticSum<size_t> (16, Util::arrSize<T> ()...)> compatId (uint16_t bcdVersion, uint16_t wIndex, T&&... functions) {
			return Util::concatArrays<Util::EncChar> (
					Util::encodeMulti (
						static_cast<uint32_t> (Util::staticSum<size_t> (16, Util::arrSize<T> ()...)),
						bcdVersion,
						wIndex,
						static_cast<uint8_t> (sizeof...(functions))
					),
					std::array<Util::EncChar, 7> {{}},
					std::forward<T> (functions)...
				);
		}

		/**
		 * Definiert eine Microsoft Compat ID Funktion, z.B. zur Identifikation eines Geräts als WinUSB-Device. Die Parameter compatibleID
		 * und subcompatibleID sind einfache ASCII-Strings, die ggf. mit Nullen aufgefüllt werden müssen. Sie können über Util::encodeString
		 * erzeugt werden, z.B. Util::encodeString ("WINUSB\0\0").
		 */
		usb_always_inline constexpr std::array<Util::EncChar, 24> compatIdFunction (uint8_t bFirstInterfaceNumber, std::array<Util::EncChar, 8> compatibleID, std::array<Util::EncChar, 8> subCompatibleID) {
			return Util::concatArrays<Util::EncChar> (
					Util::encode (bFirstInterfaceNumber),
					std::array<Util::EncChar, 1> {{1}},
					compatibleID,
					subCompatibleID,
					std::array<Util::EncChar, 6> {{}}
					);
		}
	}

	namespace CDC {
		/**
		 * Baut CDC-Spezifische Deskriptoren zu einem großen Deskriptor zusammen, welcher an einen Konfigurations-Deskriptor
		 * übergeben werden kann. Nach dem "bcdCDC"-Parameter kann eine beliebig lange Folge an CDC-Deskriptoren folgen.
		 */
		template <typename... T>
		usb_always_inline constexpr std::array<Util::EncChar, Util::staticSum<size_t> (5, Util::arrSize<T> ()...)> classSpecific (uint16_t bcdCDC, T&&... sub) {
			return Util::concatArrays<Util::EncChar> (
					Util::encodeMulti (
						uint8_t { 5 },
						uint8_t { 0x24 }, // CS_INTERFACE
						uint8_t { 0 }, // Header
						bcdCDC
					),
					std::forward<T> (sub)...
				);
		}
		/**
		 * Definiert einen "Union Interface" Deskriptor für CDC. bControlInterface gibt das Master-Interface an, darauf dürfen
		 * beliebig viele Indices von Sub-Interfaces folgen.
		 */
		template <typename... T>
		usb_always_inline constexpr std::array<Util::EncChar, 4+sizeof...(T)> unionInterface (uint8_t bControlInterface, T&&... bSubordinateInterfaces) {
			return Util::encodeMulti (
					static_cast<uint8_t> (4+sizeof...(T)),
					uint8_t { 0x24 }, // CS_INTERFACE
					uint8_t { 6 }, // Union Functional Descriptor
					bControlInterface,
					static_cast<uint8_t> (bSubordinateInterfaces)...
					);
		}
		/**
		 * Definiert einen Deskriptor für CDC Call Management.
		 */
		usb_always_inline constexpr std::array<Util::EncChar, 5> callManagement (uint8_t bmCapabilities, uint8_t bDataInterface) {
			return Util::encodeMulti (
					uint8_t { 5 },
					uint8_t { 0x24 }, // CS_INTERFACE
					uint8_t { 1 }, // Call Management Subtype
					bmCapabilities,
					bDataInterface
					);
		}
		/**
		 * Definiert einen Deskriptor für das CDC Abstract Control Model.
		 */
		usb_always_inline constexpr std::array<Util::EncChar, 4> ACM (uint8_t bmCapabilities) {
			return Util::encodeMulti (
					uint8_t { 4 },
					uint8_t { 0x24 }, // CS_INTERFACE
					uint8_t { 2 }, // ACM Subtype
					bmCapabilities);
		}

	}
}

#endif /* USB_DESC_HELPER_HH_ */
