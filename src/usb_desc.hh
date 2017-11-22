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

#ifndef USB_DESC_HH_
#define USB_DESC_HH_

#include <cstddef>
#include <array>
#include <cstdint>
#include "usb_desc_helper.hh"

/**
 * Diese einfache Klasse sammelt Informationen zu einem Deskriptor. An den Konstruktor wird eine Referenz auf die Konstante
 * des Deskriptors übergeben und dadurch Adresse & Größe ermittelt.
 */
struct Descriptor {
	template <size_t N>
	constexpr Descriptor (const std::array<Util::EncChar, N>& data_, D_TYPE type_, uint8_t index_) : data (& data_[0]), length (N), type (type_), index (index_) {}

	const Util::EncChar* data;
	uint8_t length;
	D_TYPE type;
	uint8_t index;
};
const Descriptor* getUsbDescriptor (D_TYPE type, uint8_t index);

#endif /* USB_DESC_HH_ */
