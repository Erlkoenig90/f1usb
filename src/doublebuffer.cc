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

#include <algorithm>
#include "doublebuffer.hh"

/**
 * Bereitet den Schreibvorgang vor. Gibt einen Zeiger auf beschreibbare Daten zurück. Die Anzahl beschreibbarer Bytes
 * kann per writeAvailable() abgefragt werden.
 */
uint8_t* DoubleBuffer::write () {
	if (m_inFill == singleSize ())
		// Kein Platz zum schreiben
		return nullptr;
	else {
		// Berechne Zeiger
		m_writing = true;
		return buffer (m_writeIndex) + m_inFill;
	}
}

/// Gibt die Anzahl derzeit beschreibbarer Bytes zurück.
size_t DoubleBuffer::writeAvailable () const {
	return singleSize () - m_inFill;
}

/**
 * Schließt einen Schreibvorgang ab. Falls möglich und Eingabepuffer voll, werden die Puffer getauscht.
 * length muss die Anzahl geschriebener Bytes angeben und <= writeAvailable() sein.
 */
void DoubleBuffer::writeFinish (size_t length) {
	m_writing = false;
	m_inFill += length;
	if (m_inFill == singleSize ())
		swap ();
}

/**
 * Bereitet den Lesevorgang vor. Gibt einen Zeiger auf lesbare Daten zurück. Die Anzahl lesbarer Bytes
 * kann per readAvailable() abgefragt werden.
 */
uint8_t* DoubleBuffer::read () {
	if (m_outPtr == m_outFill)
		// Nichts mehr zum Lesen da
		return nullptr;
	else
		// Berechne Zeiger
		return buffer (static_cast<uint8_t> (1-m_writeIndex)) + m_outPtr;
}

/// Gibt die Anzahl derzeit lesbarer Bytes zurück.
size_t DoubleBuffer::readAvailable () const {
	return m_outFill - m_outPtr;
}

/**
 * Schließt einen Lesvorgang ab. Falls möglich und Eingabepuffer voll, werden die Puffer getauscht.
 * length muss die Anzahl lesbarer Bytes angeben und <= readAvailable() sein.
 */
void DoubleBuffer::readFinish (size_t length) {
	m_outPtr += length;
	if (m_outPtr == m_outFill)
		swap ();
}

/// Leert beide Puffer auf setzt sie auf Ausgangszustand zurück.
void DoubleBuffer::reset () {
	m_inFill = 0;
	m_outFill = 0;
	m_outPtr = 0;
	m_writeIndex = 0;
	m_writing = false;
}

/**
 * Tauscht die Puffer, falls: Ausgabepuffer leer, es wird gerade nicht geschrieben, Eingabepuffer nicht leer.
 * Dies wird beim Schreiben/Lesen automatisch aufgerufen, falls Eingabepuffer voll bzw. Ausgabepuffer leer.
 */
void DoubleBuffer::swap () {
	if (m_outFill == m_outPtr) {
		if (!m_writing && m_inFill > 0) {
			m_outFill = m_inFill;
			m_outPtr = 0;
			m_inFill = 0;
			m_writeIndex = static_cast<uint8_t> (1 - m_writeIndex);
		} /* else {	// Nicht nötig. Setzt falls möglich den Puffer auf Ausgangszustand zurück
			m_outFill = 0;
			m_outPtr = 0;
		} */
	}
}
