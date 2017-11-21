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
 * Hier wird ein einfacher Doppelpuffer implementiert, welcher die Transfers per USB & UART synchronisiert.
 */

#ifndef DOUBLEBUFFER_HH_
#define DOUBLEBUFFER_HH_

#include <cstdint>
#include <cstddef>

/**
 * Implementiert einen einfachen Doppelpuffer. Es können immer ganze Datenblöcke gelesen bzw. geschrieben werden.
 * Dem Konstruktor müssen Zeiger und Größe des tatsächlichen Speichers übergeben werden.
 * Der Puffer kann per DMA gelesen/geschrieben werden.
 *
 * Zum Schreiben sollten die Folge der Aufrufe sein:
 * - writeAvailable()
 * - write()
 * - ggf. Warten (DMA)
 * - writeFinish()
 *
 * Zum Lesen sollten die Folge der Aufrufe sein:
 * - readAvailable()
 * - read()
 * - ggf. Warten (DMA)
 * - readFinish()
 */
class DoubleBuffer {
	public:
		constexpr DoubleBuffer (uint8_t* buffer, size_t size) : m_buffer (buffer), m_size (size / 2), m_inFill (0), m_outFill (0), m_outPtr (0), m_writeIndex (0), m_writing (false) {}

		uint8_t* write ();
		size_t writeAvailable () const;
		void writeFinish (size_t length);

		uint8_t* read ();
		size_t readAvailable () const;
		void readFinish (size_t length);

		void reset ();
		void swap ();
	private:
		size_t singleSize () const { return m_size; }
		uint8_t* buffer (uint8_t index) const { return m_buffer + (index * m_size); }

		/**
		 * Zeiger auf externe Puffer-Instanz. Indem der Puffer nicht direkt als Member-Variable angelegt wird,
		 * kann die DoubleBuffer-Instanz direkt per "constexpr" initialisiert werden, und der Puffer per "bss"-Section.
		 */
		uint8_t* const m_buffer;
		/// Speichert die Größe eines einzelnen der beiden Puffer
		const size_t m_size;

		/// Wie viele Bytes im Eingabepuffer beschrieben sind
		size_t m_inFill;
		/// Wie viele Bytes im Ausgabepuffer beschrieben sind
		size_t m_outFill;
		/// Wie viele Bytes im Ausgabepuffer gelesen wurden (<= m_outFill)
		size_t m_outPtr;
		/// 1 oder 0 - gibt an, welcher Puffer gerade beschrieben wird. Der Puffer 1-m_writeIndex wird gelesen.
		uint8_t m_writeIndex;
		/**
		 * Ob gerade eine Schreiboperation im Gange ist; verhindert das Tauschen des Puffers. Zum Lesen ist kein
		 * solches Flag nötig, weil die Puffer nur getauscht werden können wenn alles gelesen wurde und somit kein
		 * Lesevorgang im Gange ist.
		 */
		bool m_writing;
};


#endif /* DOUBLEBUFFER_HH_ */
