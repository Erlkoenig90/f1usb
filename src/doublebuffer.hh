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
		constexpr DoubleBuffer () : m_buffer {}, m_inFill (0), m_outFill (0), m_outPtr (0), m_writeIndex (0), m_writing (false) {}
		/// Gibt die Größe pro Puffer an
		static constexpr size_t size = 1024;

		uint8_t* write ();
		size_t writeAvailable () const;
		void writeFinish (size_t length);

		uint8_t* read ();
		size_t readAvailable () const;
		void readFinish (size_t length);

		void reset ();
		void swap ();
	private:
		/// Richte Puffer an 16-Byte-Grenze aus, damit DMA immer funktioniert
		alignas(16) uint8_t m_buffer [2][size];
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
