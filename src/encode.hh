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
 * In dieser Datei werden Hilfsfunktionen definiert, die von den Funktionen aus usb_desc_helper.hh aufgerufen werden.
 * Diese ermöglichen es, Integer-Daten in std::array<EncChar,...>-Instanzen zu kodieren, und beliebig viele std::array
 * -Instanzen zu konkatenieren, woraus letztlich die Deskriptoren aufgebaut werden. EncChar ist dabei ein Alias für uint8_t.
 * Diese gesamte Verarbeitung geschieht "constexpr", d.h. vom Compiler, sodass das Ergebnis als Konstante im Flash
 * abgelegt werden kann. Werden die Funktionen aus usb_desc_helper.hh nicht genutzt, kann diese Datei gelöscht werden.
 */

#ifndef ENCODE_HH_
#define ENCODE_HH_

#include <utility>
#include <array>
#include <type_traits>
#include <cstddef>
#include "util.hh"

namespace Util {
	template <typename T>
	using Plain = std::remove_cv_t<std::remove_reference_t<T>>;

	namespace Helper {
		template <typename T>
		struct ArrSize;

		template <typename T, size_t N>
		struct ArrSize<std::array<T, N>> {
			static constexpr size_t value = N;
		};

		template <typename T, size_t N>
		struct ArrSize<T [N]> {
			static constexpr size_t value = N;
		};
	}

	/// Gibt die Anzahl Element des per "T" spezifizierten Array-Typs an (std::array oder normales Array).
	template <typename T>
	usb_always_inline constexpr size_t arrSize () { return Helper::ArrSize<Plain<T>>::value; }

	namespace Helper {
		template <typename T>
		struct CRef2 {
			using Type = const T&;
		};
		template <typename T>
		struct CRef2<const T> {
			using Type = const T&;
		};
		template <typename T>
		struct CRef2<T&> {
			using Type = const T&;
		};
		template <typename T>
		struct CRef2<const T&> {
			using Type = const T&;
		};
		template <typename T>
		struct CRef2<T&&> {
			using Type = const T&;
		};
		template <typename T>
		using CRef = typename CRef2<T>::Type;

		template <typename T, size_t Length, typename... Seqs>
		struct ConcatArray;

		template <typename T, size_t Length, size_t... Seq1>
		struct ConcatArray<T, Length, std::index_sequence<Seq1...>> {
			template <typename Arr1>
			usb_always_inline static constexpr std::array<T, Length> get (Arr1&& arr1) {
				CRef<Arr1> a1 = arr1;
				return {{ a1 [Seq1]... }};
			}
		};

		template <typename T, size_t Length, size_t... Seq1, size_t... Seq2>
		struct ConcatArray<T, Length, std::index_sequence<Seq1...>, std::index_sequence<Seq2...>> {
			template <typename Arr1, typename Arr2>
			usb_always_inline static constexpr std::array<T, Length> get (Arr1&& arr1, Arr2&& arr2) {
				CRef<Arr1> a1 = arr1; CRef<Arr2> a2 = arr2;
				return {{ a1 [Seq1]..., a2 [Seq2]... }};
			}
		};

		template <typename T, size_t Length, size_t... Seq1, size_t... Seq2, size_t... Seq3>
		struct ConcatArray<T, Length, std::index_sequence<Seq1...>, std::index_sequence<Seq2...>, std::index_sequence<Seq3...>> {
			template <typename Arr1, typename Arr2, typename Arr3>
			usb_always_inline static constexpr std::array<T, Length> get (Arr1&& arr1, Arr2&& arr2, Arr3&& arr3) {
				CRef<Arr1> a1 = arr1; CRef<Arr2> a2 = arr2; CRef<Arr3> a3 = arr3;
				return {{ a1 [Seq1]..., a2 [Seq2]..., a3 [Seq3]... }};
			}
		};

		template <typename T, size_t Length, size_t... Seq1, size_t... Seq2, size_t... Seq3, size_t... Seq4>
		struct ConcatArray<T, Length, std::index_sequence<Seq1...>, std::index_sequence<Seq2...>, std::index_sequence<Seq3...>, std::index_sequence<Seq4...>> {
			template <typename Arr1, typename Arr2, typename Arr3, typename Arr4, typename... RestArr>
			usb_always_inline static constexpr std::array<T, Length> get (Arr1&& arr1, Arr2&& arr2, Arr3&& arr3, Arr4&& arr4) {
				CRef<Arr1> a1 = arr1; CRef<Arr2> a2 = arr2; CRef<Arr3> a3 = arr3; CRef<Arr4> a4 = arr4;

				return {{ a1 [Seq1]..., a2 [Seq2]..., a3 [Seq3]..., a4 [Seq4]... }};
			}
		};

		template <typename T, size_t Length, size_t... Seq1, size_t... Seq2, size_t... Seq3, size_t... Seq4, typename... RestSeq>
		struct ConcatArray<T, Length, std::index_sequence<Seq1...>, std::index_sequence<Seq2...>, std::index_sequence<Seq3...>, std::index_sequence<Seq4...>, RestSeq...> {
			static constexpr size_t size = sizeof...(Seq1) + sizeof...(Seq2) + sizeof...(Seq3) + sizeof...(Seq4);

			template <typename Arr1, typename Arr2, typename Arr3, typename Arr4, typename... RestArr>
			usb_always_inline static constexpr std::array<T, Length> get (Arr1&& arr1, Arr2&& arr2, Arr3&& arr3, Arr4&& arr4, RestArr&&... rest) {
				CRef<Arr1> a1 = arr1; CRef<Arr2> a2 = arr2; CRef<Arr3> a3 = arr3; CRef<Arr4> a4 = arr4;
				return ConcatArray<T, Length, std::make_index_sequence<size>, RestSeq...>
					::get (std::array<T, size> {{ a1 [Seq1]..., a2 [Seq2]..., a3 [Seq3]..., a4 [Seq4]... }}, std::forward<RestArr> (rest)...);
			}

		};
	}

	using EncChar = uint8_t;

	/**
	 * Konkateniert die per Parameter angegebene Folge von Arrays zu einem Array. In "T" muss der Element-Typ angegeben werden.
	 * Es wird ein 4x-Fold angewendet, um immer jeweils 4 Arrays auf einmal zu konkatenieren.
	 */
	template <typename T, typename... Arrays>
	usb_always_inline constexpr std::array<T, (arrSize<Arrays> () + ...)> concatArrays (Arrays&&... arrays) {
		return Helper::ConcatArray<T, (arrSize<Arrays> () + ...), std::make_index_sequence<arrSize<Arrays> ()>...>::get (std::forward<Arrays> (arrays)...);
	}

	namespace Helper {
		template <typename T, size_t... I>
		usb_always_inline constexpr std::array<EncChar, sizeof(T)> encodeUInt (T&& val, std::index_sequence<I...>) {
			return {{ static_cast<EncChar> ((val >> (8 * I)) & 0xFF) ... }};
		}

		template <typename T, size_t N, size_t... I>
		usb_always_inline constexpr std::array<EncChar, sizeof...(I)> encodeString (const T (&string) [N], std::index_sequence<I...>) {
			return {{ static_cast<EncChar> (static_cast<std::make_unsigned_t<T>> ( string [I/sizeof(T)] ) >> ((I%sizeof(T))*8))... }};
		}
	}

	/// Kodiert einen vorzeichenlosen Integer in ein std::array<EncChar,...>.
	template <typename T>
	usb_always_inline constexpr std::enable_if_t<std::is_unsigned<Plain<T>>::value, std::array<EncChar, sizeof(T)>> encode (T&& val) {
		return Helper::encodeUInt (std::forward<T> (val), std::make_index_sequence<sizeof(T)> {});
	}
	/// Kodiert einen vorzeichenbehafteten Integer in ein std::array<EncChar,...>.
	template <typename T>
	usb_always_inline constexpr std::enable_if_t<std::is_signed<Plain<T>>::value, std::array<EncChar, sizeof(T)>> encode (T&& val) {
		return Helper::encodeUInt (static_cast<std::make_unsigned_t<Plain<T>>> (val), std::make_index_sequence<sizeof(T)> {});
	}
	/// Kodiert einen enum-Wert in ein std::array<EncChar,...>, in dem der zugrundeliegende Typ genutzt wird.
	template <typename T>
	usb_always_inline constexpr decltype(encode(std::declval<std::underlying_type_t<std::enable_if_t<std::is_enum<Plain<T>>::value, Plain<T>>>>())) encode (T&& val) {
		return encode (static_cast<std::underlying_type_t<Plain<T>>> (std::forward<T> (val)));
	}

	/**
	 * Wandelt ein String-Literal (char, charXX_t) in ein std::array<EncChar,...> um, wobei das terminierende 0-Zeichen
	 * abgeschnitten wird. char16_t/char32_t -Strings (typ. UTF-16/32) werden als little endian kodiert.
	 */
	template <typename T, size_t N>
	usb_always_inline constexpr std::array<EncChar, sizeof(T)*(N-1)> encodeString (const T (&string) [N]) {
		return Helper::encodeString (string, std::make_index_sequence<sizeof(T)*(N-1)> {});
	}

	namespace Helper {
		template <typename T, size_t N, typename ArrT, size_t... I>
		usb_always_inline constexpr std::array<EncChar, N * arrSize<decltype(encode (std::declval<T> ()))> ()> encodeArray (ArrT&& arr, std::index_sequence<I...>) {
			return concatArrays<EncChar> (encode (std::forward<ArrT> (arr) [I])...);
		}
	}
	/**
	 * Wandelt ein Array beliebigen Typs in ein std::array<EncChar,...> um, wobei die Elemente einzeln per entsprechender
	 * encode()-Funktion umgewandelt werden.
	 */
	template <typename T, size_t N>
	usb_always_inline constexpr std::array<EncChar, N * arrSize<decltype(encode (std::declval<T> ()))> ()> encode (std::array<T, N>&& arr) {
		return Helper::encodeArray<T, N> (std::move (arr), std::make_index_sequence<N> {});
	}
	/**
	 * Wandelt ein Array beliebigen Typs in ein std::array<EncChar,...> um, wobei die Elemente einzeln per entsprechender
	 * encode()-Funktion umgewandelt werden.
	 */
	template <typename T, size_t N>
	usb_always_inline constexpr std::array<EncChar, N * arrSize<decltype(encode (std::declval<T> ()))> ()> encode (const std::array<T, N>& arr) {
		return Helper::encodeArray<T, N> (arr, std::make_index_sequence<N> {});
	}
	/**
	 * Wandelt ein Array beliebigen Typs in ein std::array<EncChar,...> um, wobei die Elemente einzeln per entsprechender
	 * encode()-Funktion umgewandelt werden.
	 */
	template <typename T, size_t N>
	usb_always_inline constexpr std::array<EncChar, N * arrSize<decltype(encode (std::declval<T> ()))> ()> encode (T (&&arr) [N]) {
		return Helper::encodeArray<T, N> (std::move (arr), std::make_index_sequence<N> {});
	}
	/**
	 * Wandelt ein Array beliebigen Typs in ein std::array<EncChar,...> um, wobei die Elemente einzeln per entsprechender
	 * encode()-Funktion umgewandelt werden.
	 */
	template <typename T, size_t N>
	usb_always_inline constexpr std::array<EncChar, N * arrSize<decltype(encode (std::declval<T> ()))> ()> encode (const T (&arr) [N]) {
		return Helper::encodeArray<T, N> (arr, std::make_index_sequence<N> {});
	}
	/// Gibt den Parameter direkt zurück.
	template <size_t N> usb_always_inline constexpr std::array<EncChar, N> encode (std::array<EncChar, N>&& arr) { return arr;	}
	/// Gibt den Parameter direkt zurück.
	template <size_t N> usb_always_inline constexpr std::array<EncChar, N> encode (const std::array<EncChar, N>& arr) { return arr; }

	/// Kodiert die einzelnen Parameter per entsprechender encode()-Funktion, und gibt die konkatenierten Ergebnisse zurück.
	template <typename... T>
	usb_always_inline constexpr std::array<EncChar, (arrSize<decltype(encode (std::declval<T> ()))> () + ...)> encodeMulti (T&&... args) {
		return concatArrays<EncChar> (encode (args)...);
	}
}


#endif /* ENCODE_HH_ */
