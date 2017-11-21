# f1usb
Dies ist ein einfaches Beispiel für die Entwicklung eines USB FullSpeed Device auf Basis des STM32F103 ohne die Verwendung externer Bibliotheken, nur durch Low-Level-Registerzugriffe. Es sind drei Varianten gezeigt:
* Der "master"-Branch zeigt ein (erweiterbares) Beispiel für eigene Requests auf dem Default Control Endpoint zum Setzen und Abfragen von LED's, und einem simplen Loopback auf dem Bulk Endpoint 1. Das Device ist als "WinUSB Device" deklariert und braucht somit ab Windows 8 keine Treiber, es kann bspw. mit [libusb](http://libusb.info) direkt darauf zugegriffen werden. Siehe dazu auch das Projekt [usbclient](https://github.com/Erlkoenig90/usbclient).
* Im "minimal"-Branch ist eine gekürzte Variante mit gleicher Funktion.
* Der "vcp"-Branch implementiert einen 3-fachen virtuellen COM-Port ("VCP") (auch als USB-Serial/RS232-Adapter bekannt) auf Basis der Standard-Klasse CDC-ACM, die direkt ohne Treiber genutzt werden kann.

Der Code ist für den [STM32F103RBT6](http://www.st.com/en/microcontrollers/stm32f103rb.html) eingestellt. Als Pinout wird das des [Olimexino-STM32](https://www.olimex.com/Products/Duino/STM32/OLIMEXINO-STM32/open-source-hardware) angenommen (insb. mit PC12 zum Einschalten des 1,5kΩ-Widerstands auf der D+ -Leitung). Beides kann aber leicht angepasst werden.

Im [Code](src/usb_desc.cc) sind die fiktiven VID und PID 0xDEAD bzw. 0xBEEF eingestellt; dies sollte auf korrekte Werte angepasst werden.

Das Projekt ist als [eclipse-cdt](https://www.eclipse.org/cdt/) Projekt unter Nutzung des [GNU MCU Eclipse-Plugins](https://gnu-mcu-eclipse.github.io/) eingerichtet. Es funktioniert mit der GCC-Distribution [GNU Arm Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm) in der Version 6.3.1 (Juni 2017). 

Auf der [Releases](https://github.com/Erlkoenig90/f1usb/releases)-Seite sind fertige Binaries für den direkten Download auf den Controller verfügbar.

# Lizenz
Dieser Code steht unter der BSD-Lizenz, siehe dazu die Datei [LICENSE](LICENSE).