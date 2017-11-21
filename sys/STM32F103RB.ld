/* Grundkonfiguration */

ENTRY(Reset_Handler)

/* Speicherbereiche des Controllers */
MEMORY {
	FLASH		: ORIGIN = 0x8000000,	LENGTH = 128K
	SRAM		: ORIGIN = 0x20000000,	LENGTH =  20K
	USBBUF		: ORIGIN = 0x40006000,	LENGTH = 1024
}

/* Initialer Stack-Pointer (vom Starupcode geladen) */
_estack = 0x20005000;
/* Gewünschte Heap-Größe, für syscalls.c */
_HeapSize = 0;
/* Gewünschte Stack-Größe */
_StackSize = 0x400;

SECTIONS {
	/* Der eigentliche Programmcode & Konstanten */
	.Text : {
		/* Der Interrupt-Vektor muss zuerst in den Programmspeicher */
		KEEP (*(.isr_vector))
		
		. = ALIGN(4);
		*(.text)			/* Programmcode */
		*(.text*)
		
		. = ALIGN(4);
		
		*(.rodata)			/* Konstanten */
		*(.rodata*)
	} >FLASH
	.EhFrame : {
		/* Für Exceptions */
		PROVIDE (__eh_frame_start = .);
		*(.eh_frame)
	} >FLASH

	/* Für Exceptions */
	.ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
	.ARM.exidx : ALIGN_WITH_INPUT {
		__exidx_start = .;
		*(.ARM.exidx*)
		__exidx_end = .;
	} >FLASH
	
	.Init : {
		. = ALIGN(4);
		
		KEEP (*(.init))
		
		. = ALIGN(4);
	} > FLASH
	.Fini : {
		. = ALIGN(4);
		
		KEEP (*(.fini))
		
		. = ALIGN(4);
	} > FLASH
	.PreInit : {
		. = ALIGN(4);
		
		__preinit_array_start = .;
		KEEP (*(.preinit_array*))
		__preinit_array_end = .;
		
		. = ALIGN(4);
	} > FLASH
	.InitArray : {
		. = ALIGN(4);
		
		__init_array_start = .;
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array*))
		__init_array_end = .;
		
		. = ALIGN(4);
	} > FLASH
		
	/* Variablen für den SRAM */
	.SramData : {
		. = ALIGN(4);
		_sdata1 = .;	/* Definiere Symbole für Adressen im RAM */
		
		*(.data)
		*(.data*)
		
		. = ALIGN(4);
		_edata1 = .;
	} >SRAM AT> FLASH
	/* Adresse der Initial-Werte der Variablen im Flash */
	_sidata1 = LOADADDR (.SramData);
    
    /* Mit 0 zu initialisierende Variablen im SRAM */
	.SramBss (NOLOAD) : {
		. = ALIGN(4);
		_sbss1 = .;
		
		*(.bss)
		*(.bss*)
		*(COMMON)
		
		. = ALIGN(4);
		_ebss1 = .;
	} >SRAM

	/* Bereich für Daten deren Größe erst zur Laufzeit bekannt ist - Stack und Heap */
	.DynData : {
		. = ALIGN(4);
		/* Für syscalls.c */
		_BeginHeap = .;
		
		/* Reserviere Platz. Falls das nicht passt, zeigt der Linker eine Fehlermeldung */
		. = . + _HeapSize;
		. = . + _StackSize;
		
		. = ALIGN(4);
	} >SRAM
	
	.UsbBuffer (NOLOAD) : {
		UsbBufBegin = .;
		*(.usbbuf)
		*(.usbbuf*)
	} > USBBUF
}
