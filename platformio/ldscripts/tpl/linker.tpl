/*
 *
 * Hysonglet
 * PlatformIO default linker script template for synwit
 *
 */


/* Entry Point */
ENTRY(Reset_Handler)

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Stack_Size = $stack;    /* required amount of stack */
_Min_Heap_Size = 0x200;      /* required amount of heap  */


/* Specify the memory areas */
MEMORY
{
	FLASH (arx) : ORIGIN = 0x00000000,    LENGTH = $rom
    RAM   (arw) : ORIGIN = 0x20000000,    LENGTH = $ram
}

/* Define output sections */
SECTIONS
{
	. = ORIGIN(FLASH);
    .text :
    {
		KEEP(*(.isr_vector))

		. = ALIGN(4);

        *(.text)
        *(.text*)
		*(.glue_7)         /* glue arm to thumb code */
		*(.glue_7t)        /* glue thumb to arm code */
		*(.eh_frame)

		KEEP (*(.init))
		KEEP (*(.fini))

		. = ALIGN(4);

        *(.rodata*)
    } > FLASH

	.preinit_array :
	{
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array*))
		PROVIDE_HIDDEN (__preinit_array_end = .);
	} >FLASH
	.init_array :
	{
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array*))
		PROVIDE_HIDDEN (__init_array_end = .);
	} >FLASH
	.fini_array :
	{
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP (*(SORT(.fini_array.*)))
		KEEP (*(.fini_array*))
		PROVIDE_HIDDEN (__fini_array_end = .);
	} >FLASH

	. = ALIGN(4);
	__data_load__ = LOADADDR(.data);
	.data :
	{
		__data_start__ = .;

		*(.data)
		*(.data*)

		. = ALIGN(4);
		__data_end__ = .;
	} > RAM AT> FLASH

	. = ALIGN(4);
	.bss :
	{
		__bss_start__ = .;

        *(.bss)
        *(.bss*)
        *(COMMON)

        . = ALIGN(4);
        __bss_end__ = .;
	} > RAM
	. = ALIGN(4);

	/* User_heap_stack section, used to check that there is enough RAM left */
	._user_heap_stack :
	{
		. = ALIGN(4);
		PROVIDE ( end = . );
		PROVIDE ( _end = . );
		. = . + _Min_Heap_Size;
		. = . + _Min_Stack_Size;
		. = ALIGN(4);
	} >RAM

	__StackTop   = ORIGIN(RAM) + LENGTH(RAM);
}
