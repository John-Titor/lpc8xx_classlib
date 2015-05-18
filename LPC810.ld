/* NXP LPC810 */
MEMORY
{
        FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x00001000 /* 4 Kbytes */
        RAM (rwx) :  ORIGIN = 0x10000000, LENGTH = 0x00000400 /* 1 Kbyte */
}

ENTRY(_vectors)

SECTIONS
{
        .text :
        {
                _text = .;
                KEEP(*(.vectors))
                *(.text*)
                *(.rodata*)
                . = ALIGN(4);
                _etext = .;
        } > FLASH
        
        .init_section : {
                _sinit = ABSOLUTE(.);
                KEEP(*(.init_array .init_array.*))
                _einit = ABSOLUTE(.);
        } > FLASH

        .data : AT (_einit)
        {        
                _data = .;
                *(vtable)
                *(.data*)
                _edata = .;
        } > RAM

        .bss :
        {
                _bss = .;
                *(.bss*)
                *(COMMON)
                _ebss = .;
        } > RAM

        _estack = ORIGIN(RAM) + LENGTH(RAM);
}

