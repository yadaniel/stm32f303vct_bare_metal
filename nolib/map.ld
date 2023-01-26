ENTRY(ResetHandler)

MEMORY
{

    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 48K

    /* FLASH (rx) : ORIGIN = 0x0400, LENGTH = 1K */
    /* SRAM (rwx) : ORIGIN = 0x1000, LENGTH = 1K */

}

/* VMA = virtual memory address => default and used for resolving symbols when linking */
/* LMA = load memory address */

SECTIONS
{
    .text : {
        . = ALIGN(4);
        _stext = .;
        __text_start__ = _stext;

        KEEP(*(.isr_vector));   /* overrides --gc-sections */
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)

        . = ALIGN(4);
        _etext = .;
        __text_end__ = _etext;

    } >FLASH

    /* __addr__ = LOADADDR(.) */
    /* __addr__ = LOADADDR(output_section_name) */
    /* __addr__ = ABSOLUTE(.) */

    .bss : {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;  /* _bss_start__ will not consume memory, it overlays the variable at this address */

        *(.bss)
        *(.bss.*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >SRAM

    __la_data__ = LOADADDR(.data);
    .data : {
        . = ALIGN(4);
        _sdata = .;
        __data_start__ = _sdata;

        *(.data)
        *(.data.*)

        . = ALIGN(4);
        _edata = .;
        __data_end__ = _edata;
    } >SRAM AT >FLASH

}

