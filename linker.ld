
MEMORY
{
  ram (rwx)  : ORIGIN = 0x20000000, LENGTH = 8K
  flash (rx) : ORIGIN = 0x08000000, LENGTH = 32K
}

_stack_size = 2K;


_flash_start = ORIGIN(flash);
_flash_size = LENGTH(flash);




ENTRY(Reset_Handler)

SECTIONS
{
  .text :
  {
    _text_start = .;

    KEEP(*(.vector_table))

    . = ALIGN(4);
    *(.text*)
    . = ALIGN(4);

    _text_end = .;
  } > flash

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata*)
    . = ALIGN(4);
  } > flash
  
  .stack :
  {
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _stack_size;
    _end_of_stack = .;
  } > ram
  
  .rtt :
  {
      KEEP(*(.rtt))
      . = ALIGN(4);
  } > ram
  
  _data_start_in_flash = LOADADDR(.data);

  .data : 
  {
    . = ALIGN(4);
    _data_start_in_ram = .;
    *(.data*)
    . = ALIGN(4);
    _data_end_in_ram = .;
  } > ram AT > flash

  . = ALIGN(4);
  
  .bss (NOLOAD) :
  {
    _bss_start = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _bss_end = .;
  } > ram



  /DISCARD/ :
  {
    *(.ARM.exidx)

    *(.init)
    *(.fini)
    
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }
}


