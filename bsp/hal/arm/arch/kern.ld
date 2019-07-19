/*
 * Linker script for kernel
 */
INCLUDE config.ld
OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(kernel_start)
PHDRS
{
	text PT_LOAD FILEHDR PHDRS AT(SIZEOF_HEADERS);
	data PT_LOAD;
}
SECTIONS
{

	.text CONFIG_KERNEL_TEXT + SIZEOF_HEADERS :AT(SIZEOF_HEADERS){
		*(.text)
		*(.glue_7)
		*(.glue_7t)
	} : text = 0xff

	. = ALIGN(4);
	.rodata : {
		*(.rodata)
		*(.rodata.*)
	} : text

	.data ALIGN(4) : {
		*(.data)
	} : data = 0xff

	. = ALIGN(4);
	__bss = . ;
	.bss .    : {
		*(.bss)
		*(COMMON)
	}
	. = ALIGN(4);
	__end = . ;

	/DISCARD/ : { *(.comment .note) }
}
