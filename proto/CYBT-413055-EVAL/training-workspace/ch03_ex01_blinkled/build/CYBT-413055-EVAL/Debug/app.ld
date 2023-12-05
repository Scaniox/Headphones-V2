OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
/* pram_patch_begin=0x270400 pram_patch_end=0x279B98 pram_end=0x280000 */
/* ram_patch_begin=0x2129AC ram_patch_end=0x212EEC ram_end=0x270000 */
/* aon_patch_begin=0x283434 aon_patch_end=0x2837C0 aon_end=0x284000 */
/* app_ram_begin=0x212EEC app_ram_end=0x26FC00 */
/* FLASH0_BEGIN_ADDR=0x500000 FLASH0_LENGTH=0x100000 */
/* FLASH0_SS=0x500000 */
/* FLASH0_VS=0x501000 */
/* FLASH0_DS=0x503000 */
MEMORY
{
	ram (rwx) : ORIGIN = 0x212EEC, LENGTH = 0x5CD14
	aon (rwx) : ORIGIN = 0x2837C0, LENGTH = 0x840
	static_section (r) : ORIGIN = 0x500C00, LENGTH = 0x400
	log_section (r) : ORIGIN = 0x81000004, LENGTH = 0x100000
}
EXTERN(spar_irom_begin spar_irom_end spar_irom_length);
EXTERN(spar_iram_begin spar_iram_end spar_iram_length);
EXTERN(spar_iram_data_begin spar_iram_data_end spar_iram_data_length);
EXTERN(spar_iram_bss_begin spar_iram_bss_end spar_iram_bss_length);
EXTERN(spar_irom_data_begin);
EXTERN(aon_iram_end);
PROVIDE(spar_irom_length = spar_irom_end - spar_irom_begin);
PROVIDE(spar_iram_length = spar_iram_end - spar_iram_begin);
PROVIDE(spar_iram_data_length = spar_iram_data_end - spar_iram_data_begin);
PROVIDE(spar_iram_bss_length = spar_iram_bss_end - spar_iram_bss_begin);
SECTIONS
{
	spar_irom_begin = 0x212EEC;
	.text : 
	{
		CREATE_OBJECT_SYMBOLS
		*(.app_init_code)
		*(.emb_text)
		*(.text)
		*(.text_in_ram)
		*(.text.*)
		*(EmHeAacDec)
		*(.gnu.linkonce.t.*)
		*(.glue_7t)
		*(.glue_7)
		*(.vfp11_veneer)
		*(.v4_bx)
		*(.ARM.extab*)
		*(.gnu.linkonce.armextab.*)
		*(.ARM.exidx*)
		*(.gnu.linkonce.armexidx.*)
		*(.vpc_decoder_section)
	} >ram
	.rodata : ALIGN (4)
	{
		CREATE_OBJECT_SYMBOLS
		*(const_drom_var)
		*(.rodata)
		*(.constdata*)
		*(.rodata.*)
		*(.gnu.linkonce.r.*)
		*(.data.rom)
		spar_irom_end = .;
	} >ram
	.data : ALIGN (4)
	{
		CREATE_OBJECT_SYMBOLS
		spar_iram_begin = .;
		spar_iram_data_begin = .;
		*(.data)
		*(.data.*)
		*(.gnu.linkonce.d.*)
		*(.init_patch_table)
		spar_iram_data_end = .;
	} >ram
	spar_irom_data_begin = spar_iram_data_begin;
	.bss : ALIGN (4)
	{
		CREATE_OBJECT_SYMBOLS
		spar_iram_bss_begin = .;
		*(.bss)
		*(.bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		spar_iram_bss_end = .;
		spar_iram_end = .;
	} >ram
	.setup : ALIGN (4)
	{
		CREATE_OBJECT_SYMBOLS
		*(.init_code)
		*(.init_text)
		*(.spar_setup)
	} >ram
	.aon : ALIGN (4)
	{
		CREATE_OBJECT_SYMBOLS
		aon_iram_begin = .;
		*(always_on_ram_var)
		*(.aon_code)
		*(.data_in_retention_ram)
		*(.code_in_retention_ram)
		aon_iram_end = .;
	} >aon
	.static_area : ALIGN (4)
	{
		CREATE_OBJECT_SYMBOLS
		static_area_begin = .;
		KEEP(*(.data_in_static_area))
		static_area_end = .;
	} >static_section
	.log_section : ALIGN (4)
	{
		CREATE_OBJECT_SYMBOLS
		KEEP(*(log_data))
	} >log_section
}
