/*
 * Custom linker script sections for Xi 640 ETH thermal camera
 * Places large frame buffers in external HyperRAM (0x90000000)
 */

SECTIONS
{
	/* HyperRAM section for large thermal frame buffers */
	.hyperram (NOLOAD) :
	{
		. = ALIGN(8);
		__hyperram_start = .;
		*(.hyperram)
		*(.hyperram.*)
		. = ALIGN(8);
		__hyperram_end = .;
	} > HYPERRAM
}
