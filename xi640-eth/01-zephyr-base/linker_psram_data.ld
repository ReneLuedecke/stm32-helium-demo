/* PSRAM-Datenbereich für große Frame-Buffer – OHNE eigenes SECTIONS { }! */

.psram_data (NOLOAD) :
{
    __psram_data_start = .;
    *(.psram_data)
    *(.psram_data.*)
    __psram_data_end = .;
} > PSRAM
