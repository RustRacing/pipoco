/* Memory layout for RP2350B */
MEMORY {
    /* RP2350B has 520KB of SRAM and external flash */
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 520K
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    /* Second stage bootloader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;
