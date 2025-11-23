/* STM32F405RGTx memory layout */
MEMORY
{
  /* Reserve last 256 KiB (sectors 10 and 11) for KV storage */
  FLASH : ORIGIN = 0x08000000, LENGTH = 0xC0000 /* 768 KiB */
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
}
