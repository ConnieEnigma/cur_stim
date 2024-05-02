MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 4096K
    RAM   : ORIGIN = 0x20000000, LENGTH =   768K /* 2514K?, 2496K for u5a5 */
    OTP   : ORIGIN = 0x0bfa0000, LENGTH =  512
}
/* For stm32 u575 */
/* the dataset shows it has 786k RAM. But actually we only use 192 + 64 + 512 = 768k */

/* For stm32 u5a5 */
/* the dataset shows it has 2514k RAM. But actually we only use 192 + 64 + ... = 2496k */