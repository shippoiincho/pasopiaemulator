// Dummy ROM file for Prebuild binary

#define ROMBASE 0x10050000u

uint8_t *basicrom=(uint8_t *)(ROMBASE);
uint8_t *cgrom   =(uint8_t *)(ROMBASE+ 0x8000);
uint8_t *kanjirom=(uint8_t *)(ROMBASE+0x10000);