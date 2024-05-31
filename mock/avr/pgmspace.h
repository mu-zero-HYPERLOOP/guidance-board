#pragma once
// this header only mocks the memory regions header from the 
// teensy 4.1 core libraries, including the 
// following keywords.

// - DMAMEM : RAM2 not has fast as RAM1
// - EXTMEM : External flash controller (really slow)
// - PROGMEM : In Flash, but still reasonably fast (slower than DMAMEM)
// - FASTRUN : RAM1 fastest possible performance (placed in FLASH as well).
// - FLASHMEM: Flash memory region for not copied to RAM1 and can therefor relativly slow
//     if not in the RAM1 cache. Should be used for startup code.

#define DMAMEM
#define EXTMEM
#define PROGMEM
#define FASTRUN
#define FLASHMEM

