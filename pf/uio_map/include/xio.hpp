#pragma once

#define Xil_Out32(Addr, Value)\
    *(volatile uint32_t*)(Addr) = (uint32_t)(Value)

#define Xil_In32(Addr)\
    *(volatile uint32_t*)(Addr)
