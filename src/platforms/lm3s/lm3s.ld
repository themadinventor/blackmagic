MEMORY
{
	rom (rx) : ORIGIN = 0x00000000, LENGTH = 256K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 64K 
}

INCLUDE libopencm3_lm3s.ld
