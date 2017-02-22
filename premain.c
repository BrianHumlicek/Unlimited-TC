void __premain() 
{ 
	(*((volatile unsigned char*)(0x0030))) = 0x3C; //PPAGE set to 0x3C
} 

