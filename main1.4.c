/* Revision History
0.1
- first stable working release
- implement 2 timing retard outputs, one on a timer
- add outpc parameter flag to monitor retard output status
0.2
- begin vNet implementation
    - read horizontile and vertical accel from vNet
- change fox_uint to use shifts (<< 15) instead of multiply and divide
0.3.1
-  added the changelog
-  expand vNet communications
	- add vNet connect and transmit ability
-  add outpc parameters integrated_velocity, integrated_position, tire_speed, tire_slip
-  add inpram parameters tire_growth, tire_size, g_factor
-  commented pragma statements to reduce compiler warnings
1.3.2 - modified canset file and code for simplicity
1.3.3
- working vnet code, tested wednesday 9/23/09
1.3.4
- removed vNet PIDS 30-34 xmin-ymax
- added inertial slip target

*/
#include "hcs12def.h"      /* common defines and macros */
#include "flash.h"         /* flashburner defines, structures */
#include <string.h>

#define FAR_TEXT1_ATTR __attribute__ ((far)) __attribute__ ((section (".text1")))
#define TEXT1_ATTR __attribute__ ((section (".text1")))
#define EEPROM_ATTR __attribute__ ((section (".eeprom")))
#define INTERRUPT
#define POST_INTERRUPT __attribute__((interrupt))
#define POST_INTERRUPT_TEXT3  __attribute__ ((section (".text3"))) __attribute__((interrupt))
#define ENABLE_INTERRUPTS __asm__ __volatile__ ("cli");
#define DISABLE_INTERRUPTS __asm__ __volatile__ ("sei");
#define NEAR
#define VECT_ATTR __attribute__ ((section (".vectors")))
extern void _start(void);       /* Startup routine */

//ISR prototypes
INTERRUPT void UnimplementedISR(void) POST_INTERRUPT;
INTERRUPT void ISR_IGNIn(void) POST_INTERRUPT;
INTERRUPT void ISR_OSSIn(void) POST_INTERRUPT;
INTERRUPT void ISR_TimerOverflow(void) POST_INTERRUPT;
INTERRUPT void ISR_Foreground_Loop(void) POST_INTERRUPT;
INTERRUPT void ISR_SCI_Comm(void) POST_INTERRUPT;
INTERRUPT void CanTxIsr(void) POST_INTERRUPT;
INTERRUPT void CanRxIsr(void) POST_INTERRUPT;

typedef void (* NEAR tIsrFunc)(void);
const tIsrFunc _vect[] VECT_ATTR = {      /* Interrupt table */
        UnimplementedISR,                 /* vector 63	RESERVED */
        UnimplementedISR,                 /* vector 62	RESERVED */
        UnimplementedISR,                 /* vector 61	RESERVED */
        UnimplementedISR,                 /* vector 60	RESERVED */
        UnimplementedISR,                 /* vector 59	RESERVED */
        UnimplementedISR,                 /* vector 58	VREG LVI */
        UnimplementedISR,                 /* vector 57	PWM Emergency Shutdown */
        UnimplementedISR,                 /* vector 56	PORT P */
        UnimplementedISR,                 /* vector 55	RESERVED */
        UnimplementedISR,                 /* vector 54	RESERVED */
        UnimplementedISR,                 /* vector 53	RESERVED */
        UnimplementedISR,                 /* vector 52	RESERVED */
        UnimplementedISR,                 /* vector 51	RESERVED */
        UnimplementedISR,                 /* vector 50	RESERVED */
        UnimplementedISR,                 /* vector 49	RESERVED */
        UnimplementedISR,                 /* vector 48	RESERVED */
        UnimplementedISR,                 /* vector 47	RESERVED */
        UnimplementedISR,                 /* vector 46	RESERVED */
        UnimplementedISR,                 /* vector 45	RESERVED */
        UnimplementedISR,                 /* vector 44	RESERVED */
        UnimplementedISR,                 /* vector 43	RESERVED */
        UnimplementedISR,                 /* vector 42	RESERVED */
        UnimplementedISR,                 /* vector 41	RESERVED */
        UnimplementedISR,                 /* vector 40	RESERVED */
        CanTxIsr,                         /* vector 39	CAN transmit */
        CanRxIsr,                         /* vector 38	CAN recieve */
        CanRxIsr,                         /* vector 37	CAN Errors */
        UnimplementedISR,                 /* vector 36	CAN Wake up */
        UnimplementedISR,                 /* vector 35	FLASH */
        UnimplementedISR,                 /* vector 34	RESERVED */
        UnimplementedISR,                 /* vector 33	RESERVED */
        UnimplementedISR,                 /* vector 32	RESERVED */
        UnimplementedISR,                 /* vector 31	RESERVED */
        UnimplementedISR,                 /* vector 30	RESERVED */
        UnimplementedISR,                 /* vector 29	CRG Self Clock Mode */
        UnimplementedISR,                 /* vector 28	CRG PLL Lock */
        UnimplementedISR,                 /* vector 27	RESERVED */
        UnimplementedISR,                 /* vector 26	RESERVED */
        UnimplementedISR,                 /* vector 25	RESERVED */
        UnimplementedISR,                 /* vector 24	PORT J */
        UnimplementedISR,                 /* vector 23	RESERVED */
        UnimplementedISR,                 /* vector 22	aTD */
        UnimplementedISR,                 /* vector 21	RESERVED */
        ISR_SCI_Comm,                     /* vector 20	SCI */
        UnimplementedISR,                 /* vector 19	SPI */
        UnimplementedISR,                 /* vector 18	Pulse Accululator Input Edge */
        UnimplementedISR,                 /* vector 17	Pulse Accumulator Overflow */
        ISR_TimerOverflow,                /* vector 16	Standard Timer Overflow */
        UnimplementedISR,                 /* vector 15	Standard Timer Channel 7 */
        UnimplementedISR,                 /* vector 14	Standard Timer Channel 6 */
        UnimplementedISR,                 /* vector 13	Standard Timer Channel 5 */
        UnimplementedISR,                 /* vector 12	Standard Timer Channel 4 */
        UnimplementedISR,                 /* vector 11	Standard Timer Channel 3 */
        ISR_IGNIn,                        /* vector 10	Standard Timer Channel 2 */
        UnimplementedISR,                 /* vector 09	Standard Timer Channel 1 */
        ISR_OSSIn,                        /* vector 08	Standard Timer Channel 0 */
        ISR_Foreground_Loop,              /* vector 07	Real Time Interrupt */
        UnimplementedISR,                 /* vector 06	IRQ */
        UnimplementedISR,                 /* vector 05	XIRQ */
        UnimplementedISR,                 /* vector 04	SWI */
        UnimplementedISR,                 /* vector 03	Unimplemented Instruction Trap */
        UnimplementedISR,                 /* vector 02	COP Failure Reset */
        UnimplementedISR,                 /* vector 01	Clock Monitor Reset */
        _start                            /* Reset vector */
   };

#define NO_TBLES 	5
#define VNET_INTERVAL 19	//RTI/8 ticks between vnet channel data packets (1 tick = .001024s) 19=.0194s=50samples/second
#define NUM_VNET_CHANNELS 13
#define VNET_TYPE 0x1000
#define NO_CAN_RCV_ADDR 4
#define NPORT    7

#define	MSG_CMD	0
#define	MSG_REQ	1
#define	MSG_RSP	2
#define	MSG_XSUB	3
#define NO_CANMSG 10
#define NO_CANBOARDS 16
#define CANID_THIS_BOARD 0
// Error status words: 
//    -bits 0-7 are current errors
//    -bits 8-15 are corresponding latched errors
#define	XMT_ERR			0x0101
#define	CLR_XMT_ERR		0xFFFE
#define	XMT_TOUT		0x0202
#define	CLR_XMT_TOUT	0xFFFD
#define	RCV_ERR			0x0404
#define	CLR_RCV_ERR		0xFFFB
#define	SYS_ERR		0x0808

#define oss_time_pairs	30
#define slip_limit_pairs 15
#define tire_growth_pairs 10

// User inputs - 1 set in flash, 1 in ram
typedef struct {
	unsigned int	rev_limit,
					rev_limit_window,
					torque_limit_window,
					rpm_teeth,
					oss_teeth,
					second_retard_time,
					retard_off_time,
					oss_time[2][oss_time_pairs],
					tire_growth[2][tire_growth_pairs],
					slip_limit[2][slip_limit_pairs],
					tire_circumference,	//inches 8bpp
					g_factor,	//ft/s/s 10bpp
					rpm_LF,		//rpm lag factor 15bpp
					oss_LF,
					slip_LF;
	unsigned char	vnet_enable,
					standard_limiting,
					inertial_limiting;
} inputs1;

//VNet config array
typedef struct {
	unsigned int	serial_number;
	unsigned int	record_rate;
	unsigned int	vnet_id;
	unsigned char	sensor_type,
					digits_before,
					digits_after;
	int				chart_min;
	char			chart_min_exp;
	int				chart_max;
	char			chart_max_exp,
					data_exp;
}vnet_config;

//#pragma ROM_VAR INP_ROM

// flash copy of inputs - initialized

#define SECTOR_BYTES         1024 // bytes
#define SECTOR_WORDS         ((int)(SECTOR_BYTES/2))
#define N_SECTORS(theStruct) ((int)((sizeof(theStruct)+SECTOR_BYTES-1)/SECTOR_BYTES))
#define N_PADDING(theStruct) ((int)(N_SECTORS(theStruct)*SECTOR_BYTES - sizeof(theStruct)))

const inputs1 in1flash EEPROM_ATTR = {
	9000,	//rev_limit
	100,	//rev_limit_window
	100,	//torque_limit_window
	4,		//IGN teeth
	100,	//OSS teeth
	146,	//second retard time limit	0.001024	0.150 seconds
	20,		//retard off time
	{{000,100,200,300,400,500,600,700,800,900,1000,1200,1400,1600,1800,2000,2400,2800,3200,3600,4000,4400,4800,5200,5600,6000,6400,6800,7200,7600},	//OSS_limit_time
	{ 231,266,311,353,401,443,493,535,588,630, 684, 765, 838, 922, 995,1063,1187,1306,1512,1599,1689,1774,1851,1934,2001,2062,2129,2196,2211,2211}},
	{{100, 200, 400, 600, 800, 1000, 1200, 1400, 1600, 1800},	//Tire_growth RPM
	{32768,32768,32768,32768,32768,32768,32768,32768,32768,32768}},	//Percentage 15bpp
	{{  0,  20,   40,  60,  80, 100, 120, 140, 160, 180, 200, 220, 240, 260, 280},	//Inertial Speed ft/s
	{1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024,1024}},	//Target slip percent 10bpp
	26880,	//tire_circumference	8bpp	105
	32946,	//g_factor	10bpp	32.174
	32768,	//rpm lag factor	15bpp
	32768,	//oss lag factor	15bpp
	32768,	//slip slag factor	15bpp
	1,		//VNet Enable
	1,		//standard limiting
	0		//inertial limiting
};
const unsigned char in1padding[N_PADDING(inputs1)] EEPROM_ATTR ={0}; 
/*
	FNAME		= "Sensor Type"
			"0 - Not Selected\n"
			"1 - Time (S)\n"
		 	"2 - Speed (rpm)\n"
			"3 - Ratio (%)\n"
			"4 - Bit Flags\n"
			"5 - Position (ft)\n"
			"6 - Velocity (ft/s)\n"
			"7 - Acceleration (ft/s/s)\n"
			"8 - Raw Data (int)\n"

1  seconds
2  run_timer
3  oss
4  oss_limit_time
5  oss_pulse_count
6  integrated_velocity
7  integrated_position
8  flag
9  debug_1
10 debug_2
11 tire_slip
12 tire speed
13 tire_slip_target
*/
const vnet_config vnet_config_flash[NUM_VNET_CHANNELS] EEPROM_ATTR = {
//	serial	record	vnet	sensor	digits	digits	chart	chart	chart	chart	data
//	number	rate	id		type	before	after	min		min_exp	max		max_exp	exponent
{	0x8A0,	50,		0x5A0,	1,		3,		0,		0,		0,		240,	0,		0},
{	0x8A1,	50,		0x5A1,	1,		2,		3,		0,		0,		10,		0,		-3},
{	0x8A2,	50,		0x3B8,	2,		4,		0,		0,		0,		2000,	0,		0},
{	0x8A3,	50,		0x5A2,	2,		4,		0,		0,		0,		2000,	0,		0},
{	0x8A4,	50,		0x5A3,	8,		5,		0,		0,		0,		20000,	0,		0},
{	0x8A5,	50,		0x3BC,	6,		3,		1,		0,		0,		350,	0,		-1},
{	0x8A6,	50,		0x3A3,	5,		4,		1,		0,		0,		1500,	0,		-1},
{	0x8A7,	50,		0x527,	4,		1,		0,		0,		0,		6,		0,		0},
{	0x8A8,	50,		0x5A4,	8,		5,		0,		0,		0,		65535,	0,		0},
{	0x8A9,	50,		0x5A5,	8,		5,		0,		0,		0,		65535,	0,		0},
{	0x8AA,	50,		0x5A6,	3,		2,		3,		0,		0,		3,		0,		-3},
{	0x8AB,	50,		0x5A7,	6,		3,		1,		0,		0,		350,	0,		-1},
{	0x8AC,	50,		0x5A8,	3,		2,		3,		0,		0,		3,		0,		-3}
};
const unsigned char vnetpadding[N_PADDING(vnet_config_flash)] EEPROM_ATTR ={0}; 

//#pragma ROM_VAR DEFAULT

//#pragma ROM_VAR OVF_ROM

const char RevNum[20] =  {    // revision no:
 // only change for major rev and/or interface change.
  "Unlimited TC v0.4  "	//19 bytes
},
Signature[32] = {            // program title.
 // Change this every time you tweak a feature.
  "ver 0.4.0 firmware"
 };
//#pragma ROM_VAR DEFAULT

// ram copy of inputs
inputs1 inpram;

vnet_config vnet_config_ram[NUM_VNET_CHANNELS];

const unsigned long baud_const = 115200;

//clock counter, etc
unsigned int mms,millisec,second_retard_timer,retard_off_timer,burn_flag;
unsigned long lmms,ltch_lmms,rcv_timeout,adc_lmms,rpm_divisor,oss_divisor,background_counter;
unsigned char flocker,next_adc,first_adc,txmode,tble_idx,burn_idx;


unsigned int TC0_overflow,TC2_overflow; //timer overflow counters

unsigned int txcnt,txgoal,rxoffset,rxnbytes,rxcnt;

// CAN variables
//unsigned long cansendclk;
unsigned short can_status;
//unsigned char can_clr_stat,can_reset,can_id;
unsigned char can_reset,cantx_buffer_index,cantx_buffer_length,canrx_buffer_length,transmit_semaphore;
unsigned char cantx_buffer[16][14];
unsigned char canrx_buffer[10];

// pointers for spare port pins
volatile unsigned char *pPTMpin[8], *pPTTpin[8], *pPTApin0;
unsigned char dummyReg,lst_pval[NPORT];

// allocate space in ram for flash burner core
volatile unsigned char RamBurnPgm[36]; 

// rs232 Outputs to pc
typedef struct {
	unsigned int	seconds,
					run_timer,
					rpm,
					oss,
					rev_limit_level,
					torque_limit_level,
					oss_limit_time,
					oss_pulse_count,
					ign_pulse_count,
					tire_slip,	//percent 10bpp
					tire_slip_target,	//percent 10bpp
					debug_1,
					debug_2;

	unsigned char	flag;

	int				accel_g,
					vert_g;
	long		
					integrated_accel,		//ft/s/s x10
					integrated_velocity,	//ft/s x1000
					integrated_position,	//ft x1000
					tire_speed;				//ft/s x1000
} variables;

variables outpc, txbuf;

//#pragma ROM_VAR OVF_ROM

typedef struct {
   unsigned int *addrRam;
   unsigned int *addrFlash;
   unsigned int  n_bytes;
} tableDescriptor;

const tableDescriptor tables[NO_TBLES] =  { 
  { (unsigned int *)&inpram, (unsigned int *)&in1flash,       sizeof(inputs1)         }, 
  { (unsigned int *)&txbuf,  NULL,                            sizeof(txbuf)           },
  { (unsigned int *)&outpc,  NULL,                            sizeof(outpc)           },
  { NULL,                    (unsigned int *)&RevNum,         sizeof(RevNum)          },
  { (unsigned int *)&vnet_config_ram,(unsigned int *)&vnet_config_flash, sizeof(vnet_config_flash)}
};

//#ifndef GCC_BUILD
//#pragma ROM_VAR DEFAULT
//#endif

#define tableInit(iTable) (void)memcpy(tables[iTable].addrRam, tables[iTable].addrFlash, tables[iTable].n_bytes)
#define tableByteRam(iTable, iByte)   ((unsigned char *)tables[iTable].addrRam + iByte)
#define tableWordRam(iTable, iWord)   (tables[iTable].addrRam + iWord)
#define tableByteFlash(iTable, iByte) ((unsigned char *)tables[iTable].addrFlash + iByte)
#define tableWordFlash(iTable, iWord) (tables[iTable].addrFlash + iWord)
#define tableBytes(iTable)            (tables[iTable].n_bytes)
#define tableWords(iTable)            ((tables[iTable].n_bytes+1)/2) // Round up

//#pragma ROM_VAR DEFAULT


// Prototypes - Note: ISRs prototyped above.
void main(void) FAR_TEXT1_ATTR;
unsigned int fox_uint(unsigned int x, unsigned char pairs, unsigned int* function[2][pairs]);
void get_adc(char chan1, char chan2);
void switch_adc(unsigned char next_adc);
void accel_integrator(unsigned int sample_time);

void fburner(unsigned int* progAdr, unsigned int* bufferPtr, 
  					 unsigned int no_words) TEXT1_ATTR;
void tburner(char erase_write, unsigned int* addr, unsigned int word) FAR_TEXT1_ATTR; 
//int intrp_2ditable(unsigned int x, int y, unsigned char nx, unsigned char ny,
//  unsigned int * x_table, int * y_table, int * z_table) TEXT1_ATTR;
void CanInit(void) TEXT1_ATTR;
void can_xsub01(void);
void Flash_Init(unsigned long oscclk) TEXT1_ATTR;
void Flash_Erase_Sector(unsigned int *address) FAR_TEXT1_ATTR;
void Flash_Write_Word(unsigned int *address, unsigned int data) TEXT1_ATTR;
//int intrp_1dctable(int x, unsigned char n, int * x_table, 
//  char sgn, unsigned char * z_table) TEXT1_ATTR;
//int intrp_2dctable(unsigned int x, int y, unsigned char nx, unsigned char ny,
//  unsigned int * x_table, int * y_table, unsigned char * z_table) TEXT1_ATTR;

//#pragma CODE_SEG OTHER_ROM

void ego_calc(void) TEXT1_ATTR;
unsigned char afrLF_calc(long t) TEXT1_ATTR; 

//#pragma CODE_SEG DEFAULT

extern void reboot(void) FAR_TEXT1_ATTR;
extern void monitor(void) FAR_TEXT1_ATTR;
extern void SpSub(void);
extern void NoOp(void);


void main(void) {
//vars for vNet code....
unsigned char can_buffer[14];
unsigned int PID;
unsigned char message_serial, connected_channel_index;
//general purpose vars
int ix;
long ltmp;

//PPAGE = 0x3C;
  // initalize PLL - reset default is Oscillator clock
  // 8 MHz oscillator, PLL freq = 48 MHz, 24 MHz bus, 
  //  divide by 16 for timer of 2/3 usec tic
  PLLCTL &= 0xBF;     // Turn off PLL so can change freq
  SYNR = 0x02;        // set PLL/ Bus freq to 48/ 24 MHz
  REFDV = 0x00;
  PLLCTL |= 0x40;     // Turn on PLL
  // wait for PLL lock
  while (!(CRGFLG & 0x08));
  CLKSEL = 0x80;      // select PLL as clock
  // wait for clock transition to finish
  for (ix = 0; ix < 60; ix++);
    
  // open flash programming capability
  Flash_Init(8000);
  //inp_spare used to force inpflash, flashve_table into sectors.
  // Must use in program or it won't use up the entire sector. POS CW 
  // ignores the pragma making this stupid statement necessary.
  ix = in1padding[0];
  if((int)RamBurnPgm & 0x0001)	{   // odd address - cpy to even one
    (void)memcpy((void *)RamBurnPgm,NoOp,1);         // cpy noop to 1st location
    (void)memcpy((void *)&RamBurnPgm[1],SpSub,32);   // cpy flashburn core pgm to ram
  }
  else
    (void)memcpy((void *)RamBurnPgm,SpSub,32);       // cpy flashburn core pgm to ram

  // load all user inputs from Flash to RAM
  tableInit(0);
  tableInit(4);
  
  // set up i/o ports
  //    - port M2 is fast idle solenoid
  //    - port M3 is inj led
  //    - port M4 is accel led
  //    - port M5 is warmup led
  //    - port E0 is flex fuel sensor input
  //    - port E4 is fuel pump
  //    - port P5 is bootload pin (input)
  //    - port T6 is IAC Coil A
  //    - port T7 is IAC Coil B
  //    - port B4 is IAC Enable
  //    - port A0 is Knock Enable (if set, means retard timing)
  DDRM |= 0xFC;    // port M - all outputs, full drive by default
  DDRE |= 0x10;	   // port E4 - output, E0 is input
  DDRT |= 0xF0;    // port T4-7 - outputs, coincides with TIOS
  DDRB |= 0x10;    // port B4 - output
  DDRA |= 0x01;    // port A0 - output
  //  Set pointers to real port addresses
  for(ix = 0; ix < 8; ix++)  {
    pPTMpin[ix] = pPTM;
    pPTTpin[ix] = pPTT;
  }
  pPTApin0 = pPORTA;
  // reset those pointers to pins which are to be used as alternate outputs
  // turn off fidle solenoid, leds
  *pPTMpin[2] &= ~0x04;
  *pPTMpin[3] &= ~0x08;
  *pPTMpin[4] &= ~0x10;
  *pPTMpin[5] &= ~0x20;
  PORTE &= ~0x10;            // turn off fuel pump
  *pPTTpin[6] &= ~0x40;      // turn off IAC coils
  *pPTTpin[7] &= ~0x80;
  *pPTApin0 &= ~0x01;    // no knock signal
  
  // set all unused (even unbonded) ports to inputs with pullups
  DDRA &= 0x01;
  DDRB &= 0x10;
  DDRE &= 0x10;
  PUCR |= 0x13;    // enable pullups for ports E, B and A
  DDRP = 0x00;
  PERP = 0xFF;     // enable pullup resistance for port P 
  DDRJ &= 0x3F;
  PERJ |= 0xC0;    // enable pullup resistance for port J6,7
  DDRS &= 0xF3;
  PERS |= 0x0C;    // enable pullup resistance for port S2,3

  // set up CRG RTI Interrupt for .128 ms clock. CRG from 8MHz oscillator.
  mms = 0;        // .128 ms tics
  millisec = 0;   // 1.024 ms clock (8 tics) for adcs
  lmms = 0;
  //cansendclk = 7812;
  ltch_lmms = 0;
  burn_flag = 0;
  RTICTL = 0x10;   // load timeout register for .128 ms (smallest possible)
  CRGINT |= 0x80;  // enable interrupt
  CRGFLG = 0x80;   // clear interrupt flag (0 writes have no effect)

  // Set up SCI (rs232): SCI BR reg= BusFreq(=24MHz)/16/baudrate
//inpram.baud = 115200; // Use this for debugging corrupted flash sectors.
  SCI0BDL = (unsigned char)(1500000/baud_const);
  ltmp = (150000000/baud_const) - ((long)SCI0BDL*100);
  if(ltmp > 50)SCI0BDL++;   // round up 
//inpram.baud = in1flash.baud; // See above.
  SCI0CR1 = 0x00;
  SCI0CR2 = 0x24;   // TIE=0,RIE = 1; TE=0,RE =1
  txcnt = 0;
  rxcnt = 0;
  txmode = 0;
  txgoal = 0;
  rcv_timeout = 0xFFFFFFFF;

  //   TC0: Ignition event
  //   TC1: Unused
  //   TC2: OSS input
  //   TC3: Unused
  //   TC4: Timing retard trigger output
  //   TC5: Ignition output


  // Set prescaler to 16. This divides bus clk (= PLLCLK/2 = 24 MHz)
  //   by 16. This gives 1.5 MHz timer, 1 tic = 2/3 us.
  MODRR = 0x00;
  TSCR2 = 0x04;
  TSCR2 |= 0x80;   // enable timer overflow interrupt (TOI)
  TIOS = 0xF0;	//TC0,TC2 input capture, all else are outputs, coinsides with DDRT
  TCTL1 = 0x00;	//No output compare channels are used
  TCTL2 = 0x00;
  TCTL3 = 0x00;
  TCTL4 = 0x31;	//TC2 capture on rising/falling, TC0 capture on rising only
  TIE = 0x05;	//enable TC0, TC2 interrupts
//PTT &= ~0x20;	//Turn coil off
  PTT |= 0x20;	//Turn second retrad off
  PTT |= 0x10;	//Turn first retard off
  

  // set up ADC processing
  // ATD0
  //    - AN0 is MAP
  //    - AN1 is MAT
  //    - AN2 is CLT
  //    - AN3 is TPS
  //    - AN4 is BAT
  //    - AN5 is EGO1 (NB or WB)
  //    - AN6 is Spare or BARO (BaroOption =2) or EGO2 (EgoOption = 2 or 4)
  //    - AN7 is Spare or KNOCK
  // Set up ADCs so they continuously convert, then read result registers
  //  every millisecond
  next_adc = 0;	   	// specifies next adc channel to be read
  ATD0CTL2 = 0x40;  // leave interrupt disabled, set fast flag clear
  ATD0CTL3 = 0x00;  // do 8 conversions/ sequence
  ATD0CTL4 = 0x67;  // 10-bit resoln, 16 tic cnvsn (max accuracy),
                    // prescaler divide by 16 => 2/3 us tic x 18 tics
  ATD0CTL5 = 0xB0;  // right justified,unsigned, continuous cnvsn,
                    // sample 8 channels starting with AN0
  ATD0CTL2 |= 0x80;  // turn on ADC0
  // wait for ADC engine charge-up or P/S ramp-up
  for(ix = 0; ix < 160; ix++)  {
    while(!(ATD0STAT0 >> 7));	 // wait til conversion complete
    ATD0STAT0 = 0x80;
  }
  adc_lmms = lmms;

  // Initialize variables
  flocker = 0;  

//	set variable block addresses to be used for CAN communications
//  canvar_blkptr[0] = (char *)&inpram;
//  canvar_blkptr[1] = (char *)&outpc;
//  for(ix = 2;ix < NO_VAR_BLKS; ix++)  {	 // rest spares for now
//    canvar_blkptr[ix] = 0;
//  }
//	Initialize CAN comms
  can_reset = 0;
//	can_id = 0;    // MS-II is processor 0
	CanInit();

  // make Ignition IC highest priority interrupt
  HPRIO = 0xEA;	//TC2

  // enable global interrupts
  ENABLE_INTERRUPTS
  TSCR1 = 0x80;	//Start the IC/OC timer
  //  main loop
	rpm_divisor = (unsigned long)90000000 / (unsigned long)inpram.rpm_teeth;
	oss_divisor = (unsigned long)90000000 / (unsigned long)inpram.oss_teeth;
	second_retard_timer = 0;
	background_counter = 0;
  for (;;)  {
	//this logic performs the speed vs. time lookup, and determines if any corrective action is needed
	outpc.oss_limit_time = fox_uint(outpc.run_timer, oss_time_pairs, &inpram.oss_time[0][0]);

	outpc.tire_speed = ((((unsigned long)fox_uint(outpc.oss, tire_growth_pairs, &inpram.tire_growth[0][0]) * (unsigned long)inpram.tire_circumference) >> 15) * (unsigned long)outpc.oss ) / (unsigned long)184;	//184 coeff for inches/min x 256 -> ft/s x 1000

	if (outpc.integrated_velocity > 0)	//Prevent divide by zero
		outpc.tire_slip += (int)((((long)((outpc.tire_speed << 10) / outpc.integrated_velocity) - (long)outpc.tire_slip) * (long)inpram.slip_LF) >> 15);
	else
		outpc.tire_slip = 0;

	outpc.tire_slip_target = fox_uint((unsigned int)(outpc.integrated_velocity / 1000), slip_limit_pairs, &inpram.slip_limit[0][0]);

/*	outpc.oss_limit_time = 6000;
	if (outpc.oss > outpc.oss_limit_time)
		outpc.torque_limit_level = (((((outpc.oss - outpc.oss_limit_time) * 10) / (inpram.torque_limit_window + 1)) * 8) / 10) + 1;	//window + 1 is to eliminate possible DIV/0
	else
		outpc.torque_limit_level = 0;
	if (outpc.torque_limit_level > 9)
		outpc.torque_limit_level = 9;
	
	//rev limiter calculation logic
	if (outpc.rpm > inpram.rev_limit)
		outpc.rev_limit_level = (((((outpc.rpm - inpram.rev_limit) * 10) / (inpram.rev_limit_window + 1)) * 8) / 10) + 1;
	else
		outpc.rev_limit_level = 0;
	if (outpc.rev_limit_level > 9)
		outpc.rev_limit_level = 9;
*/
	//if any limiting is desired, enable the external retard
	if ((outpc.oss > outpc.oss_limit_time) && (inpram.standard_limiting == 1)){
		retard_off_timer = inpram.retard_off_time;
		PTT &= ~0x10;	//retard on
		if (second_retard_timer > inpram.second_retard_time)
			PTT &= ~0x20;	//retard on
	}
	else if((outpc.tire_slip > outpc.tire_slip_target) && (inpram.inertial_limiting == 1)){
		retard_off_timer = inpram.retard_off_time;
		PTT &= ~0x10;	//retard on
		if (second_retard_timer > inpram.second_retard_time)
			PTT &= ~0x20;	//retard on
	}
	else if(retard_off_timer == 0){
		PTT |= 0x10;	//retard first retard off
		PTT |= 0x20;	//retard second retard off
		second_retard_timer = 0;
	}

//--------------------------------------------------------------------------------------
//Racepak vNet interface
//This section handles all device configuration, and data transmission
//Sensor data receiving is handled directly in the CanRxISR
//--------------------------------------------------------------------------------------
//Sensor Channel Transmit-----
	if((transmit_semaphore == 0xCC) && (inpram.vnet_enable == 1)){	//semaphore set by RTI
		for(ix=0;ix < NUM_VNET_CHANNELS;ix++){
			can_buffer[0] = 0x00;
			can_buffer[1] = 0x00;
			can_buffer[2] = (unsigned char)(vnet_config_flash[ix].vnet_id >> 8);
			can_buffer[3] = (unsigned char)(vnet_config_flash[ix].vnet_id & 0xFF);
			can_buffer[4] = 0x01;
			can_buffer[5] = vnet_config_flash[ix].data_exp;
			
			if(ix == 0){	//"if" statement used because "switch" caused linker error.  Original switch statement below
				can_buffer[6] = (unsigned char)(outpc.seconds & 0xFF);
				can_buffer[7] = (unsigned char)((outpc.seconds >> 8) & 0x7F);
			}
			else if(ix == 1){
				can_buffer[6] = (unsigned char)(outpc.run_timer & 0xFF);
				can_buffer[7] = (unsigned char)((outpc.run_timer >> 8) & 0x7F);
			}
			else if(ix == 2){
				can_buffer[6] = (unsigned char)(outpc.oss & 0xFF);
				can_buffer[7] = (unsigned char)((outpc.oss >> 8) & 0x7F);
			}
			else if(ix == 3){
				can_buffer[6] = (unsigned char)(outpc.oss_limit_time & 0xFF);
				can_buffer[7] = (unsigned char)((outpc.oss_limit_time >> 8) & 0x7F);
			}
			else if(ix == 4){
				can_buffer[6] = (unsigned char)(outpc.oss_pulse_count & 0xFF);
				can_buffer[7] = (unsigned char)((outpc.oss_pulse_count >> 8) & 0x7F);
			}
			else if(ix == 5){
				ltmp = outpc.integrated_velocity / 100;
				can_buffer[6] = (unsigned char)(ltmp & 0xFF);
				can_buffer[7] = (unsigned char)((ltmp >> 8) & 0xFF);
			}
			else if(ix == 6){
				ltmp = outpc.integrated_position / 100;
				can_buffer[6] = (unsigned char)(ltmp & 0xFF);
				can_buffer[7] = (unsigned char)((ltmp >> 8) & 0xFF);
			}
			else if(ix == 7){	//Flags
				can_buffer[6] = (PTIT & 0x30)  >> 4;
				can_buffer[7] = 0x00;
			}
			else if(ix == 8){
				can_buffer[6] = (unsigned char)(outpc.debug_1 & 0xFF);
				can_buffer[7] = (unsigned char)(outpc.debug_1 >> 8);
			}
			else if(ix == 9){
				can_buffer[6] = (unsigned char)(outpc.debug_2 & 0xFF);
				can_buffer[7] = (unsigned char)(outpc.debug_2 >> 8);
			}
			else if(ix == 10){
				ltmp = ((unsigned long)outpc.tire_slip * 1000) >> 10;
				can_buffer[6] = (unsigned char)(ltmp & 0xFF);
				can_buffer[7] = (unsigned char)((ltmp >> 8) & 0x7F);
			}
			else if(ix == 11){
				ltmp = outpc.tire_speed / 100;
				can_buffer[6] = (unsigned char)(ltmp & 0xFF);
				can_buffer[7] = (unsigned char)((ltmp >> 8) & 0x7F);
			}
			else if(ix == 12){
				ltmp = (unsigned long)(outpc.tire_slip_target * 1000) >> 10;
				can_buffer[6] = (unsigned char)(ltmp & 0xFF);
				can_buffer[7] = (unsigned char)((ltmp >> 8) & 0x7F);
			}
			/*
			
			switch(ix){
			case 0:
				can_buffer[5] = 0x00;	//0 exponent
				can_buffer[6] = (unsigned char)(outpc.seconds & 0xFF);
				can_buffer[7] = (unsigned char)((outpc.seconds >> 8) & 0x7F);
				break;
			case 1:
				can_buffer[5] = 0x00;	//0 exponent
				can_buffer[6] = (unsigned char)(outpc.run_timer & 0xFF);
				can_buffer[7] = (unsigned char)((outpc.run_timer >> 8) & 0x7F);
				break;
			case 2:	//fails
				can_buffer[5] = 0x00;	//0 exponent
				can_buffer[6] = (unsigned char)(outpc.oss & 0xFF);
				can_buffer[7] = (unsigned char)((outpc.oss >> 8) & 0x7F);
				break;
			case 3:	//fails
				can_buffer[5] = 0x00;	//0 exponent
				can_buffer[6] = (unsigned char)(outpc.oss_limit_time & 0xFF);
				can_buffer[7] = (unsigned char)((outpc.oss_limit_time >> 8) & 0x7F);
				break;
			case 4:
				can_buffer[5] = 0x00;	//0 exponent
				can_buffer[6] = (unsigned char)(outpc.oss_pulse_count & 0xFF);
				can_buffer[7] = (unsigned char)((outpc.oss_pulse_count >> 8) & 0x7F);
				break;
			case 5:
				can_buffer[5] = 0x00;	//0 exponent
				can_buffer[6] = (unsigned char)(outpc.integrated_velocity & 0xFF);
				can_buffer[7] = (unsigned char)((outpc.integrated_velocity >> 8) & 0x7F);
				break;
			case 6:	//fails
				can_buffer[5] = 0x00;	//0 exponent
				can_buffer[6] = (unsigned char)(outpc.integrated_position & 0xFF);
				can_buffer[7] = (unsigned char)((outpc.integrated_position >> 8) & 0x7F);
				break;
			case 7:	//fails....
				can_buffer[5] = 0x00;	//0 exponent
				can_buffer[6] = (outpc.flag & 0x30) >> 4;
				can_buffer[7] = 0x00;
				break;
			case 8:
				can_buffer[5] = 0x00;	//0 exponent
				can_buffer[6] = (unsigned char)(outpc.debug_1 & 0xFF);
				can_buffer[7] = (unsigned char)(outpc.debug_1 >> 8);
				break;
			case 9:
				can_buffer[5] = 0x00;	//0 exponent
				can_buffer[6] = (unsigned char)(outpc.debug_2 & 0xFF);
				can_buffer[7] = (unsigned char)(outpc.debug_2 >> 8);
				break;
			}	//End switch
*/
			can_buffer[12] = 0x04;	//4 data bytes
			can_buffer[13] = 0x00;	//optional MSCAN priority

			if(cantx_buffer_length < 15){	//If there is room in the FIFO buffer
				CANTIER = 0;	//Disable CanTxISR
				(void)memcpy(&cantx_buffer[(cantx_buffer_index + cantx_buffer_length) & 0x0F][0],&can_buffer[0],14);
				cantx_buffer_length++;
				if(CANTFLG == 0)	//MSCAN buffers are full
					CANTIER = 0x07;	//Enable all MSCAN transmit interrupts
				else{	//Otherwise, trigger next logical transmit interrupt
					CANTBSEL = CANTFLG;
					CANTIER = CANTBSEL;
				}	//end if(CANTFLG == 0)
			}	//end if(cantx_buffer_length < 15)			
		}	//end for(ix=0;ix < NUM_VNET_CHANNELS;ix++)
		transmit_semaphore = 0;
	}	//end if(transmit_semaphore == 0xCC)

//Config receive-----------------
	if((canrx_buffer_length) && (inpram.vnet_enable == 1)){
		can_buffer[12] = 0;	//flag to enable transmit at the end of select, set to disable
		switch(canrx_buffer[0]){	//canrx_buffer[0] holds index into can_rcv_addr, set in CanRxISR
		case 2:	//CAN ID (00 00 00 08) Config Broadcast
			if(!(canrx_buffer[1] == 0x10))	//check the first data byte to be 0x10, if not, break
				break;
			cantx_buffer_length = 0;
			for(ix = 0;ix < NUM_VNET_CHANNELS;ix++){
				can_buffer[0] = 0x00;
				can_buffer[1] = 0x00;
				can_buffer[2] = (unsigned char)(vnet_config_flash[ix].serial_number >> 8);	//device serial high byte	0x27
				can_buffer[3] = (unsigned char)(vnet_config_flash[ix].serial_number & 0xFF);	//device serial low byte (10,00) 0x15
				can_buffer[4] = (unsigned char)(VNET_TYPE & 0xFF);	//device type low byte
				can_buffer[5] = (unsigned char)(VNET_TYPE >> 8);		//device type high byte
				can_buffer[6] = (unsigned char)(vnet_config_flash[ix].vnet_id & 0xFF);
				can_buffer[7] = (unsigned char)(vnet_config_flash[ix].vnet_id >> 8);
				can_buffer[8] = (unsigned char)(vnet_config_flash[ix].record_rate & 0xFF);
				can_buffer[9] = (unsigned char)(vnet_config_flash[ix].record_rate >> 8);
				can_buffer[10] = vnet_config_flash[ix].data_exp;
				can_buffer[11] = 0x01;
				can_buffer[12] = 0x88;
				can_buffer[13] = 0x00;

				if(cantx_buffer_length < 15){	//If there is room in the FIFO buffer
					CANTIER = 0;	//Disable CanTxISR
					(void)memcpy(&cantx_buffer[(cantx_buffer_index + cantx_buffer_length) & 0x0F][0],&can_buffer[0],14);
					cantx_buffer_length++;
					if(CANTFLG == 0)	//MSCAN buffers are full
						CANTIER = 0x07;	//Enable all MSCAN transmit interrupts
					else{	//Otherwise, trigger next logical transmit interrupt
						CANTBSEL = CANTFLG;
						CANTIER = CANTBSEL;
					}	//end if(CANTFLG == 0)
				}	//end if(cantx_buffer_length < 15)			
			}	//end for(ix = 0;ix < NUM_VNET_CHANNELS;ix++)
			break;
		case 3:	//CAN ID (00 00 07 F8) Config General
			message_serial++;	//increment config message message serial
			if(canrx_buffer[1] == 0x40)	//lookup serial_number of connect message
				for(connected_channel_index=0;connected_channel_index < NUM_VNET_CHANNELS;connected_channel_index++)
					if(canrx_buffer[3] == (unsigned char)(vnet_config_flash[connected_channel_index].serial_number & 0xFF))
						if(canrx_buffer[4] == (unsigned char)(vnet_config_flash[connected_channel_index].serial_number >> 8)){
							message_serial = canrx_buffer[2];
							break;
						}
			//if we have a valid connected channel, and message sequence is correct
			if((connected_channel_index < NUM_VNET_CHANNELS) & (canrx_buffer[2] == message_serial)){
				switch(canrx_buffer[1]){
				case 0x40:
					can_buffer[0] = 0x00;
					can_buffer[1] = 0x00;
					can_buffer[2] = 0x07;
					can_buffer[3] = 0xF9;
					can_buffer[4] = 0xFF;
					can_buffer[5] = 0x00;
					can_buffer[6] = message_serial;
					can_buffer[7] = vnet_config_flash[connected_channel_index].serial_number >> 8;
					can_buffer[8] = 0x00;
					can_buffer[9] = 0x00;
					can_buffer[10] = 0x00;
					can_buffer[11] = 0x00;
					can_buffer[12] = 0x08;
					can_buffer[13] = 0x00;
					break;
				case 0x21:	//General config by PID
					//parse PID from message
					PID = (canrx_buffer[4] & ~0x10) << 8;
					PID |= canrx_buffer[3];
					//setup response message
					can_buffer[0] = 0x00;
					can_buffer[1] = 0x00;
					can_buffer[2] = 0x07;
					can_buffer[3] = 0xF9;
					can_buffer[4] = 0xFF;
					can_buffer[5] = 0x00;
					can_buffer[6] = message_serial;
					can_buffer[7] = 0x00;
					switch(PID){	//parse by PID number
					case 0x4001:	//PID 0x4001 Copy user table to temp table
						tableInit(4);

						can_buffer[8] = 0x00;
						can_buffer[9] = 0x00;
						can_buffer[10] = 0x00;
						can_buffer[11] = 0x00;
						can_buffer[12] = 0x08;
						break;
					case 0x4002:	//PID 0x4002 Reboot/Disconnect
						can_buffer[8] = 0x00;
						can_buffer[9] = 0x00;
						can_buffer[10] = 0x00;
						can_buffer[11] = 0x00;
						can_buffer[12] = 0x08;
						break;
					case 0x4042:	//PID 0x4042 Reboot/Disconnect
						can_buffer[8] = 0x00;
						can_buffer[9] = 0x00;
						can_buffer[10] = 0x00;
						can_buffer[11] = 0x00;
						can_buffer[12] = 0x08;
						break;
					case 0x2003:	//Product version
						can_buffer[8] = 0x03;
						can_buffer[9] = 0x00;
						can_buffer[10] = 0x00;
						can_buffer[11] = 0x00;
						can_buffer[12] = 0x08;
						break;
					case 0x0022:	//Sub channel number
						can_buffer[8] = connected_channel_index;
						can_buffer[9] = 0x00;
						can_buffer[10] = 0x00;
						can_buffer[11] = 0x00;
						can_buffer[12] = 0x08;
						break;
					case 0x0020:	//Sensor Type
						can_buffer[8] = vnet_config_ram[connected_channel_index].sensor_type;
						can_buffer[9] = 0x00;
						can_buffer[10] = 0x00;
						can_buffer[11] = 0x00;
						can_buffer[12] = 0x08;
						break;
					case 0x0001:	//vNet ID
						can_buffer[8] = vnet_config_ram[connected_channel_index].vnet_id & 0xFF;
						can_buffer[9] = vnet_config_ram[connected_channel_index].vnet_id >> 8;
						can_buffer[10] = 0x00;
						can_buffer[11] = 0x00;
						can_buffer[12] = 0x08;
						break;
					case 0x0021:	//Logger Sample Rate
						can_buffer[8] = vnet_config_ram[connected_channel_index].record_rate & 0xFF;
						can_buffer[9] = vnet_config_ram[connected_channel_index].record_rate >> 8;
						can_buffer[10] = 0x00;
						can_buffer[11] = 0x00;
						can_buffer[12] = 0x08;
						break;
					case 0x0034:	//ChartMin
						can_buffer[7] = 0x00;
						can_buffer[8] = vnet_config_ram[connected_channel_index].chart_min & 0xFF;
						can_buffer[9] = vnet_config_ram[connected_channel_index].chart_min >> 8;
						can_buffer[10] = vnet_config_ram[connected_channel_index].chart_min_exp;
						can_buffer[11] = 0x00;
						can_buffer[12] = 0x08;
						break;
					case 0x0035:	//ChartMax
						can_buffer[8] = vnet_config_ram[connected_channel_index].chart_max & 0xFF;
						can_buffer[9] = vnet_config_ram[connected_channel_index].chart_max >> 8;
						can_buffer[10] = vnet_config_ram[connected_channel_index].chart_max_exp;
						can_buffer[11] = 0x00;
						can_buffer[12] = 0x08;
						break;
					case 0x0037:	//Digits before
						can_buffer[8] = vnet_config_ram[connected_channel_index].digits_before & 0xFF;
						can_buffer[9] = 0x00;
						can_buffer[10] = 0x00;
						can_buffer[11] = 0x00;
						can_buffer[12] = 0x08;
						break;
					case 0x0038:	//Digits after
						can_buffer[8] = vnet_config_ram[connected_channel_index].digits_after & 0xFF;
						can_buffer[9] = 0x00;
						can_buffer[10] = 0x00;
						can_buffer[11] = 0x00;
						can_buffer[12] = 0x08;
						break;
					default:	//unknown PID, stop config
						connected_channel_index = 0xFF;	//set connected channel to an invalid index
						break;
					}	//end switch(PID)
					can_buffer[13] = 0x00;
					break;
				case 0x07:	//Config disconnect message
					if(canrx_buffer[3] == 0x01)
						if(canrx_buffer[4] == 0x00)
							if(canrx_buffer[5] == (vnet_config_ram[connected_channel_index].serial_number & 0xFF))
								if(canrx_buffer[6] == (vnet_config_ram[connected_channel_index].serial_number >> 8)){
									//burn ram config to flash here.... I think....
									can_buffer[0] = 0x00;
									can_buffer[1] = 0x00;
									can_buffer[2] = 0x07;
									can_buffer[3] = 0xF9;
									can_buffer[4] = 0xFF;
									can_buffer[5] = 0x00;
									can_buffer[6] = message_serial;
									can_buffer[7] = 0x00;
									can_buffer[8] = (vnet_config_ram[connected_channel_index].serial_number & 0xFF);
									can_buffer[9] = (vnet_config_ram[connected_channel_index].serial_number >> 8);
									can_buffer[10] = 0x00;
									can_buffer[11] = 0x00;
									can_buffer[12] = 0x08;
									can_buffer[13] = 0x00;
								} //end if(canrx_buffer[6] == (vnet_config_ram[connected_channel_index].serial_number >> 8))
					connected_channel_index = 0xFF;
					break;
				default:	//unknown config message, stop config
					connected_channel_index = 0xFF;	//set connected channel to an invalid index
					break;
				}	//end switch(canrx_buffer[1])

				if(can_buffer[12]){	//flag to enable transmit
					CANTIER = 0;	//Disable CanTxISR
					cantx_buffer_length = 0;
					(void)memcpy(&cantx_buffer[(cantx_buffer_index + cantx_buffer_length) & 0x0F][0],&can_buffer[0],14);
					cantx_buffer_length++;
					if(CANTFLG == 0)	//MSCAN buffers are full
						CANTIER = 0x07;	//Enable all MSCAN transmit interrupts
					else{	//Otherwise, trigger next logical transmit interrupt
						CANTBSEL = CANTFLG;
						CANTIER = CANTBSEL;
					}	//end if(CANTFLG == 0)
				}	//if(can_buffer[12])
			}	//end if((connected_channel_index < NUM_VNET_CHANNELS) & (canrx_buffer[2] == message_serial))
			else
				connected_channel_index = 0xFF;
		}	//end switch(canrx_bufer[0])
		canrx_buffer_length = 0;
	}//end if(canrx_buffer_length)

	/*if((lmms - adc_lmms) > 78)  {          // every 10 ms (78 x .128 ms clk)
      adc_lmms = lmms;
      // read 10-bit ADC results, convert to engineering units and filter
      next_adc++;
      if(next_adc > 7)next_adc = 1;
      switch_adc(next_adc);
    }*/
    
/***************************************************************************
**
** Check whether to burn flash
**
**************************************************************************/
BURN_FLASH:
    if (burn_flag) {
      // burn flash 512 byte(256 word) sector(s)
      fburner(tableWordFlash(burn_idx, 0), tableWordRam(burn_idx, 0), tableWords(burn_idx));
      if(burn_flag >= tableWords(burn_idx))  {
        burn_flag = 0;
        flocker = 0;
      }
      else
        burn_flag++;
    }

/***************************************************************************
**
**  Check for serial receiver timeout
**
**************************************************************************/
    if(lmms > rcv_timeout)  {
      txmode = 0;    // break out of current receive sequence
      rcv_timeout = 0xFFFFFFFF;
    }
  
/***************************************************************************
**
**  Check for CAN reset
**
**************************************************************************/
	  if(can_reset)  {
		  /* Re-initialize CAN comms */
		  CanInit();
		  can_reset = 0;
	  }
	background_counter++;	//counts background loops per second
    
  }     //  END Main while(1) Loop
}

//#pragma CODE_SEG DEFAULT        
//#pragma CODE_SEG NON_BANKED
INTERRUPT void ISR_Foreground_Loop(void){
	static unsigned int second;
	static unsigned int vnet_timer;
	static unsigned int run_timer;
	CRGFLG = 0x80;  // clear RTI interrupt flag
	second++;
	lmms++;   // free running clock(.128 ms tics) good for ~ 150 hrs
	mms++;    // in .128 ms tics - reset every 8 tics = 1.024 ms

	if (second > 7812){
		second = 0;
		outpc.seconds++;
	}

	// check mms to generate other clocks
	if(mms < 8)goto CLK_DONE;
	mms = 0;

	//This code is not 100% solid.  Possibility exists for pulse counter to zero out, and another new pulse come in
	//before this executes, and zeros the run_timer.  This would result in outpc.oss_pulse_counter equaling zero,
	//but outpc.run_timer would not be zero, and synchronized with it.
	if (outpc.oss_pulse_count == 0)
		run_timer = 0;
	else if(run_timer < 65535){
		run_timer++;
		outpc.run_timer = (unsigned int)(((unsigned long)run_timer << 10) / 1000);
	}

	vnet_timer++;
	if (vnet_timer > VNET_INTERVAL){
		transmit_semaphore = 0xCC;	//Set flag to allow the background loop to transmit all real time data
		vnet_timer = 0;
	}
	if (((PTIT & 0x10) == 0) && (second_retard_timer < 65535))	//First retard on, and not overflowing timer
		second_retard_timer++;
	if(retard_off_timer > 0)
		retard_off_timer--;

	millisec++;     // actually 1.024 ms
	if(millisec > 97) {	// .10035 sec clocks
		outpc.debug_1 = background_counter;
		background_counter = 0;
		millisec = 0;
	}
   
CLK_DONE:
	return;
}

INTERRUPT void ISR_IGNIn(void){
	const unsigned char rev_limit[90] ={1,1,1,1,1,1,1,1,1,
										0,1,1,1,1,1,1,1,1,
										0,1,1,1,1,0,1,1,1,
										0,1,1,0,1,1,0,1,1,
										0,1,1,0,1,0,1,1,0,
										0,1,0,1,0,1,0,1,0,
										0,1,0,0,1,0,0,1,0,
										0,0,1,0,0,0,1,0,0,
										0,0,0,0,1,0,0,0,0,
										0,0,0,0,0,0,0,0,0};
	static char cyl_count;
	static unsigned int old_TC2;
	unsigned long interval;

//	PTT &= ~0x20;	//immediately discharge the coil.  if this is a chargng cycle, it will be turned on below.
	if ((PTIT & 0x04) == 0x04){	//Falling edge just occured
		if (outpc.ign_pulse_count < 65535)
			outpc.ign_pulse_count++;

		if (outpc.ign_pulse_count > 1){
			interval = (((unsigned long)TC2_overflow << 16) + TC2) - old_TC2;

		old_TC2 = TC2;
		TC2_overflow = 0;

		outpc.rpm += (int)((((long)(rpm_divisor / interval) - (long)outpc.rpm) * (long)inpram.rpm_LF) >> 15);
		//outpc.rpm = rpm_divisor / interval;
		}
	}
/*	else{					//Rising edge just occured
		if ((outpc.rpm < 2000) || ((outpc.rev_limit_level == 0) && (outpc.torque_limit_level == 0))){	//RPM below which all limiting is inhibited
			PTT |= 0x20;	//switch the coil into a charging state, and release retard
			cyl_count = 0;
		}
		else if (outpc.torque_limit_level > outpc.rev_limit_level){	//determine which is a greater priority rev/torque limiting
			if (rev_limit[outpc.torque_limit_level * 9 + cyl_count] == 1)	//check if event is allowed based on torquelimit condition
				PTT |= 0x20;	//charge coil
			cyl_count++;
		}
		else{
			if (rev_limit[outpc.rev_limit_level * 9 + cyl_count] == 1)	//check if event is allowed based on revlimit condition
				PTT |= 0x20;
			cyl_count++;
		}

		if (cyl_count > 8)
			cyl_count=0;
	}
*/	TFLG1 = 0x04;	//clear the interrupt flag for this timer channel
	return;
}

//#pragma CODE_SEG DEFAULT        
//#pragma CODE_SEG NON_BANKED        
INTERRUPT void ISR_OSSIn(void){
	unsigned long interval;
	static unsigned int old_TC0, old_oss, skipped_teeth;

	if (outpc.oss_pulse_count < 65535)
		outpc.oss_pulse_count++;

	if (outpc.oss_pulse_count == 1){	//Initialize the tachometer, so first measurement is accurate
		old_TC0 = TC0;
		TC0_overflow = 0;
		skipped_teeth = 1;
	}
	else{
		interval = (TC0 + ((unsigned long)TC0_overflow << 16)) - old_TC0;

		if (interval < 3000)	//mechanism to skip teeth at high speeds to minimize overhead, and improve granularity
			skipped_teeth++;
		else{
			old_oss = outpc.oss;
			outpc.oss += (int)((((long)((oss_divisor * (unsigned long)skipped_teeth) / interval) - (long)outpc.oss) * (long)inpram.oss_LF) >> 15);
			//outpc.oss = ((oss_divisor * (unsigned long)skipped_teeth) / interval);
			old_TC0 = TC0;
			TC0_overflow = 0;
			skipped_teeth = 1;
		}
	}
	TFLG1 = 0x01;	//clear the interrupt flag for this channel
	return;
}

//#pragma CODE_SEG DEFAULT        
//#pragma CODE_SEG NON_BANKED        
INTERRUPT void ISR_TimerOverflow(void){
	//These counters will overflow once every 0.043690666666 seconds
	if (TC0_overflow < 2)
		TC0_overflow++;
	else{
		outpc.oss = 0;
		outpc.run_timer = 0;
		outpc.oss_pulse_count = 0;
		outpc.torque_limit_level = 0;
	}

	if (TC2_overflow < 5)
		TC2_overflow++;
	else{	//1 overflow = 343.3 RPM assuming 4PPR 57.2 rpm
//		PTT &= ~0x20;	//Make sure coil is off/firing polarity
//		PTT |= 0x20;	//Turn off second retard
//		PTT |= 0x10;	//Turn off first retard
		outpc.rev_limit_level = 0;
		outpc.ign_pulse_count = 0;
		outpc.rpm = 0;
	}

	TFLG2 = 0x80;	//clear interrupt flag for this channel
	return;
}
unsigned int fox_uint(unsigned int x, unsigned char pairs,unsigned int* function[2][pairs]){
	unsigned char i;
	unsigned int y;
	unsigned long x_high, x_low, y_high, y_low, percent;

	for (i=0;i < pairs;i++){
		if ((unsigned int)function[0][i] >= x)	//>= prevents divide by zero below
			break;
	}
	if (i == 0){	//X is below the range of the function lookup table, clip to the end value
		y = (unsigned int)function[1][0];
	}
	else if (i < pairs){	//X is within the range of the table, perform lookup
		x_high = (unsigned int)function[0][i];	//GCC kept throwing "invalid assignment to binary -" if I tried to using fuction directly, so I indirected it with these variables
		x_low = (unsigned int)function[0][i -1];
		y_high = (unsigned int)function[1][i];
		y_low = (unsigned int)function[1][i - 1];

		percent = (((unsigned long)x - x_low) << 15) / (x_high - x_low);
		y = (unsigned int)((((y_high - y_low) * percent) >> 15) + y_low);
	}
	else{	//X is above the range of the function lookup table, clip to the end value
		y = (unsigned int)function[1][pairs - 1];
	}
	return y;
}

void accel_integrator(unsigned int sample_time){
	
	//sample_time seconds x 1000
	//integrated_acceleration ft/s/s x 10
	//integrated_velocity ft/s x 1000
	//integrated_position ft x 1000
	//The equations have been tuned to perform accurately with sample times 20 - 100.  Shorter times can add up
	//undue error due to results of the equations being quantized to the nearest sample unit.  For position, this
	//unit is 0.012".  Assuming the worst, position will accumulate 0.012" error with every sample.  Longer times
	//can cause t3 to overflow, or (C * t3) to overflow.  Additionally, longer sample times reduce the resolution
	//of c, which is made worse by its increased significance, due to the longer time.
	//The v300 Accelerometer resolution is +-6.5G's@10bits.  This works out to +-0.012695 G's, or +-0.4084 ft/s/s.
	//This is well above the resolution of the equations at 0.1 ft/s/s.  

	static long b;
	long c, t2, t3;

	if(sample_time == 0){
		outpc.integrated_position = 0;
		outpc.integrated_velocity = 0;
	}
	else{
		t2 = sample_time * sample_time;
		t3 = (t2 * sample_time) / 6;
		t2 = t2 >> 1;

		//	a = b + ct
		c = ((outpc.integrated_accel - b) << 10) / (long)sample_time;

		//	x(t) = x(0) + (v(0) * t) + (b * t2/2) + (c * t3/6)
		outpc.integrated_position += (((outpc.integrated_velocity * (long)sample_time) + (((b * t2) + ((c * t3) >> 10)) / 10)) / 1000);

		//	v(t) = v(0) + bt + (c * t2/2)
		outpc.integrated_velocity += (((b * (long)sample_time)  + ((c * t2) >> 10)) / 10);
	}
	b = outpc.integrated_accel;
	return;
}


/*void switch_adc(unsigned char next_adc) {
	switch (next_adc){
	case 0:		// skip map (in Timer Int)
	case 3:		// skip tps (in Timer Int)
	case 5:		// do ego1 in ego algorithm
	case 1:
	case 2:
	case 4:
		get_adc(next_adc,next_adc);    // get one channel on each pass
		break;
	case 6:      // do ego2 in ego algorithm
	case 7:      // if knock option selected, do in knock algorithm
	}
}
*/
/*
void get_adc(char chan1, char chan2)  {
char chan;
long adcval;
  
    for (chan = chan1; chan <= chan2; chan++)  {
    switch(chan)  {
      case 0:
        adcval = (long)inpram.map0 + 
          ((long)(inpram.mapmax - inpram.map0) * ATD0DR0) / 1023; // kPa x 10
        if(first_adc)
          outpc.map = (short)adcval;
        else
          outpc.map += (short)((inpram.mapLF * (adcval - outpc.map)) / 100);
        break;
      case 1:
        adcval = (long)inpram.mat0 + 
          ((long)inpram.matmult * matfactor_table[ATD0DR1]) / 100; // deg F or C x 10
        if(first_adc)
          outpc.mat = (short)adcval;
        else
          outpc.mat += (short)((inpram.adcLF * (adcval - outpc.mat)) / 100);
        break;
      case 2:
        adcval = (long)inpram.clt0 + 
          ((long)inpram.cltmult * cltfactor_table[ATD0DR2]) / 100; // deg F or C x 10
        if(first_adc)
          outpc.clt = (short)adcval;
        else
          outpc.clt += (short)((inpram.adcLF * (adcval - outpc.clt)) / 100);
        break;
      case 3:
        adcval = (ATD0DR3 - (long)inpram.tps0) * 1000 / 
          (inpram.tpsmax - inpram.tps0);                           // % x 10            
        if(first_adc)
          outpc.tps = (short)adcval;
        else
          outpc.tps += (short)((inpram.tpsLF * (adcval - outpc.tps)) / 100);
        break;
      case 4:
        adcval = (long)inpram.batt0 + 
          ((long)(inpram.battmax - inpram.batt0) * ATD0DR4) / 1023; // V x 10
        if(first_adc)
          outpc.batt = (short)adcval;
        else
          outpc.batt += (short)((inpram.adcLF * (adcval - outpc.batt)) / 100);
        break;
      case 5:
        if(inpram.EgoOption >= 1)  {
          // check if sensor bad (near limits)
          if((ATD0DR5 < 3) || (ATD0DR5 > 1020))
            bad_ego_flag |= 0x01;
          else
            bad_ego_flag &= ~0x01;
          adcval = (long)inpram.ego0 + 
            ((long)inpram.egomult * egofactor_table[ATD0DR5]) / 100; // afr x 10
          if(first_adc)
            outpc.ego1 = (short)adcval;
          else
            outpc.ego1 += (short)((inpram.egoLF * (adcval - outpc.ego1)) / 100);
        }
        break;
      case 6:
        if(inpram.BaroOption == 2)  {
          adcval = (long)inpram.baro0 + 
            ((long)(inpram.baromax - inpram.baro0) * ATD0DR6) / 1023; // kPa x 10
          if(first_adc)
            outpc.baro = (short)adcval;
          else
            outpc.baro += (short)((inpram.adcLF * (adcval - outpc.baro)) / 100);
          outpc.ego2 = outpc.ego1; 
        } 
        else if((inpram.EgoOption == 2) || (inpram.EgoOption == 4))  {
          // check if sensor bad (near limits)
          if((ATD0DR6 < 3) || (ATD0DR6 > 1020))
            bad_ego_flag |= 0x02;
          else
            bad_ego_flag &= ~0x02;
          adcval = (long)inpram.ego0 + 
            ((long)inpram.egomult * egofactor_table[ATD0DR6]) / 100; // afr x 10
          if(first_adc)
            outpc.ego2 = (short)adcval;
          else
            outpc.ego2 += (short)((inpram.egoLF * (adcval - outpc.ego2)) / 100);
        }
        else  {
          outpc.ego2 = outpc.ego1; 
        }
        break;
      case 7:
        adcval = (long)inpram.knock0 + 
          ((long)(inpram.knockmax - inpram.knock0) * ATD0DR7) / 1023; // V x 100
        if(first_adc)
          outpc.knock = (short)adcval;
        else
          outpc.knock += (short)((inpram.knkLF * (adcval - outpc.knock)) / 100);
        break;
      default:
        break;
    }			 // end of switch
  }				 // end of for loop
  return;
}
*/
/*
#pragma CODE_SEG  NON_BANKED

int intrp_1dctable(int x, unsigned char n, int * x_table, char sgn, 
        unsigned char * z_table)  {
  int ix;
  long interp, interp3;
  // bound input arguments
  if(x > x_table[n-1])  {
    if(!sgn)
      return((int)z_table[n -1]);
    else
      return((int)((char)z_table[n -1]));
  }
  if(x < x_table[0])  {
    if(!sgn)
      return((int)z_table[0]);
    else
      return((int)((char)z_table[0]));
  }
  for(ix = n - 2; ix > -1; ix--)  { 
  	if(x > x_table[ix])  {
   		break;
  	}
  }
  if(ix < 0)ix = 0;
 
  interp =	x_table[ix + 1] - x_table[ix];
  if(interp != 0)  {
    interp3 = (x - x_table[ix]);
    interp3 = (100 * interp3);
    interp = interp3 / interp;
  }
  if(!sgn)
    return((int)(z_table[ix] +
	    interp * (z_table[ix+1] - z_table[ix])/ 100));
	else
    return((int)((char)z_table[ix] +
	    interp * ((char)z_table[ix+1] - (char)z_table[ix])/ 100));
}

int intrp_2dctable(unsigned int x, int y, unsigned char nx, unsigned char ny,
  unsigned int * x_table, int * y_table, unsigned char * z_table)  {
  int ix,jx;
  long interp1, interp2, interp3;
  // bound input arguments
  if(x > x_table[nx-1])x = x_table[nx-1];
  else if(x < x_table[0])x = x_table[0];
  if(y > y_table[ny-1])y = y_table[ny-1];
  else if(y < y_table[0])y = y_table[0];
  // Find bounding indices in table
  for(ix = ny - 2; ix > -1; ix--)  {  // Start w highest index
	//  because will generally have least time for calculations at hi y
  	if(y > y_table[ix])  {
   		break;
  	}
  }
  if(ix < 0)ix = 0;
  for(jx = nx - 2; jx > -1; jx--)  {  // Start w highest index
	// because will generally have least time for calculations at hi x
	  if(x > x_table[jx])  {
	    break;
    }
  }
  if(jx < 0)jx = 0;
  // do 2D interpolate
  interp1 = y_table[ix + 1] - y_table[ix];
  if(interp1 != 0)  {
    interp3 = (y - y_table[ix]); 
    interp3 = (100 * interp3); 
    interp1 = interp3 / interp1; 
  }
  interp2 =	x_table[jx + 1] - x_table[jx];
  if(interp2 != 0)  {
    interp3 = (x - x_table[jx]); 
    interp3 = (100 * interp3); 
    interp2 = interp3 / interp2; 
  }
  return((int)(((100 - interp1) * (100 - interp2) * z_table[ix*nx+jx]
	  + interp1 * (100 - interp2) * z_table[(ix+1)*nx+jx]
	  + interp2 * (100 - interp1) * z_table[ix*nx+jx+1]
	  + interp1 * interp2 * z_table[(ix+1)*nx+jx+1]) / 10000));
}
*/

/**************************************************************************
**
** SCI Communications
**
** Communications is established when the PC communications program sends
** a command character - the particular character sets the mode:
**
** "a" = send all of the realtime display variables (outpc structure) via txport.
** "w"+<offset lsb>+<offset msb>+<nobytes>+<newbytes> = 
**    receive updated data parameter(s) and write into offset location
**    relative to start of data block
** "e" = same as "w" above, followed by "r" below to echo back value
** "r"+<offset lsb>+<offset msb>+<nobytes>+<newbytes> = read and
**    send back value of a data parameter or block in offset location
** "y" = verify inpram data block = inpflash data block, return no. bytes different.
** "b" = jump to flash burner routine and burn a ram data block into a flash 
**    data block.
** "c" = Test communications - echo back Seconds
** "Q" = Send over Embedded Code Revision Number
** "S" = Send program title.
**
**************************************************************************/
//#pragma CODE_SEG  NON_BANKED

INTERRUPT void ISR_SCI_Comm(void)  {
  char dummy;
  int ix;
  static int vfy_fail,rd_wr;
  static unsigned char CANid,ibuf;
  static unsigned char next_txmode;
  static unsigned int tble_word,ntword;
  
#define getCANid             40
#define getTableId           41
#define setBurningParameters 99
  
  // if RDRF register not set, => transmit interrupt
  if(!(SCI0SR1 & 0x20))goto XMT_INT;

// Receive Interrupt
  // Clear the RDRF bit by reading SCISR1 register (done above), then read data
  //  (in SCIDRL reg).
  // Check if we are receiving new input parameter update
txgoal = 0;
rcv_timeout = 0xFFFFFFFF;

switch(txmode)  {

case 0:

  switch(SCI0DRL)  {
    case 'a':				 // send back all real time ram output variables
  next_txmode = 1;
  txmode = getCANid;
  break;

    case 'w':		  // receive new ram input data and write into offset location
  next_txmode = 5;
  txmode = getCANid;
  rd_wr = 1;
  break;

    case 'e':		  // same as 'w', but verify by echoing back value
  next_txmode = 5;
  txmode = getCANid;
  rd_wr = 2;
  break;

    case 'r':		  // read and send back ram input data from offset location
  next_txmode = 5;
  txmode = getCANid;
  rd_wr = 0;
  break;

    case 'y':      // Verify that a flash data block matches a
  next_txmode = 2; //  corresponding ram data block
  txmode = getCANid;
  break;
   	
    case 'b':        // burn a block of ram input values into flash
  next_txmode = setBurningParameters;
  txmode      = getCANid;
  break;

    case 't':        // update a flash table with following serial data
  txmode = 20;
  break;

    case '!':        // start receiving reinit/reboot command
  txmode = 30;
  break;

    case 'c':        // send back seconds to test comms
  txcnt = 0;         
  txmode = 1;
  txgoal = 2;        // seconds is 1st 2 bytes of outpc structure
  tble_idx = 1;
  txbuf.seconds = outpc.seconds;
  SCI0DRL = *(char *)&txbuf;
  SCI0CR2 |= 0x88;    // xmit enable & xmit interrupt enable
  break;

    case 'Q':         // send code rev no.
  txcnt = 0;
  txmode = 4;
  txgoal = 20;
  SCI0DRL = RevNum[0]; 
  SCI0CR2 |= 0x88;    // xmit enable & xmit interrupt enable
  break;

    case 'S':         // send program title
  txcnt = 0;
  txmode = 5;
  txgoal = 32;
  SCI0DRL = Signature[0]; 
  SCI0CR2 |= 0x88;    // xmit enable & xmit interrupt enable
  break;

    default:
  break;
  }     // End of switch for received command
    break;

case getCANid: // Get CAN id for current command.
   CANid = SCI0DRL;
//   if (CANid < NO_CANBOARDS && inpram.board_id_type[CANid] != 0)
//     txmode = getTableId;
//   else
     txmode = getTableId;
   break;
case getTableId: // Get table id for current command.
   tble_idx = SCI0DRL;
   if (tble_idx >= NO_TBLES)
     txmode = 0;
   else
     txmode = next_txmode;
   next_txmode = 0;
   if (txmode == 1) { 
     txcnt = 0;
     txgoal = tableBytes(tble_idx);
     // load all output variables into txbuf to avoid incoherent word data
	 outpc.flag = PTIT;
     (void)memcpy(tableByteRam(tble_idx, 0),&outpc,sizeof(outpc));
     SCI0DRL = *tableByteRam(tble_idx, 0);
     SCI0CR2 |= 0x88;    // xmit enable & xmit interrupt enable
   }
   else if (txmode == setBurningParameters && flocker == 0) { // Burn command
     flocker     = 0xCC;    // set semaphore to prevent burning flash thru runaway code
     burn_flag   = 1;
     burn_idx = tble_idx;
     txmode      = 0;			 // handle burning in main loop
   } 
   else if(txmode == 2)  {
     vfy_fail = 0;
     //save_page = PPAGE;
     //PPAGE = 0x3C;
     for(ix = 0; ix < tables[tble_idx].n_bytes; ix++)  {
       if(*tableByteFlash(tble_idx,ix) != *tableByteRam(tble_idx,ix))
       vfy_fail++;
     }
     //PPAGE = save_page;     
     txcnt = 0;
     txgoal = 2;
     SCI0DRL = *(char *)&vfy_fail;   
     SCI0CR2 |= 0x88;    // xmit enable & xmit interrupt enable
   }
   break;

case 5:
  	rxoffset = (SCI0DRL << 8); // byte offset(msb) from start of inpram
  	txmode++;
  	break;

case 6:
  	rxoffset |= SCI0DRL;    // byte offset(lsb) from start of inpram
  	txmode++;
  	break;
  	
case 7:
  	rxnbytes = (SCI0DRL << 8); 	// no. bytes (msb)
  	txmode++;
  	break;

case 8:
  	rxnbytes |= SCI0DRL;		// no. bytes (lsb)
  	rxcnt = 0;
  	if(rd_wr == 0)  {		    // read & send back input data
  	  txcnt = 0;
  	  txmode = 3;
  	  txgoal = rxnbytes;
  	  SCI0DRL = *tableByteRam(tble_idx, rxoffset);
  	  SCI0CR2 |= 0x88;        // xmit enable & xmit interrupt enable
  	}
  	else  {                   // write data to input data block
  	  // buffer to maintain coherence of inputs while awaiting serial bytes
  	  if((rxnbytes > 1) && (rxnbytes < sizeof(txbuf)))
  	    ibuf = 1;
  	  else
  	    ibuf = 0; 
  	  txmode++;
  	}
  	break;

case 9:
  	// Check for data overrun.  Note: this
  	// still not bullet proof because could write half the bytes
  	// then have overrun; need to store and write all at once if
  	// all is valid.
  	if((SCI0SR1 & 0x08) == 0)  {   // none yet- write data.
  	  if(!ibuf)
  	    *tableByteRam(tble_idx, rxoffset + rxcnt) = SCI0DRL;
  	  else
  	    *((char *)&txbuf + rxcnt) = SCI0DRL;
  	}
  	rxcnt++;
  	if(rxcnt >= rxnbytes)  {
  	  if(ibuf)
  	    (void)memcpy(tableByteRam(tble_idx, rxoffset),(char *)&txbuf,rxnbytes);
  	  dummy = SCI0DRL;      // clear OR bit (read sci0sr1(done), sci0drl)
  	  if(rd_wr == 1)
  	    txmode = 0;         // done receiving/writing all the data
  	  else  {
  	    txcnt = 0;				// send back data just written
  	    txmode = 3;				//	 for verification
  	    txgoal = rxnbytes;
  	    SCI0DRL = *tableByteRam(tble_idx, rxoffset);
  	    SCI0CR2 |= 0x88;    // xmit enable & xmit interrupt enable
  	  }
  	}
  	break;

case 20:	 // d/load & burn tables - *** Do while engine NOT turning ***
  	tble_idx = SCI0DRL;    // type of table
  	ntword = 0;
  	flocker = 0xCC; // set semaphore to prevent burning flash thru runaway code
  	//save_page = PPAGE;
  	//PPAGE = 0x3C;
	tburner(0, tableWordFlash(tble_idx, 0), tableWords(tble_idx));   // erase table
  	//PPAGE = save_page;
  	flocker = 0;
  	txmode++;
  	break;

case 21:
  	tble_word = (SCI0DRL << 8);    // msb
  	txmode++;
  	break;

case 22:
  tble_word |= SCI0DRL;            // lsb
  flocker = 0xCC; // set semaphore to prevent burning flash thru runaway code
  //save_page = PPAGE;
  //PPAGE = 0x3C;
  tburner(1, tableWordFlash(tble_idx, ntword), tble_word);   // write table word
  //PPAGE = save_page;
  flocker = 0;
  ntword++;
  if(ntword < tableWords(tble_idx))
    txmode = 21;
  else  {
    txmode = 0;
//    ign_reset();
  }
  break;

case 30:
  	if(SCI0DRL == '!')    // receive 2nd ! for reboot signal
  	  txmode++;
  	else if(SCI0DRL == 'x')  {  // received reinit signal (!x)
//  	  reinit_flag = 1;
  	  txmode = 0;
  	}
  	else
  	  txmode = 0;
  	break;

case 31:
  	if(SCI0DRL == 'x')  {   // received complete reboot signal(!!x)
  	    // go to start of bootload pgm
  	    //asm (jmp $F800);
  	    (void)reboot();
  	}
  	else if(SCI0DRL == '!')  {   // received complete reload signal(!!!)
  	    SCI0CR1 = 0x00;
  	    SCI0CR2 = 0x00;      // disable Tx, Rx
  	    // go to start of monitor code
  	    //asm (jmp $F842);
  	    (void)monitor();
  	}
  	txmode = 0;
  	break;

default:
  dummy = SCI0DRL;   // dummy read of reg to clear int flag
  break;

  }     // End of case switch for received data
    
  if((txgoal == 0) && (txmode > 0))
	rcv_timeout = lmms + 3906;   // 0.5 sec timeout
  return;

// Transmit Interrupt
XMT_INT:
  // Clear the TDRE bit by reading SCISR1 register (done), then write data
  txcnt++;
  if(txcnt < txgoal)  {
    if(txmode == 3)  {
	  SCI0DRL = *tableByteRam(tble_idx, rxoffset + txcnt);
    }
    else if(txmode == 4)  {
  	  SCI0DRL = RevNum[txcnt];
    }
    else if(txmode == 5)  {
  	  SCI0DRL = Signature[txcnt];
    }
    else if(txmode == 2)  {
  	  SCI0DRL = *((char *)&vfy_fail + txcnt);
    } 
	else  {
      SCI0DRL = *tableByteRam(tble_idx, txcnt);
    }
  } 
  else  {   // done transmitting
    txcnt = 0;
    txgoal = 0;
    txmode = 0;
    SCI0CR2 &= ~0x88;    // xmit disable & xmit interrupt disable
	SCI0CR2 |= 0x24;     // rcv, rcvint re-enable

  } 
  return;
}														 

//#pragma CODE_SEG DEFAULT        

void CanInit(void)
{
	/* Set up CAN communications */
	/* Enable CAN, set Init mode so can change registers */
	CANCTL1 |= 0x80;
	CANCTL0 |= 0x01;
	cantx_buffer_index = 0;
	cantx_buffer_length = 0;

	while(!(CANCTL1 & 0x01));  // make sure in init mode
	
	/* Set Can enable, use IPBusclk (24 MHz),clear rest */
	CANCTL1 = 0xC0;
	//CANBTR0 SJW1 SJW0  BRP5  BRP4  BRP3  BRP2  BRP1  BRP0
	//CANBTR1 SAMP TSG22 TSG21 TSG20 TSG13 TSG12 TSG11 TSG10
	/* Set timing for 250Kbits/ sec */
	CANBTR0 = 0x4B;  /* SJW=2,BR Prescaler=12(24MHz CAN clk) */
	CANBTR1 = 0xB2;  /* SAMP = 1, Set time quanta: tseg2=4,tseg1=3 
	              (8 Tq total including sync seg (=1)) */
	CANIDAC = 0x00;   /* 2 32-bit acceptance filters */
	/* CAN message format:
	 Reg Bits: 7 <-------------------- 0
	  IDR0:    |---var_off(11 bits)----|  (Header bits 28 <-- 21)
	  IDR1:    |cont'd 1 1 --msg type--|  (Header bits 20 <-- 15)
	  IDR2:    |---From ID--|--To ID---|  (Header bits 14 <--  7)
	  IDR3:    |--var_blk-|--spare--rtr|  (Header bits  6 <-- 0,rtr)
	*/  
	/* Set identifier acceptance and mask registers to accept 
	     messages only for can_id or device #15 (=> all devices) */
	/* 1st 32-bit filter bank-to mask filtering, set bit=1 */
	CANIDMR0 = 0xFF;
	CANIDMR1 = 0xFF;
	CANIDMR2 = 0xFF;
	CANIDMR3 = 0xFF;
	CANIDAR0 = 0x38;
	CANIDAR1 = 0x00;
	CANIDAR2 = 0x00;
	CANIDAR3 = 0x00;
	/* 2nd 32-bit filter bank */
	CANIDMR4 = 0xFF;
	CANIDMR5 = 0xFF;
	CANIDMR6 = 0xFF;
	CANIDMR7 = 0xFF;
	CANIDAR4 = 0x38;
	CANIDAR5 = 0x10;
	CANIDAR6 = 0x00;
	CANIDAR7 = 0x00;
	
	/* clear init mode */
	CANCTL0 &= 0xFE;  
	/* wait for synch to bus */
	while(!(CANCTL0 & 0x10));
	
	/* no xmit yet */
	CANTIER = 0x00;
	/* clear RX flag to ready for CAN recv interrupt */
	CANRFLG = 0xC3;
	/* set CAN rcv full interrupt bit */
	CANRIER = 0x01;
	return;
}

//#pragma CODE_SEG  NON_BANKED

INTERRUPT void CanTxIsr(void){

	while((CANTFLG > 0) && (cantx_buffer_length > 0)){	//we have a free MSCAN buffer, and we have data to send
		CANTBSEL = CANTFLG;    // select MSCAN xmit buffer -CANTFLG(bitmap of what buffers are empty)

		//format the header data and load into MSCAN buffer
		if((cantx_buffer[cantx_buffer_index][12] & 0x80) == 0){	//flag for 11bit header
			CAN_TB0_IDR0 = (cantx_buffer[cantx_buffer_index][2] << 5) | (cantx_buffer[cantx_buffer_index][3] >> 3);
			CAN_TB0_IDR1 = cantx_buffer[cantx_buffer_index][3] << 5;	//RTR and IDE bits 0,0
		}
		else{	//flag for 29bit header
			CAN_TB0_IDR0 = (cantx_buffer[cantx_buffer_index][0] << 3) | (cantx_buffer[cantx_buffer_index][1] >> 5);
			CAN_TB0_IDR1 = (cantx_buffer[cantx_buffer_index][1] << 3) | 0x18;	//Set SRR and IDE bits to 1,1
			CAN_TB0_IDR1 |= (cantx_buffer[cantx_buffer_index][1] << 1) & 0x06;
			CAN_TB0_IDR1 |= cantx_buffer[cantx_buffer_index][2] >> 7;
			CAN_TB0_IDR2 = (cantx_buffer[cantx_buffer_index][2] << 1) | (cantx_buffer[cantx_buffer_index][3] >> 7);
			CAN_TB0_IDR3 = cantx_buffer[cantx_buffer_index][3] << 1;	//Leave RTR bit 0
		}
		//load data, length, priority into MSCAN buffer
		(void)memcpy((char *)(&CAN_TB0_DSR0), &cantx_buffer[cantx_buffer_index][4],10);

		if(cantx_buffer_index > 14)	//increment the FIFO ring index
			cantx_buffer_index = 0;
		else
			cantx_buffer_index++;
		cantx_buffer_length--;	//counter for number of messages in que

		CANTIER |= CANTBSEL;  //flag this buffer for interrupt when empty
		CANTFLG = CANTBSEL;  //flag this buffer as full
	}

	if(cantx_buffer_length == 0)	//if buffer is empty, don't bother generating a interrupt to refill
		CANTIER = 0x00;
	return;
}

INTERRUPT void CanRxIsr(void){

const unsigned char can_rx_addr[NO_CAN_RCV_ADDR][4] = {
	{0x00, 0x00, 0x03, 0xA0},	//Horizontal Accelerometer
	{0x00, 0x00, 0x03, 0xA1},	//Vertical Accelerometer
	{0x00, 0x00, 0x00, 0x08},	//Config broadcast message
	{0x00, 0x00, 0x07, 0xF8}	//Config Messages
};
static unsigned int old_run_timer;

unsigned char can_ID[4];

unsigned int ix, itmp;

    /* CAN Recv Interrupt */
    if((CANRFLG & 0x01) && (inpram.vnet_enable == 1)){
		//Parse CAN identifier
		if(CAN_RB_IDR1 & 0x08){	//29bit ID received
			can_ID[0] = CAN_RB_IDR0 >> 3;
			can_ID[1] = CAN_RB_IDR0 << 5;
			can_ID[1] |= (CAN_RB_IDR1 & 0xE0) >> 3;
			can_ID[1] |= (CAN_RB_IDR1 & 0x06) >> 1;
			can_ID[2] = CAN_RB_IDR1 << 7;
			can_ID[2] |= CAN_RB_IDR2 >> 1;
			can_ID[3] = CAN_RB_IDR2 << 7;
			can_ID[3] |= CAN_RB_IDR3 >> 1;
		}
		else{	//11bit ID received
			can_ID[0] = 0x00;
			can_ID[1] = 0x00;
			can_ID[2] = CAN_RB_IDR0 >> 5;
			can_ID[3] = CAN_RB_IDR0 << 3;
			can_ID[3] |= CAN_RB_IDR1 >> 5;
		}
		//Check the received identifier against known addresses
		for(ix = 0;ix < NO_CAN_RCV_ADDR;ix++)
			if(can_ID[3] == can_rx_addr[ix][3])
				if(can_ID[2] == can_rx_addr[ix][2])
					if(can_ID[1] == can_rx_addr[ix][1])
						if(can_ID[0] == can_rx_addr[ix][0])
							goto parse_switch;
		CANRFLG = 0xC3;
		return;

parse_switch:
		switch(ix){	//switch based on matching index into can_rx_addr array
		case 0:	//Horizontal accel	NOTE! should be updated to include exponent!
			outpc.accel_g = CAN_RB_DSR3 << 8;
			outpc.accel_g |= CAN_RB_DSR2;
			outpc.integrated_accel = ((long)outpc.accel_g * (long)inpram.g_factor) / 102400;
			itmp = outpc.run_timer - old_run_timer;
			old_run_timer = outpc.run_timer;
			accel_integrator(itmp);
			break;
		case 1:	//Vertical accel	NOTE! should be updated to include exponent!
			outpc.vert_g =  CAN_RB_DSR3 << 8;
			outpc.vert_g |= CAN_RB_DSR2;
			break;
		case NO_CAN_RCV_ADDR:	//CAN ID doesn't match any known address
			break;
		default:	//CAN ID matches an address that is not a real time data packet, eg: config message
			if(canrx_buffer_length == 0){
				canrx_buffer[0] = ix;	//index to can_rx_addr matching identifier
				(void)memcpy(&canrx_buffer[1],(char *)(&CAN_RB_DSR0),9);
				canrx_buffer_length = 1;
			}
		}
    }

	if((CANRFLG & ~0x72) != 0)  {
        // Rcv error or overrun on receive
        can_status |= RCV_ERR;
        //reset = 1;
    }
    /* clear RX buf full, err flags to ready for next rcv int */
    /*  (Note: can't clear err count bits) */
    CANRFLG = 0xC3;
	
    return;
}

void can_xsub01(void)  {
  return;
}
/*
int intrp_2ditable(unsigned int x, int y, unsigned char nx, unsigned char ny,
  unsigned int * x_table, int * y_table, int * z_table)  {
  int ix,jx;
  long interp1, interp2, interp3;
  // bound input arguments
  if(x > x_table[nx-1])x = x_table[nx-1];
  else if(x < x_table[0])x = x_table[0];
  if(y > y_table[ny-1])y = y_table[ny-1];
  else if(y < y_table[0])y = y_table[0];
  // Find bounding indices in table
  for(ix = ny - 2; ix > -1; ix--)  {  // Start w highest index
	//  because will generally have least time for calculations at hi y
  	if(y > y_table[ix])  {
   		break;
  	}
  }
  if(ix < 0)ix = 0;
  for(jx = nx - 2; jx > -1; jx--)  {  // Start w highest index
	// because will generally have least time for calculations at hi x
	  if(x > x_table[jx])  {
	    break;
    }
  }
  if(jx < 0)jx = 0;
  // do 2D interpolate
  interp1 = y_table[ix + 1] - y_table[ix];
  if(interp1 != 0)  {
    interp3 = (y - y_table[ix]); 
    interp3 = (100 * interp3); 
    interp1 = interp3 / interp1; 
  }
  interp2 =	x_table[jx + 1] - x_table[jx];
  if(interp2 != 0)  {
    interp3 = (x - x_table[jx]); 
    interp3 = (100 * interp3); 
    interp2 = interp3 / interp2; 
  }
  return((int)(((100 - interp1) * (100 - interp2) * z_table[ix*nx+jx]
	  + interp1 * (100 - interp2) * z_table[(ix+1)*nx+jx]
	  + interp2 * (100 - interp1) * z_table[ix*nx+jx+1]
	  + interp1 * interp2 * z_table[(ix+1)*nx+jx+1]) / 10000));
}
*/
//#pragma CODE_SEG  NON_BANKED
INTERRUPT void UnimplementedISR(void) {
   /* Unimplemented ISRs trap.*/
   return;
}
//#pragma CODE_SEG DEFAULT        


/******************************************************************************
Arguments:	

      progAdr		Pointer to the start of the destination 
			  Flash location to be programmed
      bufferPtr	Pointer to the start of the source data 
      size      Number of WORDS to be programmed
                  
    This function will program non-paged flash only 
******************************************************************************/
void fburner(unsigned int* progAdr, unsigned int* bufferPtr, 
                                          unsigned int no_words)
{
unsigned char sect,no_sectors;
																
  	if(flocker != 0xCC)return; // if not set, we got here unintentionally
  	// sector = 1024 bytes (512 words)
  	if(burn_flag == 1)  {
  	  no_sectors = (unsigned char)(no_words / SECTOR_WORDS);
  	  if(no_words > (no_sectors * SECTOR_WORDS))no_sectors++;
  	  for (sect = 0; sect < no_sectors; sect++)  {  	  
  	    Flash_Erase_Sector(progAdr + sect * SECTOR_WORDS);
  	  }
  	}
  	Flash_Write_Word(progAdr + burn_flag-1, *(bufferPtr + burn_flag-1));
    	
	return;
}

/*  Burn tables in flash one sector at time */
void tburner(char erase_write, unsigned int* addr, unsigned int wrd) 
{
  unsigned char sect,nsect;
      
  	if(flocker != 0xCC)return; // if not set, we got here unintentionally
  	if(erase_write == 0)  { // in this mode word= no words in table
  	  nsect = (unsigned char)(wrd/SECTOR_WORDS);
  	  if(wrd > (nsect*SECTOR_WORDS))nsect++;
  	  for (sect = 0; sect < nsect; sect++)  {
  	    Flash_Erase_Sector(addr + sect * SECTOR_WORDS);
  	  }
  	} else  {
  	  Flash_Write_Word(addr, wrd);
  	}
  	return;
}

//*****************************************************************************
//* Function Name: Flash_Init
//* Description : Initialize Flash NVM for HCS12 by programming
//* FCLKDIV based on passed oscillator frequency, then
//* uprotect the array, and finally ensure PVIOL and
//* ACCERR are cleared by writing to them.
//*
//*****************************************************************************
void Flash_Init(unsigned long oscclk)  {
unsigned char fclk_val;
unsigned long temp;
  /* Next, initialize FCLKDIV register to ensure we can program/erase */
  temp = oscclk;
  if (oscclk >= 12000) {
    fclk_val = oscclk/8/200 - 1; /* FDIV8 set since above 12MHz clock */
    FCLKDIV = FCLKDIV | fclk_val | FDIV8;
  }
  else
  {
    fclk_val = oscclk/8/200 - 1;
    FCLKDIV = FCLKDIV | fclk_val;
  }
  FPROT = 0xFF; /* Disable all protection (only in special modes)*/
  FSTAT = FSTAT | (PVIOL|ACCERR);/* Clear any errors */
  return;
}
void Flash_Erase_Sector(unsigned int *address)  {
  FSTAT = (ACCERR | PVIOL); // clear errors
  (*address) = 0xFFFF;/* Dummy store to page to be erased */
  FCMD = ERASE;
/*********************************************************************
;* Allow final steps in a flash prog/erase command to execute out
;* of RAM while flash is out of the memory map
;* This routine can be used for flash word-program or erase commands
;*
;********************************************************************/
/* Execute the sub out of ram  */
#ifdef GCC_BUILD
__asm__ __volatile__ ("ldaa #0x80");
__asm__ __volatile__ ("jsr  RamBurnPgm");
#else
   asm {
     ldaa  #CBEIF
     jsr  RamBurnPgm
   };
#endif
   return;
}

void Flash_Write_Word(unsigned int *address, unsigned int data)  {
  FSTAT = (ACCERR | PVIOL); // clear errors
  (*address) = data; // Store desired data to address being programmed
  FCMD = PROG; // Store programming command in FCMD
/* Execute the sub out of ram  */
__asm__ __volatile__ ("ldaa #0x80");
__asm__ __volatile__ ("jsr  RamBurnPgm");
  return;
}
