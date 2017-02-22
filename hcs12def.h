/* PORT definitions */
#define PORTA    (*((volatile unsigned char*)(0x0000)))
#define pPORTA   ((volatile unsigned char*)(0x0000))
#define DDRA     (*((volatile unsigned char*)(0x0002))) 
#define PORTB    (*((volatile unsigned char*)(0x0001)))
#define DDRB     (*((volatile unsigned char*)(0x0003)))
#define PORTE    (*((volatile unsigned char*)(0x0008)))
#define DDRE     (*((volatile unsigned char*)(0x0009))) 
#define PUCR     (*((volatile unsigned char*)(0x000C))) 
#define PORTK    (*((volatile unsigned char*)(0x0032)))
#define DDRK     (*((volatile unsigned char*)(0x0033)))
#define PTT      (*((volatile unsigned char*)(0x0240)))
#define pPTT     ((volatile unsigned char*)(0x0240))
#define PTIT     (*((volatile unsigned char*)(0x0241)))
#define DDRT     (*((volatile unsigned char*)(0x0242)))
#define RDRT     (*((volatile unsigned char*)(0x0243)))
#define PERT     (*((volatile unsigned char*)(0x0244)))
#define PPST     (*((volatile unsigned char*)(0x0245)))
#define MODRR    (*((volatile unsigned char*)(0x0247)))
#define PTS      (*((volatile unsigned char*)(0x0248)))
#define DDRS     (*((volatile unsigned char*)(0x024A)))
#define PERS     (*((volatile unsigned char*)(0x024C)))
#define PTM      (*((volatile unsigned char*)(0x0250)))
#define pPTM     ((volatile unsigned char*)(0x0250))
#define DDRM     (*((volatile unsigned char*)(0x0252))) 
#define PTP      (*((volatile unsigned char*)(0x0258)))
#define DDRP     (*((volatile unsigned char*)(0x025A)))
#define PERP     (*((volatile unsigned char*)(0x025C)))
#define PTH      (*((volatile unsigned char*)(0x0260)))
#define DDRH     (*((volatile unsigned char*)(0x0262))) 
#define PTJ      (*((volatile unsigned char*)(0x0268)))
#define DDRJ     (*((volatile unsigned char*)(0x026A)))
#define PERJ     (*((volatile unsigned char*)(0x026C)))

/* PPAGE register */
#define PPAGE    (*((volatile unsigned char*)(0x0030))) 

/* HPRIO Interrupt Reg */
#define HPRIO    (*((volatile unsigned char*)(0x001F))) 

/* CRG/PLL definitions */
#define SYNR     (*((volatile unsigned char*)(0x0034)))
#define REFDV    (*((volatile unsigned char*)(0x0035)))
#define CRGFLG   (*((volatile unsigned char*)(0x0037)))
#define CRGINT   (*((volatile unsigned char*)(0x0038)))
#define CLKSEL   (*((volatile unsigned char*)(0x0039))) 
#define PLLCTL   (*((volatile unsigned char*)(0x003A)))
#define RTICTL   (*((volatile unsigned char*)(0x003B)))  

/* Flash regs */
#define FCLKDIV  (*((volatile unsigned char*)(0x0100)))
#define FSEC     (*((volatile unsigned char*)(0x0101)))
#define FCNFG    (*((volatile unsigned char*)(0x0103)))
#define FPROT    (*((volatile unsigned char*)(0x0104)))
#define FSTAT    (*((volatile unsigned char*)(0x0105)))
#define FCMD     (*((volatile unsigned char*)(0x0106)))

/* Timer definitions */
#define TIOS   (*((volatile unsigned char*)(0x0040))) 
#define CFORC  (*((volatile unsigned char*)(0x0041))) 
#define OC7M   (*((volatile unsigned char*)(0x0042))) 
#define OC7D   (*((volatile unsigned char*)(0x0043))) 
#define TCNT   (*((volatile unsigned short*)(0x0044))) 
#define TSCR1  (*((volatile unsigned char*)(0x0046))) 
#define TTOV   (*((volatile unsigned char*)(0x0047))) 
#define TCTL1  (*((volatile unsigned char*)(0x0048)))
#define TCTL2  (*((volatile unsigned char*)(0x0049)))
#define TCTL3  (*((volatile unsigned char*)(0x004A)))
#define TCTL4  (*((volatile unsigned char*)(0x004B)))
#define TIE    (*((volatile unsigned char*)(0x004C)))
#define TSCR2  (*((volatile unsigned char*)(0x004D)))
#define TFLG1  (*((volatile unsigned char*)(0x004E)))	//Timer overflow flag 0x80
#define TFLG2  (*((volatile unsigned char*)(0x004F)))	//Timer overflow flag 0x80
#define TC0    (*((volatile unsigned short*)(0x0050)))
#define TC1    (*((volatile unsigned short*)(0x0052)))
#define TC2    (*((volatile unsigned short*)(0x0054)))
#define TC3    (*((volatile unsigned short*)(0x0056)))
#define TC4    (*((volatile unsigned short*)(0x0058)))
#define TC5    (*((volatile unsigned short*)(0x005A)))
#define TC6    (*((volatile unsigned short*)(0x005C)))
#define TC7    (*((volatile unsigned short*)(0x005E)))
#define PACTL  (*((volatile unsigned char*)(0x0060)))
#define PAFLG  (*((volatile unsigned char*)(0x0061)))
#define MCCTL  (*((volatile unsigned char*)(0x0066)))
#define MCFLG  (*((volatile unsigned char*)(0x0067)))
#define MCCNT  (*((volatile unsigned short*)(0x0076)))

/* PWM definitions */
#define PWME   (*((volatile unsigned char*)(0x00E0)))
#define PWMPOL (*((volatile unsigned char*)(0x00E1)))
#define PWMCLK (*((volatile unsigned char*)(0x00E2)))
#define PWMPRCLK (*((volatile unsigned char*)(0x00E3)))
#define PWMCAE  (*((volatile unsigned char*)(0x00E4)))
#define PWMCNT0 (*((volatile unsigned char*)(0x00EC)))
#define PWMCNT1 (*((volatile unsigned char*)(0x00ED)))
#define PWMCNT2 (*((volatile unsigned char*)(0x00EE)))
#define PWMCNT3 (*((volatile unsigned char*)(0x00EF)))
#define PWMCNT4 (*((volatile unsigned char*)(0x00F0)))
#define PWMPER0 (*((volatile unsigned char*)(0x00F2)))
#define PWMPER1 (*((volatile unsigned char*)(0x00F3)))
#define PWMPER2 (*((volatile unsigned char*)(0x00F4)))
#define PWMPER3 (*((volatile unsigned char*)(0x00F5)))
#define PWMPER4 (*((volatile unsigned char*)(0x00F6)))
#define PWMDTY0 (*((volatile unsigned char*)(0x00F8)))
#define PWMDTY1 (*((volatile unsigned char*)(0x00F9)))
#define PWMDTY2 (*((volatile unsigned char*)(0x00FA)))
#define PWMDTY3 (*((volatile unsigned char*)(0x00FB)))
#define PWMDTY4 (*((volatile unsigned char*)(0x00FC)))
#define PWMSCLA (*((volatile unsigned char*)(0x00E8)))
#define PWMSCLB (*((volatile unsigned char*)(0x00E9)))

/* ATD0 (ADC) definitions */
#define ATD0CTL0   (*((volatile unsigned char*)(0x0080)))
#define ATD0CTL1   (*((volatile unsigned char*)(0x0081)))
#define ATD0CTL2   (*((volatile unsigned char*)(0x0082)))
#define ATD0CTL3   (*((volatile unsigned char*)(0x0083)))
#define ATD0CTL4   (*((volatile unsigned char*)(0x0084)))
#define ATD0CTL5   (*((volatile unsigned char*)(0x0085)))
#define ATD0STAT0  (*((volatile unsigned char*)(0x0086)))
#define ATD0STAT1  (*((volatile unsigned char*)(0x008B)))
#define ATD0DIEN   (*((volatile unsigned char*)(0x008D)))
#define PORTAD0    (*((volatile unsigned char*)(0x008F)))
#define ATD0DR0    (*((volatile unsigned short*)(0x0090)))
#define ATD0DR1    (*((volatile unsigned short*)(0x0092)))
#define ATD0DR2    (*((volatile unsigned short*)(0x0094)))
#define ATD0DR3    (*((volatile unsigned short*)(0x0096)))
#define ATD0DR4    (*((volatile unsigned short*)(0x0098)))
#define ATD0DR5    (*((volatile unsigned short*)(0x009A)))
#define ATD0DR6    (*((volatile unsigned short*)(0x009C)))
#define ATD0DR7    (*((volatile unsigned short*)(0x009E)))

/* SCI0 definitions */
#define SCI0BDH    (*((volatile unsigned char*)(0x00C8))) 
#define SCI0BDL    (*((volatile unsigned char*)(0x00C9))) 
#define SCI0CR1    (*((volatile unsigned char*)(0x00CA))) 
#define SCI0CR2    (*((volatile unsigned char*)(0x00CB))) 
#define SCI0SR1    (*((volatile unsigned char*)(0x00CC))) 
#define SCI0DRL    (*((volatile unsigned char*)(0x00CF))) 

/* SPI definitions */
#define SPI0CR1     (*((volatile unsigned char*)(0x00D8)))
#define SPI0CR2     (*((volatile unsigned char*)(0x00D9)))
#define SPI0BR      (*((volatile unsigned char*)(0x00DA)))
#define SPI0SR      (*((volatile unsigned char*)(0x00DB)))
#define SPI0DR      (*((volatile unsigned char*)(0x00DD)))
 
 /* CAN section */
  #define CAN_BASE ((volatile unsigned char*)0x0140)

  #define CANCTL0		CAN_BASE[0x0]
  #define CANCTL1		CAN_BASE[0x1]
  #define CANBTR0		CAN_BASE[0x2]
  #define CANBTR1		CAN_BASE[0x3]
  #define CANRFLG		CAN_BASE[0x4]
  #define CANRIER		CAN_BASE[0x5]
  #define CANTFLG		CAN_BASE[0x6]
  #define CANTIER		CAN_BASE[0x7]
  #define CANTARQ		CAN_BASE[0x8]
  #define CANTAAK		CAN_BASE[0x9]
  #define CANTBSEL		CAN_BASE[0xA]
  #define CANIDAC		CAN_BASE[0xB]
  #define CANRXERR		CAN_BASE[0xE]
  #define CANTXERR		CAN_BASE[0xF]

  #define CANIDAR0		CAN_BASE[0x10]
  #define CANIDAR1		CAN_BASE[0x11]
  #define CANIDAR2		CAN_BASE[0x12]
  #define CANIDAR3		CAN_BASE[0x13]
  #define CANIDAR4		CAN_BASE[0x18]
  #define CANIDAR5		CAN_BASE[0x19]
  #define CANIDAR6		CAN_BASE[0x1A]
  #define CANIDAR7		CAN_BASE[0x1B]

  #define CANIDMR0		CAN_BASE[0x14]
  #define CANIDMR1		CAN_BASE[0x15]
  #define CANIDMR2		CAN_BASE[0x16]
  #define CANIDMR3		CAN_BASE[0x17]
  #define CANIDMR4		CAN_BASE[0x1C]
  #define CANIDMR5		CAN_BASE[0x1D]
  #define CANIDMR6		CAN_BASE[0x1E]
  #define CANIDMR7		CAN_BASE[0x1F]

  #define CAN_RB_IDR0	CAN_BASE[0x20]
  #define CAN_RB_IDR1	CAN_BASE[0x21]
  #define CAN_RB_IDR2	CAN_BASE[0x22]
  #define CAN_RB_IDR3	CAN_BASE[0x23]

  #define CAN_RB_DSR0	CAN_BASE[0x24]
  #define CAN_RB_DSR1	CAN_BASE[0x25]
  #define CAN_RB_DSR2	CAN_BASE[0x26]
  #define CAN_RB_DSR3	CAN_BASE[0x27]
  #define CAN_RB_DSR4	CAN_BASE[0x28]
  #define CAN_RB_DSR5	CAN_BASE[0x29]
  #define CAN_RB_DSR6	CAN_BASE[0x2A]
  #define CAN_RB_DSR7	CAN_BASE[0x2B]

  #define CAN_RB_DLR	CAN_BASE[0x2C]

  #define CAN_RB_TBPR	CAN_BASE[0x2D]

  #define CAN_TB0_IDR0	CAN_BASE[0x30]
  #define CAN_TB0_IDR1	CAN_BASE[0x31]
  #define CAN_TB0_IDR2	CAN_BASE[0x32]
  #define CAN_TB0_IDR3	CAN_BASE[0x33]

  #define CAN_TB0_DSR0	CAN_BASE[0x34]
  #define CAN_TB0_DSR1	CAN_BASE[0x35]
  #define CAN_TB0_DSR2	CAN_BASE[0x36]
  #define CAN_TB0_DSR3	CAN_BASE[0x37]
  #define CAN_TB0_DSR4	CAN_BASE[0x38]
  #define CAN_TB0_DSR5	CAN_BASE[0x39]
  #define CAN_TB0_DSR6	CAN_BASE[0x3A]
  #define CAN_TB0_DSR7	CAN_BASE[0x3B]

  #define CAN_TB0_DLR	CAN_BASE[0x3C]

  #define CAN_TB0_TBPR	CAN_BASE[0x3D]
