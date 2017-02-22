  MEMORY
  {
    page0   (rwx) : ORIGIN = 0x0, LENGTH = 0x3000
    data    (rw)  : ORIGIN = 0x3000, LENGTH = 0x1000
    text1   (rx)  : ORIGIN = 0x100000, LENGTH = 0x4000
    text2   (rx)  : ORIGIN = 0x200000, LENGTH = 0x4000
    eeprom  (rx)  : ORIGIN = 0x4000, LENGTH = 0x2C00
    text3   (rx)  : ORIGIN = 0x6C00, LENGTH = 0x1400
    text    (rx)  : ORIGIN = 0xC000, LENGTH = 0x3EFF
  }
  PROVIDE (_stack = 0x3fff);
