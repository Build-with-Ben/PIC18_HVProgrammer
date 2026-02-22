enum MemorySpace { 
  FLASH_ID, 
  CONFIG_DEVICE, 
  UNKNOWN };

enum InputResult {
  INPUT_IN_PROGRESS,
  INPUT_COMPLETE,
  INPUT_TIMEOUT,
  INPUT_ABORT
};

enum WrBitState {
  WR_BIT_CLEAR,
  WR_BIT_SET,
  WR_BIT_TIMEOUT
};

// ------------------- Various memory addresses ----------------
#define  CONFIG_ADDR          0x300000   // CONFIG register addresses 0x300000..0x30000D
#define  BULK_ERASE_ADDR1     0x3C0005   // Main memeory erase target
#define  BULK_ERASE_ADDR2     0x3C0004   // System config erase target    
#define  CONFIG4L_ADDR        0x300006   // CONFIG4L register address
#define  CONFIG1L_ADDR        0x300000   // CONFIG1L register address
#define  CONFIG1H_ADDR        0x300001   // CONFIG1L register address
#define  USER_ID_ADDR         0x200000   // User ID address used for testing
#define  EEPROM_ADDR          0xF00000   // Start of EEPROM memory
#define  EECON1_ADDR          0x000FA6   // Start of EECON1 register

// --------------------- Bulk Erase Keys ------------------------
#define  CONFIG_ERASE_KEY     0x82       // Erase key for configuration bits
#define  EEPROM_ERASE_KEY     0x84       // Erase key for EEPROM bits
#define  USER_ID_ERASE_KEY    0x88       // Erase key for User ID bits
#define  ERASE_ALL_KEY        0x8F       // Erase everything


// --------------------- EECON1 Register Instructions -----------
#define  EEPGD_SET            0x8EA6     // Selects the program/config space for reads/writes
#define  EEPGD_CLEAR          0x9EA6     // Clears the EEPGD bit 
#define  CFGS_SET             0x8CA6     // Selects the configuration space
#define  CFGS_CLEAR           0x9CA6     // Clears the CFGS bit
#define  WREN_SET             0x84A6     // Enables writes
#define  WREN_CLEAR           0x94A6     // Disables writes
#define  WR_SET               0x82A6     // Begins write cycle
#define  WR_CLEAR             0x92A6     // End write cycle
#define  FREE_SET             0x88A6     // Enable erases
#define  FREE_CLEAR           0x98A6     // Disable erases

// --------------------- Known Masks ---------------------------
#define  CONFIG4L_BIT_MASK    0xC5       // CONFIG4L register bits 5, 4, 3, 1 are unimplemented
#define  USER_ID_BIT_MASK     0xFF       // all bits are free game
// add more later

// ------------------- Config Btye Definitions -----------------
#define  CONFIG_BYTES_TO_READ 14         // The K22 config space exposes 14 bytes at 0x300000..0x30000D.

// ------------------- 4-bit Commands --------------------------
#define  TBLWRITE_CMD          0x0C        // Table Write 4 bit command (bin 1100)
#define  TBLWRITE_PI2_CMD      0x0D        // Table Write Post Increment (bin 1101)
#define  TBLWRITE_START_CMD    0x0F        // Table Write Start Programming (bin 1111)
#define  TBLWRITE_START_PI_CMD 0x0E        // Table Write Start Programming PI (bin 1110)
#define  TBLREAD_PI_CMD        0x09        // Table Read Post Increment 4 bit command(bin 1001)
#define  TBLREAD_CMD           0x08        // Table Read No pre or post inc/dec
#define  COREINSTR_CMD         0x00        // Core instruction 4 bit command (bin 0000)
#define  TABLAT_SHIFT_OUT      0x02        // Shift out tablat register (bin 0010)

// ------------------- Pin mappings ---------------------
#define  PIN_VDD              2           // PIC18 power supply
#define  PIN_MCLR             3           // Data direction register for DATA port
#define  PIN_DATA             4           // PGD = Data = RB6 on PIC
#define  PIN_CLOCK            5           // PGC = Clk = RB7 on PIC

// All delays are in microseconds.
#define  DELAY_TPGC_P6        1           // Delay between clock edges
#define  DELAY_SETTLE         50          // Delay for lines to settle for reset
#define  DELAY_TPPDP          5           // Hold time after raising MCLR
#define  DELAY_THLD0          5           // Input Data Hold Time from PGC (P4)
#define  DELAY_TDLY1          5           // Delay Between 4-Bit Command and Command Operand
#define  DELAY_TDLY2          5           // Delay Between Last PGC falling edge of Command Byte to First PGC (raising) of Read of Data Word
#define  WR_POLL_TIMEOUT      50000       // Timeout while polling WR bit

// ------------------- Literals --------------------------------
#define  TRUE                 1
#define  FALSE                0