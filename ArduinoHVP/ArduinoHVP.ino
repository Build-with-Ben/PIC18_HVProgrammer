#pragma  GCC optimize ("O0")

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
#define  EEPGD_SET             0x8EA6     // Selects the program/config space for reads/writes
#define  EEPGD_CLEAR           0x9EA6     // Clears the EEPGD bit 
#define  CFGS_SET              0x8CA6     // Selects the configuration space
#define  CFGS_CLEAR            0x9CA6     // Clears the CFGS bit
#define  WREN_SET              0x84A6     // Enables writes
#define  WREN_CLEAR            0x94A6     // Disables writes
#define  WR_SET                0x82A6     // Begins write cycle
#define  WR_CLEAR              0x92A6     // End write cycle

// --------------------- Known Masks ---------------------------
#define  CONFIG4L_BIT_MASK     0xC5       // CONFIG4L register bits 5, 4, 3, 1 are unimplemented
#define  USER_ID_BIT_MASK      0xFF       // all bits are free game
// add more later

// ------------------- Config Btye Definitions -----------------
#define  CONFIG_BYTES_TO_READ 14         // The K22 config space exposes 14 bytes at 0x300000..0x30000D.

// ------------------- 4-bit Commands --------------------------
#define  TBLWRITE_CMD        0x0C        // Table Write 4 bit command (bin 1100)
#define  TBLWRITE_PI_CMD     0x0D        // Table Write Post Increment (bin 1101)
#define  TBLWRITE_START_CMD  0x0F        // Table Write Start Programming (bin 1111)
#define  TBLREAD_PI_CMD      0x09        // Table Read Post Increment 4 bit command(bin 1001)
#define  TBLREAD_CMD         0x08        // Table Read No pre or post inc/dec
#define  COREINSTR_CMD       0x00        // Core instruction 4 bit command (bin 0000)

// ------------------- Pin mappings ---------------------
#define  PIN_VDD             2           // PIC18 power supply
#define  PIN_MCLR            3           // Data direction register for DATA port
#define  PIN_DATA            4           // PGD = Data = RB6 on PIC
#define  PIN_CLOCK           5           // PGC = Clk = RB7 on PIC

// All delays are in microseconds.
#define  DELAY_TPGC_P6        1           // Delay between clock edges
#define  DELAY_SETTLE         50          // Delay for lines to settle for reset
#define  DELAY_TPPDP          5           // Hold time after raising MCLR
#define  DELAY_THLD0          5           // Input Data Hold Time from PGC (P4)
#define  DELAY_TDLY1          5           // Delay Between 4-Bit Command and Command Operand
#define  DELAY_TDLY2          5           // Delay Between Last PGC falling edge of Command Byte to First PGC (raising) of Read of Data Word

// ------------------- Literals --------------------------------
#define  TRUE                 1
#define  FALSE                0

// -------------- global declarations --------------------------
//volatile int input=0;
uint8_t dataBit = 0;
static uint8_t config[CONFIG_BYTES_TO_READ];
enum MemorySpace { 
  FLASH_ID, 
  CONFIG_DEVICE, 
  UNKNOWN };

//-------------- PIC18F Common Instructions ------------------------------

void caseUsed() {}
void myCase5() __attribute__((used));
void myCase5() {
    Serial.println("CASE 5");
}

static inline void setClkHigh() { 
  digitalWrite(PIN_CLOCK, HIGH); 
  delayMicroseconds(DELAY_TPGC_P6); 
  }
  
static inline void setClkLow() { 
  digitalWrite(PIN_CLOCK, LOW);  
  delayMicroseconds(DELAY_TPGC_P6);  
  }

//Create and send 4 bit opcpde command
//takes in a byte 'cmd' (i.e 0x00 or 0000 1101)
void send4Cmd(uint8_t cmd){
    pinMode(PIN_DATA, OUTPUT);
    for (uint8_t i = 0; i < 4; ++i) {
        setClkHigh();
        digitalWrite(PIN_DATA, (cmd & 1) ? HIGH : LOW);
        setClkLow();
        cmd >>= 1;
    }
    digitalWrite(PIN_DATA, LOW);
    delayMicroseconds(50); // delay to differeniate between opcode and payload
}

//Create and send 16 bit operand payload
//takes in a two byte payload (i.e 0x0000)
void send16Payload(uint16_t payload){
    for (uint8_t i = 0; i < 16; ++i) {
        setClkHigh();
        if ((payload >> i) & 0x01) {
            digitalWrite(PIN_DATA, HIGH);
        } else {
            digitalWrite(PIN_DATA, LOW);
        }
        delayMicroseconds(DELAY_TDLY1);
        setClkLow();
    }
    digitalWrite(PIN_DATA, LOW);
    delayMicroseconds(50); // delay to differeniate payload and SoF
}

// Send 8 zero operand clocks for TBLWT
static inline void send8OperandZero() {
  
  pinMode(PIN_DATA, OUTPUT);
  digitalWrite(PIN_DATA, LOW);   // operand bit = 0
  for (uint8_t i = 0; i < 8; ++i) {
    setClkHigh();
    delayMicroseconds(DELAY_TDLY1); // After writing 8 dummy bits, 
    setClkLow();
    delayMicroseconds(DELAY_TDLY1); // After writing 8 dummy bits, 
  }
}

// Unlock sequence needed for writes
void hwUnlock(){
  sendCoreInstr(0x0E55); // MOVLW 0x55
  sendCoreInstr(0x6EA7); // MOVWF EECON2
  sendCoreInstr(0x0EAA); // MOVLW 0xAA
  sendCoreInstr(0x6EA7); // MOVWF EECON2
}

// Sets EECON1 bits based on target location and enables writes 
void cfgEECON1Bits (uint32_t addr) {

    sendCoreInstr(WREN_CLEAR); // force a clear to be safe

    if (addr >= EEPROM_ADDR) {
        sendCoreInstr(EEPGD_CLEAR);
        sendCoreInstr(CFGS_CLEAR); 
    } 
    else if (addr >= CONFIG_ADDR){
        sendCoreInstr(EEPGD_SET);
        sendCoreInstr(CFGS_SET);
    }
    else if (addr >= USER_ID_ADDR) {
        sendCoreInstr(EEPGD_SET);
        sendCoreInstr(CFGS_CLEAR);
    } 
    else {
        sendCoreInstr(EEPGD_SET);
        sendCoreInstr(CFGS_CLEAR);
    }

    sendCoreInstr(WREN_SET); // enable writes
}

//  sendCoreInstr(WREN_CLEAR); // BCF EECON1, WREN  (Set to 0) - reset to idle state
//  sendCoreInstr(EEPGD_SET); // BSF EECON1, EEPGD (Set to 1) - point to flash memory
//  sendCoreInstr(CFGS_CLEAR); // BCF EECON1, CFGS  (Set to 0) - point to flash/eeprom
//  sendCoreInstr(WREN_SET); // BSF EECON1, WREN  (Set to 1) - prime uC for writing

// Send a Table Write 4-bit 'cmd' plus operand clocks
// Must pass in a valid table write command
static void sendTBLWT(uint8_t cmd) {
  send4Cmd(cmd);
  send8OperandZero();
  send8OperandZero();
  delayMicroseconds(500);   // Makes it clear from frame to frame
}

// Read a single config byte at addr (assumes program mode already entered)
uint8_t readSingleByte(uint32_t addr) {

    setTBLPTR(addr);
    
    // Force a set of EECON1 EEPGD
    sendCoreInstr(0x8EA6); // BSF EECON1, EEPGD

    if (addr >= 0x300000) {
        sendCoreInstr(0x8CA6); // BSF EECON1, CFGS
    } 
    else {
        sendCoreInstr(0x9CA6); // BCF EECON1, CFGS
    }

    
    return tblRead(TBLREAD_CMD);
}

//Sends Core instruction (0b0000) 4 bit command plus 16 bit payload arg (data)
//Used in many operations to setup registers for use
void sendCoreInstr(uint16_t data){
  send4Cmd(COREINSTR_CMD);  // 0b0000
  send16Payload(data);      // data payload
  delayMicroseconds(500);   // Makes it clear from frame to frame
}

// Memory in the address space, 0000000h to 3FFFFFh,
// is addressed via the Table Pointer register (TBLPTR), 
// which is comprised of three Pointer registers: U H L
// ----------------------------------------
// This function sets TBLPTR (U:H:L) to a 24-bit (3 byte) 
// address via input arg 'addr' 
static void setTBLPTR(uint32_t addr) {

  uint8_t U = (addr >> 16) & 0xFF; // takes first byte of add and applies to U
  uint8_t H = (addr >>  8) & 0xFF; // takes second byte of add and applies to H
  uint8_t L = (addr >>  0) & 0xFF; // takes third byte of add and applies to L

  sendCoreInstr(0x0E00 | U);  // Core instruction MOVLW U and Bitwise OR 0E and U
  sendCoreInstr(0x6EF8);      // Core instruction MOVWF TBLPTRU - Move to TBLPRTU

  sendCoreInstr(0x0E00 | H);  // Core instruction MOVLW H and Bitwise OR 0E and H 
  sendCoreInstr(0x6EF7);      // Core instruction MOVWF TBLPTRH - Move to TBLPRTH

  sendCoreInstr(0x0E00 | L);  // Core instruction MOVLW L and Bitwise OR 0E and L 
  sendCoreInstr(0x6EF6);      // Core instruction MOVWF TBLPTRL - Move to TBLPRTL
}

// Reads data in TBLPTR and returns the result in b
// Takes in a 'cmd' (i.e TBLRD*, TBLRD*+, etc)
static uint8_t tblRead(uint8_t cmd) {
  
  send4Cmd(cmd);
  send8OperandZero();

  // Turn PIN_DATA into input before the last 8 clocks to sample data (LSB->MSB)
  // And allow Arduino time to switch to INPUT before reading
  pinMode(PIN_DATA, INPUT);
  delayMicroseconds(100); 

  uint8_t b = 0;
  for (uint8_t i = 0; i < 8; ++i) { 
    setClkHigh();
    delayMicroseconds(15); // Give the PIC time to drive the line HIGH/LOW
    setClkLow();
    delayMicroseconds(30); // Stability delay
    if (digitalRead(PIN_DATA)){ //if HIGH, set corresponding bit in b
      b |= (1u << i);
    }
  }

  // Reset Data line
  pinMode(PIN_DATA, OUTPUT);
  digitalWrite(PIN_DATA, LOW);

  // Add 0.5 ms delay before next command for easier debugging
  delayMicroseconds(500);  // 0.5 millisecond
  
  return b;
}

int i;

// Read the 14 configuration bytes at 0x300000...0x30000D into 'bytes'.
// Returns number of bytes read.
size_t readAllConfigBytes(uint8_t* bytes) {
  
  if (!bytes) return 0; // check if pointer is NULL, return zero if yes
  Serial.print("Starting at Address 0x");
  Serial.println(CONFIG_ADDR, HEX);

  // Set Table pointer address
  setTBLPTR(CONFIG_ADDR);
  
  // Select config space for reading
  sendCoreInstr(EEPGD_SET); // BSF EECON1, EEPGD (bit 6)
  sendCoreInstr(CFGS_SET); // BSF EECON1, CFGS  (bit 7)
  
  for (size_t i = 0; i < CONFIG_BYTES_TO_READ; ++i){
    bytes[i] = tblRead(TBLREAD_PI_CMD);
  }
  return CONFIG_BYTES_TO_READ;
}

void enterProgramMode() {

  //CLK and DATA lines must be pulled LOW first
  //MCLR must have a higher voltage than VDD to enter HVP
  //VDD is supplied by external power supply (regular ol' battery)
  //Goal is to switch VDD on via FET or transistor from external batt

  // Lower MCLR, VDD, DATA, and CLOCK to start in a reset/power off state
  pinMode(PIN_MCLR, OUTPUT);
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_VDD, OUTPUT);
  digitalWrite(PIN_MCLR, LOW);
  digitalWrite(PIN_DATA, LOW);
  digitalWrite(PIN_CLOCK, LOW);
  digitalWrite(PIN_VDD, LOW);

  // Wait for the lines to settle.
  delay(500);

  // VDD first, then MCLR
  digitalWrite(PIN_VDD, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_MCLR, HIGH);

  delay(10);

}

void exitProgramMode() {

    // Lower MCLR, VDD, DATA, and CLOCK.
    digitalWrite(PIN_MCLR, LOW);
    digitalWrite(PIN_VDD, LOW);
    delay(1);
    
    // Clear the lines
    digitalWrite(PIN_DATA, LOW);
    digitalWrite(PIN_CLOCK, LOW);

    // Float the DATA and CLOCK pins.
    pinMode(PIN_DATA, INPUT);
    pinMode(PIN_CLOCK, INPUT);
}

void printMenu() {
  Serial.println("Enter one of the following values to execute a programming sequence:");
  Serial.println("1 - Test Erase, Write, and Read");
  Serial.println("2 - Read All Configuration Bits");
  Serial.println("3 - Enable Low Voltage Programming");
  Serial.println("4 - Read Specific Register");
  Serial.println("5 - Write Specific Register");
  Serial.println("6 - Get Device ID");
  Serial.println("7 - Test/Debug Menu");
  Serial.println("8 - Erase everything!");
  Serial.println("9 - Erase Specific Register");
  Serial.println();
}

void printTestMenu() {
  Serial.println("Enter one of the following values to execute a test sequence:");
  Serial.println("1 - Test VDD Write");
  Serial.println("2 - Test MCLR Write");
  Serial.println();
}


/*
 * Write a single byte of 'value' to address 'addr' by....
 * 1. Performs setting Table pointer to 'addr via U, H, and L'
 * 2. Moves specified 'value' into TABLAT
 * 3. Sets memory space (config or EEPROM) and enables writes
 * 4. Peforms a TBLWT which loads value from TABLAT
 * 5. Peforms write unlock sequences
 * 6. Starts the write (via WR bit)
 * 7. Wait and verify write was successful
 * 
 */
//void writeByte(uint32_t addr, uint8_t value)
//{
//    // Force clear EECON1 bits
//    sendCoreInstr(CFGS_CLEAR); 
//    sendCoreInstr(EEPGD_CLEAR);
//
//    setTBLPTR(addr);
//
//    // Detemine write mode based on address
//    // where EEPROM: 0xF00000+ | Config: 0x300000+ | UserID: 0x200000+
//    if (addr >= 0x300000 || (addr >= 0x200000 && addr < 0x300000)) {
//        sendCoreInstr(CFGS_SET);
//    }
//    
//    // Load TABLAT value
//    sendCoreInstr(0x0E00 | value);  // MOVLW val
//    sendCoreInstr(0x6EF5);          // MOVWF TABLAT
//
//    sendCoreInstr(WREN_SET);  // BSF EECON1, WREN
//
//    
//    //sendTBLWT(TBLWRITE_PI_CMD);
//    send4Cmd(TBLWRITE_CMD);    // 0x0C
//    send16Payload(0x0000 | value); // Data is in the low byte
//    delayMicroseconds(50);
//
//    // Send Required unlock commands
//    sendCoreInstr(0x0E55);  // MOVLW 0x55
//    sendCoreInstr(0x6EA7);  // MOVWF EECON2
//    sendCoreInstr(0x0EAA);  // MOVLW 0xAA
//    sendCoreInstr(0x6EA7);  // MOVWF EECON2
//
//    // Start write by setting WR bit
//    sendCoreInstr(WR_SET);  // BSF EECON1, WR
//
//    sendCoreInstr(0x0000); 
//    sendCoreInstr(0x0000);
//
//    // Wait at least 5ms
//    delay(10);
//
//    // Clear WREN bit
//    sendCoreInstr(WREN_CLEAR);
//}

void writeByte(uint32_t addr, uint8_t value)
{
    // 1. Reset EECON1 for EEPROM (EEPGD=0, CFGS=0)
    sendCoreInstr(0x9EA6); // BCF EECON1, EEPGD
    sendCoreInstr(0x9CA6); // BCF EECON1, CFGS

    setTBLPTR(addr);

    // 2. Load the data byte using the NVM Load Command (0x02)
    // This bypasses the Table Write buffer and goes straight to the NVM latch
    send4Cmd(0x02); 
    send16Payload(value); // PIC takes the lower 8 bits

    // 3. Enable Writes
    sendCoreInstr(WREN_SET); // BSF EECON1, WREN

    // 4. Required Unlock Sequence
    sendCoreInstr(0x0E55); sendCoreInstr(0x6EA7); // MOVLW 55, MOVWF EECON2
    sendCoreInstr(0x0EAA); sendCoreInstr(0x6EA7); // MOVLW AA, MOVWF EECON2

    // 5. Start Write
    delayMicroseconds(10);
    sendCoreInstr(WR_SET); // BSF EECON1, WR

    // 6. Provide "Processor Clocks" for the internal state machine
    // The PIC needs PGC toggles to advance its internal write timer
    for(int i=0; i<5; i++) {
        sendCoreInstr(0x0000); // NOPs
    }

    // 7. Physical delay for EEPROM cell chemistry
    delay(10); 

    // 8. Disable Writes
    sendCoreInstr(WREN_CLEAR);
}

uint16_t prepareEraseKey(uint32_t address) {  

    Serial.println("Selecting Erase Mode...");
    if (address >= 0xF00000) {
      Serial.println("Erase Mode: EEPROM Memory");
      return 0x0084;        
    } 
    else if (address >= 0x300000){
      Serial.println("Erase Mode: Configuration Memory");
      return 0x0082;  
    }
    else if (address >= 0x200000) {
        Serial.println("Erase Mode: User IDs");
        return 0x0088;  
    }
    else if (address == 0xFFFFFF) {
        Serial.println("Erase Mode: Full Chip");
        return 0x0F8F;  
    }
    else {
      Serial.println("Erase Mode: Full Chip");
      return 0x0F8F;
    }

    return 0x0000;
}

void bulkErase(uint32_t targetAddr) {

    //Prepare EECON1 bits based on address
    cfgEECON1Bits(targetAddr);
    
    // Prepare erase based on address
    uint16_t eraseKey = prepareEraseKey(targetAddr);

    if (eraseKey == 0x0000) {
        Serial.println("Error: Invalid Erase Address");
        return;
    }

    // Write first unlock key
    setTBLPTR(BULK_ERASE_ADDR1);
    send4Cmd(TBLWRITE_CMD);          // Command 1100 (Table Write)
    send16Payload(eraseKey >> 8);    // Send first byte of key
    delayMicroseconds(100);
 
    // Write specific region key
    setTBLPTR(BULK_ERASE_ADDR2);
    send4Cmd(TBLWRITE_CMD);          // Command 1100 (Table Write)
    send16Payload(eraseKey & 0xFF);  // Send second byte of key
    delayMicroseconds(100);

    // NOP triggers start
    sendCoreInstr(0x0000); // Command 0000 (Core Instruction)

    // Force data line low while erasing
    pinMode(PIN_DATA, OUTPUT);
    digitalWrite(PIN_DATA, LOW);

    // Send clocks during erase (Required for P11 time)
    for(int i = 0; i < 1000; i++) {
        send16Payload(0x0000); 
    }

    // Wait for Erase Time (typically 5ms-10ms)
    delay(15);
    
    Serial.println("Erase Complete.");
}

void eraseEverything() {
    Serial.println("!!! PERFORMING FULL CHIP ERASE !!!");
    enterProgramMode();

    // 1. Prepare EECON1
    sendCoreInstr(0x9EA6); // BCF EECON1, EEPGD 
    sendCoreInstr(0x8CA6); // BSF EECON1, CFGS

    // Table Write 0x0F0F to 0x3C0005 ---
    setTBLPTR(0x3C0005);
    send4Cmd(TBLWRITE_CMD);    // <--- The "1100" Command
    send16Payload(0x0F0F);     // <--- The "0F 0F" Data
    delayMicroseconds(100);

    // Table Write 0x8F8F to 0x3C0004 ---
    setTBLPTR(0x3C0004);
    send4Cmd(TBLWRITE_CMD);    // <--- The "1100" Command
    send16Payload(0x8F8F);     // <--- The "8F 8F" Data
    delayMicroseconds(100);

    // Execute Erase ---
    sendCoreInstr(0x0000);     // NOP (Starts the internal erase)

    // Hold PGD low and wait for the hardware to finish
    pinMode(PIN_DATA, OUTPUT);
    digitalWrite(PIN_DATA, LOW);

    for(int i = 0; i < 1000; i++) {
        send16Payload(0x0000); 
    }
    
    exitProgramMode();
    Serial.println("Chip erase sequence complete.");
}

void verifyWrite(uint32_t addr, uint8_t expected, uint8_t mask){
  
    //Verify write was successful
    uint8_t actual = readSingleByte(addr);
    if ((actual & mask) != (expected & mask)) {
      Serial.println("Write verification FAILED");
    } else {
      Serial.println("Write verified OK");
    }
}

void nakedConfigRead() {
    enterProgramMode();

    // 1. Force state for CONFIG/DEVICE ID space
    sendCoreInstr(0x8EA6); // BSF EECON1, EEPGD (Must be 1)
    sendCoreInstr(0x8CA6); // BSF EECON1, CFGS  (Must be 1)
    sendCoreInstr(0x0000); // NOP - give it a cycle to breathe
    sendCoreInstr(0x0000); // NOP
    delay(1);

    // Set Pointer to CONFIG1H
    setTBLPTR(0x300001);

    // Perform Read
    Serial.println("Reading 0x300001 (Should be ~0x23)...");
    uint8_t val = tblRead(TBLREAD_CMD); 

    exitProgramMode();
    Serial.print("Result: 0x");
    Serial.println(val, HEX);
}

// --------------------- Menu Option Handlers -----------------

void handleReadDeviceID()
{
    const uint32_t ADDR_DEVIDL = 0x3FFFFE;
    const uint32_t ADDR_DEVIDH = 0x3FFFFF;

    enterProgramMode();

    // Ensure program memory is selected
    sendCoreInstr(EEPGD_SET);   // EEPGD = 1
    sendCoreInstr(CFGS_SET);  // CFGS  = 1

    Serial.println("Reading Device ID...");

    setTBLPTR(ADDR_DEVIDL);
    uint8_t devidL = tblRead(TBLREAD_PI_CMD);

    // The post-increment moved us to 0x3FFFFF automatically
    uint8_t devidH = tblRead(TBLREAD_PI_CMD);

    exitProgramMode();

    Serial.print("Device ID Low  (DEVIDL) = 0x");
    Serial.println(devidL, HEX);
    Serial.print("Device ID High (DEVIDH) = 0x");
    Serial.println(devidH, HEX);

    uint16_t fullID = ((uint16_t)devidH << 8) | devidL;

    Serial.print("Full Device ID = 0x");
    Serial.println(fullID, HEX);
}

void handleReadAllConfig(uint8_t* config) {
    Serial.println("Reading Config bits...");
    enterProgramMode();
    size_t count = readAllConfigBytes(config);
    Serial.print("Number of bytes read: "); 
    Serial.println(count);
    if (count > 0) {
        Serial.println("Config bytes:");

        for (size_t i = 0; i < count; i++) {

            // CONFIG register index (CONFIG1, CONFIG2, etc.)
            size_t cfgNum = (i / 2) + 1;      

            Serial.print("CONFIG");
            Serial.print(cfgNum);

            // low or high byte?
            if ((i & 1) == 0)
                Serial.print("L = 0x");
            else
                Serial.print("H = 0x");

            if (config[i] < 0x10) Serial.print("0");  // leading zero
            Serial.println(config[i], HEX);
        }
    }
    exitProgramMode();
}

void handleTestMCLR() {
    Serial.println("Testing MCLR pin...");
    digitalWrite(PIN_MCLR, HIGH);
    delay(3000);
    digitalWrite(PIN_MCLR, LOW);
}

void handleTestVdd(){
    Serial.println("Testing VDD pin...");
    digitalWrite(PIN_VDD, HIGH);
    delay(3000);
    digitalWrite(PIN_VDD, LOW);
}

void handleEnableLVP(){
    Serial.println("Enabling LVP mode...");
    enterProgramMode();

    // Read CONFIG4L (address 0x300006) so we know its current value
    uint8_t cfg4L = readSingleByte(CONFIG4L_ADDR);

    Serial.println("Reading current CONFIG4L register value...");
    Serial.print("Current CONFIG4L Value = 0x");
    Serial.println(cfg4L, HEX);

    // Bitwise OR LVP bit (bit 2) with current CONFIG4L register value
    cfg4L |= 0x04;

    Serial.println("Setting new CONFIG4L register value...");
    Serial.print("New CONFIG4L Value = 0x");
    Serial.println(cfg4L, HEX);

    // Write new value back to CONFIG4L register
    writeByte(CONFIG4L_ADDR, cfg4L);

    verifyWrite(CONFIG4L_ADDR, cfg4L, CONFIG4L_BIT_MASK);

    exitProgramMode();
  
}

void handleManualRead() {
  Serial.println("ENTER MEMORY ADDRESS (e.g., 3FFFFE):");
  
  // Wait for the next input
  while (Serial.available() == 0) { ; } 
  
  String addrStr = Serial.readStringUntil('\n');
  addrStr.trim();
  
  // Convert Hex String to Long Integer
  uint32_t customAddr = strtoul(addrStr.c_str(), NULL, 16);
  
  Serial.print("Attempting to read address: 0x");
  Serial.println(customAddr, HEX);
  
  enterProgramMode();
  
  uint8_t result = readSingleByte(customAddr);
  Serial.print("The value at address 0x");
  Serial.print(customAddr, HEX);
  Serial.print(" = 0x");
  Serial.println(result, HEX);
  
  exitProgramMode();
}

void handleManualWrite(){
  
}

void handleTestErsWrtRd() {

  uint8_t testVal = 0x00A1;
  uint32_t testAddr = 0x200000;
  
  Serial.println("Testing Erase, Write, and Read");
  enterProgramMode();
  bulkErase(testAddr); // erases test address
  testWrite(testVal, testAddr); // Writes value to test address
  uint8_t result = simpleReadBack(testAddr); // reads test address
  if (result == testVal){
    Serial.println("Write Sucessfull!");
  }
  exitProgramMode();

}

void handleManualErase(){

  Serial.println("ENTER MEMORY ADDRESS (e.g., 3FFFFE):");
  
  // Wait for the next user input
  while (Serial.available() == 0) { ; } 
  
  String addrStr = Serial.readStringUntil('\n');
  addrStr.trim();
  
  // Convert Hex String to Long Integer
  uint32_t customAddr = strtoul(addrStr.c_str(), NULL, 16);
  
  Serial.print("Attempting to erase address: 0x");
  Serial.println(customAddr, HEX);
  
  enterProgramMode();
  
  bulkErase(customAddr);
  
  uint8_t result = readSingleByte(customAddr);
  Serial.print("The value at address 0x");
  Serial.print(customAddr, HEX);
  Serial.print(" = 0x");
  Serial.println(result, HEX);
  
  exitProgramMode();
}

void handleTestOptions (){
    
    printTestMenu();
  
    if (Serial.available() > 0) {
    
      // Read a full line and trim CR/LF so newline characters
      // sent by the Serial Monitor don't get interpreted as commands.
      String line = Serial.readStringUntil('\n');
      line.trim(); // removes leading/trailing whitespace including '\r'
  
      if (line.length() == 0) {
        // empty line (only newline/CR was sent) — skip processing
        return;
      }

      char input = line.charAt(0);
      
      if (input >= '1' && input <= '2') {
        switch (input){
        case '1': handleTestMCLR(); break;
        case '2': handleTestVdd(); break;
        break;
      }
    } 
  }
}

// Configures write bits, Loads 0xA5 into TABLAT, and Starts Write Sequence
void testWrite(uint16_t val, uint32_t addr) {
          
  Serial.print("Writing 0x");
  Serial.print(val, HEX);
  Serial.print(" to register 0x");
  Serial.print(addr, HEX);
  Serial.println("...");
  
  cfgEECON1Bits(addr); // configure EECON1 bits for 0x200000
  delayMicroseconds(100);
  
  // Set pointer to User ID address
  setTBLPTR(USER_ID_ADDR);   
  
  // Write 0xA5 to pointer location
  send4Cmd(TBLWRITE_PI_CMD); 
  send16Payload(val);

  // Triger start of write cycle
  send4Cmd(TBLWRITE_START_CMD); 
  send16Payload(0x0000); // Send zeros as payload

  // Hardware Unlock Sequence (Required to set the WR bit)
  hwUnlock();

  // Trigger the actual Write
  sendCoreInstr(WR_SET); // BSF EECON1, WR

  // Force data line LOW
  pinMode(PIN_DATA, OUTPUT);
  digitalWrite(PIN_DATA, LOW);

  // Keep the clock moving for the duration of the write
  for(int i = 0; i < 1000; i++) {
    setClkHigh();
    delayMicroseconds(1);
    setClkLow();
    delayMicroseconds(1);
  }
  delay(5);
  
  // Clear bits for next operation
  sendCoreInstr(WREN_CLEAR); 
  setTBLPTR(EECON1_ADDR);   
  sendCoreInstr(EEPGD_CLEAR);
  sendCoreInstr(CFGS_CLEAR); 
}

static simpleReadBack(uint32_t addr) {
  
  setTBLPTR(addr);
  sendCoreInstr(0x8EA6); // BSF EECON1, EEPGD
  
  if (addr >= 0x300000) {
      sendCoreInstr(0x8CA6); 
  } else {
      sendCoreInstr(0x9CA6); 
  }

  // Command 0x08 is "Table Read" (No increment)
  uint8_t result = tblRead(0x08); 

  // Print to terminal
  Serial.println("Verifying write...");
  Serial.print("New value at 0x");
  Serial.print(addr, HEX);
  Serial.print(" = 0x");
  Serial.println(result, HEX);

  return result;
}

// only runs once
void setup() {

  // Establish serial link with PC
  Serial.begin(9600);
  while (!Serial) {
      ;  // wait for serial port to connect. Needed for native USB port only
    }

  // Hold the PIC in the powered down/reset state until we are ready for it.
  digitalWrite(PIN_MCLR, LOW);
  digitalWrite(PIN_VDD, LOW);

  // Set up control lines
  // Clock and data are floating until the first PIC command.
  pinMode(PIN_CLOCK, INPUT);
  pinMode(PIN_DATA, INPUT);
  pinMode(PIN_VDD, OUTPUT);
  pinMode(PIN_MCLR, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  printMenu();

}

  //Main loop looks for serial input from host 
  //to begin programming sequence chosen and 
  //outputs serial information based on status
void loop() {
  
  unsigned long startTime;
  unsigned long elapsedTime;
  uint8_t status1 = 0;
  uint8_t status2 = 0;

  if (Serial.available() > 0) {

    // Read a full line and trim CR/LF so newline characters
    // sent by the Serial Monitor don't get interpreted as commands.
    String line = Serial.readStringUntil('\n');
    line.trim(); // removes leading/trailing whitespace including '\r'

    if (line.length() == 0) {
      // empty line (only newline/CR was sent) — skip processing
      return;
    }

    char input = line.charAt(0);
    
    if (input >= '1' && input <= '9') {

      Serial.println();

      switch (input) {

        case '1': handleTestErsWrtRd(); break;

        case '2': handleReadAllConfig(config); break;

        case '3': handleEnableLVP(); break;

        case '4': handleManualRead(); break;
        
        case '5': handleManualWrite(); break;

        case '6': handleReadDeviceID(); break;

        case '7': handleTestOptions(); break;

        case '8': eraseEverything(); break;

        case '9': handleManualErase(); break;
          break;

        default:
          // Should never reach here because of the range check
          Serial.print("Default hit! input=0x");
          Serial.println((int)input, HEX);
          break;
      }
      
      Serial.println("Action(s) completed. Enter new command.");
      Serial.println("\n");
      printMenu();
    }
    else {
        Serial.println("Not a valid input. Try Again.");
        Serial.println("\n");
        printMenu();
    }
    
  }
}
