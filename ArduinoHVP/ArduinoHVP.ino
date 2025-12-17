#pragma  GCC optimize ("O0")

// ------------------- Various memory addresses ----------------
#define  CONFIG_ADDR          0x300000   // CONFIG register addresses 0x300000..0x30000D
#define  BULK_ERASE_ADDR1     0x3C0005   // Main memeory erase target
#define  BULK_ERASE_ADDR2     0x3C0004   // System config erase target    
#define  CONFIG4L_ADDR        0x300006   // CONFIG4L register address
#define  CONFIG1L_ADDR        0x300000   // CONFIG1L register address
#define  CONFIG1H_ADDR        0x300001   // CONFIG1L register address
#define  USER_ID_ADDR         0x200000   // User ID address used for testing

// --------------------- Bulk Erase Keys ------------------------
#define  CONFIG_ERASE_KEY     0x82       // Erase key for configuration bits
#define  EEPROM_ERASE_KEY     0x84       // Erase key for EEPROM bits
#define  USER_ID_ERASE_KEY    0x88       // Erase key for User ID bits


// --------------------- EECON1 Register Instructions -----------
#define  EEPGD_SET             0x8EA6     // Selects the program/config space for reads/writes
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

// -------------- global declarations --------------------------
//volatile int input=0;
uint8_t dataBit = 0;
static uint8_t config[CONFIG_BYTES_TO_READ];

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
    
    //Data is latched on falling edge of clock.
    //Loop through 4 times by setting CLOCK HIGH and then
    //writng DATA (LOW or HIGH) based on cmd arg
    //Finally, latch DATA by writing CLOCK LOW
    pinMode(PIN_DATA, OUTPUT);

    for (uint8_t i = 0; i < 4; ++i) {
      setClkHigh();
      digitalWrite(PIN_DATA, (cmd & 1) ? HIGH : LOW); // send LSB first
      setClkLow();
      cmd >>= 1;
  }
  // Force the line low after the sequence
  digitalWrite(PIN_DATA, LOW);
  delayMicroseconds(50); // makes it clear when opcode is over and operand begins
}

//Create and send 16 bit operand payload
//takes in a two byte payload (i.e 0x0000)
void send16Payload(uint16_t payload){
    
    //Data is latched on falling edge of clock.
    //Loop through 4 times by setting CLOCK HIGH and then
    //writng DATA (LOW or HIGH) based on cmd arg
    //Finally, latch DATA by writing CLOCK LOW
    for (uint8_t i = 0; i < 16; ++i) {
      setClkHigh();
      dataBit = payload & 1;
      if (dataBit == HIGH){
        digitalWrite(PIN_DATA, HIGH); // LSB first
      }
      else {
        digitalWrite(PIN_DATA, LOW); // LSB first
      }
      delayMicroseconds(DELAY_TDLY1);
      setClkLow();
      payload >>= 1;
  }
  // Force the line low after the sequence
  digitalWrite(PIN_DATA, LOW);
  delayMicroseconds(50); // makes it clear when operand is over and opcode begins
}

// Send 8 zero operand clocks for TBLWT
static inline void send8OperandZero() {
  pinMode(PIN_DATA, OUTPUT);
  for (uint8_t i = 0; i < 8; ++i) {
    digitalWrite(PIN_DATA, LOW);   // operand bit = 0
    setClkHigh();
    setClkLow();
    delayMicroseconds(1); // After writing 8 dummy bits, 
  }
}

// Send a Table Write 4-bit 'cmd' plus operand clocks
// Must pass in a valid table write command
static void sendTBLWT(uint8_t cmd) {
  send4Cmd(cmd);
  send8OperandZero();
  send8OperandZero();
  delayMicroseconds(500);   // Makes it clear from frame to frame
}

// Read a single config byte at addr (assumes program mode already entered)
uint8_t readSingleByte(uint32_t addr, uint16_t cfgBit, uint16_t eepgdBit) {
  
  delayMicroseconds(5);
  setTBLPTR(addr);
  sendCoreInstr(eepgdBit); // BSF EECON1, EEPGD (bit 6)
  sendCoreInstr(cfgBit); // BSF EECON1, CFGS  (bit 7)
  delayMicroseconds(5);
  uint8_t b = tblReadPI(); // TBLRD post increment
  return b;
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

// Table Read: returns just 1 byte at TBLPTR
static uint8_t tblRead() {
  
  Serial.println("Performing TBLRD*...");
  send4Cmd(TBLREAD_CMD);

  // First 1 byte (8 clock cycles) = operand (0x00); keep PGD low as output
  send8OperandZero();

  // Turn PGD to input before the last 8 clocks to sample data (LSB->MSB)
  pinMode(PIN_DATA, INPUT);
  delayMicroseconds(DELAY_TDLY1);
  delayMicroseconds(DELAY_TDLY1);

  uint8_t b = 0;
  for (uint8_t i = 0; i < 8; ++i) { 
    setClkHigh();
    if (digitalRead(PIN_DATA)){ //if HIGH, set corresponding bit in b
      b |= (1u << i);
    }
    else {
    }
    setClkLow();
  }
  Serial.print("Register Value: 0x");
  Serial.println(b, HEX);

  // Delay between next command
  delayMicroseconds(500);
  
  return b;
}

// Table Read, post-increment: returns 1 byte at TBLPTR then TBLPTR++
static uint8_t tblReadPI() {
  
  Serial.println("Performing TBLRD*+...");
  send4Cmd(TBLREAD_PI_CMD);

  // First 1 byte (8 clock cycles) = operand (0x00); keep PGD low as output
  send8OperandZero();

  // Turn PIN_DATA into input before the last 8 clocks to sample data (LSB->MSB)
  // And allow Arduino time to switch to INPUT before reading
  pinMode(PIN_DATA, INPUT);
  delayMicroseconds(2); 

  uint8_t b = 0;
  for (uint8_t i = 0; i < 8; ++i) { 
    setClkHigh();
    delayMicroseconds(1); // Give the PIC time to drive the line HIGH/LOW
    if (digitalRead(PIN_DATA)){ //if HIGH, set corresponding bit in b
      b |= (1u << i);
    }
    setClkLow();
    delayMicroseconds(1); // Stability delay
  }

  // Reset Data line
  pinMode(PIN_DATA, OUTPUT);
  digitalWrite(PIN_DATA, LOW);
  
  Serial.print("Register Value: 0x");
  Serial.println(b, HEX);

  // Add 0.5 ms delay before next command for easier debugging
  delayMicroseconds(500);  // 0.5 millisecond
  
  return b;
}

int i;

// Read the 14 configuration bytes at 0x300000..0x30000D into 'out'.
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
    bytes[i] = tblReadPI();
  }
  return CONFIG_BYTES_TO_READ;
}

void enterProgramMode() {

  //CLK and DATA lines must be pulled LOW first
  //MCLR must have a higher voltage than VDD to enter HVP
  //VDD is supplied by external power supply (regular ol' battery)
  //Goal is to switch VDD on via FET or transistor from external batt

  // Lower MCLR, VDD, DATA, and CLOCK to start in a reset/power off state
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_DATA, OUTPUT);
  digitalWrite(PIN_MCLR, LOW);
  digitalWrite(PIN_VDD, LOW);
  digitalWrite(PIN_DATA, LOW);
  digitalWrite(PIN_CLOCK, LOW);

  // Wait for the lines to settle.
  delayMicroseconds(200);

  digitalWrite(PIN_VDD, HIGH);
  delay(10);
  digitalWrite(PIN_MCLR, HIGH);
  delay(10);
  
  // Switch DATA and CLOCK into outputs.
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
}

void exitProgramMode() {

    // Lower MCLR, VDD, DATA, and CLOCK.
    digitalWrite(PIN_MCLR, LOW);
    delay(1);
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
  Serial.println("5 - Get Device ID");
  Serial.println("6 - Test VDD Write");
  Serial.println("7 - Test MCLR Write");
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
void writeByte(uint32_t addr, uint8_t value, uint16_t cfgBit, uint16_t eepgdBit)
{
    setTBLPTR(addr);

    // Load TABLAT value
    sendCoreInstr(0x0E00 | value);  // MOVLW val
    sendCoreInstr(0x6EF5);          // MOVWF TABLAT

    // Select memory type and register space then enable writes
    sendCoreInstr(eepgdBit);  // BSF EECON1, EEPGD
    sendCoreInstr(cfgBit);  // BSF EECON1, CFGS
    sendCoreInstr(WREN_SET);  // BSF EECON1, WREN
    
    sendTBLWT(TBLWRITE_PI_CMD);

    // Send Required unlock commands
    sendCoreInstr(0x0E55);  // MOVLW 0x55
    sendCoreInstr(0x6EA7);  // MOVWF EECON2
    sendCoreInstr(0x0EAA);  // MOVLW 0xAA
    sendCoreInstr(0x6EA7);  // MOVWF EECON2

    // Start write by setting WR bit
    sendCoreInstr(WR_SET);  // BSF EECON1, WR

    // Wait at least 5ms
    delay(10);

    // Clear WREN bit
    sendCoreInstr(WREN_CLEAR);
}

void bulkErase(uint16_t key, uint32_t targetAddr) {
    
    Serial.println("Performing Bulk Erase...");

    sendCoreInstr(CFGS_SET); 

    // Set table pointer to address that controls erase of config bits
    setTBLPTR(targetAddr);

    // Load erase key into TABLAT
    sendCoreInstr(0x0E00 | (key & 0xFF));    // MOVLW key into W
    sendCoreInstr(0x6EF5);                   // MOVWF W to TABLAT

    // Set EECON1 for a physical write
    sendCoreInstr(WREN_SET);                 // BSF EECON1, WREN

    sendTBLWT(TBLWRITE_CMD);
    
    // Send Required unlock commands
    sendCoreInstr(0x0E55);  // MOVLW 0x55
    sendCoreInstr(0x6EA7);  // MOVWF EECON2
    sendCoreInstr(0x0EAA);  // MOVLW 0xAA
    sendCoreInstr(0x6EA7);  // MOVWF EECON2

    // Start write by setting WR bit
    sendCoreInstr(WR_SET);  // BSF EECON1, WR

    // Wait for Erase Time (typically 5ms-10ms)
    delay(15);

    sendCoreInstr(WREN_CLEAR); 
    sendCoreInstr(CFGS_CLEAR);
    
    Serial.println("Bulk Erase Complete. Config bits should be 0xFF.");
}

void verifyWrite(uint32_t addr, uint8_t expected, uint8_t mask){
  
    //Verify write was successful
    uint8_t actual = readSingleByte(addr, CFGS_SET, EEPGD_SET);
    if ((actual & mask) != (expected & mask)) {
      Serial.println("Write verification FAILED");
    } else {
      Serial.println("Write verified OK");
    }
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
    uint8_t devidL = tblReadPI();

    // The post-increment moved us to 0x3FFFFF automatically
    uint8_t devidH = tblReadPI();

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
    uint8_t cfg4L = readSingleByte(CONFIG4L_ADDR, CFGS_SET, EEPGD_SET);

    Serial.println("Reading current CONFIG4L register value...");
    Serial.print("Current CONFIG4L Value = 0x");
    Serial.println(cfg4L, HEX);

    // Bitwise OR LVP bit (bit 2) with current CONFIG4L register value
    cfg4L |= 0x04;

    Serial.println("Setting new CONFIG4L register value...");
    Serial.print("New CONFIG4L Value = 0x");
    Serial.println(cfg4L, HEX);

    // Write new value back to CONFIG4L register
    writeByte(CONFIG4L_ADDR, cfg4L, CFGS_SET, EEPGD_SET);

    verifyWrite(CONFIG4L_ADDR, cfg4L, CONFIG4L_BIT_MASK);

    exitProgramMode();
  
}

void handleManualRead() {
  Serial.println("ENTER HEX ADDRESS (e.g., 3FFFFE):");
  
  // Wait for the next input
  while (Serial.available() == 0) { ; } 
  
  String addrStr = Serial.readStringUntil('\n');
  addrStr.trim();
  
  // Convert Hex String to Long Integer
  uint32_t customAddr = strtoul(addrStr.c_str(), NULL, 16);
  
  Serial.print("Attempting to read address: 0x");
  Serial.println(customAddr, HEX);
  
  enterProgramMode();
  
  uint8_t result = readSingleByte(customAddr, CFGS_SET, EEPGD_SET);
  Serial.print("The value at address 0x");
  Serial.print(customAddr, HEX);
  Serial.print(" = 0x");
  Serial.println(result, HEX);
  
  exitProgramMode();
}

void handleTestErsWrtRd(uint8_t value, uint32_t addr) {

  enterProgramMode();
  
  Serial.print("Performing erase at test address: ");
  Serial.println(BULK_ERASE_ADDR2, HEX);
  bulkErase(USER_ID_ERASE_KEY, BULK_ERASE_ADDR2);

  // Write 0x01 to 0x200000
  Serial.print("Writing 0x01 to  ");
  Serial.println(addr, HEX);
  writeByte(addr, value, CFGS_SET, EEPGD_SET);

  Serial.print("Verify write was successful ");
  verifyWrite(addr, value, USER_ID_BIT_MASK);

  exitProgramMode();

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
    
    if (input >= '1' && input <= '7') {

      Serial.println();

      switch (input) {

        case '1': handleTestErsWrtRd(0x01, USER_ID_ADDR); break; //write 0x01 to 0x200000

        case '2': handleReadAllConfig(config); break;

        case '3': handleEnableLVP(); break;

        case '4': handleManualRead(); break;

        case '5': handleReadDeviceID(); break;
        
        case '6': handleTestVdd(); break;

        case '7': handleTestMCLR(); break;

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
