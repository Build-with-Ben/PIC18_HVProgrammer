#pragma GCC optimize ("O0")

// ------------------- Various memory addresses ----------------
#define  CONFIG_ADDR          0x300000   // CONFIG register addresses 0x300000..0x30000D
#define  BULK_ERASE_ADDR1     0x3C0005   // Bulk erase is broken up in two registers
#define  BULK_ERASE_ADDR2     0x3C0004   // ...    
#define  CONFIG4L_ADDR        0x300006   // CONFIG4L register address
#define  CONFIG1L_ADDR        0x300000   // CONFIG1L register address
#define  CONFIG1H_ADDR        0x300001   // CONFIG1L register address


// --------------------- EECON1 Register Instructions ------------
#define EEPGD_SET             0x8EA6     // Selects the program/config space for reads/writes
#define CFGS_SET              0x8CA6     // Selects the configuration space
#define CFGS_CLEAR            0x9CA6     // Clears the CFGS bit
#define WREN_SET              0x84A6     // Enables writes
#define WREN_CLEAR            0x94A6     // Disables writes
#define WR_SET                0x82A6     // Begins write cycle
#define WR_CLEAR              0x92A6     // End write cycle

// ------------------- Config Btye Definitions ----------
#define  CONFIG_BYTES_TO_READ 14         // The K22 config space exposes 14 bytes at 0x300000..0x30000D.

// ------------------- 4-bit Commands -------------------
#define  TBLWRITE_CMD        0x0C        // Table Write 4 bit command (bin 1100)
#define  TBLREAD_PI_CMD      0x09        // Table Read Post Increment 4 bit command(bin 1001)
#define  TBLREAD_CMD         0x08        // Table Read No pre or post inc/dec
#define  COREINSTR_CMD       0x00        // Core instruction 4 bit command (bin 0000)

// ------------------- Pin mappings ---------------------
#define  PIN_VDD      2     // PIC18 power supply
#define  PIN_MCLR     3     // Data direction register for DATA port
#define  PIN_DATA     4     // PGD = Data = RB6 on PIC
#define  PIN_CLOCK    5     // PGC = Clk = RB7 on PIC

// All delays are in microseconds.
#define DELAY_TPGC_P6   1       // Delay between clock edges
#define DELAY_SETTLE    50      // Delay for lines to settle for reset
#define DELAY_TPPDP     5       // Hold time after raising MCLR
#define DELAY_THLD0     5       // Input Data Hold Time from PGC (P4)
#define DELAY_TDLY1     5       // Delay Between 4-Bit Command and Command Operand
#define DELAY_TDLY2     5       // Delay Between Last PGC falling edge of Command Byte to First PGC (raising) of Read of Data Word

// -------------- global declarations --------------------------
//volatile int input=0;
uint8_t dataBit = 0;
static uint8_t config[CONFIG_BYTES_TO_READ];

//-------------- Helper Functions ------------------------------

void caseUsed() {}
void myCase5() __attribute__((used));
void myCase5() {
    Serial.println("CASE 5");
}

// Quick calls to write clock HIGH or LOW with built-in delay
static inline void setClkHigh() { digitalWrite(PIN_CLOCK, HIGH); delayMicroseconds(DELAY_TPGC_P6); }
static inline void setClkLow() { digitalWrite(PIN_CLOCK, LOW);  delayMicroseconds(DELAY_TPGC_P6);  }

//Create and send 4 bit command
//takes in a byte command (i.e 0x00)
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

  delayMicroseconds(50);
}

//Create and send 16 bit payload
//takes in a two byte payload (i.e 0x0000)
void send16Payload(uint16_t payload){
    
    //Data is latched on falling edge of clock.
    //Loop through 4 times by setting CLOCK HIGH and then
    //writng DATA (LOW or HIGH) based on cmd arg
    //Finally, latch DATA by writing CLOCK LOW
    for (uint8_t i = 0; i < 16; ++i) {
    setClkHigh();
    //Serial.println("Debug: PIN_CLK = HIGH");
    dataBit = payload & 1;
    if (dataBit == HIGH){
      digitalWrite(PIN_DATA, HIGH); // LSB first
      //Serial.print("Debug: PIN_DATA = ");
      //Serial.println(dataBit);
    }
    else {
      digitalWrite(PIN_DATA, LOW); // LSB first
      //Serial.print("Debug: PIN_DATA = ");
      //Serial.println(dataBit);
    }
    delayMicroseconds(DELAY_TDLY1);
    setClkLow();
    //Serial.println("Debug: PIN_CLK = LOW");
    payload >>= 1;
  }
  delayMicroseconds(50);
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

// Send TBLWT 4-bit command plus operand clocks (per datasheet)
static void sendTBLWT() {
  send4Cmd(TBLWRITE_CMD);   // 0x0C
  send8OperandZero();
}

// Read a single config byte at addr (assumes program mode already entered)
uint8_t readSingleConfigByte(uint32_t addr) {
  
  delayMicroseconds(5);
  setTBLPTR(addr);
  sendCoreInstr(EEPGD_SET); // BSF EECON1, EEPGD (bit 6)
  sendCoreInstr(CFGS_SET); // BSF EECON1, CFGS  (bit 7)
  delayMicroseconds(5);
  uint8_t b = tblReadPI(); // TBLRD post increment
  return b;
}

//Sends Core instruction (0b0000) 4 bit command plus 16 bit payload arg (data)
//Used in many operations to setup registers for use
void sendCoreInstr(uint16_t data){
  send4Cmd(COREINSTR_CMD);  // 0b0000
  send16Payload(data);      // data payload
  delayMicroseconds(500);
}

// Memory in the address space, 0000000h to 3FFFFFh,
// is addressed via the Table Pointer register (TBLPTR), 
// which is comprised of three Pointer registers: U H L
// ----------------------------------------
// This function sets TBLPTR (U:H:L) to a 24-bit (3 byte) 
// address via input arg 'addr' 
static void setTBLPTR(uint32_t addr) {

  //Serial.print("Where ");

  uint8_t U = (addr >> 16) & 0xFF; // takes first byte of add and applies to U
  //Serial.print("U = 0x");
  //Serial.print(U, HEX);

  uint8_t H = (addr >>  8) & 0xFF; // takes second byte of add and applies to H
  //Serial.print(", H = 0x");
  //Serial.print(H, HEX);

  uint8_t L = (addr >>  0) & 0xFF; // takes third byte of add and applies to L
  //Serial.print(", and L = 0x");
  //Serial.println(L, HEX);

  //Serial.print("Core Instruction (0b0000): MOVLW U 0x");
  //Serial.println(0x0E00 | U, HEX);
  sendCoreInstr(0x0E00 | U);  // Core instruction MOVLW U and Bitwise OR 0E and U

  //Serial.print("Core Instruction (0b0000): MOVWF U 0x");
  //Serial.println(0x6EF8, HEX);
  sendCoreInstr(0x6EF8);      // Core instruction MOVWF TBLPTRU - Move to TBLPRTU

  //Serial.print("Core Instruction (0b0000): MOVLW H 0x");
  //Serial.println(0x0E00 | H, HEX);
  sendCoreInstr(0x0E00 | H);  // Core instruction MOVLW H and Bitwise OR 0E and H 

  //Serial.print("Core Instruction (0b0000): MOVWF H 0x");
  //Serial.println(0x6EF7, HEX);
  sendCoreInstr(0x6EF7);      // Core instruction MOVWF TBLPTRH - Move to TBLPRTH

  //Serial.print("Core Instruction (0b0000): MOVLW L 0x");
  //Serial.println(0x0E00 | L, HEX);
  sendCoreInstr(0x0E00 | L);  // Core instruction MOVLW L and Bitwise OR 0E and L 

  //Serial.print("Core Instruction (0b0000): MOVWF L 0x");
  //Serial.println(0x6EF6, HEX);
  sendCoreInstr(0x6EF6);      // Core instruction MOVWF TBLPTRL - Move to TBLPRTL
}

// Table Read, post-increment: returns 1 byte at TBLPTR then TBLPTR++
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
  delay(0.5);
  
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
  Serial.println("1 - Erase Configuration Bits");
  Serial.println("2 - Read Configuration Bits");
  Serial.println("3 - Enter Program Mode");
  Serial.println("4 - Exit Program Mode");
  Serial.println("5 - Test MCLR Write");
  Serial.println("6 - Get Device ID");
  Serial.println("7 - Enable Low Voltage Programming");
  Serial.println("8 - Test VDD Write");
  Serial.println("9 - Read Specific Register");
  Serial.println();
}

void readDeviceID()
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

void writeConfigByte(uint32_t configAddress, uint8_t value)
{
//    uint8_t U = (configAddress >> 16) & 0xFF;
//    uint8_t H = (configAddress >>  8) & 0xFF;
//    uint8_t L = (configAddress >>  0) & 0xFF;

    setTBLPTR(configAddress);

//    // Load TBLPTR address
//    sendCoreInstr(0x0E00 | U);  // MOVLW U
//    sendCoreInstr(0x6EF8);      // MOVWF TBLPTRU
//    sendCoreInstr(0x0E00 | H);  
//    sendCoreInstr(0x6EF7);      // MOVWF TBLPTRH
//    sendCoreInstr(0x0E00 | L);  
//    sendCoreInstr(0x6EF6);      // MOVWF TBLPTRL

    // Load TABLAT value
    sendCoreInstr(0x0E00 | value);  // MOVLW val
    sendCoreInstr(0x6EF5);          // MOVWF TABLAT

    // Select config space, enable writes
    sendCoreInstr(EEPGD_SET);  // BSF EECON1, EEPGD
    sendCoreInstr(CFGS_SET);  // BSF EECON1, CFGS
    sendCoreInstr(WREN_SET);  // BSF EECON1, WREN
    
    sendTBLWT();

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

    uint8_t verify = readSingleConfigByte(configAddress);
    if (verify != value) {
      Serial.println("Write verification FAILED");
    } else {
      Serial.println("Write verified OK");
    }
}

//------------------- Switch Case Helpers ------------------

void configErase() {
    Serial.println("Performing Bulk Erase...");
    enterProgramMode();

    // Set table pointer to address that controls erase of config bits
    setTBLPTR(BULK_ERASE_ADDR2);

    // Set EECON1 for a physical write
    sendCoreInstr(WREN_SET);  // BSF EECON1, WREN

    sendTBLWT();
    
    // Send Required unlock commands
    sendCoreInstr(0x0E55);  // MOVLW 0x55
    sendCoreInstr(0x6EA7);  // MOVWF EECON2
    sendCoreInstr(0x0EAA);  // MOVLW 0xAA
    sendCoreInstr(0x6EA7);  // MOVWF EECON2

    // Start write by setting WR bit
    sendCoreInstr(WR_SET);  // BSF EECON1, WR

    // Wait for Erase Time (typically 5ms-10ms)
    delay(15);

    sendCoreInstr(WREN_CLEAR);  // BSF EECON1, WREN
    
    exitProgramMode();
    Serial.println("Bulk Erase Complete. Config bits should be 0xFF.");
}

void readConfigCase(uint8_t* config) {
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

void readConfigSingleCase(uint32_t addr) {
  
    Serial.println("Reading Config bits...");
    enterProgramMode();
    size_t value = readSingleConfigByte(addr);
    Serial.print("Config byte @ 0x");
    Serial.print(addr, HEX);
    Serial.print(" = 0x");
    Serial.println(value, HEX);
    exitProgramMode();
}

void testMCLR() {
    Serial.println("Testing MCLR pin...");
    digitalWrite(PIN_MCLR, HIGH);
    delay(3000);
    digitalWrite(PIN_MCLR, LOW);
}

void testVdd(){
    Serial.println("Testing VDD pin...");
    digitalWrite(PIN_VDD, HIGH);
    delay(3000);
    digitalWrite(PIN_VDD, LOW);
}

void enableLVP(){
    Serial.println("Enabling LVP mode...");
    enterProgramMode();

    // Read CONFIG4L (address 0x300006) so we know its current value
    uint8_t cfg4L = readSingleConfigByte(CONFIG4L_ADDR);

    Serial.println("Reading current CONFIG4L register value...");
    Serial.print("Current CONFIG4L Value = 0x");
    Serial.println(cfg4L, HEX);

    // Bitwise OR LVP bit (bit 2) on CONFIG4L register
    cfg4L |= 0x04;

    Serial.println("Setting new CONFIG4L register value...");
    Serial.print("New CONFIG4L Value = 0x");
    Serial.println(cfg4L, HEX);

    // Write new value back to CONFIG4L register
    writeConfigByte(CONFIG4L_ADDR, cfg4L);

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
  
  readConfigSingleCase(customAddr);
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
    
    if (input >= '1' && input <= '9') {

      Serial.println();

      switch (input) {

        case '1': configErase(); break;

        case '2': readConfigCase(config); break;

        case '3': enterProgramMode(); break;
        
        case '4': exitProgramMode(); break;
        
        case '5': testMCLR(); break;

        case '6': readDeviceID(); break;

        case '7': enableLVP(); break;

        case '8': testVdd(); break;

        case '9': handleManualRead(); break;

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
