#pragma GCC optimize ("O0")

// ------------------- Desired addresses ----------------
#define  CONFIG_ADDR          0x300000   // CONFIG register addresses 0x300000..0x30000D
#define  BULK_ERASE_ADDR1     0x3C0005   // Bulk erase is broken up in two registers
#define  BULK_ERASE_ADDR2     0x3C0004   // ...    

// ------------------- Config Btye Definitions ----------
#define  CONFIG_BYTES_TO_READ 14      // The K22 config space exposes 14 bytes at 0x300000..0x30000D.

// ------------------- 4-bit Commands -------------------
#define  TBLWRITE_CMD        0x0C      // Table Write 4 bit command (bin 1100)
#define  TBLREAD_PI_CMD      0x09      // Table Read Post Increment 4 bit command(bin 1001)
#define  COREINSTR_CMD       0x00      // Core instruction 4 bit command (bin 0000)

// ------------------- Pin mappings ---------------------
#define  PIN_VDD      2     // PIC18 power supply
#define  PIN_MCLR     3     // Data direction register for DATA port
#define  PIN_DATA     4     // PGD = Data = RB6 on PIC
#define  PIN_CLOCK    5     // PGC = Clk = RB7 on PIC

#ifndef PIN_VDD_EN
#define PIN_VDD_EN    -1
#endif

// All delays are in microseconds.
#define DELAY_SETTLE 50 // Delay for lines to settle for reset
#define DELAY_TPPDP 5 // Hold time after raising MCLR
#define DELAY_THLD0 5 // Input Data Hold Time from PGC (P4)
#define DELAY_TDLY1 5 // Delay Between 4-Bit Command and Command Operand
#define DELAY_TDLY2 5 // Delay Between Last PGC falling edge of Command Byte to First PGC (raising) of Read of Data Word

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
static inline void setClkHigh() { digitalWrite(PIN_CLOCK, HIGH); delayMicroseconds(DELAY_TDLY1); }
static inline void setClkLow() { digitalWrite(PIN_CLOCK, LOW);  delayMicroseconds(DELAY_TDLY1);  }

static inline void setVdd(bool on) {
#if PIN_VDD_EN >= 0
  digitalWrite(PIN_VDD_EN, on ? HIGH : LOW);
#endif
}

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
    //Serial.println("Debug: PIN_CLK = HIGH");
    dataBit = cmd & 1;
    if (dataBit == HIGH){
      digitalWrite(PIN_DATA, HIGH); // LSB first
      //Serial.print("Debug: PIN_DATA = ");
      //Serial.println(dataBit);
    }
    else{
      digitalWrite(PIN_DATA, LOW); // LSB first
      //Serial.print("Debug: PIN_DATA = ");
      //Serial.println(dataBit);
    }
    delayMicroseconds(DELAY_TDLY1);
    setClkLow();
    //Serial.println("Debug: PIN_CLK = LOW");
    cmd >>= 1;
  }
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
}

//Sends Core instruction (0b0000) 4 bit command plus 16 bit payload arg (data)
//Used in many operations to setup registers for use
void sendCoreInstr(uint16_t data){
  send4Cmd(COREINSTR_CMD);  // 0b0000
  send16Payload(data);      // data payload
}

// Memory in the address space, 0000000h to 3FFFFFh,
// is addressed via the Table Pointer register (TBLPTR), 
// which is comprised of three Pointer registers: U H L
// ----------------------------------------
// This function sets TBLPTR (U:H:L) to a 24-bit (3 byte) 
// address via input arg 'addr' 
static void setTBLPTR(uint32_t addr) {

  Serial.print("Where ");

  uint8_t U = (addr >> 16) & 0xFF; // takes first byte of add and applies to U
  Serial.print("U = 0x");
  Serial.print(U, HEX);

  uint8_t H = (addr >>  8) & 0xFF; // takes second byte of add and applies to H
  Serial.print(", H = 0x");
  Serial.print(H, HEX);

  uint8_t L = (addr >>  0) & 0xFF; // takes third byte of add and applies to L
  Serial.print(", and L = 0x");
  Serial.println(L, HEX);

  Serial.print("Core Instruction (0b0000): MOVWF U 0x");
  Serial.println(0x0E00 | U, HEX);
  sendCoreInstr(0x0E00 | U);  // Core instruction MOVLW U and Bitwise OR 0E and U 

  Serial.print("Core Instruction (0b0000): MOVLW U 0x");
  Serial.println(0x6EF8, HEX);
  sendCoreInstr(0x6EF8);      // Core instruction MOVWF TBLPTRU - Move to TBLPRTU

  Serial.print("Core Instruction (0b0000): MOVWF H 0x");
  Serial.println(0x0E00 | H, HEX);
  sendCoreInstr(0x0E00 | H);  // Core instruction MOVLW H and Bitwise OR 0E and H 

  Serial.print("Core Instruction (0b0000): MOVLW H 0x");
  Serial.println(0x6EF7, HEX);
  sendCoreInstr(0x6EF7);      // Core instruction MOVWF TBLPTRH - Move to TBLPRTH

  Serial.print("Core Instruction (0b0000): MOVWF L 0x");
  Serial.println(0x0E00 | L, HEX);
  sendCoreInstr(0x0E00 | L);  // Core instruction MOVLW L and Bitwise OR 0E and L 

  Serial.print("Core Instruction (0b0000): MOVLW L 0x");
  Serial.println(0x6EF6, HEX);
  sendCoreInstr(0x6EF6);      // Core instruction MOVWF TBLPTRL - Move to TBLPRTL
}

// Table Read, post-increment: returns 1 byte at TBLPTR then TBLPTR++
static uint8_t tblReadPI() {
  //Serial.println("Table Read - Post Increment (0b1001): TBLRD *+");
  send4Cmd(TBLREAD_PI_CMD);

  // First 1 byte (8 clock cycles) = operand (0x00); keep PGD low as output
  pinMode(PIN_DATA, OUTPUT);
  for (uint8_t i = 0; i < 8; ++i) { 
    digitalWrite(PIN_DATA, LOW); 
    setClkHigh(); 
    setClkLow(); 
    }

  // Turn PGD to input before the last 8 clocks to sample data (LSB->MSB)
  pinMode(PIN_DATA, INPUT);
  delayMicroseconds(DELAY_TDLY1);
  delayMicroseconds(DELAY_TDLY1);

  uint8_t b = 0;
  for (uint8_t i = 0; i < 8; ++i) { 
    setClkHigh();
    if (digitalRead(PIN_DATA)){ //if HIGH, set corresponding bit in b
      b |= (1u << i);
      //Serial.println("Debug: PIN DATA Value = HIGH");
      //Serial.print("Debug: Updated PIN Date Byte Value = 0b");
      //Serial.println(b, BIN);
    }
    else {
      //Serial.println("Debug: PIN DATA Value = LOW");
      //Serial.print("Debug: Updated PIN Date Byte Value = 0b");
      //Serial.println(b, BIN);
    }
    setClkLow();
  }
  //Serial.print("Debug: PIN DATA Value (BIN) = 0b");
  //Serial.println(b, BIN);
  //Serial.print("Debug: PIN DATA Value (HEX) = 0x");
  //Serial.println(b, HEX);
  //Serial.print("Debug: PIN DATA Value (DEC) = ");
  //Serial.println(b, DEC);
  return b;
}

int i;

// Read the 14 configuration bytes at 0x300000..0x30000D into 'out'.
// Returns number of bytes read.
size_t readConfigBits(uint8_t* bits) {
  if (!bits) return 0; // check if pointer is NULL, return zero if yes
  Serial.print("Starting at Address 0x");
  Serial.println(CONFIG_ADDR, HEX);
  setTBLPTR(CONFIG_ADDR);
  for (size_t i = 0; i < CONFIG_BYTES_TO_READ; ++i){ // do this 14 times
    bits[i] = tblReadPI();
    //Serial.println(bits[i]);
  }
  return CONFIG_BYTES_TO_READ;
}

void enterProgramMode() {

  //CLK and DATA lines must be pulled LOW first
  //MCLR must have a higher voltage than VDD to enter HVP
  //VDD is supplied by external power supply (regular ol' battery)
  //Goal is to switch VDD on via FET or transistor from external batt

  // Lower MCLR, VDD, DATA, and CLOCK to start in a reset/power off state
  digitalWrite(PIN_MCLR, LOW);
  digitalWrite(PIN_VDD, LOW);
  digitalWrite(PIN_DATA, LOW);
  digitalWrite(PIN_CLOCK, LOW);

  // Wait for the lines to settle.
  delay(10);

  digitalWrite(PIN_MCLR, HIGH);
  delay(10);
  digitalWrite(PIN_VDD, HIGH);
  delay(10);
  
  // Switch DATA and CLOCK into outputs.
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
}

void exitProgramMode() {

    // Lower MCLR, VDD, DATA, and CLOCK.
    digitalWrite(PIN_MCLR, LOW);
    digitalWrite(PIN_VDD, LOW);
    digitalWrite(PIN_DATA, LOW);
    digitalWrite(PIN_CLOCK, LOW);

    // Float the DATA and CLOCK pins.
    pinMode(PIN_DATA, INPUT);
    pinMode(PIN_CLOCK, INPUT);
}

void printHex1(unsigned int value) {
    if (value >= 10)
        Serial.print((char)('A' + value - 10));
    else
        Serial.print((char)('0' + value));
}

void printHex4(unsigned int word) {
    printHex1((word >> 12) & 0x0F);
    printHex1((word >> 8) & 0x0F);
    printHex1((word >> 4) & 0x0F);
    printHex1(word & 0x0F);
}

void printHex8(unsigned long word) {
    unsigned int upper = (unsigned int)(word >> 16);
    if (upper) printHex4(upper);
    printHex4((unsigned int)word);
}

void printMenu() {
  Serial.println("Enter one of the following values to execute a programming sequence:");
  Serial.println("1 - Bulk Erase");
  Serial.println("2 - Read Configuration Bits");
  Serial.println("3 - Enter Program Mode");
  Serial.println("4 - Exit Program Mode");
  Serial.println("5 - Test MCLR Write");
  Serial.println("6 - Get Device ID");
  Serial.println();
}

void readDeviceID()
{
    const uint32_t ADDR_DEVIDL = 0x3FFFFE;
    const uint32_t ADDR_DEVIDH = 0x3FFFFF;

    enterProgramMode();

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

//------------------- Switch Case Helpers ------------------

void doBulkErase() {
    Serial.println("Performing Bulk Erase...");
    enterProgramMode();
    // your erase code here
    exitProgramMode();
}

void readConfigCase(uint8_t* config) {
    Serial.println("Reading Config bits...");
    enterProgramMode();
    size_t count = readConfigBits(config);
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

void testMCLR() {
    Serial.println("Testing MCLR pin...");
    digitalWrite(PIN_MCLR, HIGH);
    delay(3000);
    digitalWrite(PIN_MCLR, LOW);
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
    
    if (input >= '1' && input <= '6') {

      Serial.println();

      switch (input) {

        case '1': doBulkErase(); break;

        case '2': readConfigCase(config); break;

        case '3': enterProgramMode(); break;
        
        case '4': exitProgramMode(); break;
        
        case '5': testMCLR(); break;

        case '6': readDeviceID(); break;

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
