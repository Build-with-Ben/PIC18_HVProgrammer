// ------------------- Desired addresses ----------------
#define  CONFIG_ADDR          0x300000   // CONFIG register addresses 0x300000..0x30000D
#define  BULK_ERASE_ADDR1     0x3C0005   // Bulk erase is broken up in two registers
#define  BULK_ERASE_ADDR2     0x3C0004   // ...    

// ------------------- Config Btye Definitions ----------
#define  CONFIG_BYTES_TO_READ 0x0E      // The K22 config space exposes 14 bytes at 0x300000..0x30000D.

// ------------------- 4-bit Commands -------------------
#define  TBLWRITE_CMD        0x0C      // Table Write 4 bit command (bin 1100)
#define  TBLREAD_PI_CMD      0x08      // Table Read Post Increment 4 bit command(bin 1001)
#define  COREINSTR_CMD       0x00      // Core instruction 4 bit command (bin 0000)

// ------------------- Pin mappings ---------------------
#define  PIN_VDD      1     // PIC18 power supply
#define  PIN_MCLR     2     // Data direction register for DATA port
#define  PIN_DATA     3     // PGD = Data = RB6 on PIC
#define  PIN_CLOCK    4     // PGC = Clk = RB7 on PIC

// All delays are in microseconds.
#define DELAY_SETTLE 50 // Delay for lines to settle for reset
#define DELAY_TPPDP 5 // Hold time after raising MCLR
#define DELAY_THLD0 5 // Input Data Hold Time from PGC (P4)
#define DELAY_TDLY1 1 // Delay Between 4-Bit Command and Command Operand
#define DELAY_TDLY2 1 // Delay Between Last PGC falling edge of Command Byte to First PGC (raising) of Read of Data Word

//-------------- Helper Functions ------------------------------

// Quick calls to write clock HIGH or LOW with built-in delay
static inline void setClkHigh() { digitalWrite(PIN_CLOCK, HIGH); delayMicroseconds(DELAY_TDLY1); }
static inline void setClkLow() { digitalWrite(PIN_CLOCK, LOW);  delayMicroseconds(DELAY_TDLY1);  }

//Create and send 4 bit command
//takes in a byte command (i.e 0x00)
void send4Cmd(uint8_t  cmd){
    
    //Data is latched on falling edge of clock.
    //Loop through 4 times by setting CLOCK HIGH and then
    //writng DATA (LOW or HIGH) based on cmd arg
    //Finally, latch DATA by writing CLOCK LOW
    for (uint8_t bit = 0; bit < 4; ++bit) {
        setClkHigh();
        //Check if the least significant bit (LSB) of cmd is 1.
        //If so, send a HIGH signal on the data line; otherwise, send LOW.
        if (cmd & 1)
            digitalWrite(PIN_DATA, HIGH); 
        else
            digitalWrite(PIN_DATA, LOW);
        delayMicroseconds(DELAY_TDLY1); // delay needed between writes
        setClkLow();
        cmd >>= 1;
        bit++;
    }
}

//Create and send 16 bit payload
//takes in a two byte payload (i.e 0x0000)
void send16Payload(uint16_t payload){
    
    //Data is latched on falling edge of clock.
    //Loop through 4 times by setting CLOCK HIGH and then
    //writng DATA (LOW or HIGH) based on cmd arg
    //Finally, latch DATA by writing CLOCK LOW
    for (uint8_t bit = 0; bit < 16; ++bit) {
        setClkHigh();
        //Check if the least significant bit (LSB) of cmd is 1.
        //If so, send a HIGH signal on the data line; otherwise, send LOW.
        if (payload & 1)
            digitalWrite(PIN_DATA, HIGH); 
        else
            digitalWrite(PIN_DATA, LOW);
        delayMicroseconds(DELAY_TDLY1); // delay needed between writes
        setClkLow();
        payload >>= 1;
        bit++;
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

// Table Read, post-increment: returns 1 byte at TBLPTR then TBLPTR++
static uint8_t tblReadPI() {
  send4Cmd(TBLREAD_PI_CMD);

  // First 2 bytes (8 clock cycles) = operand (0x00); keep PGD low as output
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
    if (digitalRead(PIN_DATA)) b |= (1u << i);
    setClkLow();
  }
  return b;
}

// Read the 14 configuration bytes at 0x300000..0x30000D into 'out'.
// Returns number of bytes read.
size_t readConfigBits(uint8_t* out) {
  if (!out) return 0; // check if pointer is NULL, return zero if yes
  setTBLPTR(CONFIG_ADDR);
  for (size_t i = 0; i < CONFIG_BYTES_TO_READ; ++i) // do this 14 times
    out[i] = tblReadPI();
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
  delayMicroseconds(DELAY_SETTLE);

  // Switch DATA and CLOCK into outputs.
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);

  // Raise MCLR, then VDD.
  digitalWrite(PIN_MCLR, HIGH);
  delayMicroseconds(DELAY_TPPDP);
  digitalWrite(PIN_VDD, HIGH);
  delayMicroseconds(DELAY_THLD0);
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

// declarations
int input=0;


// only runs once
void setup() {

  // Establish serial link with PC
  Serial.begin(9600);
  while (!Serial) {
      ;  // wait for serial port to connect. Needed for native USB port only
    }

  Serial.println("\n");
  Serial.println("Enter one of the following values to execute a programming sequence");
  Serial.println("1 - Bulk Erase");
  Serial.println("2 - Read Configuration Bits");
  Serial.println("3 - TBD");
  Serial.println("4 - TBD");
  Serial.println("\n");

  // Set up control lines initially as outputs
  pinMode(PIN_VDD, OUTPUT);
  pinMode(PIN_MCLR, OUTPUT);
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
}

  //Main loop looks for serial input from host 
  //to begin programming sequence chosen and 
  //outputs serial information based on status
void loop() {

  uint8_t config[CONFIG_BYTES_TO_READ];

  if (Serial.available() > 0) {

    // read the incoming byte, returns ASCII value
    input = Serial.read();

    // execute certain sequences based on input (looking for 1, 2, 3, or 4)
    // 1, 2, 3, or 4 = ASCII value of 49-51
    // check if input is in range
    if (input >= 49 && input <= 51) {
      switch (input) {

        // Check if Bulk Erase is selected
        case 49:
          Serial.println("You entered: 1");
          Serial.println("Executing bulk erase program sequence");
          Serial.println("Entering program mode");
          enterProgramMode();
          // Todo: Add bulk erase commands and sequence
          Serial.println("Eraseing data");
          exitProgramMode();
          Serial.println("Action(s) completed. Returning to program options.");
          Serial.println("\n");
          break;
        
        // Check if configuration bits read operation is selected
        case 50:
          Serial.println("You entered: 2");
          Serial.println("Executing configuration bit data read");

          Serial.println("Entering program mode...");
          enterProgramMode();

          Serial.println("Reading configuration bits...");
          size_t count = readConfigBits(config);  // Read configuration memory store in config[], retuns num bytes read

          // Check if read succeeded
          if (count == CONFIG_BYTES_TO_READ) {
              Serial.println("Config bytes read successfully:");

              // Print results
              for (size_t i = 0; i < CONFIG_BYTES_TO_READ; i++) {
                  // Determine config word index (i/2 because each word has L+H)
                  size_t wordIndex = i / 2 + 1;  

                  Serial.print("CONFIG");
                  Serial.print(wordIndex);

                  if (i % 2 == 0) {
                      Serial.print("L = 0x");   // Even address → Low byte
                  } else {
                      Serial.print("H = 0x");   // Odd address → High byte
                  }

                  if (config[i] < 0x10) Serial.print("0"); // leading zero
                  Serial.println(config[i], HEX);
              }
          } else {
              Serial.println("Error: Could not read configuration bytes.");
          }
          Serial.println("Exiting program mode...");
          exitProgramMode();
          Serial.println("Action(s) completed. Returning to program options.");
          Serial.println("\n");
          break; 
        case 51:
          Serial.println("You entered: 3");
          Serial.println("Todo..."); 
          Serial.println("Action(s) completed. Returning to program options.");
          Serial.println("\n");
          break;
        case 52:
          Serial.println("You entered: 4");
          Serial.println("Todo...");
          Serial.println("Action(s) completed. Returning to program options.");
          Serial.println("\n");
          break;
      } 
    }
    else {
        Serial.println("Not a valid input. Try Again.");
        Serial.println("\n");
    }
    Serial.println("Enter one of the following values to execute a programming sequence");
    Serial.println("1 - Bulk Erase");
    Serial.println("2 - Read Configuration Bits");
    Serial.println("3 - TBD");
    Serial.println("4 - TBD");
    Serial.println("\n");
  }
}
