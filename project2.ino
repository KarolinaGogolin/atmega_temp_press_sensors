#define F_CPU 16000000UL

//IO expander
#define ADDRESS_WRITE_IO 0x40
#define IODIRA 0x00  // IODIRA register address (GPIOA direction control)
#define GPIOA 0x12   // GPIOA register address (output values for GPIOA)

//temp sensor
// values refer to table 3 in datasheet
#define ADDRESS_READ 0x91 //10010001 // 1001 is for read and write for DS1261 then 3 zeroes correspond to (A2,A1,A0) and the last bit is read //datasheet ref: page 8 SLAVE ADDRESS
#define ADDRESS_WRITE 0x90//10010000 // 1001 is for read and write for DS1261 then 3 zeroes correspond to (A2,A1,A0) and the last bit is write

#define CONFIG_COMMAND 0xAC
#define CONVERT_T_COMMAND 0xEE
#define ACCESS_TH_COMMAND 0xA1
#define ACCESS_TL_COMMAND 0xA2
#define READ_TEMP_COMMAND 0xAA

//pressure sensor
#define BMP280_ADDRESS_WRITE 0xEE
#define BMP280_ADDRESS_READ 0xEF

// #include <avr/io.h>
#include <util/delay.h>
#include <Arduino.h>

const uint8_t NUMBER_DISPLAYED[] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x80};

//calibration variables for pressure
int32_t t_fine;

uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

#define CALIBRATION_BYTES 24
uint8_t calibration_data[CALIBRATION_BYTES];

void initialiseMCP23017()
{ 
//initial config
  bus_start();
  write_byte(ADDRESS_WRITE_IO);
  write_byte(IODIRA); //IODRA address
  write_byte(0x00); //setting output mode
  bus_stop();

  bus_start();
  write_byte(ADDRESS_WRITE_IO);
  write_byte(IODIRA+1); //IODRB address
  write_byte(0x00); //setting output mode
  bus_stop();
}

void initialiseDS1621()
{ 
//initial config
  bus_start();
  write_byte(ADDRESS_WRITE);
  write_byte(CONFIG_COMMAND);
  write_byte(0x00); //setting non-stop mode
  bus_stop();

  bus_start();
  write_byte(ADDRESS_WRITE);
  write_byte(CONVERT_T_COMMAND); //start temperature conversion
  bus_stop();
}

void initialise_BMP280()
{
  bus_start();
  write_byte(BMP280_ADDRESS_WRITE);
  write_byte(0xE0); //reset command
  write_byte(0xB6); //reset value
  bus_stop();

  _delay_ms(10);

  bus_start();
  write_byte(BMP280_ADDRESS_WRITE);
  write_byte(0xF4); // ctrl_meas register
  write_byte(0x27); // x1 temp, x1 pressure 11- normal mode

  read_calibration_data();

   _delay_ms(100); //delay to get valid first reading
}

// // Function to read calibration parameters and store them in global variables
void read_calibration_data() {
    bus_start();
    write_byte(BMP280_ADDRESS_WRITE);
    write_byte(0x88);                 // Start of calibration data
    bus_stop();

    bus_start();
    write_byte(BMP280_ADDRESS_READ); // Device read address
    for (int i = 0; i < CALIBRATION_BYTES-1; ++i) {
        calibration_data[i] = read_byte(1);
    }
    calibration_data[CALIBRATION_BYTES-1] = read_byte(0);
    bus_stop();

    // Parse calibration data by combining 2 indexes into one variable
    dig_T1 = (calibration_data[1] << 8) | calibration_data[0];
    dig_T2 = (calibration_data[3] << 8) | calibration_data[2];
    dig_T3 = (calibration_data[5] << 8) | calibration_data[4];

    dig_P1 = (calibration_data[7] << 8) | calibration_data[6];
    dig_P2 = (calibration_data[9] << 8) | calibration_data[8];
    dig_P3 = (calibration_data[11] << 8) | calibration_data[10];
    dig_P4 = (calibration_data[13] << 8) | calibration_data[12];
    dig_P5 = (calibration_data[15] << 8) | calibration_data[14];
    dig_P6 = (calibration_data[17] << 8) | calibration_data[16];
    dig_P7 = (calibration_data[19] << 8) | calibration_data[18];
    dig_P8 = (calibration_data[21] << 8) | calibration_data[20];
    dig_P9 = (calibration_data[23] << 8) | calibration_data[22];
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t compensate_temperature(int32_t raw_temp) {
    int32_t var1, var2, T;
    var1 = ((((raw_temp >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((raw_temp >> 4) - ((int32_t)dig_T1)) * ((raw_temp >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t compensate_pressure(int32_t raw_press) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    p = 1048576 - raw_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (uint32_t)p; // Pressure in Pa
}

float read_temperature()
{
   uint8_t data[3];

   bus_start();
    write_byte(BMP280_ADDRESS_WRITE);
    write_byte(0xFA); // Send register address to read press
    bus_stop();

    bus_start();
    write_byte(BMP280_ADDRESS_READ); // Device read address
    data[0] = read_byte(1);        // Read MSB
    data[1] = read_byte(1);        // Read MSB
    data[2] = read_byte(0);        // Read XLSB
    bus_stop();

    //MSB needs to be in bits 19-12 so shift left 12
    //LSB needs to be in bits 11-4 so shift left 4
    //XLSB needs to be in bits 3-0 but it contains data in upper 4 bits so shift right by 4
    uint32_t temp = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4);
    
    return ((float)compensate_temperature(temp))/100.0;
}

float read_raw_pressure()
{
    uint8_t data[3];

    bus_start();
    write_byte(BMP280_ADDRESS_WRITE); // Device write address
    write_byte(0xF7); // Send register address to read pressure
    bus_stop();

    bus_start();
    write_byte(BMP280_ADDRESS_READ); // Device read address
    data[0] = read_byte(1);        // Read MSB
    data[1] = read_byte(1);        // Read LSB
    data[2] = read_byte(0);        // Read XLSB
    bus_stop();

    uint32_t pressure = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4);

    return ((float)compensate_pressure(pressure))/256.0;  // Return pressure in Pa
}

void bus_start()
{
  PORTC |= (1 << PC4) | (1 << PC5); // SDA and SCL high
  PORTC &= ~(1 << PC4);            // SDA low (start cond)
  _delay_us(4);
  PORTC &= ~(1 << PC5);            // SCL low
}

void bus_stop()
{
  PORTC &= ~(1 << PC4); // SDA low
  PORTC |= (1 << PC5);  // SCL high
  _delay_us(4);
  PORTC |= (1 << PC4);  // SDA high
}

void write_byte(uint8_t data)
{
  //starting with i=7 so MSB
  for (int i = 7; i >= 0; --i)
  {
      if (data & (1 << i)) PORTC |= (1 << PC4); //if data is 1 pull SDA high
      else PORTC &= ~(1 << PC4); //else pull SDA low

      //pulse the clock
      _delay_us(2);
      PORTC |= (1 << PC5); //SCL high
      _delay_us(2);
      PORTC &= ~(1 <<PC5); // SCL low
  }

   // Release the SDA line (set as input to read ACK)
    DDRC &= ~(1 << PC4); // Set SDA as input and send ACK

  //toggle to clock ACK bit
    _delay_us(2);
    PORTC |= (1 << PC5); // SCL high 
    _delay_us(2);
    PORTC &= ~(1 << PC5); // SCL low

    DDRC |= (1 << PC4); // Set SDA back to output mode
}

uint8_t read_byte(uint8_t ack)
{
    uint8_t data = 0; // Initialize empty data

    // Configure SDA as input to read data
    DDRC &= ~(1 << PC4); // Set SDA (PC4) as input

    for (int i = 7; i >= 0; --i)
    {
        _delay_us(2);
        PORTC |= (1 << PC5); // SCL high (start clock pulse)

        //SDA is read on the rising edge of SCL
        if (PINC & (1 << PC4)) data |= (1 << i); // If SDA is high, set the bit in 'data'
        else data &= ~(1 << i); // If SDA is low, clear the bit in 'data'

        _delay_us(2);
        PORTC &= ~(1 << PC5); // SCL low (end clock pulse)
    }

    DDRC |= (1 << PC4);       // Configure SDA as output
      if (ack) PORTC &= ~(1 << PC4); // SDA low for ACK
      else PORTC |= (1 << PC4);  // SDA high

    _delay_us(2);
    PORTC |= (1 << PC5); // SCL high
    _delay_us(2);
    PORTC &= ~(1 << PC5); // SCL low

    return data;
}

int* transform_temp(float temperature)
{
  if (temperature < 0) temperature = -temperature;

  static int temp[3] = {0}; 
  temp[2] = (int)(temperature * 10) % 10; 
  
  int integer_part = (int)temperature;
  temp[1] = integer_part % 10;  // Ones digit
  temp[0] = (integer_part / 10) % 10;  // Tens digit

  return temp;
}

void update_display(float temperature) {
  int* temp = transform_temp(temperature);  // Get digits

  bus_start();
  write_byte(ADDRESS_WRITE_IO);  // MCP23017 address with write
  write_byte(GPIOA);  // GPIOA address
    if (temperature < 0)  write_byte(NUMBER_DISPLAYED[temp[0]] | 0x80); // if the temp is negative turn the red LED on
    else write_byte(NUMBER_DISPLAYED[temp[0]]);  
  bus_stop();

  bus_start();
  write_byte(ADDRESS_WRITE_IO);  // MCP23017 address with write
  write_byte(GPIOA+1);  // GPIOB address
    if (temp[2] != 0) write_byte(NUMBER_DISPLAYED[temp[1]] | 0x80); // if the temp is not an integer (has .50 at the end) also write 1 to yellow LED  
    else write_byte(NUMBER_DISPLAYED[temp[1]]); //todo writhe the led pin);  
  bus_stop();
}

float read_temp()
{
  bus_start();
  write_byte(ADDRESS_WRITE);  // DS1621 Address with Write
  write_byte(READ_TEMP_COMMAND);  // Read Temperature Command (0xAA)
  bus_stop();

  bus_start();
  write_byte(ADDRESS_READ);
  uint8_t msb = read_byte(1); // int part                       //sending ACK to indicate there is more data to read
  uint8_t lsb = read_byte(0); // decimal part in 0.5 increments //sending NACK to indicate the end
  bus_stop();

  float temp = msb; // setting int temp

  if (lsb & 0x80) temp += 0.5;// if lsb == 100000000 then add a 0.5

  return temp;
}

int main(void)
{
  Serial.begin(9600); 

  DDRC |= (1 << PC4) | (1 << PC5);  // A4 (SDA) and A5 (SCL) set to outputs
  PORTC |= (1 << PC4) | (1 << PC5); // Enabling pullup resistors

  initialiseDS1621();
  initialiseMCP23017();
  initialise_BMP280();

  while(1)
  {
    // Read all measurements first
    float ds_temp = read_temp();
    float bmp_temp = read_temperature();
    float raw_pressure = read_raw_pressure();
    
    //display temp
    update_display(ds_temp);
    
    // Printing
    Serial.println("DS1621 Temperature: " + String(ds_temp, 2) + "°C");
    Serial.println("BMP280 Temperature: " + String(bmp_temp,2) + "°C");
    Serial.println("BMP280 Pressure: " + String(raw_pressure,2) + " Pa");
    
    Serial.flush();
    
    _delay_ms(10000);
  }
  return 0;
}
