/*
Author: Vikram Seshadri
5/9/2019
  
Time to Vac. Fixture v2
 - LED indication for Gauge Pressure ranges
 - Data storage in accordance to the template
 - Serial port data visualization
 - Optimizing data capture
 - Logs every 2 sec.

Description
This code is uploaded onto the Feather 32u4 uSD Adalogger board.

The pressure fixture has the following functionality:
1. Measures Pressure in the surrounding
2. Measures Temperature in the surrounding
3. Indicates Gauge pressure range using RGB LED
4. Indicates battery level every 10 minutes using the RGB LED
5. Logs Sample, Time, Pressure, Temperature, Ambient Pressure and Differential Pressure if SD Card is present
  
Gauge Pressure ranges:
Look for the LED color to determine the gauge pressure range
    RED                diff_pressure > -40
    PURPLE             diff_pressure <= -40 && diff_pressure > -50
    WHITE              diff_pressure <= -50 && diff_pressure > -60
    BLUE               diff_pressure <= -60 && diff_pressure > -70
    GREEN              diff_pressure <= -70 
*/

#include <SD.h>
#include <SPI.h>

/******* Variables *******/

#define SDSS  4
#define SAVE_INTERVAL 2000 //save to SD every 2s //Modify this for data capture rates (2000 - 2s)

//pins
const int red = 5;
const int green = 10;
const int blue = 6;
const int chip_select_4 = 11;
const int chip_select_5 = 12;
const int program_select = 13;

int timesec = SAVE_INTERVAL / 1000;

//commands
const int chipReset = 0x1E;
const int convertP = 0x48; // OSR = 4096, sub 0x02 to increment
const int convertT = 0x58; //OSR = 4096, sub 0x02 to incremenet
const int readADC = 0x00;
const int readPROM = 0xA0; //0xA000 is first, 0xAE is last, 8 values, increment by 0x02

//matrix for calibration coefficents
uint16_t coeff[] = {0, 0, 0, 0, 0, 0, 0, 0}; // 1-6 are the coefficents for data compensation
int32_t dT = 0;

//dtostrf char arrays
char u[6]; //differential pressure
char v[6]; //ambient pressure
char w[6];  //count
char x[12]; //timer
char y[6]; //pressure
char z[6]; //temperature

String dataString = "";

/*************************************************************/
/******* Functions *******/
/* LED Functions */

//setup pins for LED
void setup_led()
{
  pinMode(9, INPUT); //pin w/ voltage divider off bat
  pinMode(8, OUTPUT); //pin for sd LED

  //RGB pins
  pinMode(green, OUTPUT);
  digitalWrite(green, HIGH);

  pinMode(blue, OUTPUT);
  digitalWrite(blue, HIGH);

  pinMode(red, OUTPUT);
  digitalWrite(red, HIGH);
}

//check battery voltage
float checkBattery()
{
  //This returns the current voltage of the battery on a Feather 32u4.
  float measuredvbat = analogRead(9);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat;
}

//light led based on battery charge
void led()
{
  float batVoltage = checkBattery();  //get battery voltage

  if (batVoltage < 3.4) {
    //shine red light if bat low
    digitalWrite(red, LOW);
    digitalWrite(green, HIGH);
    digitalWrite(blue, HIGH);
  }

  else {
    //shine green if bat high
    digitalWrite(red, HIGH);
    digitalWrite(green, LOW);
    digitalWrite(blue, HIGH);
  }
}

void led_off()
{
  digitalWrite(red, HIGH);
  digitalWrite(green, HIGH);
  digitalWrite(blue, HIGH);
}

//function to return LED color based on Gauge pressure
//Range can be adjusted based on need

void sd_led_on(bool rgb_on, float diff_pressure)
{
  if (!rgb_on)
  {
    if(diff_pressure > -40.00)
    {
      digitalWrite(red, LOW);
    }
    else if(diff_pressure <= -40.00 && diff_pressure > -50.00)
    {
      digitalWrite(blue, LOW); //Purple LED blinks every (SAVE_INTERVAL / 1000) seconds
      digitalWrite(red, LOW);
    }
    else if(diff_pressure <= -50.00 && diff_pressure > -60.00)
    {
      digitalWrite(blue, LOW); //White LED blinks every (SAVE_INTERVAL / 1000) seconds
      digitalWrite(red, LOW);
      digitalWrite(green, LOW);
    }
    else if(diff_pressure <= -60.00 && diff_pressure > -70.00)
    {
      digitalWrite(blue, LOW); //Blue LED blinks every (SAVE_INTERVAL / 1000) seconds
    }
    else if(diff_pressure <= -70.00)
    {
      digitalWrite(green, LOW); //Green LED blinks every (SAVE_INTERVAL / 1000) seconds
    }        
  }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Sensor Functions */

//Setup SPI
void start_spi()
{
  //set spi settings
  /*
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV8);
  */
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

}

void end_spi()
{
  SPI.endTransaction();
}

//Select Sensor
void select()
{
  digitalWrite(chip_select_4, LOW);
  digitalWrite(chip_select_5, LOW);
}

//Deselect Sensor
void deselect()
{
  digitalWrite(chip_select_4, HIGH);
  digitalWrite(chip_select_5, HIGH);
}

//Setup Sensor
void setup_sensor()
{
  // reset sensor
  select();
  SPI.transfer(chipReset);
  deselect();
  delay(500);

  //read PROM
  int j = 0;
  for (int i = readPROM; i < 0xAE; i = i + 0x02) {
    if (j == 0) {
      select();
      SPI.transfer(i); //send command
      coeff[j] = SPI.transfer(0xFF); //read response byte
      coeff[j] = coeff[j] << 8; //shift
      coeff[j] = coeff[j] | SPI.transfer(0xFF); //read response byte
      deselect();
    }

    else {
      select();
      SPI.transfer(i); //send command
      coeff[j] = SPI.transfer(0x00); //read response byte
      coeff[j] = coeff[j] << 8; //shift
      coeff[j] = coeff[j] | SPI.transfer(0x00); //read response byte
      deselect();
    }

    j = j + 1;
    delay(10);
  }
}

//Read Digital Pressure
unsigned long read_pressure()
{
  //conversion sequence for pressure
  start_spi();
  delay(50);
  select();
  SPI.transfer(convertP);
  deselect();
  delay(10);


  //read ADC
  unsigned long pressure_resp = 0;
  unsigned long pressure_0 = 0;
  select();
  SPI.transfer(0x00); //send command
  pressure_resp = SPI.transfer(0x00); //read response byte
  pressure_0 = pressure_resp << 16; //shift raw output

  pressure_resp = SPI.transfer(0x00); //read response byte
  pressure_0 = pressure_0 | (pressure_resp << 8); //shift raw and combine with byte

  pressure_resp = SPI.transfer(0x00);
  pressure_0 = pressure_0 | pressure_resp; //combine final byte
  deselect();
  end_spi();

  return pressure_0;
}

//Read Digital Temperature
unsigned long read_temperature()
{
  //conversion sequence for temperature
  start_spi();
  delay(50);
  select();
  SPI.transfer(convertT);
  deselect();
  delay(10);

  //read ADC
  uint32_t temperature_msb = 0;
  uint32_t temperature_mb = 0;
  uint32_t temperature_lsb = 0;
  int32_t temperature_0 = 0;

  select();
  if (SPI.transfer(0x00)) {
    temperature_msb = SPI.transfer(0b00000000);
    temperature_0 = temperature_msb << 16;
    //Serial.print("MSB: ");
    // Serial.print(temperature_msb);

    temperature_mb = SPI.transfer(0b00000000);
    temperature_0 = temperature_0 | (temperature_mb << 8);
    //Serial.print("  MB: ");
    //Serial.print(temperature_mb);

    temperature_lsb = SPI.transfer(0b00000000);
    temperature_0 = temperature_0 | temperature_lsb;
    //Serial.print("  LSB: ");
    //Serial.print(temperature_lsb);
    deselect();
  }

  else {
    deselect();
    delay(10);
    read_temperature();
  }

  end_spi();

  return temperature_0;
}

float calc_temp(int32_t temp)
{
  //calc compensated temperature
  dT = temp - (((int64_t)coeff[5]) * ((int64_t)256));
  int32_t temperature = (int32_t)(2000 + (((int64_t)dT * coeff[6]) / ((int64_t)8388608))); //(int32_t)( ((int32_t)2000) + ( (int32_t)( ((int64_t)dT)*((int64_t)coeff[6]))/((int64_t)8388608)));

  return temperature;
}

float calc_press(unsigned long pressure)
{
  //calc compensated pressure
  int64_t off = ((int64_t)coeff[2]) * ((int64_t)131072) + (((int64_t)coeff[4]) * ((int64_t)dT)) / ((int64_t)64);
  int64_t sens = ((int64_t)coeff[1]) * ((int64_t)65536) + (((int64_t)coeff[3]) * ((int64_t)dT)) / ((int64_t)128);
  long pressure_1 = (((int64_t)pressure) * sens / ((int64_t)2097152) - off) / ((int64_t)32768);

  return pressure_1;
}

/*************************************************************/
/* SD Class */
class CSVFile
{
  public:
    char filenamechar[100];
    File file;
    //SdFat SD;
    String base_filename;
    int file_number;
    String full_filename;

    CSVFile(String bfilename)
    {
      base_filename = bfilename;
      file_number = 1;
      full_filename = bfilename + String(file_number) + String(F(".csv"));
      full_filename.toCharArray(filenamechar, 100);
      //SD.begin(SDSS, SPI_HALF_SPEED);
    }

    void beginNow()
    {
      SD.begin(SDSS);
    }

    void openFile()
    {
      file = SD.open(filenamechar, FILE_WRITE);
    }

    void closeFile()
    {
      file.close();
    }

    void writeToCSV(String data)
    {
      file.print(data);
      file.print(String(F("\r\n")));
    }

    void writeHeaders()
    {
      file.println(String(F("Sample #,Time (days),Pressure (kPa),Temperature (C),Ambient Pressure (kPa),Gauge Pressure (kPa)"))); //Headers
    }

    void _update_file_number()
    {
      // See if the file already exists, if so, see if it has _###
      // If file doesn't exist, add _###
      // If file does exists, make _### 1 more

      while (SD.exists(filenamechar)) {
        file_number += 1;
        full_filename = base_filename + String(file_number) + String(F(".csv"));
        full_filename.toCharArray(filenamechar, 100);
      }
    }
};

//setup csv file object
String filename = String(timesec) + "SPres";
CSVFile csv(filename);
/*************************************************************/
/******* Operation *******/

void setup()
{
  //Setup Board
  setup_led();
  pinMode(program_select, OUTPUT);
  digitalWrite(program_select, LOW); // set program select to SPI

  //set cs pins
  pinMode(SDSS, OUTPUT);
  pinMode(chip_select_4, OUTPUT);
  pinMode(chip_select_5, OUTPUT);

  deselect(); //Deselect Sensor
  delay(500);

  //start serial and SPI
  Serial.begin(115200);
  SPI.begin();

  //setup sensor
  start_spi();
  setup_sensor();
  end_spi();

  Serial.print("Initializing SD card...");
  delay(500);

  if (SD.begin(SDSS)) {
    //setup SD card
    Serial.println("Card Initialized");
    csv.beginNow();
    csv._update_file_number();
    csv.openFile();
    csv.writeHeaders();
    csv.closeFile();
  }

  else {
    Serial.println("No SD card found");
  }

  delay(300);

  //quickly display battery life
  led();
  delay(1700);
  led_off();
}

void loop()
{
  static byte prevMillis = 0; // temp time variable for pressure sensor
  static bool tempFlag = 0;
  static unsigned long pressure_0 = 0;
  static int32_t temp_0 = 0;
  static float temperature_1 = 0;
  static float pressure_1 = 0;
  static float amb_pressure = 0;
  static float diff_pressure = 0;
  static unsigned long loop_timer = millis(); // temp time variable for SD

  static float rgb_count = 0.0; //for battery to reach 10 minutes
  static float count = 0.0; //sample number
  static bool led_on = 0;
  static float battery = 0;
  static bool low_bat = 0;
  double timer = 0;

  /* check battery */

  battery = checkBattery();
  if (battery < 3.25) {
    low_bat = 1;
  }

  else {
    low_bat = 0;
  }

  /* if battery is good record values */
  if (!low_bat) {
    //control for getting data -> 0-(SAVE_INTERVAL/2)s get temp, (SAVE_INTERVAL/2)s - SAVE_INTERVAL s get pressure
    if (millis() - prevMillis >= (SAVE_INTERVAL / 2)) {
      if (!tempFlag) {
        //read temperature
        temp_0 = read_temperature();
        temperature_1 = calc_temp(temp_0);
        delay(300);
      }

      else {
        //read pressure
        pressure_0 = read_pressure();
        pressure_1 = calc_press(pressure_0);
        delay(300);
      }

      tempFlag = !tempFlag;
      prevMillis = millis();
    }

    // control for SD -> record every SAVE_INTERVAL s
    if (millis() - loop_timer >= SAVE_INTERVAL) {
      loop_timer += SAVE_INTERVAL;

      csv.openFile();
      sd_led_on(led_on, diff_pressure); //turn on
      delay(20);

      dtostrf(count, 5, 0, w);
      dataString += w;
      dataString += ',';
      delay(20);

      timer = millis() / (1000);
      timer = timer / (86400); //time in days
      dtostrf(timer, 3, 8, x);
      dataString += x;
      dataString += ',';
      delay(20);

      pressure_1 = pressure_1 / 1000;
      dtostrf(pressure_1, 3, 2, y);
      dataString += y;
      dataString += ',';
      delay(20);

      temperature_1 = temperature_1 / 100;
      dtostrf(temperature_1, 3, 2, z);
      dataString += z;
      dataString += ',';
      delay(20);

      if (count < 3) {
        amb_pressure = amb_pressure + (pressure_1 / 3);
        diff_pressure = 0;
        if (count == 2) {
          dtostrf(amb_pressure, 3, 2, v);
          dataString += v;
          dataString += ',';
        }
      }

      else {
        diff_pressure = pressure_1 - amb_pressure;
        dtostrf(amb_pressure, 3, 2, v);
        dataString += v;
        dataString += ',';
      }

      dtostrf(diff_pressure, 3, 2, u);
      dataString += u;
      sd_led_on(led_on, diff_pressure); //indicates the gauge pressure range
      csv.writeToCSV(dataString);
      delay(20);

      csv.closeFile();

      Serial.print("Sample#: "); Serial.println(w);
      Serial.print("Time(sec.): "); Serial.println(millis() / 1000);
      Serial.print("Pressure(kPa): "); Serial.println(y);
      Serial.print("Temperature(C): "); Serial.println(z);
      Serial.print("Ambient Pressure(kPa): "); Serial.println(v);
      Serial.print("Gauge Pressure(kPa): "); Serial.println(u);
      Serial.print('\n');

      dataString = "";
      led_off(); //turn off led
      rgb_count = rgb_count + 1;
      count = count + 1;
    }

    //control for LED -> every 10 min -> dont want to waste power
    if (rgb_count >= (600000 / SAVE_INTERVAL)) {
      led(); //Turn on Battery indicator every 10 minutes
      rgb_count = 0.0;
      led_on = 1;
    }

    if (led_on) {
      if (rgb_count > 2) {
        led_off(); // turn LED off after 30 secs
        led_on = 0;
      }
    }
  }

  /* Light LED continuously if low bat -> not recording data */
  else {
    led();
  }
}
