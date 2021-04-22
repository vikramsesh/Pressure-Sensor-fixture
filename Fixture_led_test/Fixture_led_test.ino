 /* 
 *  Author: Vikram Seshadri
 *  02/20/2019
 *  
 *  Requirements: 
 *  1. Feather 32u4 uSD Adalogger board
 *  2. Pressure fixture PCB
 *  
 *  Description:
 *  This is a test code for the pressure fixture LEDs. This code helps to check if the LED on the pressure fixture PCB emits all the colors required.
 *  This code is necessary as gauge pressure range can be noted using these LEDs.
 *  
 */
const int red = 5;
const int green = 10;
const int blue = 6;

void setup() {

  //pinMode(9, OUTPUT);
  pinMode(8,OUTPUT);

  //RGB pins
  pinMode(green, OUTPUT);
  digitalWrite(green, HIGH);

  pinMode(blue, OUTPUT);
  digitalWrite(blue, HIGH);

  pinMode(red, OUTPUT);
  digitalWrite(red, HIGH); 

  Serial.begin(115200);

}

void loop() 
{

  digitalWrite(red,LOW);      //Red LED
  delay (2000);
  digitalWrite(red,HIGH);
  delay (2000);

  digitalWrite(green,LOW);    //Green LED
  delay (2000);
  digitalWrite(green,HIGH);
  delay (2000);

  digitalWrite(blue,LOW);     //Blue LED
  delay (2000);
  digitalWrite(blue,HIGH);
  delay (2000);

  digitalWrite(green,LOW);    //Cyan
  digitalWrite(blue,LOW);
  delay (2000);
  digitalWrite(green,HIGH);
  digitalWrite(blue,HIGH);
  delay (2000);

  digitalWrite(red,LOW);      //Purple
  digitalWrite(blue,LOW);
  delay (2000);
  digitalWrite(red,HIGH);
  digitalWrite(blue,HIGH);
  delay (2000);

  digitalWrite(red,LOW);      //Yellow
  digitalWrite(green,LOW);
  delay (2000);
  digitalWrite(red,HIGH);
  digitalWrite(green,HIGH);
  delay (2000);

  digitalWrite(red,LOW);      //White
  digitalWrite(green,LOW);
  digitalWrite(blue,LOW);
  delay (2000);
  digitalWrite(red,HIGH);
  digitalWrite(green,HIGH);
  digitalWrite(blue,HIGH);
  delay (2000);
      
}
