/* Lawrence Bandini

   Function:
   1. Send GPS data from a HELTEC WIRELESS TRACKER (esp32) device over RF
   https://github.com/rapbando/HELTEC-LoRaSender-GPS-TRACKER
 * */
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_st7735.h"
#include "HT_TinyGPS++.h"

// Objects for GNSS parsing and serial communication
TinyGPSPlus GPS;
HT_st7735 st7735;


// Pin definitions for GNSS module communication
const int GNSS_RXPin = 34;
const int GNSS_TXPin = 33;
const int GNSS_RSTPin = 35;
const int GNSS_PPS_Pin = 36;

double Bat_threshold = 3.3;

#define Vext 3

#define RF_FREQUENCY                                868125000 // Hz

#define TX_OUTPUT_POWER                             21        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       11         // [SF7..SF12]
#define LORA_CODINGRATE                             4         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        16         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 110 // Define the payload size here

#define VBAT_READ 1
#define ADC_CTRL 2

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

double txNumber;

char LAT[8];
char LON[8];


bool lora_idle = true;

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );

float readBatLevel() {
  pinMode(ADC_CTRL, OUTPUT);
  digitalWrite(ADC_CTRL, HIGH);
  delay(50); // let GPIO stabilize
  int analogValue = analogRead(VBAT_READ);
  Serial.println("BatLevel = " + String(analogValue));
  float voltage = 4.9 * analogValue;
  Serial.println("Voltage = " + String(voltage));
  return voltage;

}


void setup() {
  pinMode(1, INPUT);
  pinMode(18, OUTPUT);

  delay(100);
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);




  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                     LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                     LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                     true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

  delay(100);
//  st7735.st7735_init();

}

void loop()
{
  float bV = readBatLevel();
  if (bV > Bat_threshold)
  {
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, HIGH); // Give power to GNSS module and display
    digitalWrite(18, HIGH); // Turn ON LED
    Serial1.begin(115200, SERIAL_8N1, GNSS_TXPin, GNSS_RXPin);  // Start GNSS module communication
  //  st7735.st7735_fill_screen(ST7735_BLACK);
 //   delay(100);
 //   st7735.st7735_write_str(0, 0, (String)"Init...");
    Serial.println("Waiting 30 sec for GNSS fix...");
    delay(30000); // Give time for GNSS fix


    while (Serial1.available() > 0)
    {

      if (Serial1.peek() != '\n')
      {
        GPS.encode(Serial1.read());
      }
      else
      {
        Serial1.read();
        if (GPS.date.day() == 0)
        {
          continue;
        }

 //       st7735.st7735_fill_screen(ST7735_BLACK);
 //       st7735.st7735_write_str(0, 0, (String)"f TX 868.1 MHz");
        digitalWrite(18, HIGH); // Turn ON LED
         
        String date_str = "D: " + (String)GPS.date.year() + ":" + (String)GPS.date.month() + ":" + (String)GPS.date.day();
//        Serial.println(date_str);
        
        String time_str = "T: " + (String)GPS.time.hour() + ":" + (String)GPS.time.minute() + ":" + (String)GPS.time.second();
//        Serial.println(time_str);
 //       st7735.st7735_write_str(0, 20, time_str);
        
        float latitude = GPS.location.lat();
//        Serial.println(latitude,5);
        dtostrf(latitude, 7, 4, LAT);
        
        float longitude = GPS.location.lng();
//        Serial.println(longitude,5);
//        String LON = "LON: " + (String)longitude;
        dtostrf(longitude, 7, 4, LON);

        String Speed = "S: " + (String)GPS.speed.kmph();
//        Serial.println(Speed);





        txNumber += 0.01;
        sprintf(txpacket,"SHADY T # %0.2f \n %s \n %s \n LAT: %s \n LON: %s \n %s \n V: %f \n",txNumber, date_str, time_str, LAT, LON, Speed, bV); //start a package
        Serial.printf(txpacket);
        if ((latitude != 00)) {
          
          Serial.printf("\r\nsending packet \"%s\", length %d\r\n",txpacket,strlen(txpacket));
          Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); //send the package OTA
          lora_idle = false;
        }
        Radio.IrqProcess( );

        
        while (Serial1.read() > 0);
        
//        digitalWrite(Vext, LOW);  // Remove power to GNSS module and display after packet TX
        digitalWrite(18, LOW);  // Turn off LED
        delay(12000); // Send the OTA package every 2 min
      }

    }
  }


  else {
    digitalWrite(18, LOW);  // Turn off LED
    digitalWrite(Vext, LOW); // Remove power to GNSS module and display
//    Serial.println("Battery is drained, going to sleep 30 min...");
//    radio.sleep()
    delay(1800000);
  }
}





void OnTxDone( void )
{
  Serial.println("TX done......");
  lora_idle = true;

}

void OnTxTimeout( void )
{
  Radio.Sleep( );
  Serial.println("TX Timeout......");
  lora_idle = true;
}

