//**************************** Bibliotheken ****************************************************/
#include <Wire.h>  
#include <SoftwareSerial.h> // Use software serial since the hardware serial port is used by the USB interface. 
//Make sure to use ESP8266 version of SoftwareSerial library. See "EspSoftwareSerial".
#include <ModbusMaster.h> //https://github.com/4-20ma/ModbusMaster //For Modbus 

//************************ DDS238-4W **********************************************/
/*
* DDS238-4W Energy Meter to Modbus RS485
*
* Modbus registers:
* Total kWh: 0x0000, 0x0001, Data: 2, Format: xxxxxx.xx, Unit kWh
* Export kWh: 0x0008, 0x0009, Data: 2, Format: xxxxxx.xx, Unit kWh
* Import kWh: 0x000A, 0x000B, Data: 2, Format: xxxxxx.xx, Unit kWh
* Voltage: 0x000C, Data: 1, Format: xxx.x, Unit: V
* Current: 0x000D, Data: 1, Format: xx.xx, Unit: A
* Active Power: 0x000E, Data: 1, Format: xxx.xxx, Unit: kW
* Reactive Power: 0x000F, Data: 1, Format: xxx.xxx, Unit: kvar
* PF: 0x0010, Data: 1, Format: x.xxx, Unit:
* ID + Baud rate: 0x000C, Data: 1, Format: Byte 1 = ID, Byte 2 = Baud (01=9600,02=4800,03=2400,04=1200)
* Read Function Code 03
* Write Function Code 10
* Each "data" is 2 bytes. So Total kWh register has 4 bytes.
*/

//dds238 x registers
#define dds238_VOLTAGE                     0x000C                              //V   Data length type 0x01 - use index "2"
#define dds238_CURRENT                     0x000D                              //A   Data length type 0x01 - use index "2"
#define dds238_POWER                       0x000E                              //W   Data length type 0x01 - use index "2"
#define dds238_REACTIVE_APPARENT_POWER     0x000F                              //VAR Data length type 0x01 - use index "2"
#define dds238_POWER_FACTOR                0x0010                              //    Data length type 0x01 - use index "2"
#define dds238_FREQUENCY                   0x0011                              //Hz  Data length type 0x01 - use index "2"
#define dds238_IMPORT_ACTIVE_ENERGY        0x000A                              //Wh  Data length type 0x02 - use index "1"
#define dds238_EXPORT_ACTIVE_ENERGY        0x0008                              //Wh  Data length type 0x02 - use index "1"
#define dds238_TOTAL_ACTIVE_ENERGY         0x0000                              //Wh  Data length type 0x02 - use index "1"

double Voltage;
double Current;
double Power;
double ReactivePower;
double Import_kWh;
double Export_kWh;
double Total_kWh;
double PF;
double Frequency;
  
char Voltage_str[5];
char Current_str[8];
char Power_str[10];
char ReactivePower_str[10];
char Import_kWh_str[10];
char Export_kWh_str[10];
char Total_kWh_str[10];
char PF_str[7];
char Frequency_str[5];

//************************ Init ModbusMaster Object ********************************************/

ModbusMaster node;
SoftwareSerial mySerial(14,12); // using pins 14 for RX and 12 for TX

// Setup wird nur einmal durchlaufen ***********************************************************/
void setup() {
  Serial.begin(115000); //Serielle Verbindung starten
  Serial.println("Booting");
  
  Serial.println("Start");
  while(!Serial) { }
  
  // using SoftwareSerial initialize Modbus communication baud rate
  mySerial.begin(9600);

  // communicate with Modbus slave ID 0 over Serial.
  node.begin(0, mySerial);

} // Ende void setup()



// Continuous Program **************************************************************************/
void loop() {
  
  static uint32_t i;
  uint8_t j, result;
  uint16_t data[6];

    //***************** Read from Holding Registers = Modbus function 0x03 *********************/
    // DDS238
    //******************************************************************************************/
    result = node.readHoldingRegisters(dds238_TOTAL_ACTIVE_ENERGY, 4);
    if (result == node.ku8MBSuccess){
    for (j = 0; j <= 1; j++)
        {
          data[j] = node.getResponseBuffer(j);
          Serial.printf("Data: %d Inhalt: %d \n",j,data[j]);
        }
    Total_kWh = float(data[0]) + float(data[1])/100.000;
    dtostrf(Total_kWh,7,2,Total_kWh_str);
    Serial.printf("Total kWh: %.2f kWh \n",Total_kWh);
    Serial.printf("Total kWh: %s kWh \n",Total_kWh_str);  
    }   

    delay(100);
    result = node.readHoldingRegisters(0x000c, 2); // Voltage, unsigned WORD, XXX.X
    if (result == node.ku8MBSuccess)
    {
       data[0] = node.receive();
       Voltage = float(data[0])/10.000;
       dtostrf(Voltage,5,1,Voltage_str);
       Serial.printf("Voltage: %.1f V \n",Voltage);
       Serial.printf("Voltage: %s V \n",Voltage_str);
       }
    
    delay(100);
    result = node.readHoldingRegisters(0x000d, 2); // Current, unsigned WORD, XX.XX
    if (result == node.ku8MBSuccess)
    {
       data[0] = node.receive();
       Current = float(data[0])/100.000;
       dtostrf(Current,4,2,Current_str);
       Serial.printf("Current: %.2f A \n",Current);
       Serial.printf("Current: %s A \n",Current_str);
    }  
    
    delay(100);
    result = node.readHoldingRegisters(0x000e, 2); //Active Power, unsigned WORD, XXX.XXX
    if (result == node.ku8MBSuccess)
    {
       data[0] = node.receive();
       Power = float(data[0]);
       dtostrf(Power,5,1,Power_str);
       Serial.printf("Active Power: %.1f W \n",Power); 
       Serial.printf("Active Power: %s W \n",Power_str); 
    }     

    delay(100);
    result = node.readHoldingRegisters(0x000F, 2);  //Reactive Power, unsigned WORD, XXX.XXX
    if (result == node.ku8MBSuccess)
    {
       data[0] = node.receive();
       ReactivePower = (data[0]);
       dtostrf(ReactivePower,5,1,ReactivePower_str);
       Serial.printf("Reactive Power: %.1f VAR \n",ReactivePower); 
       Serial.printf("Reactive Power: %s VAR \n",ReactivePower_str); 
     }

    delay(100);
    result = node.readHoldingRegisters(0x0010, 2);  // PowerFactor , Format: X.XXX
    if (result == node.ku8MBSuccess)
    {
       data[0] = node.receive();
       PF = float(data[0])/1000.000;
       dtostrf(PF,5,3,PF_str);
       Serial.printf("Power Factor: %.3f \n",PF); 
       Serial.printf("Power Factor: %s \n",PF_str); 
    }    
    
    delay(100);
    result = node.readHoldingRegisters(0x00011, 2);  // Frequency, Format: XX.XX
    if (result == node.ku8MBSuccess)
    {
       data[0] = node.receive();
       Frequency = float(data[0])/100.000;
       dtostrf(Frequency,5,2,Frequency_str);
       Serial.printf("Frequency: %.2f \n",Frequency); 
       Serial.printf("Frequency: %s \n",Frequency_str); 
     }
     
    Serial.println("Fertig !");
    delay(2000);
    
   
}  // End void loop
