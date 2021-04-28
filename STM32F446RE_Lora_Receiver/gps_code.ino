#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <LoRa.h>
int uartread;


HardwareSerial UART(A1, A0);//rx tx
TinyGPSPlus gps;
void setup()
{
  Serial.begin(115200);
  UART.begin(115200); 
  Serial.println("LoRa Sender");

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  

  
      
}

void loop()
{
   
if (UART.available()) {
  uartread = UART.read();
  Serial.println(uartread);

}
  

  LoRa.beginPacket();
  //LoRa.print(uartread);
  LoRa.print("Sended");
  LoRa.endPacket();

 
 delay(5000);
}
