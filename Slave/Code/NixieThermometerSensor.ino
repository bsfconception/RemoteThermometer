//#define SLEEP_DURATION  1     // 10 sec, test purpose
//#define SLEEP_DURATION  60     // 1 min
//#define SLEEP_DURATION  300     // 5 min
#define SLEEP_DURATION  900     // 15 min

#define WIFI_AP_NAME1   "BSFClock1"
#define WIFI_AP_NAME2   "BSFClock2"
#define WIFI_AP_PWD  ""
//#define DHT_SENSOR
#define DS_SENSOR

#define COM_LED   5
#define DEV_SELECT0 14
#define DEV_SELECT1 12
#define DEV_SELECT2 13



#define DEBUG_MODE
//#define TEST_MODE
#define BOARD_LED 2

//#define FORCE_SIMULATION
#define SIMUL_ADDR  1
#define SIMUL_TEMP  -18
#define SIMUL_RSSI  -70


#define UDP_MODE




//-- Libraries Included --------------------------------------------------------------
#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

#ifdef DHT_SENSOR
  #include "DHT.h"
#endif

#ifdef DS_SENSOR
  #include <OneWire.h>
  #include <DallasTemperature.h>
#endif


long    g_CorrSleepDuration = 0;

//------------------------------------------------------------------------------------

#ifdef DHT_SENSOR
  #define DHTPIN 4  
  #define DHTTYPE DHT11   // DHT 11
  //#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
  //#define DHTTYPE DHT21   // DHT 21 (AM2301)
  DHT dht(DHTPIN, DHTTYPE);
#endif

#ifdef DS_SENSOR
  #define ONE_WIRE_BUS 4
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature sensors(&oneWire);
#endif

//------------------------------------------------------------------------------------
  // Authentication Variables
  char*       ssid;              // SERVER WIFI NAME
  char*       password;          // SERVER PASSWORD
  const String  Devicename = "Device_1";
  short g_ModuleAddr = 0;
  long g_Start;

//------------------------------------------------------------------------------------
  // WIFI Module Role & Port    
  IPAddress APlocal_IP(192, 168, 4, 2);
  //IPAddress    apIP(10, 10, 10, 1);
  IPAddress APgateway(192, 168, 4, 1);
  IPAddress APsubnet(255, 255, 255, 0);



#ifdef UDP_MODE
  unsigned int UDPPort = 2390;      // local port to send and listen to  
  WiFiUDP Udp;
#endif



//------------------------------------------------------------------------------------
  // Some Variables
  char result[32];              // Buffer big enough for 7-character float
  char packetBuffer[255];       // buffer for incoming data
  String Message;
//====================================================================================

void setup() {
short res;
  pinMode(DEV_SELECT0, INPUT_PULLUP);
  pinMode(DEV_SELECT1, INPUT_PULLUP);
  pinMode(DEV_SELECT2, INPUT_PULLUP);
  pinMode(COM_LED, OUTPUT);
  digitalWrite( COM_LED, LOW );
  g_ModuleAddr = digitalRead(DEV_SELECT0) + 2*digitalRead(DEV_SELECT1);

  digitalWrite( COM_LED, HIGH );
  delay(25);
  digitalWrite( COM_LED, LOW );



#ifdef DEBUG_MODE  
  pinMode(BOARD_LED, OUTPUT);
  g_Start = millis();
  Serial.begin(9600);
  Serial.println("");

  Serial.print("Module Address:");
  Serial.println(g_ModuleAddr );
  digitalWrite( BOARD_LED, LOW );
#endif

  g_CorrSleepDuration = random( SLEEP_DURATION / 10 ) - (SLEEP_DURATION/20);
  g_CorrSleepDuration = g_CorrSleepDuration + SLEEP_DURATION;

#ifdef DEBUG_MODE  
  Serial.print("I will sleep for:");
  Serial.println(g_CorrSleepDuration);
#endif

  
  
#ifdef UDP_MODE
  // WiFi Connect ----------------------------------------------------  
  res = Check_WiFi_and_Connect();
  if(res == 0)
  {
    for(int i=0;i<3;i++)
    {
      digitalWrite( COM_LED, HIGH );
      delay(25);
      digitalWrite( COM_LED, LOW );
      delay(150);
    }
    #ifndef TEST_MODE
      ESP.deepSleep(g_CorrSleepDuration * 1000000);
    #endif
  }
#endif


  
}

//====================================================================================

void loop() {

#ifdef UDP_MODE


  Send_Data_To_Server();


#ifdef DEBUG_MODE  
  Serial.print("Duration:");
  Serial.println(millis() - g_Start);
#endif


#ifdef TEST_MODE
  delay(5000);
#else
  delay(100);
  ESP.deepSleep(g_CorrSleepDuration * 1000000);
#endif  


#endif


}

//====================================================================================
#ifdef UDP_MODE


void Send_Data_To_Server() {
long tNow, tNow2 ;

  long rssi = WiFi.RSSI();
  
#ifdef DEBUG_MODE  
  Serial.print("RSSI:");
  Serial.println(rssi);
#endif

#ifdef DHT_SENSOR
//  float t = dht.readTemperature();
#endif

#ifdef DS_SENSOR
  #ifndef FORCE_SIMULATION
    sensors.requestTemperatures(); // Send the command to get temperatures
    tNow = (long) sensors.getTempCByIndex(0);  
    sensors.requestTemperatures(); // Send the command to get temperatures
    tNow2 = (long) sensors.getTempCByIndex(0);  
    //compare temp
    if( tNow != tNow2 )
    {
      sensors.requestTemperatures(); // Send the command to get temperatures
      tNow = (long) sensors.getTempCByIndex(0);  
    }
    Message = "ID:";
    Message = Message + (String)(g_ModuleAddr);
    Message = Message + "T:";
    Message = Message + (String)(tNow);
    Message = Message + "RSSI:";
    Message = Message + (String)(rssi);
  #else
    Message = "ID:";
    Message = Message + (String)(SIMUL_ADDR);
    Message = Message + "T:";
    Message = Message + (String)(SIMUL_TEMP);
    Message = Message + "RSSI:";
    Message = Message + (String)(SIMUL_RSSI);
  #endif


#endif
    
#ifdef DEBUG_MODE  
  Serial.print("Sending Data:");
  Serial.println(Message.c_str());
#endif  

  Udp.beginPacket(APlocal_IP, UDPPort);                         // the IP Adress must be known 
  // Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());         // this can be used, to answer to a peer, if data was received first
  Udp.write(Message.c_str());
  Udp.endPacket();                                              // this will automatically send the data

/*  
  digitalWrite( COM_LED, HIGH );
  delay(25);
  digitalWrite( COM_LED, LOW );
*/

  tNow = millis();
  
  while(1)
  {
    int packetSize = Udp.parsePacket();
    if (packetSize) 
    {
#ifdef DEBUG_MODE  
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
#endif      
      IPAddress remoteIp = Udp.remoteIP();
      
#ifdef DEBUG_MODE  
      Serial.print(remoteIp);
      Serial.print(", port ");
      Serial.println(Udp.remotePort());
#endif
      // read the packet into packetBufffer
      int len = Udp.read(packetBuffer, 255);
      if (len > 0) 
      {
        packetBuffer[len] = 0;
      }
#ifdef DEBUG_MODE  
      Serial.print("Contents:");
      Serial.println(packetBuffer);
#endif      
      digitalWrite( COM_LED, HIGH );
      delay(25);
      digitalWrite( COM_LED, LOW );
      delay(150);      
      break;                                                    // exit the while-loop
    }
    if((millis()-tNow)>2000)
    {                                   // if more then 1 second no reply -> exit
#ifdef DEBUG_MODE  
      Serial.println("timeout");
#endif      

      for(int i=0;i<2;i++)
      {
        digitalWrite( COM_LED, HIGH );
        delay(25);
        digitalWrite( COM_LED, LOW );
        delay(150);
      }
      break;                                                    // exit
    }
  }

}

//====================================================================================

short Check_WiFi_and_Connect()
{
long tNow;

#ifdef DEBUG_MODE  
    Serial.println();
    Serial.print("Wait for WiFi : ");
#endif
  
    if( digitalRead(DEV_SELECT2) )
    {
      WiFi.begin(WIFI_AP_NAME1,WIFI_AP_PWD);                                 // reconnect to the Network
     Serial.print(WIFI_AP_NAME1);
    }
    else
    {
      WiFi.begin(WIFI_AP_NAME2,WIFI_AP_PWD);                                 // reconnect to the Network
     Serial.print(WIFI_AP_NAME2);
    }
    

    tNow = millis();
    while (WiFi.status() != WL_CONNECTED) 
    {
      delay(500);
#ifdef DEBUG_MODE  
      Serial.print(".");
#endif
      if( (millis() - tNow) > 5000 )
        return(0);
    }

#ifdef DEBUG_MODE  
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: " + WiFi.localIP().toString());
#endif
    Udp.begin(UDPPort);

    return(1);
}

#endif
