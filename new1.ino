
/*--------------------------------------------------------------
  Robot Controller
  
  Hardware:     
                A0 PWM FOR L293D (speed motors)   
                D0 SD (CS)
                D1 SCL (DS3231,BMP180,GY-271,PCF8574)
                D2 SCK (DS3231,BMP180,GY-271,PCF8574)
                D3 DHT11
                D4 NodeMCU LED
                D5 SD (SCK)
                D6 SD (MISO)
                D7 SD (MOSI)
                D8 SERVO
                Ekspander(PCF8574): P0 P1 P2 P3 P4 P5 P6 P7           
                
  Date:         16 February 2020
 
  Author:       xwondunx@gmail.com
                https://gitbub.com/Wondun
--------------------------------------------------------------*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <FS.h>                   
#include <SD.h>
#include <ESP8266mDNS.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <RTClib.h>
#include <DHT.h>
#include "Arduino.h"
#include "PCF8574.h"
#include <Servo.h>

PCF8574 pcf8574(0x20);
Adafruit_BMP085 bmp;
RTC_DS3231 rtc;
Servo myservo; 


#define DHTPIN D3
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);


File plik;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

struct Data {
  float battery;
  int16_t humidity;
  int16_t speed_1;
  int16_t speed_2;
};

struct Data _data;


#define SSID "**********"               // Wi-Fi Access point SSID
#define PASSWORD "**************"       // Wi-Fi password

//#define SSID "ROBOT"               // Wi-Fi Access point SSID
//#define PASSWORD "kuba"            // Wi-Fi password
#define INDEX "/index.htm"
#define UPLOADUSER  "admin"
#define UPLOADPASS "password"
//#undef UPLOADPASS
//#define DBG_OUTPUT_PORT Serial

#ifndef STASSID
#define STASSID "**********"
#define STAPSK  "**********"
#endif

const char* ssid = STASSID;
const char* password = STAPSK;
//const char* host = "esp8266sd";

IPAddress ip(192, 168, 1, 106);  // IP address
IPAddress mask(255, 255, 255, 0); // Netmask
IPAddress gw(192, 168, 1, 1);    // Default gateway
IPAddress dns(192, 168, 30, 4);   // DNS
ESP8266WebServer server(82);      // create a server at port 82
ESP8266WebServer server2(80);      // create a server at port 80
boolean LED_state[4] = {0};       // stores the states of the LEDs

#define BUSY ;
#define IDLE ;


static bool hasSD = false;
File uploadFile;


void returnOK() {
  server2.send(200, "text/plain", "");
}

void returnFail(String msg) {
  server2.send(500, "text/plain", msg + "\r\n");
}

bool loadFromSdCard(String path) {
  String dataType = "text/plain";
  if (path.endsWith("/")) {
    path += "index.htm";
  }

  if (path.endsWith(".src")) {
    path = path.substring(0, path.lastIndexOf("."));
  } else if (path.endsWith(".htm")) {
    dataType = "text/html";
  } else if (path.endsWith(".css")) {
    dataType = "text/css";
  } else if (path.endsWith(".js")) {
    dataType = "application/javascript";
  } else if (path.endsWith(".png")) {
    dataType = "image/png";
  } else if (path.endsWith(".gif")) {
    dataType = "image/gif";
  } else if (path.endsWith(".jpg")) {
    dataType = "image/jpeg";
  } else if (path.endsWith(".ico")) {
    dataType = "image/x-icon";
  } else if (path.endsWith(".xml")) {
    dataType = "text/xml";
  } else if (path.endsWith(".pdf")) {
    dataType = "application/pdf";
  } else if (path.endsWith(".zip")) {
    dataType = "application/zip";
  }  else if (path.endsWith(".svg")) {
    dataType = "image/svg+xml";
  }

  File dataFile = SD.open(path.c_str());
  if (dataFile.isDirectory()) {
    path += "/index.htm";
    dataType = "text/html";
    dataFile = SD.open(path.c_str());
  }

  if (!dataFile) {
    return false;
  }

  if (server2.hasArg("download")) {
    dataType = "application/octet-stream";
  }

  if (server2.streamFile(dataFile, dataType) != dataFile.size()) {
    //DBG_OUTPUT_PORT.println("Sent less data than expected!");
  }

  dataFile.close();
  return true;
}

void handle2FileUpload() {
  if (server2.uri() != "/edit") {
    return;
  }
  HTTPUpload& upload = server2.upload();
  if (upload.status == UPLOAD_FILE_START) {
    if (SD.exists((char *)upload.filename.c_str())) {
      SD.remove((char *)upload.filename.c_str());
    }
    uploadFile = SD.open(upload.filename.c_str(), FILE_WRITE);
    //DBG_OUTPUT_PORT.print("Upload: START, filename: "); DBG_OUTPUT_PORT.println(upload.filename);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) {
      uploadFile.write(upload.buf, upload.currentSize);
    }
    //DBG_OUTPUT_PORT.print("Upload: WRITE, Bytes: "); DBG_OUTPUT_PORT.println(upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) {
      uploadFile.close();
    }
    //DBG_OUTPUT_PORT.print("Upload: END, Size: "); DBG_OUTPUT_PORT.println(upload.totalSize);
  }
}

void deleteRecursive(String path) {
  File file = SD.open((char *)path.c_str());
  if (!file.isDirectory()) {
    file.close();
    SD.remove((char *)path.c_str());
    return;
  }

  file.rewindDirectory();
  while (true) {
    File entry = file.openNextFile();
    if (!entry) {
      break;
    }
    String entryPath = path + "/" + entry.name();
    if (entry.isDirectory()) {
      entry.close();
      deleteRecursive(entryPath);
    } else {
      entry.close();
      SD.remove((char *)entryPath.c_str());
    }
    yield();
  }

  SD.rmdir((char *)path.c_str());
  file.close();
}

void handle2Delete() {
  if (server2.args() == 0) {
    return returnFail("BAD ARGS");
  }
  String path = server2.arg(0);
  if (path == "/" || !SD.exists((char *)path.c_str())) {
    returnFail("BAD PATH");
    return;
  }
  deleteRecursive(path);
  returnOK();
}

void handleCreate() {
  if (server2.args() == 0) {
    return returnFail("BAD ARGS");
  }
  String path = server2.arg(0);
  if (path == "/" || SD.exists((char *)path.c_str())) {
    returnFail("BAD PATH");
    return;
  }

  if (path.indexOf('.') > 0) {
    File file = SD.open((char *)path.c_str(), FILE_WRITE);
    if (file) {
      file.write((const char *)0);
      file.close();
    }
  } else {
    SD.mkdir((char *)path.c_str());
  }
  returnOK();
}

void printDirectory() {
  if (!server2.hasArg("dir")) {
    return returnFail("BAD ARGS");
  }
  String path = server2.arg("dir");
  if (path != "/" && !SD.exists((char *)path.c_str())) {
    return returnFail("BAD PATH");
  }
  File dir = SD.open((char *)path.c_str());
  path = String();
  if (!dir.isDirectory()) {
    dir.close();
    return returnFail("NOT DIR");
  }
  dir.rewindDirectory();
  server2.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server2.send(200, "text/json", "");
  WiFiClient client = server2.client();

  server2.sendContent("[");
  for (int cnt = 0; true; ++cnt) {
    File entry = dir.openNextFile();
    if (!entry) {
      break;
    }

    String output;
    if (cnt > 0) {
      output = ',';
    }

    output += "{\"type\":\"";
    output += (entry.isDirectory()) ? "dir" : "file";
    output += "\",\"name\":\"";
    output += entry.name();
    output += "\"";
    output += "}";
    server2.sendContent(output);
    entry.close();
  }
  server2.sendContent("]");
  server2.sendContent(""); // Terminate the HTTP chunked transmission with a 0-length chunk
  dir.close();
}

void handleNotFound() {
  if (hasSD && loadFromSdCard(server2.uri())) {
    return;
  }
  String message = "SDCARD Not Detected\n\n";
  message += "URI: ";
  message += server2.uri();
  message += "\nMethod: ";
  message += (server2.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server2.args();
  message += "\n";
  for (uint8_t i = 0; i < server2.args(); i++) {
    message += " NAME:" + server2.argName(i) + "\n VALUE:" + server2.arg(i) + "\n";
  }
  server2.send(404, "text/plain", message);
  //DBG_OUTPUT_PORT.print(message);
}

// checks if received HTTP request is switching on/off LEDs
// also saves the state of the LEDs
void SetLEDs(void)
{
    // LED 1 (pin D1)
    if (server.hasArg("LED1")) {                          // Checks if URL has argumen LED1 (LED1=1)
        LED_state[0] = server.arg("LED1").toInt();        // save LED state. Converting argument value from stfing to integer
        digitalWrite(D4, (LED_state[0]==1)?LOW:HIGH);     // For D1 and D4 use LOW output for 'on' state  
       if(LED_state[0]==1)przod(); else if(LED_state[0]==0) no();
       if (_data.distance <=20) {no(); }
    }
    // LED 2 (pin D2)
    if (server.hasArg("LED2")) {
        LED_state[1] = server.arg("LED2").toInt();       // save LED state
        digitalWrite(D4, (LED_state[1]==1)?LOW:HIGH);    // For D2 and D3 use HIGH output for 'on' state
      if(LED_state[1]==1)tyl(); else if(LED_state[1]==0) no();  
      //if (_data.distance <=20) {no(); }
    }
    // LED 3 (pin D3)
    if (server.hasArg("LED3")) {
        LED_state[2] = server.arg("LED3").toInt();       // save LED state
        digitalWrite(D4, (LED_state[2]==1)?LOW:HIGH);    // For D2 and D3 use HIGH output for 'on' state  
       if(LED_state[2]==1)lewo(); else if(LED_state[2]==0) no();
      // if (_data.distance <=20) {no(); }
    }
    // LED 4 (pin D4)
    if (server.hasArg("LED4")) {
        LED_state[3] = server.arg("LED4").toInt();       // save LED state
        digitalWrite(D4, (LED_state[3]==1)?LOW:HIGH);    // For D4 and D4 use LOW output for 'on' state  
       if(LED_state[3]==1)prawo(); else if(LED_state[3]==0) no();
      // if (_data.distance <=20) {no(); }
    }
}

// send the XML file with analog values, switch status
//  and LED status
String xmlResponse()
{
    String res = "";                    // String to assemble XML response values
    //float battery_val;                     // stores value read from analog inputs

    int X=23;
    int Y=53;

  
    float temp_val=0;                     
    float pressure_val=0;  
    float humidity_val=0;                   
    long wifi = 0;    
    String wifi_strenght;
                   
    DateTime now = rtc.now();
    String time=String(now.year()+String("-")+now.month()+String("-")+now.day()+String(" ")+now.hour()+String(":")+now.minute());

    res += "<?xml version = \"1.0\" ?>\n";
    res +="<inputs>\n";
    // read analog input
    // ESP8266 has only one Analog input. It's 10bit (1024 values) Measuring range is 0-1V.
    
        res += "<battery>";
        res += String(_data.battery);
        res += "</battery>\n";
		
		temp_val = bmp.readTemperature();
        res += "<temp>";
        res += String(temp_val);
        res += "</temp>\n";
        
		pressure_val = bmp.readPressure()/100;
        res += "<pressure>";
        res += String(pressure_val);
        res += "</pressure>\n";

    humidity_val = dht.readHumidity();
        res += "<humidity>";
        res += String(humidity_val);
        res += "</humidity>\n";
		
		wifi = WiFi.RSSI();
   if(wifi>=-70)wifi_strenght="Doskonały";
   else if(wifi<-70&&wifi>=-85)wifi_strenght="Dobry";
   else if(wifi<=-86&&wifi>=-100)wifi_strenght="Średni";
   else if(wifi<=-100)wifi_strenght="Słaby";
   else if(wifi<=-110)wifi_strenght="Brak sygnału";
        res += "<wifi>";
        res += wifi_strenght;
        res += "</wifi>\n";
       
    int aktualnyCzas2=100;
		aktualnyCzas2 = millis()/100;
    X=aktualnyCzas2;
    Y=aktualnyCzas2;
        res += "<OX>";
        res += String(X);
        res += "</OX>\n";

        res += "<OY>";
        res += String(Y);
        res += "</OY>\n";
        
        for(int k=0;k<8;k++){
        res += "<div class='time'>";
        res += String(time);
        res += "</div>\n";
        }
    
    // MOTOR1
    res += "<LED>";
    if (LED_state[0]) {
        res += "on";
    }
    else {
        res += "off";
    }
    res += "</LED>\n";
    // MOTOR2
    res += "<LED>";
    if (LED_state[1]) {
        res += "on";
    }
    else {
        res += "off";
    }
     res += "</LED>\n";
    // MOTOR3
    res += "<LED>";
    if (LED_state[2]) {
        res += "on";
    }
    else {
        res += "off";
    }
    res += "</LED>\n";
    // MOTOR4
    res += "<LED>";
    if (LED_state[3]) {
        res += "on";
    }
    else {
        res += "off";
    }
    res += "</LED>\n";
    
    res += "</inputs>";
    return res;
}

void ajaxInputs() {
  SetLEDs(); 
  server.sendHeader("Connection", "close");                         
  server.sendHeader("Cache-Control", "no-store, must-revalidate");  
  server.send(200, "text/xml", xmlResponse());                      
                                                                
}
void indexFile() {
    server.sendHeader("Connection", "close");                      
    server.sendHeader("Cache-Control", "no-store, must-revalidate");
    File file = SPIFFS.open(INDEX, "r");                           
    size_t sent = server.streamFile(file, "text/html");            
    file.close();                                                   
}

void setup()
{
   for(int i=0;i<8;i++) {
    pcf8574.pinMode(i, OUTPUT);
  }
  pcf8574.begin();
  myservo.attach(D8);
  myservo.write(63);
  
  //DBG_OUTPUT_PORT.begin(115200);
  //DBG_OUTPUT_PORT.setDebugOutput(true);
  //DBG_OUTPUT_PORT.print("\n");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  //DBG_OUTPUT_PORT.print("Connecting to ");
  //DBG_OUTPUT_PORT.println(ssid);

  // Wait for connection
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) {//wait 10 seconds
    delay(500);
  }
  if (i == 21) {
    //DBG_OUTPUT_PORT.print("Could not connect to");
    //DBG_OUTPUT_PORT.println(ssid);
    //while (1) {
    //  delay(500);
   // }
  }



  server2.on("/list", HTTP_GET, printDirectory);
  server2.on("/edit", HTTP_DELETE, handle2Delete);
  server2.on("/edit", HTTP_PUT, handleCreate);
  server2.on("/edit", HTTP_POST, []() {
    returnOK();
  }, handle2FileUpload);
  server2.onNotFound(handleNotFound);

  server2.begin();
  ////DBG_OUTPUT_PORT.println("HTTP server started");

  if (SD.begin(D0)) {
    //DBG_OUTPUT_PORT.println("SD Card initialized.");
    hasSD = true;
  }
      #ifndef ESP8266
      while (!Serial);
      #endif
      
    Serial.begin(115200);           // for debugging
    
    dht.begin();
    // initialize File system
    Serial.println("Initializing File system...");
    if (!SPIFFS.begin()) {
        Serial.println("ERROR - File system initialization failed!");
        return; 
    }
    Serial.println("SUCCESS - File system initialized.");
    if (!SPIFFS.exists(INDEX)) {
        Serial.println("ERROR - Can't find /index.htm file!");
        return;              
    }
    Serial.println("SUCCESS - Found /index.htm file.");

    pinMode(D4, OUTPUT);        // NodeMCU LED
    digitalWrite(D4, HIGH);
    pinMode(A0, OUTPUT);
  
                                
    WiFi.mode(WIFI_STA);        
    //WiFi.config(ip, gw, mask, dns);
    WiFi.begin(SSID, PASSWORD); 

    Serial.print("Connecting Wi-Fi.");
    while (WiFi.status() != WL_CONNECTED) {
      delay(100);
      Serial.print(".");
    }
    Serial.println();
    Serial.print("SUCCESS - Local IP: ");
    Serial.println(WiFi.localIP()); 
    server.on("/ajax_inputs", HTTP_GET, ajaxInputs);  

    server.on(INDEX, HTTP_GET, indexFile); 
    server.on("/list", HTTP_GET, listFile);                  
    server.on("/delete", HTTP_GET, handleDelete);              
    server.on("/edit", HTTP_POST, handleFile, handleFileUpload);    
    server.onNotFound(anyFile);        
    server.begin();                   


     if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  }
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP180 sensor, check wiring!");
  while (1) {}
  }
}

void loop()
{
    server.handleClient();              
   // delay(100);
    server2.handleClient();
   // delay(100);                         
   if ((_data.distance <=20) && (LED_state[0]==1) && (_data.speed_1>=50)) { no(); }

   
 while(!Serial.available());
  byte b1 = Serial.read();
  if(b1 != 0xAA) return;
  while(!Serial.available());
  byte b2 = Serial.read();
  if(b2 != 0xBB) return;
  Serial.readBytes((char*)&_data, sizeof(_data));
  Serial.print("batt: ");
  Serial.println(_data.battery);
  Serial.print("dist: ");
  Serial.println(_data.distance);
  Serial.print("spd1: ");
  Serial.println(_data.speed_1);
  Serial.print("spd2: ");
  Serial.println(_data.speed_2);

 DateTime now = rtc.now();
   String time=String(now.year()+String("-")+now.month()+String("-")+now.day()+String(" ")+now.hour()+String(":")+now.minute());
   
     Serial.print(time+":"+now.second()); Serial.print(";");
     Serial.print(bmp.readTemperature()); Serial.print(";");
     Serial.print(bmp.readPressure()/100); Serial.print(";");
     Serial.print(dht.readHumidity());

      plik = SD.open("DATA.txt", FILE_WRITE);
 if(now.minute()%10==0&&now.second()%30==0){

      plik.print(time+":"+now.second()); plik.print(";");
     plik.print(bmp.readTemperature()); plik.print(";");
     plik.print(bmp.readPressure()/100); plik.print(";");
     plik.print(dht.readHumidity());
     plik.println();
     plik.close();}


   //    if (_data.distance <=30)
  //     {

  // if(LED_state[1]==1) tyl();
//  else if(LED_state[2]==1) lewo();
//  else if(LED_state[3]==1) prawo();
//  else no();
   //    }
}

String getContentType(String filename) {
  if(server.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  else if(filename.endsWith(".svg")) return "image/svg+xml";
  return "text/plain";
}
void listFile() {
#ifdef UPLOADPASS
  if(!server.authenticate(UPLOADUSER, UPLOADPASS)) {
    return server.requestAuthentication();
  }
#endif
  String output = "<html><head><meta charset='utf-8'>\
  <title>ESP8266 - File operations</title>\
  <body>\
  <form method='POST' action='/edit' enctype='multipart/form-data'>\
  Upload file to local filesystem:<br>\
   <input type='file' name='update'>\
   <input type='submit' value='Upload file'>\
  </form>";
  String path = server.hasArg("dir")?server.arg("dir"):"/";
  Dir dir = SPIFFS.openDir(path);
  while(dir.next()){
    File entry = dir.openFile("r");
    String filename = String(entry.name());
    output += "<br>";
    output += "<a href='" + filename + "'>" + filename + "</a>&nbsp<a href='/delete?file=" + filename + "'><font color=red>delete</font></a>";
    output += "<br>";
    entry.close();
  }
  output += "</body><html>";
  server.sendHeader("Connection", "close");
  server.sendHeader("Cache-Control", "no-store, must-revalidate");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/html", output);  
}
void handleFile() {
#ifdef UPLOADPASS
  if(!server.authenticate(UPLOADUSER, UPLOADPASS)) {
    return server.requestAuthentication();
  }
#endif
  server.sendHeader("Connection", "close");
  server.sendHeader("Cache-Control", "no-store, must-revalidate");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Refresh", "5; url=/list");
  server.send(200, "text/plain", "OK");  
}
void handleFileUpload(){
  File fsUploadFile;
#ifdef UPLOADPASS
  if(!server.authenticate(UPLOADUSER, UPLOADPASS)) {
    return server.requestAuthentication();
  }
#endif
  if(server.uri() != "/edit") return;
  BUSY
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    String filename = upload.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if(upload.status == UPLOAD_FILE_WRITE){
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if(upload.status == UPLOAD_FILE_END){
    if(fsUploadFile)
      fsUploadFile.close();
  }
  IDLE
}
bool fileRead(String path){
  if(path.endsWith("/")) path += INDEX;
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
    if(SPIFFS.exists(pathWithGz))
      path += ".gz";
    server.sendHeader("Connection", "close");
    server.sendHeader("Cache-Control", "no-store, must-revalidate");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}
void anyFile() {
  BUSY
  if(!fileRead(server.uri()))
    server.send(404, "text/plain", "FileNotFound");
  IDLE
}
void handleDelete() {
#ifdef UPLOADPASS
  if(!server.authenticate(UPLOADUSER, UPLOADPASS)) {
    return server.requestAuthentication();
  }
#endif
  server.sendHeader("Connection", "close");
  server.sendHeader("Cache-Control", "no-store, must-revalidate");
  server.sendHeader("Refresh", "5; url=/list");
  String path;
  if(server.args() != 0) {
    path = server.arg(0);
    if(path != "/" && SPIFFS.exists(path)) {
      if (SPIFFS.remove(path)) {
        server.send(200, "text/plain", "OK");
        IDLE
        return;
      } else {
        server.send(404, "text/plain", "FileNotFound");
        IDLE
        return;
      }
    }
  }
  server.send(500, "text/plain", "ERROR");
  IDLE
}


void przod () {
  pcf8574.digitalWrite(0, LOW); 
  pcf8574.digitalWrite(1, HIGH);
  
  pcf8574.digitalWrite(2, LOW);
  pcf8574.digitalWrite(3, HIGH); 
  
  pcf8574.digitalWrite(4, LOW);
  pcf8574.digitalWrite(5, HIGH); 
  
  pcf8574.digitalWrite(6, LOW);
  pcf8574.digitalWrite(7, HIGH); 
  
  }
  
void tyl () {
  pcf8574.digitalWrite(0, HIGH); 
  pcf8574.digitalWrite(1, LOW);
  
  pcf8574.digitalWrite(2, HIGH);
  pcf8574.digitalWrite(3, LOW); 
  
  pcf8574.digitalWrite(4, HIGH);
  pcf8574.digitalWrite(5, LOW); 
  
  pcf8574.digitalWrite(6, HIGH);
  pcf8574.digitalWrite(7, LOW); 
  
  }

  void lewo () {
  pcf8574.digitalWrite(0, HIGH); 
  pcf8574.digitalWrite(1, LOW);
  
  pcf8574.digitalWrite(2, LOW);
  pcf8574.digitalWrite(3, HIGH); 
  
  pcf8574.digitalWrite(4, LOW);
  pcf8574.digitalWrite(5, HIGH); 
  
  pcf8574.digitalWrite(6, HIGH);
  pcf8574.digitalWrite(7, LOW);  }
  
  
void prawo () {
  pcf8574.digitalWrite(0, LOW); 
  pcf8574.digitalWrite(1, HIGH);
  
  pcf8574.digitalWrite(2, HIGH);
  pcf8574.digitalWrite(3, LOW); 
  
  pcf8574.digitalWrite(4, HIGH);
  pcf8574.digitalWrite(5, LOW); 
  
  pcf8574.digitalWrite(6, LOW);
  pcf8574.digitalWrite(7, HIGH); 
  }
  
void no () {
  pcf8574.digitalWrite(0, LOW); 
  pcf8574.digitalWrite(1, LOW);
  
  pcf8574.digitalWrite(2, LOW);
  pcf8574.digitalWrite(3, LOW); 
  
  pcf8574.digitalWrite(4, LOW);
  pcf8574.digitalWrite(5, LOW); 
  
  pcf8574.digitalWrite(6, LOW);
  pcf8574.digitalWrite(7, LOW);  }
