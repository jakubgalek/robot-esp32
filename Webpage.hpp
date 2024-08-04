#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>

#define INDEX "/index.htm"

WebServer server(80);

const char* hostname = "robot";


boolean MOTOR_state[4] = {0};

boolean AUTO_DRIVE_state = 0;    
boolean RADAR_state = 0;     


void checkMotorsAndReloadBuzzer(){  
  if (busy_motors != true) Buzzer_reload();
}

void automatic_drive_Task(void *pvParameters) {
  while (1) {
    automatic_drive();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void collect_distances_servo_Task(void *pvParameters) {

    while (1) {

      collect_distances_servo();
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void SetMOTORs() {
    // MOTOR 1
    if (server.hasArg("MOTOR1")) {
        MOTOR_state[0] = server.arg("MOTOR1").toInt();
      if (MOTOR_state[0]==1 && !(measurement3 <= 10) && !(measurement4 <= 10) && !(measurement5 <= 10)){ledcWrite(pwmChannel, 200);forward();} else if(MOTOR_state[0]==0) stop_driving();
    }

    // MOTOR 2
    if (server.hasArg("MOTOR2")) {
        MOTOR_state[1] = server.arg("MOTOR2").toInt();
     if(MOTOR_state[1]==1){ledcWrite(pwmChannel, 200);backward();} else if(MOTOR_state[1]==0) stop_driving();
    }

    // MOTOR 3
    if (server.hasArg("MOTOR3")) {
        MOTOR_state[2] = server.arg("MOTOR3").toInt();
        if(MOTOR_state[2]==1){ledcWrite(pwmChannel, 220);turn(turn_left,90,255); direction = "W lewo";} else if(MOTOR_state[2]==0) stop_driving();
    }

    // MOTOR 4
    if (server.hasArg("MOTOR4")) {
        MOTOR_state[3] = server.arg("MOTOR4").toInt();
       if(MOTOR_state[3]==1){ledcWrite(pwmChannel, 220);turn(turn_right,90,255); direction = "W prawo";} else if(MOTOR_state[3]==0) stop_driving();
    }
    // Speed Slider on website
    if (server.hasArg("Slider")) {
        int motorSpeed = server.arg("Slider").toInt(); // Get speed value from the slider
        ledcWrite(pwmChannel, motorSpeed); // Set the motor speed
        //Serial.println(motorSpeed);
    }
    // Automatic drive button on website
    if (server.hasArg("AUTO_DRIVE")) {
        AUTO_DRIVE_state = server.arg("AUTO_DRIVE").toInt();

        if(AUTO_DRIVE_state == 1)
        { 
          if (AutomaticDriveTaskHandle == NULL) {
            xTaskCreatePinnedToCore(
              automatic_drive_Task,       // Function to execute in the task
              "AutomaticDriveTask",       // Task name
              7000,                       // Stack size (adjust as needed)
              NULL,                       // Parameters supplied to the thread
              1,                          // Task priority
              &AutomaticDriveTaskHandle,  // Task handle
              0                           // Core number (0 or 1)
            );
          } 
        }
        else if(AUTO_DRIVE_state == 0)
        {
          if (AutomaticDriveTaskHandle != NULL) {
                vTaskDelete(AutomaticDriveTaskHandle);
                AutomaticDriveTaskHandle = NULL;
                stop_driving();
             }
        } 
    }
     // RADAR button on website
     if (server.hasArg("RADAR")) {
        RADAR_state = server.arg("RADAR").toInt();

        if (RADAR_state == 1) 
        {
          stopFlag=false;
        }

        else if (RADAR_state == 0) 
        {
           stopFlag=true;
        }
      }
    
}

String xmlResponseBase() {
    String res = "";  // String to assemble XML response values
    res += "<?xml version = \"1.0\" ?>\n";
    res +="<inputs>\n";

    res += "<voltageESP32>";
    res += String(readVoltageESP32());
    res += "</voltageESP32>\n";

  float temperature = getFormattedTemperature();
  char tempStr[10];
  dtostrf(temperature, 4, 1, tempStr);

    res += "<temp>";
    res += String(tempStr);
    res += "</temp>\n";

    res += "<altitude>";
    res += String(getFormattedAltitude());
    res += "</altitude>\n";

    res += "<voltageMotors>";
    res += String(readVoltageMotors());
    res += "</voltageMotors>\n";

    res += "<wifi>";
    res += signalStrength;
    res += "</wifi>\n";

    res += "<direction>";
    res += direction;
    res += "</direction>\n";
        
    res += "<MOTOR>";
    if (MOTOR_state[0]) {
        res += "on";
         //Serial.println("forward");
    } else {
        res += "off";
    }
    res += "</MOTOR>\n";
    res += "<MOTOR>";
    if (MOTOR_state[1]) {
        res += "on";
        //Serial.println("backward");
    } else {
        res += "off";
    }
    res += "</MOTOR>\n";
    res += "<MOTOR>";
    if (MOTOR_state[2]) {
        res += "on";
        //Serial.println("turn_left");
    } else {
        res += "off";
    }
    res += "</MOTOR>\n";
    res += "<MOTOR>";
    if (MOTOR_state[3]) {
        res += "on";
        //Serial.println("turn_right");
    } else {
        res += "off";
    }
    res += "</MOTOR>\n";

    res += "<AUTO_DRIVE>";
    if (AUTO_DRIVE_state) {
        res += "on";
        //Serial.println("AUTO_DRIVE");
    } else {
        res += "off";
    }
    res += "</AUTO_DRIVE>\n";

    res += "<RADAR>";
    if (RADAR_state) {
        res += "on";
        //Serial.println("RADAR");
    } else {
        res += "off";
    }
    res += "</RADAR>\n";
  

 // Start the radar data section
    res += "<radarData>\n";
    return res;

}

String xmlResponseWithRadar() {
    String res = "";
    for (int i = 0; i < 180; i++) {
        res += "<radarPoint>\n";
        res += "<angle>" + String(i) + "</angle>\n";
        res += "<distance1>" + String(radarData[i][0]) + "</distance1>\n";
        res += "<distance2>" + String(radarData[i][1]) + "</distance2>\n";
        res += "</radarPoint>\n";
    }
    res += "</radarData>\n";
    return res;
}

String xmlResponse() {
    String response = xmlResponseBase();
    // Call the radar data generation function here
    response += xmlResponseWithRadar();
    response += "</inputs>";
    return response;
}

void ajaxInputs() {
  SetMOTORs();      // Set MOTORs
  server.sendHeader("Connection", "close");                          // Headers to free connection ASAP and 
  server.sendHeader("Cache-Control", "no-store, must-revalidate");   // Don't cache response
  server.send(200, "text/xml", xmlResponse());                       // Send string from xmlResponse() as XML document to client.
}

void indexFile() {
    server.sendHeader("Connection", "close");                        // Headers again free connection and
    server.sendHeader("Cache-Control", "no-store, must-revalidate"); // Don't caching
    
    if (SPIFFS.exists(INDEX)) {
        File file = SPIFFS.open(INDEX, "r");                         // Open default index file readonly
        if (!file) {
            Serial.println("Failed to open file");
            server.send(500, "text/plain", "Failed to open file");
            return;
        }
        server.streamFile(file, "text/html");                        // Send file to client as HTML document
        file.close();
    } else {
        server.send(404, "text/plain", "File not found");
    }
}

void listFiles() {
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while(file){
        Serial.print("FILE: ");
        Serial.println(file.name());
        file = root.openNextFile();
    }
}

String getContentType(String filename) {
    if (server.hasArg("download")) return "application/octet-stream";
    else if (filename.endsWith(".htm")) return "text/html";
    else if (filename.endsWith(".html")) return "text/html";
    else if (filename.endsWith(".css")) return "text/css";
    else if (filename.endsWith(".js")) return "application/javascript";
    else if (filename.endsWith(".png")) return "image/png";
    else if (filename.endsWith(".gif")) return "image/gif";
    else if (filename.endsWith(".jpg")) return "image/jpeg";
    else if (filename.endsWith(".ico")) return "image/x-icon";
    else if (filename.endsWith(".xml")) return "text/xml";
    else if (filename.endsWith(".pdf")) return "application/x-pdf";
    else if (filename.endsWith(".zip")) return "application/x-zip";
    else if (filename.endsWith(".gz")) return "application/x-gzip";
    else if (filename.endsWith(".svg")) return "image/svg+xml";
    return "text/plain";
}

bool handleFileRead(String path) {
    // Serial.println("handleFileRead: " + path);
    if (path.endsWith("/")) path += "index.htm"; // If the path is a directory, add "index.htm"
    
    String contentType = getContentType(path); // Get the content type
    if (SPIFFS.exists(path)) { // Check if the file exists
        File file = SPIFFS.open(path, "r");
        size_t sent = server.streamFile(file, contentType); // Send file to client
        file.close();
        return true; // File found and handled
    }
    return false; // File not found
}

void Web_init() {

  // mDNS initialization
  if (!MDNS.begin(hostname)) {
    Serial.println("Error setting up mDNS");
  } else {
    MDNS.addService("http", "tcp", 80); // Advertise a service (HTTP)
    Serial.println("mDNS responder started");
  }

  // Web server initialization
    server.on("/ajax_inputs", ajaxInputs);
    server.onNotFound([]() {
        if (!handleFileRead(server.uri()))
            server.send(404, "text/plain", "Not Found");
    });
    server.begin();
    Serial.println("HTTP server started");

      if (!SPIFFS.begin(true)) {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    } 
}
