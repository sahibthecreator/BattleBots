/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

// Load Wi-Fi library
#include <WiFi.h>

// Replace with your network credentials
const char* ssid     = "Privet_Suka";
const char* password = "123456789";

const char* ssidToConnect     = "nokia 5700 xpressmusic";
const char* passwordToConnect = "11111111";

// Set web server port number to 80
WiFiServer server(80);

const byte led_gpio = 17; // the PWM pin the LED is attached to
const byte led_gpio1 = 5; // the PWM pin the LED is attached to

const byte back1 = 16; // the PWM pin the LED is attached to
const byte back2 = 18; // the PWM pin the LED is attached to

// Variable to store the HTTP request
String header;

void setup() {
  Serial.begin(115200);
  
  //-------------------MOTORS SETUP-----------------
  
  ledcAttachPin(led_gpio, 0); // assign a led pins to a channel
  ledcAttachPin(led_gpio1, 1); // assign a led pins to a channel
  ledcAttachPin(back1, 2); // assign a led pins to a channel
  ledcAttachPin(back2, 3); // assign a led pins to a channel
  ledcSetup(0, 4000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(1, 4000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(2, 4000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(3, 4000, 8); // 12 kHz PWM, 8-bit resolution

  
  
    // Connect to Wi-Fi network with SSID and password
    Serial.print("Setting Access Point…");
    // Remove the password parameter, if you want the AP (Access Point) to be open
    WiFi.softAP(ssid, password);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("Our IP address: ");
    Serial.println(IP);
  
    server.begin();

    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssidToConnect);

    WiFi.begin(ssidToConnect, passwordToConnect);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address of NOKIA EXPRESSMUSIC : ");
    Serial.println(WiFi.localIP());
}

void loop(){


  
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
        
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP32 Web Server</h1>");
            // the content of the HTTP response follows the header:
            client.print("<h2 class='button'>Click <a href=\"/F\">here</a> to go forward <br></h2>");
            client.print("<h2 class='button'>Click <a href=\"/S\">here</a> to Stop <br></h2>");
            client.print("<h2 class='button'>Click <a href=\"/B\">here</a> to go Back <br></h2>");
            client.print("<h2 class='button'>Click <a href=\"/R\">here</a> to turn Right <br></h2>");
            client.print("<h2 class='button'>Click <a href=\"/L\">here</a> to go Left <br></h2>");
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /F")) {
          ledcWrite(0, 230); // set the brightness of the LED
          ledcWrite(1, 230); // set the brightness of the LED
          ledcWrite(2, 0); // set the brightness of the LED
          ledcWrite(3, 0); // set the brightness of the LED               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /S")) {
          ledcWrite(0, 0); // set the brightness of the LED
          ledcWrite(1, 0); // set the brightness of the LED
          ledcWrite(2, 0); // set the brightness of the LED
          ledcWrite(3, 0); // set the brightness of the LED                  // GET /L turns the LED off
        }
        if (currentLine.endsWith("GET /B")) {
          ledcWrite(0, 0); // set the brightness of the LED
          ledcWrite(1, 0); // set the brightness of the LED
          ledcWrite(2, 150); // set the brightness of the LED
          ledcWrite(3, 150); // set the brightness of the LED                  // GET /L turns the LED off
        }
        if (currentLine.endsWith("GET /R")) {
          ledcWrite(0, 0); // set the brightness of the LED
          ledcWrite(1, 150); // set the brightness of the LED
          ledcWrite(2, 150); // set the brightness of the LED
          ledcWrite(3, 0); // set the brightness of the LED 
                          // GET /L turns the LED off
        }
        if (currentLine.endsWith("GET /L")) {
            ledcWrite(0, 150); // set the brightness of the LED
          ledcWrite(1, 0); // set the brightness of the LED
          ledcWrite(2, 0); // set the brightness of the LED
          ledcWrite(3, 150); // set the brightness of the LED                 // GET /L turns the LED off
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
  
}

void captureFlag(){
  int wifiSignal = getSignalStrength("Bot1") ;
  while(getSignalStrength("Bot1") < wifiSignal){
    ledcWrite(0, 230); // set the brightness of the LED
    ledcWrite(1, 230); // set the brightness of the LED
    ledcWrite(2, 0); // set the brightness of the LED
    ledcWrite(3, 0); // set the brightness of the LED
    delay(500);
    ledcWrite(0, 0); // set the brightness of the LED
    ledcWrite(1, 180); // set the brightness of the LED
    ledcWrite(2, 180); // set the brightness of the LED
    ledcWrite(3, 0); // set the brightness of the LED
    delay(500);
  }
  while(getSignalStrength("Bot1") < 35){
    ledcWrite(0, 230); // set the brightness of the LED
    ledcWrite(1, 230); // set the brightness of the LED
    ledcWrite(2, 0); // set the brightness of the LED
    ledcWrite(3, 0); // set the brightness of the LED
  }
  
  
  
  
}

int getSignalStrength(String wifiName){
  Serial.println("scan start");

    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    if (n == 0) {
        Serial.println("no networks found");
    } else {
        for (int i = 0; i < n; ++i) {
            if(WiFi.SSID(i) == wifiName){
                Serial.print(WiFi.SSID(i));
                Serial.print("Signal Strength: ");
                Serial.print(WiFi.RSSI(i));
                return WiFi.RSSI(i);
            }
        }
    }
}
