/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-web-server-slider-pwm/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

// Import required libraries
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <esp_now.h>

esp_now_peer_info_t peerInfo ={};

typedef struct test_struct  {
  String sliderValue;
  String variable;
  
} test_struct ;

test_struct myData;

//myData.x = 3;
//initialise


//see https://randomnerdtutorials.com/esp-now-one-to-many-esp32-esp8266/

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
uint8_t broadcastAddress1[] = {0x7C,0x9E,0xBD,0x47,0x6C,0xF4};
uint8_t broadcastAddress2[] = {0x94,0xB9,0x7E,0xE5,0xA8,0x5C};

const char* ssid = "***";
const char* password = "***";

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

const int output = 2;

String sliderValue = "000";
String variable;



// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

const char* PARAM_INPUT = "value";
const char* PARAM_CMD = "go";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
  <head>
    <title>ESP32-CAM Robot</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: Arial; text-align: center; margin:0px auto; padding-top: 30px;}
    .slider { -webkit-appearance: none; margin: 14px; width: 360px; height: 25px; background: #FFD65C;
      outline: none; -webkit-transition: .2s; transition: opacity .2s;}
    .slider::-webkit-slider-thumb {-webkit-appearance: none; appearance: none; width: 35px; height: 35px; background: #003249; cursor: pointer;}
    .slider::-moz-range-thumb { width: 35px; height: 35px; background: #003249; cursor: pointer; } 
      table { margin-left: auto; margin-right: auto; }
      td { padding: 8 px; }
      .button {
        background-color: #2f4468;
        border: none;
        color: white;
        padding: 10px 20px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 18px;
        margin: 6px 3px;
        cursor: pointer;
        -webkit-touch-callout: none;
        -webkit-user-select: none;
        -khtml-user-select: none;
        -moz-user-select: none;
        -ms-user-select: none;
        user-select: none;
        -webkit-tap-highlight-color: rgba(0,0,0,0);
      }
      img {  width: auto ;
        max-width: 100%% ;
        height: auto ; 
      }
    </style>
  </head>
  <body>
    <h1>ESP32 Robot DB1</h1>
    <img src="" id="photo" >
  <p><span id="textSliderValue">%SLIDERVALUE%</span></p>
  <p><input type="range" onchange="updateSliderPWM(this)" id="pwmSlider" min="0" max="255" value="%SLIDERVALUE%" step="1" class="slider"></p>
    <table>
      <tr><td colspan="3" align="center"><button class="button" onmousedown="toggleCheckbox('forward');" ontouchstart="toggleCheckbox('forward');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">Forward</button></td></tr>
      <tr><td align="center"><button class="button" onmousedown="toggleCheckbox('left');" ontouchstart="toggleCheckbox('left');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">Left</button></td><td align="center"><button class="button" onmousedown="toggleCheckbox('stop');" ontouchstart="toggleCheckbox('stop');">Stop</button></td><td align="center"><button class="button" onmousedown="toggleCheckbox('right');" ontouchstart="toggleCheckbox('right');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">Right</button></td></tr>
      <tr><td colspan="3" align="center"><button class="button" onmousedown="toggleCheckbox('backward');" ontouchstart="toggleCheckbox('backward');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">Backward</button></td></tr>                   
    </table>
   <script>
   function toggleCheckbox(x) {
     var xhr = new XMLHttpRequest();
     xhr.open("GET", "/action?go=" + x, true);
     xhr.send();
   }
   window.onload = document.getElementById("photo").src = "http://192.168.1.185" + ":81/stream";
   function updateSliderPWM(element) {
  var sliderValue = document.getElementById("pwmSlider").value;
  document.getElementById("textSliderValue").innerHTML = sliderValue;
  console.log(sliderValue);
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/slider?value="+sliderValue, true);
  xhr.send();
}
  </script>
  </body>
</html>
)rawliteral";

// Replaces placeholder with button section in your web page
String processor(const String& var){
  //Serial.println(var);
  if (var == "SLIDERVALUE"){
    return sliderValue;
  }
  return String();
}

void register_peers(){
  //Serial.begin(115200);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
   
  // register peer
  
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 2");
    return;
  }
  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 3");
    return;
  }
//  /// register third peer
//  memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
//  if (esp_now_add_peer(&peerInfo) != ESP_OK){
//    Serial.println("Failed to add peer");
//    return;
//  }
}

int robot_send(){
  Serial.print("in robot send: ");
  Serial.println(myData.variable);
  
  esp_err_t result = esp_now_send(0, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
    return 1;
  }
  else {
    Serial.println("Error sending the data");
    return 0;
  }
}

void setup(){
  myData.sliderValue = "000";
  myData.variable = "stop";
  // Serial port for debugging purposes
  Serial.begin(115200);

  WiFi.mode(WIFI_AP_STA);
  
  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(output, ledChannel);
  
  ledcWrite(ledChannel, sliderValue.toInt());

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP Local IP Address
  Serial.println(WiFi.localIP());
  register_peers();
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/slider?value=<inputMessage>
  server.on("/slider", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/slider?value=<inputMessage>
    if (request->hasParam(PARAM_INPUT)) {
      inputMessage = request->getParam(PARAM_INPUT)->value();
      myData.sliderValue = inputMessage;
      sliderValue =  myData.sliderValue;
      ledcWrite(ledChannel, sliderValue.toInt());
      if(!robot_send()) Serial.println("something wrong with robot send speed");
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.println(inputMessage);
    request->send(200, "text/plain", "OK");
  });

  // Send a GET request to <ESP_IP>/slider?value=<inputMessage>
  server.on("/action", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/slider?value=<inputMessage>
    if (request->hasParam(PARAM_CMD)) {
      inputMessage = request->getParam(PARAM_CMD)->value();
      myData.variable = inputMessage.c_str();
      variable = myData.variable;
      if(((variable== "stop") || (variable== "backward")|| (variable== "forward") ||(variable== "left") || (variable== "right"))){
        Serial.println(variable);
        if(!robot_send()) Serial.println("something wrong with robot send");
      }
      //ledcWrite(ledChannel, sliderValue.toInt());
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.println(inputMessage);
    request->send(200, "text/plain", "OK");
  });
  
  // Start server
  server.begin();
}
  
void loop() {
  
}
