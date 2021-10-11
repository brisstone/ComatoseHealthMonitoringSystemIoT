#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"

#include <ESP32Servo.h>



#include "MAX30105.h"           //MAX3010x library
#include "heartRate.h"          //Heart rate calculating algorithm
#include "spo2_algorithm.h"


MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

#include <LiquidCrystal_I2C.h>
 
LiquidCrystal_I2C I2C_LCD1(0x27, 16, 2);



uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid


byte pulseLED = 16; //Must be on PWM pin
byte readLED = 23; //Blinks with each data read


const int Buzzer1 = 16;

int LflexPin = 34;
int RflexPin = 35;




// Change these constants according to your project's design
const float VCC = 3.3;      // voltage at Ardunio 5V line
const float R_DIV = 47000.0;  // resistor used to create a voltage divider
const float flatResistance = 22000.0; // 25kresistance when flat
const float bendResistance = 25000.0;  // 100k resistance at 90 deg

float Rflex; //Resistance value
float Lflex; //Resistance value

float RRflex;
float LLflex;

float LflexThreshold = 24500.0; // Flex resistance threshold level
float RflexThreshold = 23000.0; // Flex resistance threshold level


#define DHT11_PIN 4
#define DS18B20 5
#define REPORTING_PERIOD_MS     1000

int LflexVal = 0;
int RflexVal = 0;




OneWire oneWire(DS18B20);
DallasTemperature sensors(&oneWire);


float temperature, Humidity, BPM, AvgBPM, SpO2, bodytemperature;
 

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

int temp = 1;
float beatsPerMinute;
int beatAvg;

uint32_t tsLastReport = 0;
 
/*Put your SSID & Password*/
const char* ssid = "TECNO CAMON";  // Enter SSID here
const char* password = "aaaaaaaa";  //Enter Password here


const char* http_username = "admin";
const char* http_password = "admin";

const char* PARAM_INPUT_1 = "state";



//// Uncomment the type of sensor in use:
#define DHTTYPE    DHT11     // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)
  

DHT dht(DHT11_PIN, DHTTYPE);


// Create AsyncWebServer object on port 80
AsyncWebServer server(80); 



String flexHealth1(){
        //Read rightflex
       int LADCflexVal = analogRead(LflexPin);
        int RADCflexVal = analogRead(RflexPin);
      
        
        float LVflex = LADCflexVal * VCC / 1023.0;
        float Lflex = R_DIV * (VCC / LVflex - 1.0);
       Lflex = abs(Lflex);
      
        float RVflex = RADCflexVal * VCC / 1023.0;
        float Rflex = R_DIV * (VCC / RVflex - 1.0);
        Rflex = abs(Rflex);

        RRflex = Rflex ;
        LLflex = Lflex ;

          Serial.println("Resistance: " + String(LLflex) + " ohms");
      
       Serial.println("Resistance: " + String(RRflex) + " ohms");
       
    
      //&& ((bodytemperature > 37.00 && (BPM > 120.00) && (SpO2 > 100.00) )
       
        // higher than threshold, NB: it is a negative value
        if( ((RRflex > RflexThreshold)|| (LLflex > RflexThreshold)) && ((bodytemperature < 37.00) && (BPM < 120.00) && (SpO2 < 100.00) ) ){
          
          return "Patient already recovering";
         
        }
        else if ( (RRflex < RflexThreshold)|| (LLflex < LflexThreshold)) {
          return "Checking Patient recovery1";
        }else{
          return "";
        }
      //      //threshold 2600
      //return "Checking Patient recovery"; 
}

String flexHealth2(){
  //Read rightflex
  
   int LADCflexVal = analogRead(LflexPin);
        int RADCflexVal = analogRead(RflexPin);
      
        
        float LVflex = LADCflexVal * VCC / 1023.0;
        float Lflex = R_DIV * (VCC / LVflex - 1.0);
       Lflex = abs(Lflex);
      
        float RVflex = RADCflexVal * VCC / 1023.0;
        float Rflex = R_DIV * (VCC / RVflex - 1.0);
        Rflex = abs(Rflex);


        RRflex = Rflex ;
        LLflex = Lflex ;

            Serial.println("Resistance: " + String(LLflex) + " ohms");
      
       
       Serial.println("Resistance: " + String(RRflex) + " ohms");
       
       
       
       if( ((RRflex > RflexThreshold)|| (LLflex > LflexThreshold)) && ((bodytemperature > 37.00) && (BPM > 120.00) && (SpO2 > 100.00) ) ){

        tone(Buzzer1, 4186, // C
        500); // half a second
        tone(Buzzer1, 5274, // E
        500); // half a second
        delay(500);
        
        return "Patient in critical condition";
        
        
      }
      else if ( (RRflex < RflexThreshold)|| (LLflex < LflexThreshold)) {
        return "checking patient 2 ";
      }else{
        return "";
      }
//      //threshold 2600
//return "Checking Patient condition"; 
}

String readDHTTemperature() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  float temperature = dht.readTemperature();

  // Read temperature as Fahrenheit (isFahrenheit = true)
  //float t = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(temperature)) {    
    Serial.println("Failed to read from DHT sensor!");
    
    return "--";
  }
  else {
//    Serial.println(temperature);
    return String(temperature);
  }
}

String readDHTHumidity() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float Humidity = dht.readHumidity();
  if (isnan(Humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return "--";
  }
  else {
//    Serial.println(Humidity);
    return String(Humidity);
  }
}

 
 
        


 const char index_html[] PROGMEM = R"rawliteral(<!DOCTYPE HTML>
<head>
    <title>COMATOSE PATIENT MONITORING SYSTEM</title>
    <meta name='viewport' content='width=device-width, initial-scale=1.0, maximum-scale= 1.0' >
    <meta http-equiv="refresh" content='5' >
    <link href='https://fonts.googleapis.com/css?family=Open+Sans:300,400,600' rel='stylesheet'>
        <style>
          html { font-family: 'Open Sans', sans-serif; display: block; margin: 0px auto; text-align: center;color: #444444;}
          body{margin: 0px;}
          h1 {margin: 0px;}
          h3 {margin: 0px;}
          .side-by-side{display: table-cell;vertical-align: middle;position: relative;}
          .heading{margin-top: 0px;}
          .text{font-weight: 600;font-size: 19px;width: 200px;}
          .reading{font-weight: 300;font-size: 50px;padding-right: 25px;}
          .checkHealth{font-weight: 250;font-size: 15px;padding-right: 25px;}
          .temperature .reading{color: #F29C1F;}
          .Humidity .reading{color: #3B97D3;}
          .BPM .reading{color: #FF0000;}
          .SpO2 .reading{color: #955BA5;}
          .bodytemperature .reading{color: #F29C1F;}
          .superscript{font-size: 17px;font-weight: 600;position: absolute;top: 10px;}
          .data{padding: 10px;}
          .container{display: table;margin: 0 auto;}
          .icon{width:65px}
        </style>
</head>
<body>
    <h1>COMATOSE PATIENT MONITORING SYSTEM</h1>
    <h3>Health is Wealth</h3>
    <div class='container'>
        <button onclick="logoutButton()">Logout</button>

        <div class='data temperature'>
            <div class='side-by-side icon'>
            <svg enable-background='new 0 0 19.438 54.003'height=54.003px id=Layer_1 version=1.1 viewBox='0 0 19.438 54.003'width=19.438px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><g><path d='M11.976,8.82v-2h4.084V6.063C16.06,2.715,13.345,0,9.996,0H9.313C5.965,0,3.252,2.715,3.252,6.063v30.982
            C1.261,38.825,0,41.403,0,44.286c0,5.367,4.351,9.718,9.719,9.718c5.368,0,9.719-4.351,9.719-9.718
            c0-2.943-1.312-5.574-3.378-7.355V18.436h-3.914v-2h3.914v-2.808h-4.084v-2h4.084V8.82H11.976z M15.302,44.833
            c0,3.083-2.5,5.583-5.583,5.583s-5.583-2.5-5.583-5.583c0-2.279,1.368-4.236,3.326-5.104V24.257C7.462,23.01,8.472,22,9.719,22
            s2.257,1.01,2.257,2.257V39.73C13.934,40.597,15.302,42.554,15.302,44.833z'fill=#F29C21 /></g></svg>
            </div>
            <div class='side-by-side text'>Room Temperature</div>
            <div class='side-by-side reading'>%temperature%<span class='superscript'>&deg;C</span></div>
        </div>

        <div class='data Humidity'>
            <div class='side-by-side icon'>
            <svg enable-background='new 0 0 29.235 40.64'height=40.64px id=Layer_1 version=1.1 viewBox='0 0 29.235 40.64'width=29.235px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><path d='M14.618,0C14.618,0,0,17.95,0,26.022C0,34.096,6.544,40.64,14.618,40.64s14.617-6.544,14.617-14.617
            C29.235,17.95,14.618,0,14.618,0z M13.667,37.135c-5.604,0-10.162-4.56-10.162-10.162c0-0.787,0.638-1.426,1.426-1.426
            c0.787,0,1.425,0.639,1.425,1.426c0,4.031,3.28,7.312,7.311,7.312c0.787,0,1.425,0.638,1.425,1.425
            C15.093,36.497,14.455,37.135,13.667,37.135z'fill=#3C97D3 /></svg>
            </div>
            <div class='side-by-side text'>Room Humidity</div>
            <div class='side-by-side reading'>%Humidity%<span class='superscript'>pernt</span></div>
        </div>

        <div class='data Heart Rate'>
            <div class='side-by-side icon'>
            <svg enable-background='new 0 0 40.542 40.541'height=40.541px id=Layer_1 version=1.1 viewBox='0 0 40.542 40.541'width=40.542px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><g><path d='M34.313,20.271c0-0.552,0.447-1,1-1h5.178c-0.236-4.841-2.163-9.228-5.214-12.593l-3.425,3.424
            c-0.195,0.195-0.451,0.293-0.707,0.293s-0.512-0.098-0.707-0.293c-0.391-0.391-0.391-1.023,0-1.414l3.425-3.424
            c-3.375-3.059-7.776-4.987-12.634-5.215c0.015,0.067,0.041,0.13,0.041,0.202v4.687c0,0.552-0.447,1-1,1s-1-0.448-1-1V0.25
            c0-0.071,0.026-0.134,0.041-0.202C14.39,0.279,9.936,2.256,6.544,5.385l3.576,3.577c0.391,0.391,0.391,1.024,0,1.414
            c-0.195,0.195-0.451,0.293-0.707,0.293s-0.512-0.098-0.707-0.293L5.142,6.812c-2.98,3.348-4.858,7.682-5.092,12.459h4.804
            c0.552,0,1,0.448,1,1s-0.448,1-1,1H0.05c0.525,10.728,9.362,19.271,20.22,19.271c10.857,0,19.696-8.543,20.22-19.271h-5.178
            C34.76,21.271,34.313,20.823,34.313,20.271z M23.084,22.037c-0.559,1.561-2.274,2.372-3.833,1.814
            c-1.561-0.557-2.373-2.272-1.815-3.833c0.372-1.041,1.263-1.737,2.277-1.928L25.2,7.202L22.497,19.05
            C23.196,19.843,23.464,20.973,23.084,22.037z'fill=#26B999 /></g></svg>
            </div>
            <div class='side-by-side text'>Heart Rate</div>
            <div class='side-by-side reading'>%BPM%<span class='superscript'>BPM</span></div>
        </div>

        <div class='data Blood Oxygen'>
            <div class='side-by-side icon'>
            <svg enable-background='new 0 0 58.422 40.639'height=40.639px id=Layer_1 version=1.1 viewBox='0 0 58.422 40.639'width=58.422px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><g><path d='M58.203,37.754l0.007-0.004L42.09,9.935l-0.001,0.001c-0.356-0.543-0.969-0.902-1.667-0.902
            c-0.655,0-1.231,0.32-1.595,0.808l-0.011-0.007l-0.039,0.067c-0.021,0.03-0.035,0.063-0.054,0.094L22.78,37.692l0.008,0.004
            c-0.149,0.28-0.242,0.594-0.242,0.934c0,1.102,0.894,1.995,1.994,1.995v0.015h31.888c1.101,0,1.994-0.893,1.994-1.994
            C58.422,38.323,58.339,38.024,58.203,37.754z'fill=#955BA5 /><path d='M19.704,38.674l-0.013-0.004l13.544-23.522L25.13,1.156l-0.002,0.001C24.671,0.459,23.885,0,22.985,0
            c-0.84,0-1.582,0.41-2.051,1.038l-0.016-0.01L20.87,1.114c-0.025,0.039-0.046,0.082-0.068,0.124L0.299,36.851l0.013,0.004
            C0.117,37.215,0,37.62,0,38.059c0,1.412,1.147,2.565,2.565,2.565v0.015h16.989c-0.091-0.256-0.149-0.526-0.149-0.813
            C19.405,39.407,19.518,39.019,19.704,38.674z'fill=#955BA5 /></g></svg>
            </div>
            <div class='side-by-side text'>Blood Oxygen</div>
            <div class='side-by-side reading'>%SpO2%<span class='superscript'>perct</span></div>
        </div>

        <div class='data Body Temperature'>
            <div class='side-by-side icon'>
            <svg enable-background='new 0 0 19.438 54.003'height=54.003px id=Layer_1 version=1.1 viewBox='0 0 19.438 54.003'width=19.438px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><g><path d='M11.976,8.82v-2h4.084V6.063C16.06,2.715,13.345,0,9.996,0H9.313C5.965,0,3.252,2.715,3.252,6.063v30.982
            C1.261,38.825,0,41.403,0,44.286c0,5.367,4.351,9.718,9.719,9.718c5.368,0,9.719-4.351,9.719-9.718
            c0-2.943-1.312-5.574-3.378-7.355V18.436h-3.914v-2h3.914v-2.808h-4.084v-2h4.084V8.82H11.976z M15.302,44.833
            c0,3.083-2.5,5.583-5.583,5.583s-5.583-2.5-5.583-5.583c0-2.279,1.368-4.236,3.326-5.104V24.257C7.462,23.01,8.472,22,9.719,22
            s2.257,1.01,2.257,2.257V39.73C13.934,40.597,15.302,42.554,15.302,44.833z'fill=#F29C21 /></g></svg>
            </div>
            <div class='side-by-side text'>Body Temperature</div>
            <div class='side-by-side reading'>%bodytemperature%<span class='superscript'>&deg;C</span></div>
        </div>

        
        <div class=''> 
             <div class="side-by-side checkHealth">%flexHealth1%</div>
            <div class="side-by-side text">HEALTH STATE</div>
            <div class='side-by-side checkHealth'>%flexHealth2%</div>
        </div>
        
    </div>


    <script>function toggleCheckbox(element) {
        var xhr = new XMLHttpRequest();
        if(element.checked){ 
          xhr.open("GET", "/update?state=1", true); 
          document.getElementById("state").innerHTML = "ON";  
        }
        else { 
          xhr.open("GET", "/update?state=0", true); 
          document.getElementById("state").innerHTML = "OFF";      
        }
        xhr.send();
      }
      function logoutButton() {
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/logout", true);
        xhr.send();
        setTimeout(function(){ window.open("/logged-out","_self"); }, 1000);
      }
      </script>

</body>
</HTML>)rawliteral";


const char logout_html[] PROGMEM = R"rawliteral(<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
</head>
<body>
  <p>Logged out or <a href="/">return to homepage</a>.</p>
  <p><strong>Note:</strong> close all web browser tabs to complete the logout process.</p>
</body>
</html>)rawliteral";


// Replaces placeholder with values
String processor(const String& var){
  //Serial.println(var);
  if(var == "temperature"){
    return readDHTTemperature();
  }
  else if(var == "Humidity"){
     return readDHTHumidity();
  }
  else if(var == "BPM"){
    return String(BPM);
  }
  else if(var == "SpO2"){
    return String(SpO2);
  }
  else if(var == "bodytemperature"){
    return String(bodytemperature);
  } 
  else if(var == "flexHealth1"){
    return flexHealth1();
  }
  else if(var == "flexHealth2"){
    return flexHealth2();
  }
  
  return String();
}


void onBeatDetected()
{
  Serial.println("Beat!");
}

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}


void setup() 
{
  Serial.begin(115200);
  pinMode(19, OUTPUT); //max30102 INT pin, remove
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
   pinMode(DHT11_PIN, OUTPUT);
   
   pinMode(LflexPin, INPUT);
  pinMode(RflexPin, INPUT);
  
//  pinMode(Buzzer1, OUTPUT);
//   digitalWrite(Buzzer1, HIGH);
tone(Buzzer1, 1000, // C
        500); // half a second

//  tone(Buzzer1, 5274, // E
//        500); // half a second

              
   // Initialize The I2C LCD
  I2C_LCD1.init();
  // Turn ON The Backlight
  I2C_LCD1.backlight();
  // Print A Message To The 1st Line
  I2C_LCD1.print("iNITIALIZING");
  // Set Cursor To Line 2
  I2C_LCD1.setCursor(0,1);
  // Print A Message To The 2nd Line
  I2C_LCD1.print("gETTING READY");
  

 
  delay(100);  

  // Allow allocation of all timers for buzzer
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  dht.begin();
  sensors.begin();
  
 // Initialize max30102 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED


  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

 if(!SPIFFS.begin(true)){
    Serial.println("spiff error");
    return;
  }

  Serial.println("Connecting to ");
  Serial.println(ssid);
 
  //connect to your local wi-fi network
  WiFi.begin(ssid, password);
 
  //check wi-fi is connected to wi-fi network
  while (WiFi.status() != WL_CONNECTED) {
  delay(1000);
  Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected..!");
  Serial.print("Got IP: ");  Serial.println(WiFi.localIP());

   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    if(!request->authenticate(http_username, http_password))
      return request->requestAuthentication();
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readDHTTemperature().c_str());
  });
  server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readDHTHumidity().c_str());
  });
 
  server.on("/logout", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(401);
  });

  server.on("/logged-out", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", logout_html, processor);
  });
  
   server.on("/rain", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/health.png", "image/png");
  });


//  // Send a GET request to <ESP_IP>/update?state=<inputMessage>
//  server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
//    if(!request->authenticate(http_username, http_password))
//      return request->requestAuthentication();
//    String inputMessage;
//    String inputParam;
//    // GET input1 value on <ESP_IP>/update?state=<inputMessage>
//    if (request->hasParam(PARAM_INPUT_1)) {
//      inputMessage = request->getParam(PARAM_INPUT_1)->value();
//      inputParam = PARAM_INPUT_1;
//      digitalWrite(output, inputMessage.toInt());
//    }
//    else {
//      inputMessage = "No message sent";
//      inputParam = "none";
//    }
//    Serial.println(inputMessage);
//    request->send(200, "text/plain", "OK");
////  });
 
  server.begin();
  Serial.println("HTTP server started");
 
 
 
 
}




void loop() {
 
//      DHT.read11(DHT11_PIN);
//      temperature = DHT.temperature;
//      
//      Humidity = DHT.humidity;

    

        


      //threshold 2600
      
  //    particleSensor.check(); //consider putting into if checkforbeat condition
    
  //    if (!particleSensor.available()) return; 
  
    long irValue = particleSensor.getIR();
  
    if (checkForBeat(irValue) == true)
    {

      //Read DS18B20 TEMPERATURE 
      sensors.requestTemperatures();
      bodytemperature = sensors.getTempCByIndex(0); 
     
      
//      float bodytemperature = particleSensor.readTemperature();
//
//      Serial.print("temperatureC=");
//      Serial.print(temperature, 4);
      
      //We sensed a beat!
         if (irValue > 40000){
          long delta = millis() - lastBeat;
          lastBeat = millis();
    
          beatsPerMinute = 60 / (delta / 1000.0);
          BPM = beatsPerMinute * 22;
          
         oxygen();
          
          }else{
             BPM = 0;
          }

        
      
      
                if (millis() - tsLastReport > REPORTING_PERIOD_MS) 
              {
//                Serial.print("Room Temperature: ");
//                Serial.print(temperature);
//                Serial.println("°C");
//            
//                Serial.print("Room Humidity: ");
//                Serial.print(Humidity);
//                Serial.println("%");
//            
//                Serial.print("BPM: ");
//                Serial.println(BPM);
//            
//                Serial.print("SpO2: ");
//                Serial.print(SpO2);
//                Serial.println("%");
//            
//                Serial.print("Body Temperature: ");
//                Serial.print(bodytemperature);
//                Serial.println("°C");
//            
//                Serial.println("*********************************");
//                Serial.println();
             
                tsLastReport = millis();
              }
  
      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable
  
        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
  
//       Serial.print("IR=");
//    Serial.print(irValue);
//    Serial.print(", BPM=");
//    Serial.print(beatsPerMinute);
//    Serial.print(", Avg BPM=");
//    Serial.print(beatAvg);


    if (irValue < 40000){
//      Serial.print(" No finger?");
  
    Serial.println();
    }

    float temperature = dht.readTemperature();

 
   I2C_LCD1.clear();
   I2C_LCD1.setCursor(0,0);
  I2C_LCD1.print("T:" + String(bodytemperature) + "C"); 
  I2C_LCD1.setCursor(9,0);
  I2C_LCD1.print("O2:" + String(SpO2) + "O2");  
     I2C_LCD1.setCursor(0,1);
     I2C_LCD1.print("BP:" + String(BPM) + "BPM");
     
    //&& ((bodytemperature > 37.00 && (BPM > 120.00) && (SpO2 > 100.00) )
       
        // higher than threshold, NB: it is a negative value
        if( ((RRflex > RflexThreshold)|| (LLflex > RflexThreshold)) && ((bodytemperature < 37.00) && (BPM < 120.00) && (SpO2 < 100.00) ) ){

          if((bodytemperature > 26) && (BPM > 70.00) && (SpO2 > 30.00)){
             digitalWrite(Buzzer1, HIGH);
             delay(100);
            digitalWrite(Buzzer1, LOW);
          }else if(((RRflex < RflexThreshold)|| (LLflex < RflexThreshold)) && ((bodytemperature < 37.00) && (BPM < 120.00) && (SpO2 < 100.00) )){
            digitalWrite(Buzzer1, LOW);
          }
         
          
        }
        else if ( ((RRflex > RflexThreshold)|| (LLflex > RflexThreshold)) && ((bodytemperature > 37.00) || (BPM > 120.00) || (SpO2 > 100.00) ) ) {
          digitalWrite(Buzzer1, HIGH);
          delay(100);
          digitalWrite(Buzzer1, LOW);
        }else{
           digitalWrite(Buzzer1, LOW);
        }
      //      //threshold 2600
      //return "Checking Patient recovery"; 
      
    
}

float oxygen(){
             bufferLength = 50; //buffer length of 100 stores 4 seconds of samples running at 25sps
          
            //read the first 100 samples, and determine the signal range
            for (byte i = 0 ; i < bufferLength ; i++)
            {
              while (particleSensor.available() == false) //do we have new data?
                particleSensor.check(); //Check the sensor for new data
          
              redBuffer[i] = particleSensor.getRed();
              irBuffer[i] = particleSensor.getIR();
              particleSensor.nextSample(); //We're finished with this sample so move to next sample
          
//              Serial.print(F("red="));
//              Serial.print(redBuffer[i], DEC);
//              Serial.print(F(", ir="));
//              Serial.println(irBuffer[i], DEC);
            }
          
            //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
            maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
                if(validSPO2 == 1){
                SpO2 = spo2;
                 
  
                return SpO2;
                
                }else{
                  SpO2 = 0;
                  return SpO2;
                }
                
          
            //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
            while (1)
            {
              //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
              for (byte i = 25; i < 100; i++)
              {
                redBuffer[i - 25] = redBuffer[i];
                irBuffer[i - 25] = irBuffer[i];
              }
          
              //take 25 sets of samples before calculating the heart rate.
              for (byte i = 75; i < 100; i++)
              {
                while (particleSensor.available() == false) //do we have new data?
                  particleSensor.check(); //Check the sensor for new data
          
                digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
          
                redBuffer[i] = particleSensor.getRed();
                irBuffer[i] = particleSensor.getIR();
                particleSensor.nextSample(); //We're finished with this sample so move to next sample
          
                //send samples and calculation result to terminal program through UART
                Serial.print(F("red="));
                Serial.print(redBuffer[i], DEC);
                Serial.print(F(", ir="));
                Serial.print(irBuffer[i], DEC);
          
                Serial.print(F(", HR="));
                Serial.print(heartRate, DEC);
          
                Serial.print(F(", HRvalid="));
                Serial.print(validHeartRate, DEC);
          
                Serial.print(F(", SPO2="));
                Serial.print(spo2, DEC);
          
                Serial.print(F(", SPO2Valid="));
                Serial.println(validSPO2, DEC);
              }
          
              //After gathering 25 new samples recalculate HR and SP02
              maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
             if(validSPO2 == 1){
                SpO2 = spo2;
                return SpO2;
                }else{
                  SpO2 = 0;
                  return SpO2;
                }
              
              
            }
            SpO2 = 0;
            return SpO2;
          }
    
 
 
 
