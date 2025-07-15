#include <WiFi.h>
#include <FS.h>
#include <SD.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Adafruit_TCS34725.h"
#include <math.h>

#define LED_PIN 2 // Define the GPIO for the LED
#define SD_CS_PIN 5 // SD card CS pin (adjust according to your wiring)

struct ColorEntry {
  float r, g, b;
};

struct Position {
  float x;
  float y;
};

// Link lengths (IK)
float linkLengths[] = {50.0, 50.0, 75.0};
float linkAnglesIK[] = {-1.57, 0, 1.57};
const int numLinks = 3;
const float tolerance = 0.5;

ColorEntry colorList[] = {
  ColorEntry{93.25, 92.5, 62.5}, // Empty (no object detected)
  ColorEntry{111.5, 86.5, 62},   // Purple
  ColorEntry{150, 69.5, 50.5},   // Red
  ColorEntry{144, 70, 42},       // Orange
  ColorEntry{122, 90, 40.5},     // Yellow
  ColorEntry{93.5, 111.25, 51}   // Green
};

int numColors = 6;

// WiFi credentials
const char* ssid = "Toborki GB";
const char* password = "K0rnisz0ny";

// Create a web server on port 80
WebServer server(80);

#define SERVOMIN  80 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  460 // This is the 'maximum' pulse length count (out of 4096)
#define SERVOMID  270 
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);

int servoPos[16];
int servoPosGoal[16];
bool busy = false;

long timeToUpdateServo = 0;

String inputString = "";    // A string to hold incoming data
bool stringComplete = false; // Whether the string is ready to be processed

float sns_red, sns_green, sns_blue;
int color_id = -1;

//G-code
int gCodePointer = 0;
long blockExecutionUntil = 0;
bool runGCodeExecution = false; //runGCodeProgram
int ifStack = 0;

String program[] = {
  ":START", 
  "G1 A90 B90 C60 D20 E65 F0",
  ":DISPENSE", 
  "G1 A150 B90 C10 D5 E65", 
  "G1 F180", 
  "WAIT 500", 
  "G1 F0", 
  "WAIT 500",
  ":GET_COLOR", 
  "COLOR", 
  "IF COLOR == 0",
  "WAIT 2000",
  "JMP DISPENSE", 
  "END",
  ":GRAB", 
  "G1 E90",
  "WAIT 500", 
  "G1 A150 B100 C30 D5",
  "// DROP AT PURPLE BOX", 
  "IF COLOR == 1", 
  "G1 A60 B70 C30 D50", 
  "G1 E70", 
  "END",
  "// DROP AT RED BOX", 
  "IF COLOR == 2", 
  "G1 A75 B80 C30 D30", 
  "G1 E70", 
  "END",
  "// DROP AT ORANGE BOX", 
  "IF COLOR == 3", 
  "G1 A90 B80 C30 D30", 
  "G1 E70", 
  "END",
  "// DROP AT YELLOW BOX", 
  "IF COLOR == 4", 
  "G1 A110 B80 C30 D30", 
  "G1 E70", 
  "END",
  "// DROP AT GREEN BOX", 
  "IF COLOR == 5", 
  "G1 A125 B55 C60 D20", 
  "G1 E70", 
  "END",
  "G1 E70", 
  "WAIT 1000",
  "G1 A90 B90 C60 D20 E70",
  "WAIT 250",
  "JMP START"
};

void logRequest() {
  String clientIP = server.client().remoteIP().toString(); // Get client IP
  String method = (server.method() == HTTP_GET) ? "GET" : "POST"; // Determine request method
  String path = server.uri(); // Requested path

  // Log client IP, method, and requested path
  Serial.print("[");
  Serial.print(millis());
  Serial.print("] ");
  Serial.print("Client ");
  Serial.print(clientIP);
  Serial.print(" requested ");
  Serial.print(method);
  Serial.print(" ");
  Serial.println(path);
}

void handlePreflight() {
    // Send CORS headers for preflight response
    handleCORS();
    server.send(204); // No Content response for preflight
}

void handleCORS() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

// Function to serve static files from SD card
void handleFileRequest() {
  logRequest();
  if (server.method() == HTTP_OPTIONS) {
    handlePreflight();
    return;
  }

  String path = server.uri();
  if (path.endsWith("/")) {
    path += "index.html"; // Serve index.html if root directory is accessed
  }

  String contentType = getContentType(path);

  // Try to open the requested file
  if (SD.exists(path)) {
    File file = SD.open(path);
    Serial.print("File ");
    Serial.print(path);
    Serial.println(" found!");
    server.streamFile(file, contentType);
    file.close();
  } else {
    // If file is not found, return 404
    server.send(404, "text/plain", "File Not Found");
  }
}

// Helper function to determine content type
String getContentType(String filename) {
  if (filename.endsWith(".html")) return "text/html";
  if (filename.endsWith(".css")) return "text/css";
  if (filename.endsWith(".js")) return "application/javascript";
  if (filename.endsWith(".png")) return "image/png";
  if (filename.endsWith(".jpg")) return "image/jpeg";
  if (filename.endsWith(".gif")) return "image/gif";
  if (filename.endsWith(".json")) return "application/json";
  return "text/plain";
}

// API handler for toggling LED
void handleToggleLED() {
  logRequest();
  static bool ledState = false;
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  server.send(200, "application/json", "{\"status\":\"LED toggled\"}");
}

// API handler for returning millis()
void handleGetMillis() {
  logRequest();
  unsigned long uptime = millis();
  String jsonResponse = "{\"millis\":" + String(uptime) + "}";
  server.send(200, "application/json", jsonResponse);
}


// API handler for RGB sensor
void handleGetRGB() {
  logRequest();
  String jsonResponse = "{\"R\":" + String(sns_red) + ",\"G\":" + String(sns_green) + ",\"B\":" + String(sns_blue) + "}";
  server.send(200, "application/json", jsonResponse);
}


// API handler for setting multiple servo positions via POST request
void setServoPosition() {
  logRequest();
  handleCORS();

  // Check if request body contains JSON
  if (server.hasArg("plain")) {
    // Parse the JSON
    StaticJsonDocument<400> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));

    // Handle JSON parsing error
    if (error) {
      server.send(400, "application/json", "{\"status\":\"Invalid JSON\"}");
      return;
    }

    // Create a response object
    DynamicJsonDocument response(200);

    // Loop over each key-value pair in the JSON object
    for (JsonPair kv : doc.as<JsonObject>()) {
      String key = kv.key().c_str();
      
      // Check if key is in the format "servoX" where X is an integer
      if (key.startsWith("servo") && key.length() > 5) {
        int servoID = key.substring(5).toInt(); // Extract servo ID number
        int position = kv.value().as<int>();    // Get servo position

        // Ensure the position is within valid bounds (0-180)
        if (position >= 0 && position <= 180) {
          setServoPos(servoID, position);
          response[key] = "Updated to " + String(position);
        } else {
          response[key] = "Position out of range";
        }
      } else {
        response[key] = "Invalid key format";
      }
    }

    // Send response with the status for each servo updated
    String jsonResponse;
    serializeJson(response, jsonResponse);
    server.send(200, "application/json", jsonResponse);
  } else {
    // Respond with an error if no body is provided
    server.send(400, "application/json", "{\"status\":\"No JSON body found\"}");
  }
}

void getServoPosition() {
  //logRequest();
  handleCORS();

  // Create a JSON document to store the response
  DynamicJsonDocument response(400); // Adjust size if you have more than 16 servos

  // Map raw PWM values to angles (0-180 degrees) for each servo
  for (int i = 0; i < 16; i++) { // Assuming 16 servos
    int angle = map(servoPosGoal[i], SERVOMIN, SERVOMAX, 0, 180); // Map to 0-180
    String servoKey = "servo" + String(i); // Create key in the format "servoX"
    response[servoKey] = angle;
  }

  // Serialize the response to JSON
  String jsonResponse;
  serializeJson(response, jsonResponse);

  // Send the JSON response
  server.send(200, "application/json", jsonResponse);
}


// API handler for setting multiple servo positions via POST request
void setServoPWM() {
  logRequest();

  if (server.method() == HTTP_OPTIONS) {
    handleCORS();
    server.send(204); // No Content
    return;
  }

  // Check if request body contains JSON
  if (server.hasArg("plain")) {
    // Parse the JSON
    StaticJsonDocument<400> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));

    // Handle JSON parsing error
    if (error) {
      server.send(400, "application/json", "{\"status\":\"Invalid JSON\"}");
      return;
    }

    // Create a response object
    DynamicJsonDocument response(200);

    // Loop over each key-value pair in the JSON object
    for (JsonPair kv : doc.as<JsonObject>()) {
      String key = kv.key().c_str();
      
      // Check if key is in the format "servoX" where X is an integer
      if (key.startsWith("servo") && key.length() > 5) {
        int servoID = key.substring(5).toInt(); // Extract servo ID number
        int position = kv.value().as<int>();    // Get servo position

        // Ensure the position is within valid bounds (0-180)
        if (position >= 0 && position <= 4096) {
          //pwm.setPWM(i, 0, position);
          servoPosGoal[servoID] = position;
          //servoPos[i] = position;

          //setServoPos(servoID, position);
          response[key] = "Updated to " + String(position);
        } else {
          response[key] = "Position out of range";
        }
      } else {
        response[key] = "Invalid key format";
      }
    }

    // Send response with the status for each servo updated
    String jsonResponse;
    serializeJson(response, jsonResponse);
    server.send(200, "application/json", jsonResponse);
  } else {
    // Respond with an error if no body is provided
    server.send(400, "application/json", "{\"status\":\"No JSON body found\"}");
  }
}

// API handler for running a GCode command
void runGCodeCommandHandler() {
  logRequest();
  handleCORS();

  // Check if request body contains JSON
  if (server.hasArg("plain")) {
    // Parse the JSON
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));

    // Handle JSON parsing error
    if (error) {
      server.send(400, "application/json", "{\"status\":\"Invalid JSON\"}");
      return;
    }

    // Extract the "command" field from the JSON
    if (doc.containsKey("command")) {
      String command = doc["command"].as<String>();

      // Run the GCode command
      runGCodeCommand(command, false);

      // Respond with success
      String response = "{\"status\":\"Command executed successfully\", \"command\":\"" + command + "\"}";
      server.send(200, "application/json", response);
    } else {
      // Respond with error if "command" key is missing
      server.send(400, "application/json", "{\"status\":\"Missing 'command' field\"}");
    }
  } else {
    // Respond with an error if no body is provided
    server.send(400, "application/json", "{\"status\":\"No JSON body found\"}");
  }
}


// Validate and Update setServoPos() function
void setServoPos(uint8_t n, float angle) {
  // Validate if the servo ID is within range (0-15)
  if (n < 0 || n >= 16) {
    Serial.println("Invalid servo ID");
    return;
  }

  if(angle < 0 || angle > 180){
    Serial.println("Invalid servo angle");
    return;
  }

  // Map the angle to the appropriate PWM value
  uint16_t pwmVal = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  //pwm.setPWM(n, 0, pwmVal);  // Corrected to send PWM on channel 'n'
  servoPosGoal[n] = pwmVal;
  if(n == 5){
    // servoPos[n] = pwmVal;
    if(pwmVal > 90){
      pwmVal = 90;
    }
  }

  Serial.print("Servo ");
  Serial.print(n);
  Serial.print(" set to: ");
  Serial.print(angle);
  Serial.print("deg (");
  Serial.print(pwmVal);
  Serial.println(")");

}

void setServoPulse(uint8_t n, double pulse){
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse); 
}

void setup() {
  Serial.begin(115200);
  Serial.println("Serial begin");
  
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Start with LED off

  Serial.println("TCS34725 Init");
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  tcs.setInterrupt(true); //turn off LED

  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Wire.setClock(100000);

  for(int i=0; i<16; i++){
    pwm.setPWM(i, 0, SERVOMID);
    servoPosGoal[i] = SERVOMID;
    servoPos[i] = SERVOMID;
  }

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card initialization failed!");
    return;
  }else{
    Serial.println("SD Card initialized.");
  }
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

  // Define API routes
  server.on("/api/toggle-led", HTTP_GET, handleToggleLED); // Endpoint to toggle LED
  server.on("/api/millis", HTTP_GET, handleGetMillis); // Endpoint to get uptime in millis
  server.on("/api/rgb", HTTP_GET, handleGetRGB);
  server.on("/api/servo", HTTP_POST, setServoPosition);
  server.on("/api/servo", HTTP_GET, getServoPosition);
  server.on("/api/set-pwm", HTTP_POST, setServoPWM);
  server.on("/api/gcode", HTTP_POST, runGCodeCommandHandler);
  server.on("/api/gcode", HTTP_OPTIONS, handlePreflight);

  // Route to serve files from the SD card
  server.onNotFound(handleFileRequest); // Handle all other requests as file requests

  // Start the web server
  server.begin();
  Serial.println("HTTP server started.");
}

void loop() {
  //unsigned long timeMs = millis();

  //must be first
  if(timeToUpdateServo < millis()){
    if(millis() - timeToUpdateServo > 25){ // show warning when loop is behind
      Serial.print("Warning, servo loop behind ");
      Serial.print(millis() - timeToUpdateServo);
      Serial.println("ms");
    }

    timeToUpdateServo = millis() + 16;

    bool reachedGoad = true;
    for(int i=0; i<16; i++){
      if(abs(servoPos[i] - servoPosGoal[i]) < 3){
        servoPos[i] = servoPosGoal[i];
      }else{
        if(servoPos[i] < servoPosGoal[i]){
          servoPos[i]+=2;
        }

        if(servoPos[i] > servoPosGoal[i]){
          servoPos[i]-=2;
        }
      }

      //check if all motors reached the goal
      if(servoPos[i] != servoPosGoal[i]){
        reachedGoad = false;
      }

      pwm.setPWM(i, 0, servoPos[i]);
    }
    busy = !reachedGoad;
    digitalWrite(LED_PIN, busy);
  }

  // if(timeToGetColor < timeMs && !(timeToUpdateServo < timeMs-5)){
  //   if(!readingInProgress){
  //     tcs.setInterrupt(false); // Turn on LED, start the reading
  //     readingInProgress = true;
  //     timeToGetColor = timeMs + 5;
  //   }else{
  //     tcs.getRGB(&sns_red, &sns_green, &sns_blue);
  //     tcs.setInterrupt(true); // Turn off LED
  //     readingInProgress = false;
  //     timeToGetColor = timeMs + 995;

  //     int cid = findClosestColorId(sns_red, sns_green, sns_blue);
  //     if(cid == 1){Serial.println("PURPLE");}else
  //     if(cid == 2){Serial.println("RED");}else
  //     if(cid == 3){Serial.println("ORANGE");}else
  //     if(cid == 4){Serial.println("YELLOW");}else
  //     if(cid == 5){Serial.println("GREEN");}
      
  //   }
  // }

  //execute Serial commands
  if (stringComplete) {
    runGCodeCommand(inputString, false);  // Process the command
    inputString = "";             // Clear the string for the next input
    stringComplete = false;       // Reset the flag
  }

  runGCodeProgram();

  // Handle client requests
  server.handleClient();
}
// G-Code --------------------------------------------------
void runGCodeProgram(){
  if(!busy && blockExecutionUntil < millis() && runGCodeExecution){
    
    if (gCodePointer >= sizeof(program) / sizeof(program[0])) {
      runGCodeCommand("STOP", false);
      return;
    }

    //trip leading spaces
    // int i=0;
    // while (i < program.length() && program.charAt(gCodePointer) == ' ') {
    //   i++;
    // }
    String line = program[gCodePointer];
    line.trim();
    //String line = program[gCodePointer].substring(i);

    if(isLineExecutable(line)){
      Serial.print("[G-Code] [");
      Serial.print(line);
      Serial.print("]: ");
      runGCodeCommand(line, true);
    }else{
      gCodePointer++;
    }
  }
}

bool isLineExecutable(const String& line) {
  if (line.charAt(0) == ':' || line.charAt(0) == '/' || line.length() == 0 || line.charAt(0) == ' ') {
    return false;
  }
  return true;
}


// Executes G-Code command
void runGCodeCommand(const String& command, bool advanceProgram) {
    // Find the first space to separate the command from the parameters
    int spaceIndex = command.indexOf(' ');
    String gcodeCommand;
    String parameters;

    if (spaceIndex != -1) {
        // Extract the command and parameters
        gcodeCommand = command.substring(0, spaceIndex);
        parameters = command.substring(spaceIndex + 1);
    } else {
        // No parameters, the entire string is the command
        gcodeCommand = command;
        parameters = "";
    }

    // Call the appropriate handler based on the command
    if (gcodeCommand.equals("G0")) {
        Serial.println(command);
        handleG0Command(parameters);
        if (advanceProgram) gCodePointer++;
    }
    else 
    if (gcodeCommand.equals("G1")) {
        Serial.println(command);
        handleG1Command(parameters);
        if (advanceProgram) gCodePointer++;
    }
    else if (gcodeCommand.equals("COLOR")) {
        tcs.setInterrupt(false); // Turn on LED
        delay(10);
        tcs.getRGB(&sns_red, &sns_green, &sns_blue);
        tcs.setInterrupt(true); // Turn off LED
        Serial.print("COLOR (");
        Serial.print(sns_red);
        Serial.print(", ");
        Serial.print(sns_green);
        Serial.print(", ");
        Serial.print(sns_blue);
        Serial.print(") is ID: ");
        color_id = findClosestColorId(sns_red, sns_green, sns_blue);
        Serial.println(color_id);
        if (advanceProgram) gCodePointer++;
    }
    else if(gcodeCommand.equals("JMP")){
      jumpToLabel(":"+parameters);
    }
    else if(gcodeCommand.equals("WAIT")){
      int waitTime = parameters.toInt();
      blockExecutionUntil = millis() + waitTime;
      Serial.print("Waiting ");
      Serial.print(waitTime);
      Serial.println("ms");
      if (advanceProgram) gCodePointer++;
    }
    else if(gcodeCommand.equals("START")){
      runGCodeExecution = true;
      Serial.println("STARTING THE PROGRAM");
      if (advanceProgram) gCodePointer++;
    }
    else if(gcodeCommand.equals("STOP")){
      runGCodeExecution = false;
      gCodePointer = 0;
      Serial.println("STOPPING THE PROGRAM");
    }
    else if(gcodeCommand.equals("PAUSE")){
      runGCodeExecution = false;
      Serial.println("PAUSING THE PROGRAM");
    }
    else if(gcodeCommand.equals("IF")){
      handleIfCondition(parameters);
      if (advanceProgram) gCodePointer++;
    }
    else if(gcodeCommand.equals("END")){
      handleEndCondition();
      if (advanceProgram) gCodePointer++;
    }
    else {
        // Unknown command, handle error or ignore
        Serial.println("Unknown command: " + gcodeCommand);
    }
}

void handleG0Command(const String& command) {
    // Map motor labels to motor numbers
    const uint8_t motorMap[] = {'X', 'Y', 'Z', 'E'};
    float x = 0;
    float y = 0;
    float z = 0;
    float e = 0;
    // Loop through each motor label
    for (uint8_t i = 0; i < 4; i++) {
        bool found = false;
        double val = extractValue(command, motorMap[i], found);
        if (found) {
            if(motorMap[i] == 'X') x = val;
            if(motorMap[i] == 'Y') y = val;
            if(motorMap[i] == 'Z') z = val;
            if(motorMap[i] == 'E') e = val;
        }
    }

    Serial.println("----");
    Serial.print("Solving IK for position ");
    Serial.print("X:");
    Serial.print(x);
    Serial.print(", Y:");
    Serial.print(y);
    Serial.print(", Z:");
    Serial.println(z);

    
    solveIK3D(x, y, z);

    Position p = calculatePosition(numLinks);
    Serial.print("Reached to X:");
    Serial.print(p.x);
    Serial.print(", Y:");
    Serial.println(p.y);
}

void handleG1Command(const String& command) {
    // Map motor labels to motor numbers
    const uint8_t motorMap[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P'};

    // Loop through each motor label
    for (uint8_t i = 0; i < 16; i++) {
        bool found = false;
        double angle = extractValue(command, motorMap[i], found);
        if (found) {
            // Set the servo position for the specific motor
            setServoPos(i, angle);
        }
    }
}

// Function to extract a value for a specific parameter (e.g., 'A', 'B', etc.)
double extractValue(const String& command, char param, bool& found) {
    int index = command.indexOf(param);
    if (index != -1) {
        int endIndex = command.indexOf(' ', index + 1); // Find end of the value
        if (endIndex == -1) endIndex = command.length(); // Last parameter
        String value = command.substring(index + 1, endIndex);
        found = true;
        return value.toDouble(); // Convert to double
    }
    found = false;
    return 0.0;
}

void jumpToLabel(const String& label) {
  for (int i = 0; i < sizeof(program) / sizeof(program[0]); i++) {
    if (program[i].equals(label)) {
      gCodePointer = i + 1;
      Serial.print("Jumping to '");
      Serial.print(label);
      Serial.print("'at line: ");
      Serial.println(gCodePointer);
      break;
    }
  }
}

//EXAMPLE
//COLOR == 1
void handleIfCondition(const String& parameters) {
    // Increment the IF stack
    ifStack++;

    // Find the operator ("==", "!=") and split the condition and value
    int operatorIndex1 = parameters.indexOf("==");
    int operatorIndex2 = parameters.indexOf("!=");
    bool conditionResult = false;
    
    if (operatorIndex1 == -1 && operatorIndex2 == -1) {
      Serial.print("Invalid condition: ");
      return;
    }

    String condition = parameters.substring(0, max(operatorIndex1, operatorIndex2));
    condition.trim();

    if(operatorIndex1 > 0){
      String value = parameters.substring(operatorIndex1 + 2);
      value.trim();
      int targetValue = value.toInt();
      if(targetValue == color_id){
        conditionResult = true;
      }
    }

    if(operatorIndex2 > 0){
      String value = parameters.substring(operatorIndex2 + 2);
      value.trim();
      int targetValue = value.toInt();
      if(targetValue == color_id){
        conditionResult = true;
      }
    }

    // Log the evaluation result
    if (conditionResult) {
        Serial.print("IF CONDITION '");
        Serial.print(parameters);
        Serial.println("' IS TRUE!");
    } else {
        Serial.print("IF CONDITION '");
        Serial.print(parameters);
        Serial.println("' IS FALSE!");
        skipUntilEnd(); // Skip to the next END block
    }
}

void handleEndCondition() {
  Serial.println("IF END");
  ifStack--;
  if (ifStack < 0) {
    Serial.println("Unmatched END in G-code");
    runGCodeExecution = false;
  }
}

void skipUntilEnd() {
  while (gCodePointer < sizeof(program) / sizeof(program[0])) {
    if (program[gCodePointer].equals("END")) {
      gCodePointer++;
      break;
    }
    gCodePointer++;
  }
}

// This function is called when serial data is received
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read(); // Read the incoming byte
    // If a newline character is detected, set the flag
    if (inChar == '\n') {
      stringComplete = true;
    }else{
      inputString += inChar;  
    }
  }
}

int findClosestColorId(float r, float g, float b) {
  float minDistance = 999999.0;  // Initialize to a large number
  int closestId = 0;  // Default to no match

  for (int i = 0; i < numColors; i++) {
    // Calculate the Euclidean distance between the input color and each color in the list
    float distance = sqrt(pow(r - colorList[i].r, 2) + pow(g - colorList[i].g, 2) + pow(b - colorList[i].b, 2));

    // Check if the distance is the smallest so far and within a reasonable threshold
    if (distance < minDistance && distance < 15) {
      minDistance = distance;
      closestId = i;  // Store the index of the closest color
    }
  }
  
  return closestId;
}

void solveIK3D(float x, float y, float z){
  y = y-50; //move target by 50mm for 2D IK
  float baseRot = atan2(z, x);
  float distToY = sqrt(x * x + z * z);
  solveIK(distToY, -y);

  Serial.print("2D IK space: (");
  Serial.print(distToY);
  Serial.print(", ");
  Serial.print(y);
  Serial.println(")");

  float br = mymap(baseRot, 0, M_PI, 180, 0);
  Serial.print("Base rot: ");
  Serial.print(baseRot);
  Serial.print("rad, ");
  Serial.print(br);
  Serial.println("deg");

  // Print the joint angles
  for (int i = 0; i < numLinks; i++) {
    Serial.print("Angle ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(linkAnglesIK[i]);
    Serial.print("rad, ");
    if(i==0){
      Serial.print(mymap(linkAnglesIK[i], -M_PI, 0, 0, 180)); //conver to deg
    }else{
      Serial.print(mymap(linkAnglesIK[i], -M_PI/2, M_PI/2, 0, 180)); //conver to deg
    }
    Serial.println("deg");
  }

  setServoPos(0, br);
  setServoPos(1, mymap(linkAnglesIK[0], -M_PI, 0, 180, 0));
  setServoPos(2, mymap(linkAnglesIK[1], -M_PI/2, M_PI/2, 180, 0));
  setServoPos(3, mymap(linkAnglesIK[2], -M_PI/2, M_PI/2, 180, 0));
}

// Function to calculate the position of the end of the chain
Position calculatePosition(int iter) {
  float x = 0;
  float y = 0;
  float angleSum = 0;

  for (int i = 0; i < iter && i < numLinks; i++) {
    angleSum += linkAnglesIK[i];
    x += linkLengths[i] * cos(angleSum);
    y += linkLengths[i] * sin(angleSum);
  }
  
  Position result = {x, y};
  return result;
}

// Cyclic Coordinate Descent (CCD) implementation
void solveIK(float targetX, float targetY) {
  linkAnglesIK[0] = -1.57;
  linkAnglesIK[1] = 0;
  linkAnglesIK[2] = 1.57;
  for (int iter = 0; iter < 100; iter++) {
    for (int i = numLinks - 1; i >= 0; i--) {
      Position currentPos = calculatePosition(numLinks); // End effector position
      float distToTarget = sqrt(pow(currentPos.x - targetX, 2) + pow(currentPos.y - targetY, 2));

      // Check if within tolerance
      if (distToTarget < tolerance) {
        Serial.println("Target reached.");
        return;
      }

      Position jointPos = calculatePosition(i); // Position of the current joint

      // Vectors
      float jointToEffectorX = currentPos.x - jointPos.x;
      float jointToEffectorY = currentPos.y - jointPos.y;
      float jointToTargetX = targetX - jointPos.x;
      float jointToTargetY = targetY - jointPos.y;

      // Normalize vectors
      float effectorLength = sqrt(jointToEffectorX * jointToEffectorX + jointToEffectorY * jointToEffectorY);
      float targetLength = sqrt(jointToTargetX * jointToTargetX + jointToTargetY * jointToTargetY);

      jointToEffectorX /= effectorLength;
      jointToEffectorY /= effectorLength;
      jointToTargetX /= targetLength;
      jointToTargetY /= targetLength;

      // Compute angle between vectors
      float dot = jointToEffectorX * jointToTargetX + jointToEffectorY * jointToTargetY;
      float det = jointToEffectorX * jointToTargetY - jointToEffectorY * jointToTargetX;
      float angleChange = atan2(det, dot);

      // Update joint angle
      linkAnglesIK[i] += angleChange;

      // Clamp joint angles within limits
      float minAngle = -M_PI / 2;
      float maxAngle = M_PI / 2;
      if (i == 0) {
        minAngle = -M_PI;
        maxAngle = 0;
      }

      if (linkAnglesIK[i] < minAngle) {
        linkAnglesIK[i] = minAngle;
      }
      if (linkAnglesIK[i] > maxAngle) {
        linkAnglesIK[i] = maxAngle;
      }
    }
  }

  Serial.println("IK did not converge within the iteration limit.");
}

float mymap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
