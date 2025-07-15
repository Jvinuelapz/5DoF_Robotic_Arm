let runGCodeButton;
let gcodeInput;
let heartBeatDiv;
let heartBeatTime = 0;
let syncCheckbox;

let motors = {
    m0: 90,
    m1: 90,
    m2: 90,
    m3: 90,
    m4: 90
};

let motorsGoal = {
    m0: 90,
    m1: 90,
    m2: 90,
    m3: 90,
    m4: 90
};

let gripperClosed = true;
let myFont;
let timeToUpdate = 0;
let togglePos = false;

//camera controls
let cam;
let isPanning = false;
let lastMouseX, lastMouseY;
let camAngle = 90;
let camHeight = 150;
let zoomLevel = 200;
let zoomGoal = 200;

endEffectorPos = {
    x: 0,
    y: 0,
    z: 0
};

//let server = "https://robot.xpiti.com";
//let server = "http://192.168.1.193";
let server = "";

let filesToLoad = [
    { type: 'model', key: 'armModelSeg0', path: 'arm/0.stl' },
    { type: 'model', key: 'armModelSeg1', path: 'arm/1.stl' },
    { type: 'model', key: 'armModelSeg2', path: 'arm/23.stl' },
    { type: 'model', key: 'armModelSeg3', path: 'arm/23.stl' }, // armModelSeg3 reuses armModelSeg2
    { type: 'model', key: 'armModelSeg4', path: 'arm/4.stl' },
    { type: 'model', key: 'armModelSeg4_open', path: 'arm/4_open.stl' },
    { type: 'image', key: 'armTexture', path: 'arm/arm_texture.png' }
    // { type: 'font', key: 'myFont', path: 'Roboto-Regular.ttf' },
];
  
let assets = {}; // To store loaded assets
let retryLimit = 3;
let filesLoaded = false;

function loadFileSequentially(index = 0, retries = 0) {
    if (index >= filesToLoad.length) {
        console.log("All files loaded successfully!");
        filesLoaded = true;
        // start syncing with the robot (enable the checkbox)
        syncCheckbox.checked(true);
        return;
    }

    let file = filesToLoad[index];
    console.log(`Loading ${file.type} from ${file.path}...`);

    // Choose the correct loading function based on type
    let loadFunction;
    switch (file.type) {
        case 'model':
        loadFunction = loadModel;
        break;
        case 'image':
        loadFunction = loadImage;
        break;
        case 'font':
        loadFunction = loadFont;
        break;
        default:
        console.error(`Unknown type: ${file.type}`);
        return;
    }

    // Load the file
    loadFunction(file.path, 
        (loadedAsset) => {
            // Success
            assets[file.key] = loadedAsset;
            console.log(`${file.key} loaded successfully.`);
            // Handle cases like armModelSeg3 depending on another
            if (file.key === 'armModelSeg3') {
                assets[file.key] = assets['armModelSeg2'];
            }
            loadFileSequentially(index + 1);
        },
        (err) => {
            // Failure
            if (retries < retryLimit) {
                console.warn(`Retrying to load ${file.path}... Attempt ${retries + 1}`);
                loadFileSequentially(index, retries + 1);
            } else {
                console.error(`Failed to load ${file.path} after ${retryLimit} attempts.`);
                loadFileSequentially(index + 1); // Skip to the next file
            }
        }
    );
}
  

function setup() {
    // Select the container element
    const container = document.getElementById("canvas-container");
    
    // Get the container's width and height
    const width = container.offsetWidth;
    const height = container.offsetHeight;
    
    // Create the canvas with the container's dimensions
    canvas = createCanvas(width, height, WEBGL);
    
    // Place the canvas in the container
    canvas.parent("canvas-container");

    cam = createCamera();
    setCamera(cam);

    //------------------------------------------------
    // Link the slider from the HTML to the JavaScript
    runGCodeButton = select('#run-gcode');
    runGCodeButton.mousePressed(processGCodeInput);
    gcodeInput = select('#gcode-input');
    gcodeInput.changed(processGCodeInput);

    //start pause and stop buttons
    let startButton = select('#start-robot');
    startButton.mousePressed(() => {
        sendGCodeCommand("START");
    });
    let pauseButton = select('#pause-robot');
    pauseButton.mousePressed(() => {
        sendGCodeCommand("PAUSE");
    });
    let stopButton = select('#stop-robot');
    stopButton.mousePressed(() => {
        sendGCodeCommand("STOP");
    });

    syncCheckbox = select('#sync-robot');
    heartBeatDiv = select('#heart-beat');

    // armModelSeg0 = loadModel('arm/0.stl');
    // armModelSeg1 = loadModel('arm/1.stl');
    // armModelSeg2 = loadModel('arm/23.stl');
    // armModelSeg3 = armModelSeg2;
    // armModelSeg4 = loadModel('arm/4.stl');
    // armModelSeg4_open = loadModel('arm/4_open.stl');
    // armTexture = loadImage('arm/arm_texture.png');
    // myFont = loadFont('Roboto-Regular.ttf');

    loadFileSequentially();

    //textFont(myFont);

    angleMode(DEGREES);

    print("Setup complete");
}

function windowResized() {
    // Recalculate the dimensions when the window is resized
    const container = document.getElementById("canvas-container");
    const width = container.offsetWidth;
    const height = container.offsetHeight;
    
    // Resize the canvas to fit the container
    resizeCanvas(width, height);
}

function processGCodeInput(){
    let gcode = document.getElementById("gcode-input").value;
    sendGCodeCommand(gcode);
}


// Function to fetch motor positions (non-blocking)
function fetchMotorPositions() {
    // Make a GET request to the server
    fetch(`${server}/api/servo`)
        .then(response => {
            // Check if the response is ok
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            return response.json(); // Parse JSON response
        })
        .then(motorData => {
            // Update the motor variables
            // check if the motorData is not empty
            if(motorData.servo0 == undefined){
                return;
            }
            motorsGoal.m0 = motorData.servo0;
            motorsGoal.m1 = motorData.servo1;
            motorsGoal.m2 = motorData.servo2;
            motorsGoal.m3 = motorData.servo3;
            motorsGoal.m4 = motorData.servo4;
            gripperClosed = motorData.servo4 > 85;

            heartBeatTime = millis();

            //console.log('Motor positions updated:', motors);
        })
        .catch(error => {
            console.error('Error fetching motor positions:', error);
        });
}

// Function to send a G-Code command (non-blocking) with retries
function sendGCodeCommand(cmd) {
    // Define the endpoint
    const endpoint = `${server}/api/gcode`;

    const payload = {
        command: cmd, // The G-Code command string
    };

    console.log("Sending G-Code command:", JSON.stringify(payload));

    // Define the maximum number of retries
    const maxRetries = 3;

    // Function to attempt the request
    function attemptRequest(retriesLeft) {
        // Make a POST request to the server
        fetch(endpoint, {
            method: "POST", // Use the POST HTTP method
            headers: {
                "Content-Type": "application/json", // Specify JSON content type
            },
            body: JSON.stringify(payload), // Convert the payload to JSON
        })
            .then(response => {
                // Check if the response is ok
                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }
                return response.json(); // Parse JSON response
            })
            .then(data => {
                // Handle the server's response
                console.log("G-Code command sent successfully:", data);
                gcodeInput.style('background-color', '#1b1b1b');
            })
            .catch(error => {
                console.error(`Error sending G-Code command: ${error}`);

                // Retry if there are retries left
                if (retriesLeft > 0) {
                    console.log(`Retrying... Attempts left: ${retriesLeft}`);
                    attemptRequest(retriesLeft - 1); // Retry the request
                } else {
                    console.error("Max retries reached. G-Code command failed.");
                    gcodeInput.style('background-color', '#c21717');
                }
            });
    }

    // Start the first attempt with maxRetries
    attemptRequest(maxRetries);
}


function draw() {
    background(20);

    let hbt = millis() - (heartBeatTime + 500);
    hbt = constrain(hbt, 0, 1000);
    let heartBeatColor = map(hbt, 0, 2000, 200, 20);
    heartBeatDiv.style('background-color', `rgb(0, ${heartBeatColor}, 0)`);

    if(timeToUpdate < millis()){
        timeToUpdate = millis() + 1000;
        if(syncCheckbox.checked()){
            fetchMotorPositions();
        }
    }

    if(filesLoaded){
        //lerp motors for smooth visualisation
        motors.m0 = movePosition(motors.m0, motorsGoal.m0, 1);
        motors.m1 = movePosition(motors.m1, motorsGoal.m1, 1);
        motors.m2 = movePosition(motors.m2, motorsGoal.m2, 1);
        motors.m3 = movePosition(motors.m3, motorsGoal.m3, 1);
        
        endEffectorPos = getEndEffectorPosition();

        // Drawing
        push();
        lights();
        //translate(0, 100, 0); // Move the camera back
        // rotateX(-22);
        // rotateY(30*sin(frameCount/2));
        scale(1, -1, 1);
        drawAxes(this, 100);
        drawGrid(this, 500, 10);
        drawGrid(this, 1000, 50);
        drawScene(this, endEffectorPos);
        pop();

        document.getElementById("fps-value").textContent = round(frameRate());
        document.getElementById("eepos-value").textContent = `x: ${round(endEffectorPos.x)}mm, y: ${round(endEffectorPos.y)}mm, z: ${round(endEffectorPos.z)}mm`;
    }else{
        noFill();
        stroke(255);
        strokeWeight(3);

        let rot = millis() / 2;
        let a1 = map(sin(millis()/5), -1, 1, 0, 180) + rot;
        let a2 = map(sin(millis()/6), -1, 1, 180, 360) + rot;
        arc(0, -75, 50, 50, a1, a2);
        noStroke();
        fill(50);
        box(200, 1, 100);
    }


    //lerp zoom
    zoomLevel = lerp(zoomLevel, zoomGoal, 0.05);
    // Handle rotation when no key is pressed
    if (mouseIsPressed && mouseX > 0 && mouseX < width && mouseY > 0 && mouseY < height) {
        let dx = mouseX - lastMouseX;
        let dy = mouseY - lastMouseY;
        camAngle += dx * 0.1;
        camHeight += dy * 0.5;
        camHeight = constrain(camHeight, 0, 500);
    }

    cam.camera(cos(camAngle)*zoomLevel, -camHeight, sin(camAngle)*zoomLevel, 0, -camHeight/2, 0, 0, 1, 0);

    // Update the last mouse position for rotation
    lastMouseX = mouseX;
    lastMouseY = mouseY;
}

function animateRoot(){
    motors.m0 = ( sin(frameCount/3.22) * 90 ) + 90;
    motors.m1 = ( (sin(frameCount/3.9112) * sin(frameCount/1.432)) * 90 ) + 90;
    motors.m2 = ( sin(frameCount/1.55) * 90 ) + 90;
    motors.m3 = ( sin(frameCount/2.32) * 90 ) + 90;
}

function drawGrid(buffer, size, spacing){
    let gridCount = size/spacing/2;

    buffer.stroke(255, 33);
    buffer.strokeWeight(1);
    for(let i = -gridCount; i <= gridCount; i++){
        buffer.line(-spacing * gridCount, 0, i * spacing, spacing * gridCount, 0, i * spacing);
        buffer.line(i * spacing, 0, -spacing * gridCount, i * spacing, 0, spacing * gridCount);
    }
}

function drawScene(buffer, eePos){
    // Draw the robot arm
    drawRobot(buffer, motors);
    

    // Draw the end effector position
    buffer.push();
    buffer.fill(255, 0, 0);
    buffer.translate(eePos.x, eePos.y, eePos.z);
    buffer.sphere(3);
    drawAxes(buffer, 20);
    buffer.pop();

    buffer.push();

    buffer.stroke(0);
    buffer.fill(100, 0, 150);
    buffer.translate(-72, 15, 100);
    buffer.box(30, 30, 50);

    buffer.fill(200, 0, 0);
    buffer.translate(33.5, 0, 0);
    buffer.box(30, 30, 50);

    buffer.fill(255, 150, 0);
    buffer.translate(33.5, 0, 0);
    buffer.box(30, 30, 50);

    buffer.fill(255, 255, 0);
    buffer.translate(33.5, 0, 0);
    buffer.box(30, 30, 50);
    
    buffer.fill(0, 255, 0);
    buffer.translate(33.5, 0, 0);
    buffer.box(30, 30, 50);

    buffer.pop();
}

function drawRobot(buffer, robotMotors) {
    let m0 = map(robotMotors.m0, 0, 180, -90, 90);
    let m1 = map(robotMotors.m1, 180, 0, -90, 90);
    let m2 = map(robotMotors.m2, 180, 0, -90, 90);
    let m3 = map(robotMotors.m3, 180, 0, -90, 90);
    

    let segmentHeight = 50;  // Height of each segment

    //buffer.texture(armTexture);
    buffer.fill(255);
    buffer.noStroke();

    // --- Arm Segment 0 (Base) ---
    buffer.push();

    buffer.model(assets["armModelSeg0"]); // Draw stationary base
    buffer.pop();
  
    // --- Arm Segment 1 ---
    buffer.push();
    buffer.translate(0, segmentHeight, 0);  // Move armModelSeg1 above the base by 46.25 mm
    buffer.rotateY(m0);               // Rotate armModelSeg1 horizontally around Y-axis (base rotation)
    buffer.model(assets["armModelSeg1"]);
    
    // --- Arm Segment 2 ---
    buffer.rotateX(m1);                 // Rotate armModelSeg2 vertically around X-axis
    buffer.model(assets["armModelSeg2"]);
    
    // --- Arm Segment 3 ---
    buffer.translate(0, segmentHeight, 0); // Move armModelSeg3 above armModelSeg2 by 50mm
    buffer.rotateX(m2);                 // Rotate armModelSeg3 vertically around X-axis
    buffer.model(assets["armModelSeg3"]);
    
    // --- Arm Segment 4 (End Effector) ---
    buffer.translate(0, segmentHeight, 0); // Move armModelSeg4 (end effector) above armModelSeg3 by 50mm
    buffer.rotateX(m3);                 // Rotate the end effector
    if(gripperClosed){
        buffer.model(assets["armModelSeg4"]);
    }else{
        buffer.model(assets["armModelSeg4_open"]);
    }

    buffer.fill(0);

    buffer.translate(0, segmentHeight, 0); // Move the end effector above armModelSeg4 by 50mm
    // drawAxes(10);
    // buffer.fill(200);
    // buffer.noStroke();
    // buffer.sphere(3);
  
    buffer.pop();  // End of robot arm drawing

    buffer.fill(0);
}

function drawAxes(buffer, axisLength) {
    buffer.strokeWeight(1.5);
    buffer.stroke(255, 0, 0);
    buffer.line(0, 0, 0, axisLength, 0, 0);
    buffer.stroke(0, 255, 0);
    buffer.line(0, 0, 0, 0, axisLength, 0);
    buffer.stroke(0, 0, 255);
    buffer.line(0, 0, 0, 0, 0, axisLength);

    buffer.noStroke();
    buffer.push();
    buffer.fill(255, 0, 0);
    buffer.translate(axisLength, 0, 0);
    buffer.rotateZ(-90);
    buffer.cone(3, 8);
    buffer.pop();

    buffer.push();
    buffer.fill(0, 255, 0);
    buffer.translate(0, axisLength, 0);
    buffer.cone(3, 8);
    buffer.pop();

    buffer.push();
    buffer.fill(0, 0, 255);
    buffer.translate(0, 0, axisLength);
    buffer.rotateX(90);
    buffer.cone(3, 8);
    buffer.pop();
}

function getEndEffectorPosition() {
    // base is at 0,0,0,
    let L1 = 50; //first segment, rotating horizontally
    let L2 = 50;
    let L3 = 50;
    let L4 = 75; //end effector

    let A1 = 0 - motors.m0;
    let A2 = motors.m1 - 180;
    let A3 = 90 + motors.m1 + motors.m2;
    let A4 = motors.m1 + motors.m2 + motors.m3;

    let d = cos(A2)*L2 + cos(A3)*L3 + cos(A4)*L4;
    let X = cos(A1) * d;
    let Y = L1 + sin(A2)*L2 + sin(A3)*L3 + sin(A4)*L4;
    let Z = sin(A1) * d;

    return {
        x: X,
        y: -Y + 100,
        z: Z
    }
}

function mouseWheel(event) {
    // Zoom in/out when scrolling
    zoomGoal += event.delta * 0.5;  // Adjust zoom speed here
    zoomGoal = constrain(zoomGoal, 50, 500); // Constrain zoom to desired limits
}

function movePosition(pos, goal, speed){
    if(abs(pos - goal) <= speed){
        return goal;
    }

    if(pos < goal){
        return pos + speed;
    }else{
        return pos - speed;
    }
}