// CAKE Robot Visualizer
// Fixes: left wheel sign inversion, theta accumulation, waypoint messages, scan after nav
// Based on BenchRobotics project: https://benchrobotics.com/esp-32/esp32-robot-with-odometry-and-waypoint-navigation/

import hypermedia.net.*;
import java.util.ArrayList;
import java.util.concurrent.CopyOnWriteArrayList;

float wheelBase = 0.056; // Distance between wheels in meters
float wheelRadius = 0.033; // Wheel radius in meters
float scalingFactor = 100; // Pixels per meter

UDP udp;
String robotIP = "192.168.1.13"; // <-- Replace with your robot's IP
int robotPort = 12345;

float x, y, theta;
float prevX, prevY;
float prevAngleL = 0;
float prevAngleR = 0;

// Path & waypoints
CopyOnWriteArrayList<PVector> path = new CopyOnWriteArrayList<PVector>();
ArrayList<PVector> waypoints = new ArrayList<PVector>();
int currentWaypointIndex = 0;

// Navigation settings
boolean navEnabled = false;
float angleTolerance = radians(20);
float waypointBoundary = 60; // pixels

int moveSpeed = 55;

boolean isScanning= false;

int scanStartTime = 0;
int scanDuration = 4000; // ms — adjust for your robot speed

ArrayList<PVector> obstacles = new ArrayList<PVector>();
ArrayList<String> logMessages = new ArrayList<String>();

int maxLogLines = 8;




void setup() {
  size(900, 800);
  background(255);

  x = width / 2;
  y = height / 2;
  prevX = x;
  prevY = y;

  udp = new UDP(this, 12345);
  udp.listen(true);
  udp.send("Connected", robotIP, robotPort);

  addLog("Visualizer started. Click to place waypoints.");
  addLog("W = start nav | X = stop | R = reset");
}

void draw() {
  background(30);

  // Draw grid
  drawGrid();

  // Draw obstacles (from ToF mapping)
  for (PVector obs : obstacles) {
    fill(255, 80, 80, 180);
    noStroke();
    ellipse(obs.x, obs.y, 8, 8);
  }

  // Draw path
  stroke(80, 200, 120);
  strokeWeight(1.5);
  noFill();
  beginShape();
  for (PVector p : path) {
    vertex(p.x, p.y);
  }
  
  endShape();
  strokeWeight(1);

  // Draw waypoints
  for (int i = 0; i < waypoints.size(); i++) {
    PVector wp = waypoints.get(i);
    boolean isCurrent = (i == currentWaypointIndex);

    noFill();
    stroke(isCurrent ? color(80, 255, 120) : color(80, 140, 255));
    strokeWeight(1);
    ellipse(wp.x, wp.y, waypointBoundary * 2, waypointBoundary * 2);

    fill(isCurrent ? color(80, 255, 120) : color(80, 140, 255));
    noStroke();
    ellipse(wp.x, wp.y, 10, 10);

    // Label
    fill(200);
    textSize(11);
    textAlign(CENTER);
    text("#" + i, wp.x, wp.y - waypointBoundary - 6);
  }

  // Line from robot to current waypoint
  if (!waypoints.isEmpty() && currentWaypointIndex < waypoints.size()) {
    PVector cw = waypoints.get(currentWaypointIndex);
    stroke(80, 255, 120, 120);
    strokeWeight(1);
    line(x, y, cw.x, cw.y);
  }

  // Draw robot
  drawRobot();

  // Scanning indicator
  if (isScanning) {
    if (millis() - scanStartTime > scanDuration) {
      send_command('s', 0, 0, 0, 0, 0, 0);
      isScanning = false;
      addLog("Scanning completed");
    } else {
      
      // spinning arc indicator
      float progress = (float)(millis() - scanStartTime) / scanDuration;
      stroke(255, 220, 60);
      strokeWeight(3);
      noFill();
      arc(x, y, 50, 50, -HALF_PI, -HALF_PI + TWO_PI * progress);
      strokeWeight(1);
    }
  }

  // Navigation logic
  if (navEnabled && !isScanning) {
    follow_waypoint();
  }

  // HUD overlay
  drawHUD();
}

void receive(byte[] data) {
  String inString = new String(data).trim();
  if (inString == null || inString.length() == 0) return;

  String[] parts = split(inString, ',');

  // Odometry packet: "angleL,angleR"
  if (parts.length == 2) {
    try {
      // Invert left wheel sign
      float angleL = float(parts[0]) * -1;
      float angleR = float(parts[1]);

      if (!Float.isNaN(angleL) && !Float.isNaN(angleR)) {
        float deltaAngleL = angleL - prevAngleL;
        float deltaAngleR = angleR - prevAngleR;
        prevAngleL = angleL;
        prevAngleR = angleR;

        float dL = deltaAngleL * wheelRadius;
        float dR = deltaAngleR * wheelRadius;

        float dTheta = (dR - dL) / wheelBase;

        if (abs(dTheta) > 0.0001 || abs(dL + dR) > 0.0001) {
          float dX = ((dL + dR) / 2.0) * cos(theta + dTheta / 2.0);
          float dY = ((dL + dR) / 2.0) * sin(theta + dTheta / 2.0);

          x += dX * scalingFactor;
          y += dY * scalingFactor;

          // accumulate full dTheta, not half
          theta += dTheta;

          path.add(new PVector(x, y));
          if (path.size() > 2000) path.remove(0);

          // ToF obstacle mapping while scanning
          if (isScanning && parts.length >= 3) {
            // if robot sends "angleL,angleR,distance"
            // handled below in the length==3 branch
          }
        }
      }
    } catch (NumberFormatException e) {
      println("Parse error (odom): " + e.getMessage());
    }
  }

 //Odometry + ToF packet: "angleL,angleR,distanceMM"
  else if (parts.length == 3) {
    try {
      float angleL = float(parts[0]) * -1; // FIX #1
      float angleR = float(parts[1]);
      float distMM = float(parts[2]);

      if (!Float.isNaN(angleL) && !Float.isNaN(angleR)) {
        float deltaAngleL = angleL - prevAngleL;
        float deltaAngleR = angleR - prevAngleR;
        prevAngleL = angleL;
        prevAngleR = angleR;

        float dL = deltaAngleL * wheelRadius;
        float dR = deltaAngleR * wheelRadius;
        float dTheta = (dR - dL) / wheelBase;

        if (abs(dTheta) > 0.0001 || abs(dL + dR) > 0.0001) {
          float dX = ((dL + dR) / 2.0) * cos(theta + dTheta / 2.0);
          float dY = ((dL + dR) / 2.0) * sin(theta + dTheta / 2.0);
          x += dX * scalingFactor;
          y += dY * scalingFactor;
          theta += dTheta; // FIX #2
          path.add(new PVector(x, y));
          if (path.size() > 2000) path.remove(0);
        }

        // Map obstacle from ToF distance
        if (!Float.isNaN(distMM) && distMM > 0 && distMM < 2000) {
          float distPixels = (distMM / 1000.0) * scalingFactor;
          float obsX = x + cos(theta) * distPixels;
          float obsY = y + sin(theta) * distPixels;
          obstacles.add(new PVector(obsX, obsY));
          if (obstacles.size() > 3000) obstacles.remove(0);
        }
      }
    } catch (NumberFormatException e) {
      println("Parse error (odom+tof): " + e.getMessage());
    }
  }
}

void follow_waypoint() {
  // All waypoints done
  if (waypoints.isEmpty() || currentWaypointIndex >= waypoints.size()) {
    send_command('s', 0, 0, 0, 0, 0, 0);
    if (navEnabled) {
      navEnabled = false;
      addLog("All points reached, Starting scanning");
      startScan();
    }
    return;
  }

  PVector cw = waypoints.get(currentWaypointIndex);
  float deltaX = cw.x - x;
  float deltaY = cw.y - y;
  float targetAngle = atan2(deltaY, deltaX);

  float angleDiff = targetAngle - theta;
  // Normalize to [-PI, PI]
  angleDiff = (angleDiff + PI) % TWO_PI - PI;

  float dist = dist(x, y, cw.x, cw.y);

  if (dist < waypointBoundary) {
    send_command('s', 0, 0, 0, 0, 0, 0);
    // FIX #3: Print message when waypoint is reached
    addLog(currentWaypointIndex +
           "  (" + nf(cw.x, 0, 1) + ", " + nf(cw.y, 0, 1) + ")");
    println("Reached waypoint: " + currentWaypointIndex);
    currentWaypointIndex++;
    return;
  }

  if (abs(angleDiff) > angleTolerance) {
    if (angleDiff > 0) {
      send_command('l', moveSpeed, 0, 0, 0, 0, 1);
    } else {
      send_command('r', moveSpeed, 0, 0, 0, 0, 1);
    }
  } else {
    send_command('f', moveSpeed, 0, 0, 0, 0, 1);
  }
}

void startScan() {
  addLog("Scannin: yaw " + (scanDuration / 1000.0) + " s.");
  send_command('l', moveSpeed, 0, 0, 0, 0, 0);
  scanStartTime = millis();
  isScanning = true;
}

void send_command(char direction, int speed, float P, float I, float D, int screen_select, int enable_pid) {
  if (speed < 0 || speed >= 80) return;
  
  int speedL = speed;
  int speedR = speed;
  
  String cmd = direction + "," + speedL + "," + speedR + "," + P + "," + I + "," + D + "," + screen_select + "," + enable_pid;
  udp.send(cmd, robotIP, robotPort);
  println("CMD: " + cmd);
}

void drawRobot() {
  pushMatrix();
  translate(x, y);
  rotate(theta);

  // Body
  fill(50, 120, 255);
  noStroke();
  rectMode(CENTER);
  rect(0, 0, 22, 12, 3);

  // Front marker
  fill(255, 80, 80);
  ellipse(11, 0, 7, 7);

  // Direction arrow
  stroke(255);
  strokeWeight(1.5);
  line(0, 0, 9, 0);
  strokeWeight(1);

  popMatrix();
}

void drawGrid() {
  stroke(50, 50, 50);
  strokeWeight(0.5);
  int gridStep = 50;
  for (int gx = 0; gx < width; gx += gridStep) {
    line(gx, 0, gx, height);
  }
  for (int gy = 0; gy < height; gy += gridStep) {
    line(0, gy, width, gy);
  }
  // Center cross
  stroke(70, 70, 70);
  line(width/2, 0, width/2, height);
  line(0, height/2, width, height/2);
}

void drawHUD() {
  // Status bar at bottom
  fill(0, 0, 0, 180);
  noStroke();
  rect(0, height - 140, width, 140);

  // State info
  fill(200);
  textSize(12);
  textAlign(LEFT);
  String state = navEnabled ? "NAV ON" : (isScanning ? "SCANNING" : "IDLE");
  fill(navEnabled ? color(80, 255, 120) : (isScanning ? color(255, 220, 60) : color(160)));
  text("● " + state, 12, height - 120);

  fill(180);
  text("X:" + nf(x, 0, 1) + "  Y:" + nf(y, 0, 1) + "  θ:" + nf(degrees(theta), 0, 1) + "°", 90, height - 120);
  text("Waypoint: " + currentWaypointIndex + "/" + waypoints.size(), 400, height - 120);
  text("Path pts: " + path.size(), 560, height - 120);
  text("Obstacles: " + obstacles.size(), 680, height - 120);

  // Key hints
  fill(120);
  textSize(10);
  text("[Click] Place waypoint  [W] Start nav  [X] Stop  [R] Reset  [C] Clear waypoints  [S] Manual scan", 12, height - 104);

  // Log lines
  textSize(11);
  int logStart = height - 88;
  for (int i = 0; i < logMessages.size(); i++) {
    float alpha = map(i, 0, logMessages.size() - 1, 80, 220);
    fill(180, 220, 180, alpha);
    text(logMessages.get(i), 12, logStart + i * 14);
  }
}




void addLog(String msg) {
  println(msg);
  logMessages.add(msg);
  if (logMessages.size() > maxLogLines) {
    logMessages.remove(0);
  }
}




void mousePressed() {
  // Don't place waypoint if clicking on HUD area
  if (mouseY > height - 140) return;

  PVector wp = new PVector(mouseX, mouseY);
  waypoints.add(wp);
  addLog("Точка #" + (waypoints.size() - 1) + " добавлена: (" + mouseX + ", " + mouseY + ")");
}





void keyPressed() {
  if (key == 'w' || key == 'W') {
    if (waypoints.isEmpty()) {
      addLog("No waypoints! Clicke on the map.");
    } else {
      navEnabled = true;
      currentWaypointIndex = 0;
      addLog("Navigation is starting. Point: " + waypoints.size());
    }
  }

  if (key == 'x' || key == 'X') {
    navEnabled = false;
    isScanning = false;
    send_command('s', 0, 0, 0, 0, 0, 0);
    addLog("Stoped.");
  }

  if (key == 'r' || key == 'R') {
    // Full reset
    x = width / 2;
    y = height / 2;
    theta = 0;
    prevAngleL = 0;
    prevAngleR = 0;
    path.clear();
    obstacles.clear();
    waypoints.clear();
    currentWaypointIndex = 0;
    navEnabled = false;
    isScanning = false;
    send_command('s', 0, 0, 0, 0, 0, 0);
    addLog("Reset completed");
  }

  if (key == 'c' || key == 'C') {
    waypoints.clear();
    currentWaypointIndex = 0;
    navEnabled = false;
    send_command('s', 0, 0, 0, 0, 0, 0);
    addLog("Waypoints cleared.");
  }

  if (key == 's' || key == 'S') {
    if (!isScanning) {
      addLog("Scannining from me");
      startScan();
    }
  }

  // Debug keys
  if (key == '1') send_command('f', 40, 0, 0, 0, 1, 1);
  if (key == '2') send_command('s', 0,  0, 0, 0, 0, 0);
}
