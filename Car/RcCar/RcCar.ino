#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// Wi-Fi credentials
const char* ssid = "ArucoProject";
const char* password = "ArucoProject1!";

IPAddress local_IP(192, 168, 0, 104);  // desired static IP
IPAddress gateway(192, 168, 0, 1);     // your network gateway
IPAddress subnet(255, 255, 255, 0);    // subnet mask
IPAddress dns(8, 8, 8, 8);             // optional, use your preferred DNS


// Define motor control pins
#define IN1 D1  // GPIO5
#define IN2 D2  // GPIO4
#define IN3 D3  // GPIO0
#define IN4 D4  // GPIO2

ESP8266WebServer server(80);

// Movement functions
void moveForwardShort() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(50);
  stopMotors();
}

void moveBackwardShort() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(50);
  stopMotors();
}

void turnLeftShort() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  delay(50);
  stopMotors();
}

void turnRightShort() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  delay(50);
  stopMotors();
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

}

void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

}



void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Web page UI
String webPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head><title>ESP8266 Car</title></head>
<body>
<h1>ESP8266 Car Control</h1>
<button onclick="location.href='/forward'">Forward</button><br>
<button onclick="location.href='/left'">Left</button>
<button onclick="location.href='/stop'">Stop</button>
<button onclick="location.href='/right'">Right</button><br>
<button onclick="location.href='/backward'">Backward</button>
<button onclick="location.href='/forwardShort'">Forward Short</button><br>
<button onclick="location.href='/leftShort'">Left Short</button>
<button onclick="location.href='/rightShort'">Right Short</button><br>
<button onclick="location.href='/backwardShort'">Backward Short</button>

</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);

  // Set pin modes
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();  // Ensure motors are off at start

  delay(200);/// do not change can only increase
  if (!WiFi.config(local_IP, gateway, subnet, dns)) {
    Serial.println("STA Failed to configure");
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP address: ");
  Serial.println(WiFi.localIP());

  // Define URL handlers
  server.on("/", []() {
    server.send(200, "text/html", webPage);
  });
  server.on("/forwardShort", []() {
    moveForwardShort();
    server.send(200, "text/html", "Moving Forward<br><a href='/'>Back</a>");
  });

  server.on("/backwardShort", []() {
    moveBackwardShort();
    server.send(200, "text/html", "Moving Backward<br><a href='/'>Back</a>");
  });

  server.on("/leftShort", []() {
    turnLeftShort();
    server.send(200, "text/html", "Turning Left<br><a href='/'>Back</a>");
  });

  server.on("/rightShort", []() {
    turnRightShort();
    server.send(200, "text/html", "Turning Right<br><a href='/'>Back</a>");
  });

  server.on("/forward", []() {
    moveForward();
    server.send(200, "text/html", "Moving Forward<br><a href='/'>Back</a>");
  });

  server.on("/backward", []() {
    moveBackward();
    server.send(200, "text/html", "Moving Backward<br><a href='/'>Back</a>");
  });

  server.on("/left", []() {
    turnLeft();
    server.send(200, "text/html", "Turning Left<br><a href='/'>Back</a>");
  });

  server.on("/right", []() {
    turnRight();
    server.send(200, "text/html", "Turning Right<br><a href='/'>Back</a>");
  });

  server.on("/stop", []() {
    stopMotors();
    server.send(200, "text/html", "Stopped<br><a href='/'>Back</a>");
  });

server.on("/joystick", []() {
  if (server.hasArg("x") && server.hasArg("y")) {
    int x = server.arg("x").toInt();  // -100 (left) to 100 (right)
    int y = server.arg("y").toInt();  // -100 (back) to 100 (forward)

    const int threshold = 40;  // small dead zone

    // Stop if near center
    if (abs(x) < threshold && abs(y) < threshold) {
      stopMotors();
      server.send(200, "text/plain", "Stop");
      return;
    }

    // Forward/Backward movement
    if (abs(y) > abs(x)) {
      if (y > 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        
        server.send(200, "text/plain", "Forward");
      } else {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        server.send(200, "text/plain", "Backward");
      }
    }
    // Left/Right turning
    else {
      if (x > 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        server.send(200, "text/plain", "Right");
      } else {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        server.send(200, "text/plain", "Left");
      }
    }

  } else {
    server.send(400, "text/plain", "Missing x or y");
  }
});

  server.begin();
  Serial.println("HTTP server started");
}



void loop() {
  server.handleClient();
}
