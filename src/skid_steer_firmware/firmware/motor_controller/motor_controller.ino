// Motores (PWM, INA, INB)
#define RIGHT_PWM 1
#define RIGHT_INA 2
#define RIGHT_INB 42

#define LEFT_PWM 4
#define LEFT_INA 5
#define LEFT_INB 6

/*Encoders (A, B) Debido a la descripcion                                      x   
del robot skid_steer los encoders deben                                        l     
ir cruzados para compartir el mismo sentido                                    l
de giro al avanzar o tener el giro contrario                             y <---+        regla de la mano derecha
al del otro lado. El sentido antihorario                                        z
sera el negativo y el horario el positivo*/ 
#define RIGHT_FRONT_ENC_A 40
#define RIGHT_FRONT_ENC_B 41
#define RIGHT_MID_ENC_A 38
#define RIGHT_MID_ENC_B 39
#define RIGHT_BACK_ENC_A 36
#define RIGHT_BACK_ENC_B 37

#define LEFT_FRONT_ENC_A 15
#define LEFT_FRONT_ENC_B 7
#define LEFT_MID_ENC_A 17
#define LEFT_MID_ENC_B 16
#define LEFT_BACK_ENC_A 8
#define LEFT_BACK_ENC_B 18

// ===== CANAL DE PWM =====
const int freq = 20000;
const int pwmChannelRight = 0;
const int pwmChannelLeft = 1;
const int resolution = 8;



// ===== VARIABLES DE ENCODERS =====
volatile unsigned int base_right_front_wheel_counter = 0;
volatile unsigned int base_right_mid_wheel_counter = 0;
volatile unsigned int base_right_back_wheel_counter = 0;
volatile unsigned int base_left_front_wheel_counter = 0;
volatile unsigned int base_left_mid_wheel_counter = 0;
volatile unsigned int base_left_back_wheel_counter = 0;
float base_right_front_wheel_vel = 0.0;
float base_right_mid_wheel_vel = 0.0;
float base_right_back_wheel_vel = 0.0;
float right_vel;
float base_left_front_wheel_vel = 0.0;
float base_left_mid_wheel_vel = 0.0;
float base_left_back_wheel_vel = 0.0;
float left_vel;
String base_right_front_wheel_sign = "p";
String base_right_mid_wheel_sign = "p";
String base_right_back_wheel_sign = "p";
String base_left_front_wheel_sign = "p";
String base_left_mid_wheel_sign = "p";
String base_left_back_wheel_sign = "p";

String right_wheels_sign = "p";
String left_wheels_sign = "p";

float right_wheel_vel = 0.0;
float left_wheel_vel = 0.0;

const float delay_coefficient = 50.0e-3;
const int delay_ = 50;

// ===== DECLARACIONES DE FUNCIONES =====
void IRAM_ATTR rightFrontEncoderCallback();
void IRAM_ATTR rightMidEncoderCallback();
void IRAM_ATTR rightBackEncoderCallback();
void IRAM_ATTR leftFrontEncoderCallback();
void IRAM_ATTR leftMidEncoderCallback();
void IRAM_ATTR leftBackEncoderCallback();
void processCommand(String cmd);
void controlMotor(String cmd);
void sendEncoderData();

void setup() {
  // Configura pines de motores
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_INA, OUTPUT);
  pinMode(RIGHT_INB, OUTPUT);
  ledcAttachChannel(RIGHT_PWM, freq, resolution, pwmChannelRight);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_INA, OUTPUT);
  pinMode(LEFT_INB, OUTPUT); 
  ledcAttachChannel(LEFT_PWM, freq, resolution, pwmChannelLeft);
  

  // Configuracion encoders
  pinMode(RIGHT_FRONT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_FRONT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_MID_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_MID_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_BACK_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_BACK_ENC_B, INPUT_PULLUP);
  pinMode(LEFT_FRONT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_FRONT_ENC_B, INPUT_PULLUP);
  pinMode(LEFT_MID_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_MID_ENC_B, INPUT_PULLUP);
  pinMode(LEFT_BACK_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_BACK_ENC_B, INPUT_PULLUP);

  // Configura interrupciones
  attachInterrupt(digitalPinToInterrupt(RIGHT_FRONT_ENC_A), rightFrontEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_MID_ENC_A), rightMidEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_BACK_ENC_A), rightBackEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_FRONT_ENC_A), leftFrontEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_MID_ENC_A), leftMidEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_BACK_ENC_A), leftBackEncoderCallback, RISING);
  
  // Inicialización de salidas
  ledcWrite(RIGHT_PWM, 0);
  digitalWrite(RIGHT_INA, 0);
  digitalWrite(RIGHT_INB, 0);
  ledcWrite(LEFT_PWM, 0);
  digitalWrite(LEFT_INA, 0);
  digitalWrite(LEFT_INB, 0);

  Serial.begin(230400);
}

void loop() {
  static unsigned long last_time = 0;
  // Procesar comandos seriales
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }
  if (millis() - last_time >= delay_) {
  // Calcular velocidades
  base_right_front_wheel_vel = (base_right_front_wheel_counter / delay_coefficient) * (60 / (12 * 75.00)) * 0.10472;
  base_right_mid_wheel_vel = (base_right_mid_wheel_counter / delay_coefficient) * (60 / (12 * 75.00)) * 0.10472;
  base_right_back_wheel_vel = (base_right_back_wheel_counter / delay_coefficient) * (60 / (12 * 75.00)) * 0.10472;
  right_vel = (base_right_front_wheel_vel + base_right_front_wheel_vel + base_right_back_wheel_vel)/3;
  
  base_left_front_wheel_vel = (base_left_front_wheel_counter / delay_coefficient) * (60 / (12 * 75.00)) * 0.10472;
  base_left_mid_wheel_vel = (base_left_mid_wheel_counter / delay_coefficient) * (60 / (12 * 75.00)) * 0.10472;
  base_left_back_wheel_vel = (base_left_back_wheel_counter / delay_coefficient) * (60 / (12 * 75.00)) * 0.10472;
  left_vel = (base_left_front_wheel_vel + base_left_mid_wheel_vel + base_left_back_wheel_vel)/3;
  // Enviar datos
  sendEncoderData();

  // Reiniciar contadores
  base_right_front_wheel_counter = 0;
  base_right_mid_wheel_counter = 0;
  base_right_back_wheel_counter = 0;
  base_left_front_wheel_counter = 0;
  base_left_mid_wheel_counter = 0;
  base_left_back_wheel_counter = 0;

  
  last_time = millis();
  }
}

void IRAM_ATTR rightFrontEncoderCallback() {
  base_right_front_wheel_counter++;
  base_right_front_wheel_sign = digitalRead(RIGHT_FRONT_ENC_B) ? "p" : "n";
}

void IRAM_ATTR rightMidEncoderCallback() {
  base_right_mid_wheel_counter++;
  base_right_mid_wheel_sign = digitalRead(RIGHT_MID_ENC_B) ? "p" : "n";
}

void IRAM_ATTR rightBackEncoderCallback() {
  base_right_back_wheel_counter++;
  base_right_back_wheel_sign = digitalRead(RIGHT_BACK_ENC_B) ? "p" : "n";
}

void IRAM_ATTR leftFrontEncoderCallback() {
  base_left_front_wheel_counter++;
  base_left_front_wheel_sign = digitalRead(LEFT_FRONT_ENC_B) ? "p" : "n";
}

void IRAM_ATTR leftMidEncoderCallback() {
  base_left_mid_wheel_counter++;
  base_left_mid_wheel_sign = digitalRead(LEFT_MID_ENC_B) ? "p" : "n";
}

void IRAM_ATTR leftBackEncoderCallback() {
  base_left_back_wheel_counter++;
  base_left_back_wheel_sign = digitalRead(LEFT_BACK_ENC_B) ? "p" : "n";
}

// ===== FUNCIONES DE CONTROL =====
void processCommand(String cmd) {
  int start = 0;
  int end = cmd.indexOf(',');
  
  while (end != -1) {
    String part = cmd.substring(start, end);
    controlMotor(part);
    start = end + 1;
    end = cmd.indexOf(',', start);
  }
}
// cadena proveniente de ros: rn04.63,lp04.63, Velociddad en radianes
void controlMotor(String cmd) {
  if (cmd.length() < 2) return;
  
  char side = cmd[0]; // 'r' o 'l'
  char dir = cmd[1];  // 'p' o 'n'
  float speed_rads = cmd.substring(2).toFloat(); // Velocidad en rad/s
  
  // Parámetros del motor
  const float max_speed_rads = 16.61f; // 130 RPM con reducción 75:1
  const int min_duty_cycle = 100; // Valor mínimo para vencer fricción (ajustar)
  
  // Convertir rad/s a duty cycle (0-255)
  int duty_cycle;
  if (fabs(speed_rads) < 0.001) {
    duty_cycle = 0; // Detener
  } else {
    // Normalizar y escalar a 255
    float ratio = fabs(speed_rads) / max_speed_rads;
    duty_cycle = (int)(ratio * 255);
    
    // Aplicar mínimo y máximo
    duty_cycle = max(duty_cycle, min_duty_cycle);
    duty_cycle = constrain(duty_cycle, 0, 255);
  }

  
  if (side == 'r') {
    ledcWrite(RIGHT_PWM, duty_cycle);
    digitalWrite(RIGHT_INA, dir == 'p' ? HIGH : LOW);
    digitalWrite(RIGHT_INB, dir == 'p' ? LOW : HIGH);
  }
  else if (side == 'l') {
    ledcWrite(LEFT_PWM, duty_cycle);
    digitalWrite(LEFT_INA, dir == 'p' ? HIGH : LOW);
    digitalWrite(LEFT_INB, dir == 'p' ? LOW : HIGH);
  }
}

void sendEncoderData() {
  String data;
  
  // Ruedas derechas
  data += "rf" + base_right_front_wheel_sign + String(base_right_front_wheel_vel, 2) + ",";
  data += "rm" + base_right_mid_wheel_sign + String(base_right_mid_wheel_vel, 2) + ",";
  data += "rb" + base_right_back_wheel_sign + String(base_right_back_wheel_vel, 2) + ",";
  
  // Ruedas izquierdas
  data += "lf" + base_left_front_wheel_sign + String(base_left_front_wheel_vel, 2) + ",";
  data += "lm" + base_left_mid_wheel_sign + String(base_left_mid_wheel_vel, 2) + ",";
  data += "lb" + base_left_back_wheel_sign + String(base_left_back_wheel_vel, 2) + ",";
  
  Serial.println(data);
}
