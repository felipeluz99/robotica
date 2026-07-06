#include <Arduino.h>
#include <ArduinoJson.h>
#include <math.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <string.h>

// =====================================================
// CONFIGURAÇÕES DE DEBUG
// =====================================================
//
// ATENÇÃO: no modo USB, o debug compartilha a MESMA porta
// dos comandos JSON. Se ligar DEBUG_ATIVO com a Raspberry
// conectada, os prints de debug vão se misturar com os
// comandos/telemetria no mesmo canal. Use DEBUG_ATIVO = true
// apenas em bancada (Serial Monitor, sem a Pi), ou faça a Pi
// ignorar toda linha que não comece com '{'.

// Coloque true apenas para diagnóstico em bancada.
const bool DEBUG_ATIVO = false;

// Mostra cada JSON completo recebido.
const bool DEBUG_JSON_RECEBIDO = false;

// Mostra cada byte recebido pela UART.
// Deixe false normalmente, pois gera muitos prints.
const bool DEBUG_BYTES_UART = false;

// Mostra periodicamente RPM, setpoints, PWM, erros e pulsos.
const bool DEBUG_CONTROLE = true;

// Intervalo entre relatórios periódicos.
const unsigned long INTERVALO_DEBUG_MS = 300;

unsigned long instanteDebugAnterior = 0;

// Macro para prints formatados somente pela USB.
#define DEBUGF(...)                         \
  do {                                      \
    if (DEBUG_ATIVO) {                      \
      Serial.printf(__VA_ARGS__);           \
    }                                       \
  } while (0)

#define DEBUG_PRINT(texto)                   \
  do {                                      \
    if (DEBUG_ATIVO) {                      \
      Serial.print(texto);                  \
    }                                       \
  } while (0)

#define DEBUG_PRINTLN(texto)                 \
  do {                                      \
    if (DEBUG_ATIVO) {                      \
      Serial.println(texto);                \
    }                                       \
  } while (0)

// =====================================================
// COMUNICAÇÃO SERIAL
// =====================================================

// USB nativo do ESP32 (conector micro-USB).
// É por aqui que entram os comandos JSON da Raspberry Pi
// e saem o ack e a telemetria. É a MESMA porta usada para
// gravar o sketch e (opcionalmente) para debug.
const uint32_t BAUD_USB = 115200;

// =====================================================
// LED RGB NATIVO OPCIONAL
// =====================================================
// ESP32-WROOM comum normalmente não possui LED RGB nativo.
// Se sua placa for uma ESP32-S3/C3 com LED RGB endereçável nativo,
// coloque USAR_LED_RGB_NATIVO em 1 e ajuste o pino abaixo.
// Em muitas ESP32-S3 DevKit, o LED RGB nativo fica no GPIO48.

#define USAR_LED_RGB_NATIVO 0
#define PINO_LED_RGB_NATIVO 48


// =====================================================
// GIROSCÓPIO MPU6050 — BIBLIOTECA ADAFRUIT
// =====================================================
// Requer, no Arduino IDE:
//   1) Adafruit MPU6050
//   2) Adafruit Unified Sensor
//   3) Adafruit BusIO
//
// GPIO22 já é usado pelo motor 3 neste projeto. Por isso o I2C foi
// deslocado para GPIO16 (SDA) e GPIO17 (SCL), ambos livres no mapa atual.
// O GY-521/MPU6050 deve ser alimentado em 3,3 V e compartilhar o GND do ESP32.

const int PINO_I2C_SDA = 21;
const int PINO_I2C_SCL = 4;
const uint8_t ENDERECO_MPU6050 = 0x68;

Adafruit_MPU6050 mpu;

const unsigned long INTERVALO_GIROSCOPIO_US = 5000;  // 200 Hz
const unsigned long INTERVALO_PRINT_ORIENTACAO_MS = 500;
const bool PRINT_ORIENTACAO = true;

bool giroscopioDisponivel = false;
float biasGiroXDps = 0.0f;
float biasGiroYDps = 0.0f;
float biasGiroZDps = 0.0f;
float velocidadeGiroXDps = 0.0f;
float velocidadeGiroYDps = 0.0f;
float velocidadeGiroZDps = 0.0f;
float anguloRollGraus = 0.0f;
float anguloPitchGraus = 0.0f;
float anguloYawGraus = 0.0f;
float temperaturaMpuC = 0.0f;
unsigned long ultimoGiroscopioUs = 0;
unsigned long ultimoPrintOrientacaoMs = 0;

float normalizarAnguloGraus(float angulo) {
  while (angulo > 180.0f) angulo -= 360.0f;
  while (angulo <= -180.0f) angulo += 360.0f;
  return angulo;
}

void zerarYawGiroscopio() {
  anguloYawGraus = 0.0f;
  ultimoGiroscopioUs = micros();
}

void calcularRollPitchPorAcelerometro(const sensors_event_t &aceleracao) {
  float ax = aceleracao.acceleration.x;
  float ay = aceleracao.acceleration.y;
  float az = aceleracao.acceleration.z;

  // Convenção simples para debug:
  // roll  = inclinação lateral
  // pitch = inclinação para frente/trás
  // yaw   = giro horizontal integrado pelo giroscópio Z
  anguloRollGraus = atan2f(ay, az) * 180.0f / PI;
  anguloPitchGraus = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
}

void imprimirOrientacaoSeNecessario(unsigned long agoraMs) {
  if (!PRINT_ORIENTACAO) {
    return;
  }

  if (agoraMs - ultimoPrintOrientacaoMs < INTERVALO_PRINT_ORIENTACAO_MS) {
    return;
  }

  ultimoPrintOrientacaoMs = agoraMs;

  // Linha não-JSON de propósito: o app.py ignora linhas que não começam com '{'.
  Serial.print("[ORIENTACAO] roll=");
  Serial.print(anguloRollGraus, 1);
  Serial.print(" deg | pitch=");
  Serial.print(anguloPitchGraus, 1);
  Serial.print(" deg | yaw=");
  Serial.print(anguloYawGraus, 1);
  Serial.print(" deg | gx=");
  Serial.print(velocidadeGiroXDps, 2);
  Serial.print(" dps | gy=");
  Serial.print(velocidadeGiroYDps, 2);
  Serial.print(" dps | gz=");
  Serial.print(velocidadeGiroZDps, 2);
  Serial.println(" dps");
}

bool inicializarGiroscopio() {
  Wire.begin(PINO_I2C_SDA, PINO_I2C_SCL);
  Wire.setClock(400000);
  delay(50);

  if (!mpu.begin(ENDERECO_MPU6050, &Wire)) {
    Serial.println("{\"gyro_error\":\"mpu6050_sem_resposta_adafruit\"}");
    return false;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  // Calibração do bias do giroscópio.
  // O robô deve ficar completamente parado durante esta etapa.
  const int AMOSTRAS_CALIBRACAO = 800;
  double somaX = 0.0;
  double somaY = 0.0;
  double somaZ = 0.0;
  int amostrasValidas = 0;

  for (int i = 0; i < AMOSTRAS_CALIBRACAO; i++) {
    sensors_event_t aceleracao;
    sensors_event_t giro;
    sensors_event_t temperatura;

    if (mpu.getEvent(&aceleracao, &giro, &temperatura)) {
      somaX += giro.gyro.x * 180.0 / PI;
      somaY += giro.gyro.y * 180.0 / PI;
      somaZ += giro.gyro.z * 180.0 / PI;
      amostrasValidas++;
      calcularRollPitchPorAcelerometro(aceleracao);
      temperaturaMpuC = temperatura.temperature;
    }
    delay(2);
  }

  if (amostrasValidas < AMOSTRAS_CALIBRACAO * 0.8f) {
    Serial.println("{\"gyro_error\":\"falha_na_calibracao_adafruit\"}");
    return false;
  }

  biasGiroXDps = static_cast<float>(somaX / amostrasValidas);
  biasGiroYDps = static_cast<float>(somaY / amostrasValidas);
  biasGiroZDps = static_cast<float>(somaZ / amostrasValidas);

  velocidadeGiroXDps = 0.0f;
  velocidadeGiroYDps = 0.0f;
  velocidadeGiroZDps = 0.0f;
  anguloYawGraus = 0.0f;
  ultimoGiroscopioUs = micros();
  ultimoPrintOrientacaoMs = millis();

  Serial.print("{\"gyro\":\"inicializado_adafruit\",\"bias_x_dps\":");
  Serial.print(biasGiroXDps, 4);
  Serial.print(",\"bias_y_dps\":");
  Serial.print(biasGiroYDps, 4);
  Serial.print(",\"bias_z_dps\":");
  Serial.print(biasGiroZDps, 4);
  Serial.println("}");
  return true;
}

void atualizarGiroscopio() {
  if (!giroscopioDisponivel) {
    return;
  }

  unsigned long agoraUs = micros();
  unsigned long intervaloUs = agoraUs - ultimoGiroscopioUs;

  if (intervaloUs < INTERVALO_GIROSCOPIO_US) {
    return;
  }

  ultimoGiroscopioUs = agoraUs;
  float dt = intervaloUs / 1000000.0f;

  // Ignora intervalos absurdos para não integrar saltos após bloqueios.
  if (dt <= 0.0f || dt > 0.1f) {
    return;
  }

  sensors_event_t aceleracao;
  sensors_event_t giro;
  sensors_event_t temperatura;

  if (!mpu.getEvent(&aceleracao, &giro, &temperatura)) {
    return;
  }

  velocidadeGiroXDps = giro.gyro.x * 180.0f / PI - biasGiroXDps;
  velocidadeGiroYDps = giro.gyro.y * 180.0f / PI - biasGiroYDps;
  velocidadeGiroZDps = giro.gyro.z * 180.0f / PI - biasGiroZDps;

  // Pequenas zonas mortas reduzem deriva quando o robô está parado.
  if (fabsf(velocidadeGiroXDps) < 0.35f) velocidadeGiroXDps = 0.0f;
  if (fabsf(velocidadeGiroYDps) < 0.35f) velocidadeGiroYDps = 0.0f;
  if (fabsf(velocidadeGiroZDps) < 0.35f) velocidadeGiroZDps = 0.0f;

  calcularRollPitchPorAcelerometro(aceleracao);
  temperaturaMpuC = temperatura.temperature;

  // O yaw não vem do acelerômetro; ele é integrado do giro Z.
  // Ele serve ao app.py para fechar giros de 90°/180° no AUTO.
  anguloYawGraus = normalizarAnguloGraus(
    anguloYawGraus + velocidadeGiroZDps * dt
  );

  imprimirOrientacaoSeNecessario(millis());
}

// Tempo sem comandos antes da parada automática.
const unsigned long TIMEOUT_COMANDO_MS = 1000;

// =====================================================
// ALIMENTAÇÃO DOS ENCODERS POR GPIO
// =====================================================

// GPIO32 mantido em HIGH.
const int pinoVccEncoder = 32;

// GPIO33 mantido em LOW.
const int pinoGndEncoder = 33;

const unsigned long TEMPO_ESTABILIZACAO_ENCODER_MS = 100;

// =====================================================
// MOTOR 1
// =====================================================

const int pinDirecaoA_M1 = 12;
const int pinDirecaoB_M1 = 14;
const int pinoEnable_M1  = 13;

const int enc1_A = 34;
const int enc1_B = 35;

ESP32Encoder encoderM1;
long oldPosM1 = 0;

float rotacoesM1 = 0.0f;
float setpointM1 = 0.0f;

int pwmM1 = 0;
int sentidoAnteriorM1 = 0;

// =====================================================
// MOTOR 2
// =====================================================

const int pinDirecaoA_M2 = 27;
const int pinDirecaoB_M2 = 26;
const int pinoEnable_M2  = 25; 

// GPIO36 aparece como VP.
// GPIO39 aparece como VN.
const int enc2_A = 39;
const int enc2_B = 36;

ESP32Encoder encoderM2;
long oldPosM2 = 0;

float rotacoesM2 = 0.0f;
float setpointM2 = 0.0f;

int pwmM2 = 0;
int sentidoAnteriorM2 = 0;

// =====================================================
// MOTOR 3
// =====================================================

const int pinDirecaoA_M3 = 23;   // in5
const int pinDirecaoB_M3 = 22;   // in6
const int pinoEnable_M3  = 5;    // enable3 (PWM)

const int enc3_A = 18;
const int enc3_B = 19;

ESP32Encoder encoderM3;
long oldPosM3 = 0;

float rotacoesM3 = 0.0f;
float setpointM3 = 0.0f;

int pwmM3 = 0;
int sentidoAnteriorM3 = 0;

// Últimos deltas para diagnóstico.
long ultimoDeltaPulsosM1 = 0;
long ultimoDeltaPulsosM2 = 0;
long ultimoDeltaPulsosM3 = 0;

// =====================================================
// CONFIGURAÇÕES DOS MOTORES E ENCODERS
// =====================================================

// Ajustar conforme a resolução real do encoder.
const float PULSOS_POR_REVOLUCAO = 720.0f;

// RPM para comando máximo.
const float RPM_MAXIMO = 130f;

// Inverta caso o motor físico esteja no sentido contrário.
const bool INVERTER_MOTOR_1 = false;
const bool INVERTER_MOTOR_2 = false;
const bool INVERTER_MOTOR_3 = false;

// Inverta caso o encoder informe sinal contrário.
const bool INVERTER_ENCODER_1 = false;
const bool INVERTER_ENCODER_2 = false;
const bool INVERTER_ENCODER_3 = false;

// =====================================================
// CONTROLE MANUAL DO GARFO / MOTOR 3
// =====================================================
// O terceiro joystick da interface envia fork entre -1.0 e +1.0.
// O valor é convertido diretamente em PWM para o motor do garfo,
// seguindo a mesma lógica manual dos outros comandos do robô.

const long GARFO_PULSOS_CURSO_TOTAL = 7200;  // mantido para telemetria de posição
const int PWM_MIN_GARFO = 60;
const int PWM_MAX_GARFO = 180;

float forkCommand = 0.0f;
float forkPositionAtual = 0.0f;
long posicaoGarfoPulsos = 0;
long alvoGarfoPulsos = 0;
long erroGarfoPulsos = 0;

// Estado lógico recebido da Raspberry/interface. Usado na telemetria
// e no LED RGB nativo, caso exista.
char modoEmpilhadeira[16] = "MANUAL";
char estadoEmpilhadeira[24] = "IDLE";
char faseEmpilhadeira[32] = "PARADO";
bool timeoutAtivo = false;

// PWM acima do qual a ausência de pulsos gera alerta.
const int PWM_ALERTA_ENCODER = 70;

// =====================================================
// INTERVALOS
// =====================================================

const unsigned long INTERVALO_CONTROLE_MS = 100;
const unsigned long INTERVALO_TELEMETRIA_MS = 500;

unsigned long instanteControleAnterior = 0;
unsigned long instanteTelemetriaAnterior = 0;
unsigned long ultimoComandoRecebido = 0;

// =====================================================
// COMANDOS
// =====================================================

// Faixa:
// throttle: -1.0 até +1.0
// yaw:      -1.0 até +1.0

float throttle = 0.0f;
float yaw = 0.0f;

bool comandoRecebido = false;

// Contadores de diagnóstico.
unsigned long totalJsonValidos = 0;
unsigned long totalJsonInvalidos = 0;
unsigned long totalBytesRecebidos = 0;
unsigned long totalTimeouts = 0;

// =====================================================
// PID
// =====================================================

struct ControladorPID {
  float kp;
  float ki;
  float kd;

  float integral;
  float medidaAnterior;

  bool iniciado;
};

ControladorPID pidM1 = {
  2.0f,
  0.30f,
  0.00f,
  0.0f,
  0.0f,
  false
};

ControladorPID pidM2 = {
  2.0f,
  0.30f,
  0.00f,
  0.0f,
  0.0f,
  false
};

const float LIMITE_INTEGRAL = 200.0f;

// =====================================================
// INTERRUPÇÕES DOS ENCODERS
// =====================================================

// Não coloque Serial.print dentro das interrupções.


// =====================================================
// MENSAGENS PARA RASPBERRY
// =====================================================

// Envia resposta para o Raspberry e replica na USB.
void enviarMensagem(const char *mensagem) {
  Serial.println(mensagem);

  if (DEBUG_ATIVO) {
    Serial.print("[TX RASPBERRY] ");
    Serial.println(mensagem);
  }
}

// =====================================================
// ALIMENTAÇÃO DOS ENCODERS
// =====================================================

void ligarAlimentacaoEncoders() {
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("[ENCODER POWER] Iniciando alimentação...");

  // Primeiro estabelece o GND relativo.
  digitalWrite(pinoGndEncoder, LOW);
  pinMode(pinoGndEncoder, OUTPUT);
  digitalWrite(pinoGndEncoder, LOW);

  DEBUGF(
    "[ENCODER POWER] GPIO%d configurado como GND relativo.\n",
    pinoGndEncoder
  );

  // Mantém VCC desligado inicialmente.
  digitalWrite(pinoVccEncoder, LOW);
  pinMode(pinoVccEncoder, OUTPUT);
  digitalWrite(pinoVccEncoder, LOW);

  DEBUGF(
    "[ENCODER POWER] GPIO%d configurado como saída inicialmente LOW.\n",
    pinoVccEncoder
  );

  delay(10);

  // Energiza os encoders.
  digitalWrite(pinoVccEncoder, HIGH);

  DEBUGF(
    "[ENCODER POWER] GPIO%d colocado em HIGH.\n",
    pinoVccEncoder
  );

  delay(TEMPO_ESTABILIZACAO_ENCODER_MS);

  int estadoVcc = digitalRead(pinoVccEncoder);
  int estadoGnd = digitalRead(pinoGndEncoder);

  DEBUGF(
    "[ENCODER POWER] Leitura GPIO%d VCC = %s\n",
    pinoVccEncoder,
    estadoVcc == HIGH ? "HIGH" : "LOW"
  );

  DEBUGF(
    "[ENCODER POWER] Leitura GPIO%d GND = %s\n",
    pinoGndEncoder,
    estadoGnd == LOW ? "LOW" : "HIGH"
  );

  if (estadoVcc != HIGH) {
    DEBUG_PRINTLN(
      "[ERRO] GPIO de alimentação do encoder não permaneceu HIGH."
    );
  }

  if (estadoGnd != LOW) {
    DEBUG_PRINTLN(
      "[ERRO] GPIO de GND relativo do encoder não permaneceu LOW."
    );
  }

  DEBUG_PRINTLN("[ENCODER POWER] Alimentação inicializada.");
}

void desligarAlimentacaoEncoders() {
  DEBUG_PRINTLN("[ENCODER POWER] Desligando alimentação.");

  digitalWrite(pinoVccEncoder, LOW);
  digitalWrite(pinoGndEncoder, LOW);
}

// =====================================================
// DEBUG VISUAL POR LED RGB NATIVO OPCIONAL
// =====================================================

void copiarTextoSeguro(char *destino, size_t tamanho, const char *origem) {
  if (destino == nullptr || tamanho == 0 || origem == nullptr) {
    return;
  }

  strncpy(destino, origem, tamanho - 1);
  destino[tamanho - 1] = '\0';
}

void escreverLedRgb(uint8_t r, uint8_t g, uint8_t b) {
#if USAR_LED_RGB_NATIVO
  rgbLedWrite(PINO_LED_RGB_NATIVO, r, g, b);
#else
  (void)r;
  (void)g;
  (void)b;
#endif
}

void atualizarLedEstado() {
  if (timeoutAtivo) {
    escreverLedRgb(255, 0, 0);        // vermelho: timeout/erro
    return;
  }

  if (strcmp(estadoEmpilhadeira, "ENCONTRADO") == 0) {
    escreverLedRgb(0, 255, 0);        // verde: alvo encontrado
    return;
  }

  if (strcmp(faseEmpilhadeira, "RECUPERAR_TAG") == 0) {
    escreverLedRgb(180, 0, 255);      // roxo: recuperação da tag
    return;
  }

  if (strcmp(modoEmpilhadeira, "AUTO") == 0) {
    escreverLedRgb(255, 180, 0);      // amarelo/laranja: busca automática
    return;
  }

  if (strcmp(modoEmpilhadeira, "MANUAL") == 0) {
    escreverLedRgb(0, 0, 255);        // azul: manual
    return;
  }

  escreverLedRgb(255, 255, 255);      // branco: estado desconhecido
}

// =====================================================
// PID
// =====================================================

#define PID_ESCOLHIDO(indicePid) ((indicePid) == 2 ? pidM2 : pidM1)

void resetarPID(int indicePid) {
  ControladorPID &pid = PID_ESCOLHIDO(indicePid);
  pid.integral = 0.0f;
  pid.medidaAnterior = 0.0f;
  pid.iniciado = false;
}

float calcularPID(
  int indicePid,
  float setpoint,
  float medida,
  float dt
) {
  ControladorPID &pid = PID_ESCOLHIDO(indicePid);

  if (setpoint < 0.5f || dt <= 0.0f) {
    resetarPID(indicePid);
    return 0.0f;
  }

  float erro = setpoint - medida;
  float derivada = 0.0f;

  if (pid.iniciado) {
    derivada =
      -(medida - pid.medidaAnterior) / dt;
  }

  float integralCandidata =
    pid.integral + erro * dt;

  integralCandidata = constrain(
    integralCandidata,
    -LIMITE_INTEGRAL,
    LIMITE_INTEGRAL
  );

  float saidaCandidata =
    pid.kp * erro +
    pid.ki * integralCandidata +
    pid.kd * derivada;

  bool saturadoMaximo =
    saidaCandidata > 255.0f;

  bool saturadoMinimo =
    saidaCandidata < 0.0f;

  // Anti-windup.
  if (
    (!saturadoMaximo && !saturadoMinimo) ||
    (saturadoMaximo && erro < 0.0f) ||
    (saturadoMinimo && erro > 0.0f)
  ) {
    pid.integral = integralCandidata;
  }

  float saida =
    pid.kp * erro +
    pid.ki * pid.integral +
    pid.kd * derivada;

  pid.medidaAnterior = medida;
  pid.iniciado = true;

  return constrain(saida, 0.0f, 255.0f);
}

// =====================================================
// ACIONAMENTO DOS MOTORES
// =====================================================

void acionarMotor(
  int pinDirecaoA,
  int pinDirecaoB,
  int pinoEnable,
  float comandoPWM,
  bool inverterMotor
) {
  if (inverterMotor) {
    comandoPWM = -comandoPWM;
  }

  int pwm = constrain(
    static_cast<int>(roundf(fabsf(comandoPWM))),
    0,
    255
  );

  if (pwm == 0) {
    analogWrite(pinoEnable, 0);

    digitalWrite(pinDirecaoA, LOW);
    digitalWrite(pinDirecaoB, LOW);

    return;
  }

  if (comandoPWM > 0.0f) {
    digitalWrite(pinDirecaoA, LOW);
    digitalWrite(pinDirecaoB, HIGH);
  } else {
    digitalWrite(pinDirecaoA, HIGH);
    digitalWrite(pinDirecaoB, LOW);
  }

  analogWrite(pinoEnable, pwm);
}

// =====================================================
// CONTROLE INDIVIDUAL
// =====================================================

float controlarMotor(
  const char *nomeMotor,
  int indicePid,
  float setpoint,
  float rpmMedido,
  float dt,
  int &sentidoAnterior
) {
  int sentidoAtual = 0;

  if (setpoint > 0.5f) {
    sentidoAtual = 1;
  } else if (setpoint < -0.5f) {
    sentidoAtual = -1;
  }

  if (sentidoAtual == 0) {
    if (sentidoAnterior != 0) {
      DEBUGF(
        "[%s] Setpoint zerado. Reiniciando PID.\n",
        nomeMotor
      );
    }

    resetarPID(indicePid);
    sentidoAnterior = 0;

    return 0.0f;
  }

  if (
    sentidoAnterior != 0 &&
    sentidoAtual != sentidoAnterior
  ) {
    DEBUGF(
      "[%s] Mudança de direção detectada: %d -> %d. "
      "Reiniciando PID.\n",
      nomeMotor,
      sentidoAnterior,
      sentidoAtual
    );

    resetarPID(indicePid);
  }

  sentidoAnterior = sentidoAtual;

  float setpointAbsoluto =
    fabsf(setpoint);

  float rpmNoSentidoSolicitado =
    rpmMedido * static_cast<float>(sentidoAtual);

  float intensidadePWM = calcularPID(
    indicePid,
    setpointAbsoluto,
    rpmNoSentidoSolicitado,
    dt
  );

  return intensidadePWM *
         static_cast<float>(sentidoAtual);
}

// =====================================================
// SETPOINTS
// =====================================================

void atualizarSetpoints() {
  float throttleOriginal = throttle;
  float yawOriginal = yaw;

  throttle = constrain(
    throttle,
    -1.0f,
    1.0f
  );

  yaw = constrain(
    yaw,
    -1.0f,
    1.0f
  );
/*
  if (
    throttle != throttleOriginal ||
    yaw != yawOriginal
  ) {
//    DEBUGF(
      "[COMANDO] Valores limitados: throttle %.3f -> %.3f, "
      "yaw %.3f -> %.3f\n",
      throttleOriginal,
      throttle,
      yawOriginal,
      yaw
    );
  }
*/
  float comandoM1 = throttle + yaw;
  float comandoM2 = throttle - yaw;

  float maiorComando = fmaxf(
    1.0f,
    fmaxf(
      fabsf(comandoM1),
      fabsf(comandoM2)
    )
  );

  if (maiorComando > 1.0f) {
    DEBUGF(
      "[COMANDO] Normalização diferencial aplicada. "
      "Fator = %.3f\n",
      maiorComando
    );
  }

  comandoM1 /= maiorComando;
  comandoM2 /= maiorComando;

  setpointM1 =
    comandoM1 * RPM_MAXIMO;

  setpointM2 =
    comandoM2 * RPM_MAXIMO;

  DEBUGF(
    "[COMANDO] throttle=%.3f yaw=%.3f | "
    "setpoint M1=%.2f RPM | setpoint M2=%.2f RPM\n",
    throttle,
    yaw,
    setpointM1,
    setpointM2
  );
}

// =====================================================
// CONTROLE DE POSIÇÃO DO GARFO / MOTOR 3
// =====================================================

float calcularComandoGarfo() {
  const long curso = GARFO_PULSOS_CURSO_TOTAL <= 0
    ? 1
    : GARFO_PULSOS_CURSO_TOTAL;

  forkPositionAtual = constrain(
    static_cast<float>(posicaoGarfoPulsos) / static_cast<float>(curso),
    0.0f,
    1.0f
  );

  alvoGarfoPulsos = 0;
  erroGarfoPulsos = 0;
  setpointM3 = forkCommand * 100.0f;

  if (timeoutAtivo) {
    return 0.0f;
  }

  float comando = constrain(forkCommand, -1.0f, 1.0f) * static_cast<float>(PWM_MAX_GARFO);

  if (fabsf(comando) > 0.0f && fabsf(comando) < PWM_MIN_GARFO) {
    comando = copysignf(static_cast<float>(PWM_MIN_GARFO), comando);
  }

  return comando;
}

// =====================================================
// PROCESSAMENTO DO JSON
// =====================================================

bool processarJson(const char *mensagem) {
  if (DEBUG_ATIVO && DEBUG_JSON_RECEBIDO) {
    Serial.print("[JSON RECEBIDO] ");
    Serial.println(mensagem);
  }

#if ARDUINOJSON_VERSION_MAJOR >= 7
  JsonDocument documento;
#else
  StaticJsonDocument<384> documento;
#endif

  DeserializationError erroJson =
    deserializeJson(documento, mensagem);

  if (erroJson) {
    totalJsonInvalidos++;

    DEBUGF(
      "[ERRO JSON] Falha na desserialização: %s\n",
      erroJson.c_str()
    );

    DEBUGF(
      "[ERRO JSON] Mensagem problemática: %s\n",
      mensagem
    );

    Serial.print(
      "{\"erro\":\"json_invalido\",\"detalhe\":\""
    );

    Serial.print(erroJson.c_str());
    Serial.println("\"}");

    return false;
  }

  if (!documento["reset_yaw"].isNull() && documento["reset_yaw"].as<bool>()) {
    zerarYawGiroscopio();
  }

  if (!documento["reset_fork_zero"].isNull() && documento["reset_fork_zero"].as<bool>()) {
    encoderM3.clearCount();
    oldPosM3 = 0;
    posicaoGarfoPulsos = 0;
    alvoGarfoPulsos = 0;
    erroGarfoPulsos = 0;
    forkPositionAtual = 0.0f;
    forkCommand = 0.0f;
  }

  const char *modoJson = documento["mode"] | nullptr;
  if (modoJson != nullptr) {
    copiarTextoSeguro(modoEmpilhadeira, sizeof(modoEmpilhadeira), modoJson);
  }

  const char *estadoJson = documento["state"] | nullptr;
  if (estadoJson != nullptr) {
    copiarTextoSeguro(estadoEmpilhadeira, sizeof(estadoEmpilhadeira), estadoJson);
  }

  const char *faseJson = documento["phase"] | nullptr;
  if (faseJson != nullptr) {
    copiarTextoSeguro(faseEmpilhadeira, sizeof(faseEmpilhadeira), faseJson);
  }

  if (
    documento["throttle"].isNull() ||
    documento["yaw"].isNull()
  ) {
    totalJsonInvalidos++;

    DEBUG_PRINTLN(
      "[ERRO JSON] Campo throttle ou yaw ausente."
    );

    enviarMensagem(
      "{\"erro\":\"campos_obrigatorios\","
      "\"campos\":[\"throttle\",\"yaw\"]}"
    );

    return false;
  }

  bool throttleNumerico =
    documento["throttle"].is<float>() ||
    documento["throttle"].is<int>() ||
    documento["throttle"].is<long>();

  bool yawNumerico =
    documento["yaw"].is<float>() ||
    documento["yaw"].is<int>() ||
    documento["yaw"].is<long>();

  if (!throttleNumerico || !yawNumerico) {
    totalJsonInvalidos++;

    DEBUGF(
      "[ERRO JSON] Tipo inválido. throttle numérico=%s, "
      "yaw numérico=%s\n",
      throttleNumerico ? "sim" : "nao",
      yawNumerico ? "sim" : "nao"
    );

    enviarMensagem(
      "{\"erro\":\"throttle_e_yaw_devem_ser_numericos\"}"
    );

    return false;
  }

  float novoThrottle =
    documento["throttle"].as<float>();

  float novoYaw =
    documento["yaw"].as<float>();

  if (
    !isfinite(novoThrottle) ||
    !isfinite(novoYaw)
  ) {
    totalJsonInvalidos++;

    DEBUGF(
      "[ERRO JSON] Valor não finito: throttle=%f yaw=%f\n",
      novoThrottle,
      novoYaw
    );

    enviarMensagem(
      "{\"erro\":\"throttle_ou_yaw_invalido\"}"
    );

    return false;
  }

  if (
    novoThrottle < -1.0f ||
    novoThrottle > 1.0f ||
    novoYaw < -1.0f ||
    novoYaw > 1.0f
  ) {
    DEBUGF(
      "[AVISO JSON] Valor fora da faixa -1 a 1. "
      "Será limitado. throttle=%.3f yaw=%.3f\n",
      novoThrottle,
      novoYaw
    );
  }

  throttle = constrain(
    novoThrottle,
    -1.0f,
    1.0f
  );

  yaw = constrain(
    novoYaw,
    -1.0f,
    1.0f
  );

  float novoFork = 0.0f;

  if (!documento["fork"].isNull()) {
    bool forkNumerico =
      documento["fork"].is<float>() ||
      documento["fork"].is<int>() ||
      documento["fork"].is<long>();

    if (!forkNumerico) {
      totalJsonInvalidos++;
      enviarMensagem(
        "{\"erro\":\"fork_deve_ser_numerico\"}"
      );
      return false;
    }

    novoFork = documento["fork"].as<float>();

    if (!isfinite(novoFork)) {
      totalJsonInvalidos++;
      enviarMensagem(
        "{\"erro\":\"fork_invalido\"}"
      );
      return false;
    }

    if (novoFork < -1.0f || novoFork > 1.0f) {
      DEBUGF(
        "[AVISO JSON] Valor fork fora da faixa -1 a 1. "
        "Será limitado. fork=%.3f\n",
        novoFork
      );
    }
  }

  forkCommand = constrain(
    novoFork,
    -1.0f,
    1.0f
  );

  atualizarSetpoints();

  timeoutAtivo = false;
  atualizarLedEstado();

  ultimoComandoRecebido = millis();
  comandoRecebido = true;
  totalJsonValidos++;
/*
  DEBUGF(
    "[JSON OK] Comando válido número %lu processado.\n",
    totalJsonValidos
  );
*/
  Serial.print(
    "{\"ack\":true,\"throttle\":"
  );

  Serial.print(throttle, 3);

  Serial.print(",\"yaw\":");
  Serial.print(yaw, 3);

  Serial.print(",\"fork\":");
  Serial.print(forkCommand, 3);

  Serial.print(",\"state\":\"");
  Serial.print(estadoEmpilhadeira);
  Serial.print("\",\"phase\":\"");
  Serial.print(faseEmpilhadeira);
  Serial.print("\"");

  Serial.println("}");

  return true;
}

// =====================================================
// RECEPÇÃO UART
// =====================================================

void receberDadosRaspberry() {
  static char buffer[384];

  static size_t indice = 0;
  static int profundidadeChaves = 0;
  static bool recebendoJson = false;

  while (Serial.available() > 0) {
    char caractere =
      static_cast<char>(Serial.read());

    totalBytesRecebidos++;

    if (DEBUG_ATIVO && DEBUG_BYTES_UART) {
      DEBUGF(
        "[UART BYTE] Dec=%d Hex=0x%02X Char='%c'\n",
        static_cast<unsigned char>(caractere),
        static_cast<unsigned char>(caractere),
        isPrintable(caractere) ? caractere : '.'
      );
    }

    if (caractere == '{') {
      if (!recebendoJson) {
        recebendoJson = true;
        indice = 0;
        profundidadeChaves = 0;

        DEBUG_PRINTLN(
          "[UART] Início de JSON detectado."
        );
      }

      profundidadeChaves++;
    }

    if (!recebendoJson) {
      // Ignora \n, \r e outros dados fora de um JSON.
      if (
        DEBUG_ATIVO &&
        caractere != '\n' &&
        caractere != '\r' &&
        caractere != ' ' &&
        caractere != '\t'
      ) {
        DEBUGF(
          "[AVISO UART] Byte ignorado fora de JSON: 0x%02X\n",
          static_cast<unsigned char>(caractere)
        );
      }

      continue;
    }

    if (indice >= sizeof(buffer) - 1) {
      recebendoJson = false;
      indice = 0;
      profundidadeChaves = 0;
      totalJsonInvalidos++;

      DEBUG_PRINTLN(
        "[ERRO UART] Buffer JSON excedeu 127 caracteres."
      );

      enviarMensagem(
        "{\"erro\":\"mensagem_json_muito_grande\"}"
      );

      continue;
    }

    buffer[indice++] = caractere;

    if (caractere == '}') {
      profundidadeChaves--;

      if (profundidadeChaves < 0) {
        DEBUG_PRINTLN(
          "[ERRO UART] Profundidade de chaves ficou negativa."
        );

        recebendoJson = false;
        indice = 0;
        profundidadeChaves = 0;

        continue;
      }

      if (profundidadeChaves == 0) {
        buffer[indice] = '\0';

        DEBUGF(
          "[UART] JSON completo recebido com %u caracteres.\n",
          static_cast<unsigned int>(indice)
        );

        processarJson(buffer);

        recebendoJson = false;
        indice = 0;
      }
    }
  }
}

// =====================================================
// PARADA DOS MOTORES
// =====================================================

void pararMotores() {
  DEBUG_PRINTLN(
    "[MOTORES] Executando parada dos dois motores."
  );

  throttle = 0.0f;
  yaw = 0.0f;
  forkCommand = 0.0f;

  setpointM1 = 0.0f;
  setpointM2 = 0.0f;
  setpointM3 = 0.0f;

  pwmM1 = 0;
  pwmM2 = 0;
  pwmM3 = 0;

  sentidoAnteriorM1 = 0;
  sentidoAnteriorM2 = 0;
  sentidoAnteriorM3 = 0;

  resetarPID(1);
  resetarPID(2);

  acionarMotor(
    pinDirecaoA_M1,
    pinDirecaoB_M1,
    pinoEnable_M1,
    0.0f,
    INVERTER_MOTOR_1
  );

  acionarMotor(
    pinDirecaoA_M2,
    pinDirecaoB_M2,
    pinoEnable_M2,
    0.0f,
    INVERTER_MOTOR_2
  );

  acionarMotor(
    pinDirecaoA_M3,
    pinDirecaoB_M3,
    pinoEnable_M3,
    0.0f,
    INVERTER_MOTOR_3
  );

  DEBUG_PRINTLN("[MOTORES] Motores parados.");
}

// =====================================================
// TIMEOUT
// =====================================================

void verificarTimeout(
  
  unsigned long instanteAtual
) {
  if (!comandoRecebido) {
    return;
  }
  unsigned long agora = millis();

  unsigned long tempoSemComando =
    agora - ultimoComandoRecebido;

  if (tempoSemComando > TIMEOUT_COMANDO_MS) {
    comandoRecebido = false;
    totalTimeouts++;

    DEBUGF(
      "[TIMEOUT] Nenhum comando válido por %lu ms.\n",
      tempoSemComando
    );

    DEBUGF(
      "[TIMEOUT] Total de timeouts: %lu\n",
      totalTimeouts
    );

    timeoutAtivo = true;
    pararMotores();
    atualizarLedEstado();

    enviarMensagem(
      "{\"estado\":\"timeout\","
      "\"motores\":\"parados\"}"
    );
  }
}

// =====================================================
// DEBUG DO CONTROLE
// =====================================================

void imprimirDebugControle(
  unsigned long instanteAtual
) {
  if (!DEBUG_ATIVO || !DEBUG_CONTROLE) {
    return;
  }

  if (
    instanteAtual - instanteDebugAnterior <
    INTERVALO_DEBUG_MS
  ) {
    return;
  }

  instanteDebugAnterior = instanteAtual;

  float erroM1 =
    fabsf(setpointM1) - fabsf(rotacoesM1);

  float erroM2 =
    fabsf(setpointM2) - fabsf(rotacoesM2);

  Serial.println();
  Serial.println("========== DEBUG CONTROLE ==========");

  DEBUGF(
    "Comando: throttle=%.3f | yaw=%.3f\n",
    throttle,
    yaw
  );

  DEBUGF(
    "M1: SP=%7.2f RPM | RPM=%7.2f | erro=%7.2f | "
    "PWM=%4d | pulsos=%ld | integral=%7.2f\n",
    setpointM1,
    rotacoesM1,
    erroM1,
    pwmM1,
    ultimoDeltaPulsosM1,
    pidM1.integral
  );

  DEBUGF(
    "M2: SP=%7.2f RPM | RPM=%7.2f | erro=%7.2f | "
    "PWM=%4d | pulsos=%ld | integral=%7.2f\n",
    setpointM2,
    rotacoesM2,
    erroM2,
    pwmM2,
    ultimoDeltaPulsosM2,
    pidM2.integral
  );

  DEBUGF(
    "UART: bytes=%lu | JSON válidos=%lu | "
    "JSON inválidos=%lu | timeouts=%lu\n",
    totalBytesRecebidos,
    totalJsonValidos,
    totalJsonInvalidos,
    totalTimeouts
  );

  DEBUGF(
    "Encoder power: GPIO%d=%s | GPIO%d=%s\n",
    pinoVccEncoder,
    digitalRead(pinoVccEncoder) == HIGH
      ? "HIGH"
      : "LOW",
    pinoGndEncoder,
    digitalRead(pinoGndEncoder) == LOW
      ? "LOW"
      : "HIGH"
  );

  // ---------------- ALERTAS MOTOR 1 ----------------

  if (
    abs(pwmM1) >= PWM_ALERTA_ENCODER &&
    ultimoDeltaPulsosM1 == 0 &&
    fabsf(setpointM1) > 5.0f
  ) {
    DEBUG_PRINTLN(
      "[ALERTA M1] PWM aplicado, mas nenhum pulso foi detectado."
    );

    DEBUG_PRINTLN(
      "[ALERTA M1] Verifique encoder, conversor, alimentação, "
      "GPIO34 e GPIO35."
    );
  }

  if (
    abs(pwmM1) >= 250 &&
    fabsf(erroM1) > 10.0f
  ) {
    DEBUG_PRINTLN(
      "[ALERTA M1] PID próximo da saturação máxima."
    );
  }

  if (
    fabsf(setpointM1) > 5.0f &&
    fabsf(rotacoesM1) > 5.0f &&
    setpointM1 * rotacoesM1 < 0.0f
  ) {
    DEBUG_PRINTLN(
      "[ALERTA M1] Sinal do encoder contrário ao setpoint. "
      "Considere INVERTER_ENCODER_1."
    );
  }

  // ---------------- ALERTAS MOTOR 2 ----------------

  if (
    abs(pwmM2) >= PWM_ALERTA_ENCODER &&
    ultimoDeltaPulsosM2 == 0 &&
    fabsf(setpointM2) > 5.0f
  ) {
    DEBUG_PRINTLN(
      "[ALERTA M2] PWM aplicado, mas nenhum pulso foi detectado."
    );

    DEBUG_PRINTLN(
      "[ALERTA M2] Verifique encoder, conversor, alimentação, "
      "GPIO36/VP e GPIO39/VN."
    );
  }

  if (
    abs(pwmM2) >= 250 &&
    fabsf(erroM2) > 10.0f
  ) {
    DEBUG_PRINTLN(
      "[ALERTA M2] PID próximo da saturação máxima."
    );
  }

  if (
    fabsf(setpointM2) > 5.0f &&
    fabsf(rotacoesM2) > 5.0f &&
    setpointM2 * rotacoesM2 < 0.0f
  ) {
    DEBUG_PRINTLN(
      "[ALERTA M2] Sinal do encoder contrário ao setpoint. "
      "Considere INVERTER_ENCODER_2."
    );
  }

  Serial.println("====================================");
}

// =====================================================
// CONTROLE DE VELOCIDADE
// =====================================================

void executarControle(
  unsigned long instanteAtual
) {
  unsigned long intervalo =
    instanteAtual - instanteControleAnterior;

  if (intervalo < INTERVALO_CONTROLE_MS) {
    return;
  }

  instanteControleAnterior = instanteAtual;

  if (intervalo > INTERVALO_CONTROLE_MS * 3) {
    DEBUGF(
      "[AVISO CONTROLE] Intervalo anormalmente longo: %lu ms.\n",
      intervalo
    );
  }

  float dt =
    intervalo / 1000.0f;

  long posicaoM1 = encoderM1.getCount();
  long posicaoM2 = encoderM2.getCount();
  long posicaoM3 = encoderM3.getCount();

  long deltaPulsosM1 =
    posicaoM1 - oldPosM1;

  long deltaPulsosM2 =
    posicaoM2 - oldPosM2;

  long deltaPulsosM3 =
    posicaoM3 - oldPosM3;

  ultimoDeltaPulsosM1 = deltaPulsosM1;
  ultimoDeltaPulsosM2 = deltaPulsosM2;
  ultimoDeltaPulsosM3 = deltaPulsosM3;

  oldPosM1 = posicaoM1;
  oldPosM2 = posicaoM2;
  oldPosM3 = posicaoM3;

  posicaoGarfoPulsos = INVERTER_ENCODER_3
    ? -posicaoM3
    : posicaoM3;

  rotacoesM1 =
    (deltaPulsosM1 * 60000.0f) /
    (PULSOS_POR_REVOLUCAO * intervalo);

  rotacoesM2 =
    (deltaPulsosM2 * 60000.0f) /
    (PULSOS_POR_REVOLUCAO * intervalo);

  rotacoesM3 =
    (deltaPulsosM3 * 60000.0f) /
    (PULSOS_POR_REVOLUCAO * intervalo);

  if (INVERTER_ENCODER_1) {
    rotacoesM1 = -rotacoesM1;
  }

  if (INVERTER_ENCODER_2) {
    rotacoesM2 = -rotacoesM2;
  }

  if (INVERTER_ENCODER_3) {
    rotacoesM3 = -rotacoesM3;
  }

  float saidaM1 = controlarMotor(
    "M1",
    1,
    setpointM1,
    rotacoesM1,
    dt,
    sentidoAnteriorM1
  );

  float saidaM2 = controlarMotor(
    "M2",
    2,
    setpointM2,
    rotacoesM2,
    dt,
    sentidoAnteriorM2
  );

  float saidaM3 = calcularComandoGarfo();

  pwmM1 =
    static_cast<int>(roundf(saidaM1));

  pwmM2 =
    static_cast<int>(roundf(saidaM2));

  pwmM3 =
    static_cast<int>(roundf(saidaM3));

  acionarMotor(
    pinDirecaoA_M1,
    pinDirecaoB_M1,
    pinoEnable_M1,
    saidaM1,
    INVERTER_MOTOR_1
  );

  acionarMotor(
    pinDirecaoA_M2,
    pinDirecaoB_M2,
    pinoEnable_M2,
    saidaM2,
    INVERTER_MOTOR_2
  );

  acionarMotor(
    pinDirecaoA_M3,
    pinDirecaoB_M3,
    pinoEnable_M3,
    saidaM3,
    INVERTER_MOTOR_3
  );
}

// =====================================================
// TELEMETRIA PARA O RASPBERRY
// =====================================================

void escreverTelemetria(Print &saida) {
  saida.print("{\"tipo\":\"telemetria\"");

  saida.print(",\"throttle\":");
  saida.print(throttle, 3);

  saida.print(",\"yaw\":");
  saida.print(yaw, 3);

  saida.print(",\"fork\":");
  saida.print(forkCommand, 3);

  saida.print(",\"fork_position\":");
  saida.print(forkPositionAtual, 3);

  saida.print(",\"forklift_state\":{");
  saida.print("\"mode\":\"");
  saida.print(modoEmpilhadeira);
  saida.print("\",\"state\":\"");
  saida.print(estadoEmpilhadeira);
  saida.print("\",\"phase\":\"");
  saida.print(faseEmpilhadeira);
  saida.print("\",\"timeout\":");
  saida.print(timeoutAtivo ? "true" : "false");
  saida.print(",\"led_rgb_enabled\":");
#if USAR_LED_RGB_NATIVO
  saida.print("true");
#else
  saida.print("false");
#endif
  saida.print("}");

  saida.print(",\"gyro\":{");
  saida.print("\"available\":");
  saida.print(giroscopioDisponivel ? "true" : "false");
  saida.print(",\"driver\":\"adafruit_mpu6050\"");
  saida.print(",\"roll_deg\":");
  saida.print(anguloRollGraus, 2);
  saida.print(",\"pitch_deg\":");
  saida.print(anguloPitchGraus, 2);
  saida.print(",\"yaw_deg\":");
  saida.print(anguloYawGraus, 2);
  saida.print(",\"gx_dps\":");
  saida.print(velocidadeGiroXDps, 2);
  saida.print(",\"gy_dps\":");
  saida.print(velocidadeGiroYDps, 2);
  saida.print(",\"gz_dps\":");
  saida.print(velocidadeGiroZDps, 2);
  saida.print(",\"bias_x_dps\":");
  saida.print(biasGiroXDps, 4);
  saida.print(",\"bias_y_dps\":");
  saida.print(biasGiroYDps, 4);
  saida.print(",\"bias_z_dps\":");
  saida.print(biasGiroZDps, 4);
  saida.print(",\"temp_c\":");
  saida.print(temperaturaMpuC, 2);
  saida.print("}");

  saida.print(",\"encoder_power\":{");

  saida.print("\"vcc_gpio\":");
  saida.print(pinoVccEncoder);

  saida.print(",\"gnd_gpio\":");
  saida.print(pinoGndEncoder);

  saida.print(",\"ligado\":");
  saida.print(
    digitalRead(pinoVccEncoder) == HIGH
      ? "true"
      : "false"
  );

  saida.print("},\"m1\":{");

  saida.print("\"setpoint\":");
  saida.print(setpointM1, 1);

  saida.print(",\"rpm\":");
  saida.print(rotacoesM1, 1);

  saida.print(",\"pwm\":");
  saida.print(pwmM1);

  saida.print(",\"pulsos\":");
  saida.print(ultimoDeltaPulsosM1);

  saida.print("},\"m2\":{");

  saida.print("\"setpoint\":");
  saida.print(setpointM2, 1);

  saida.print(",\"rpm\":");
  saida.print(rotacoesM2, 1);

  saida.print(",\"pwm\":");
  saida.print(pwmM2);

  saida.print(",\"pulsos\":");
  saida.print(ultimoDeltaPulsosM2);

  saida.print("},\"m3\":{");

  saida.print("\"setpoint_percent\":");
  saida.print(setpointM3, 1);

  saida.print(",\"target_pulses\":");
  saida.print(alvoGarfoPulsos);

  saida.print(",\"position_pulses\":");
  saida.print(posicaoGarfoPulsos);

  saida.print(",\"error_pulses\":");
  saida.print(erroGarfoPulsos);

  saida.print(",\"rpm\":");
  saida.print(rotacoesM3, 1);

  saida.print(",\"pwm\":");
  saida.print(pwmM3);

  saida.print(",\"pulsos\":");
  saida.print(ultimoDeltaPulsosM3);

  saida.println("}}");
}

void enviarTelemetria(
  unsigned long instanteAtual
) {
  if (
    instanteAtual - instanteTelemetriaAnterior <
    INTERVALO_TELEMETRIA_MS
  ) {
    return;
  }

  instanteTelemetriaAnterior = instanteAtual;

  escreverTelemetria(Serial);
}

// =====================================================
// DEBUG DA PINAGEM
// =====================================================

void imprimirMapaDePinos() {
  if (!DEBUG_ATIVO) {
    return;
  }

  Serial.println();
  Serial.println("========== MAPA DE PINOS ==========");

  DEBUGF(
    "Comunicacao: USB nativo (micro-USB)\n"
  );

  DEBUGF(
    "Encoder VCC relativo: GPIO%d\n",
    pinoVccEncoder
  );

  DEBUGF(
    "Encoder GND relativo: GPIO%d\n",
    pinoGndEncoder
  );

  DEBUGF(
    "M1 DIR A: GPIO%d\n",
    pinDirecaoA_M1
  );

  DEBUGF(
    "M1 DIR B: GPIO%d\n",
    pinDirecaoB_M1
  );

  DEBUGF(
    "M1 PWM: GPIO%d\n",
    pinoEnable_M1
  );

  DEBUGF(
    "M1 Encoder A: GPIO%d\n",
    enc1_A
  );

  DEBUGF(
    "M1 Encoder B: GPIO%d\n",
    enc1_B
  );

  DEBUGF(
    "M2 DIR A: GPIO%d\n",
    pinDirecaoA_M2
  );

  DEBUGF(
    "M2 DIR B: GPIO%d\n",
    pinDirecaoB_M2
  );

  DEBUGF(
    "M2 PWM: GPIO%d\n",
    pinoEnable_M2
  );

  DEBUGF(
    "M2 Encoder A: GPIO%d / VP\n",
    enc2_A
  );

  DEBUGF(
    "M2 Encoder B: GPIO%d / VN\n",
    enc2_B
  );

  Serial.println("==================================");
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  // ---------------- USB ----------------

  Serial.begin(BAUD_USB);

  delay(500);

  // Durante esta calibração (aprox. 1,6 s), não mova o robô.
  // Usa a biblioteca Adafruit_MPU6050.
  giroscopioDisponivel = inicializarGiroscopio();
  atualizarLedEstado();

  DEBUG_PRINTLN();
  DEBUG_PRINTLN("======================================");
  DEBUG_PRINTLN("      INICIALIZAÇÃO DO ESP32");
  DEBUG_PRINTLN("======================================");

  DEBUGF(
    "[SETUP] Comunicacao USB iniciada em %lu baud.\n",
    BAUD_USB
  );

  imprimirMapaDePinos();

  // ---------------- MOTORES ----------------

  DEBUG_PRINTLN("[SETUP] Configurando pinos dos motores.");

  pinMode(pinDirecaoA_M1, OUTPUT);
  pinMode(pinDirecaoB_M1, OUTPUT);
  pinMode(pinoEnable_M1, OUTPUT);

  pinMode(pinDirecaoA_M2, OUTPUT);
  pinMode(pinDirecaoB_M2, OUTPUT);
  pinMode(pinoEnable_M2, OUTPUT);

  pinMode(pinDirecaoA_M3, OUTPUT);
  pinMode(pinDirecaoB_M3, OUTPUT);
  pinMode(pinoEnable_M3, OUTPUT);

  digitalWrite(pinDirecaoA_M1, LOW);
  digitalWrite(pinDirecaoB_M1, LOW);
  analogWrite(pinoEnable_M1, 0);

  digitalWrite(pinDirecaoA_M2, LOW);
  digitalWrite(pinDirecaoB_M2, LOW);
  analogWrite(pinoEnable_M2, 0);

  digitalWrite(pinDirecaoA_M3, LOW);
  digitalWrite(pinDirecaoB_M3, LOW);
  analogWrite(pinoEnable_M3, 0);

  DEBUG_PRINTLN(
    "[SETUP] Motores configurados e inicialmente parados."
  );

  // ---------------- ALIMENTAÇÃO ENCODERS ----------------

  ligarAlimentacaoEncoders();

  // ---------------- ENTRADAS ENCODERS ----------------

  DEBUG_PRINTLN(
    "[SETUP] Configurando entradas dos encoders."
  );

  /*
    GPIO34, GPIO35, GPIO36 e GPIO39 não possuem
    pull-up interno.
  */

  // Pull-up interno da biblioteca. ATENÇÃO: não tem efeito nos
  // pinos 34/35/36/39, que são input-only sem pull-up no hardware.
  // Se o encoder for open-collector, ainda precisará de 10k externo.
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  // Quadratura completa (conta bordas dos dois canais => 4x resolução).
  encoderM1.attachFullQuad(enc1_A, enc1_B);
  encoderM2.attachFullQuad(enc2_A, enc2_B);
  encoderM3.attachFullQuad(enc3_A, enc3_B);

  encoderM1.clearCount();
  encoderM2.clearCount();
  encoderM3.clearCount();

  oldPosM1 = 0;
  oldPosM2 = 0;
  oldPosM3 = 0;

  DEBUGF(
    "[SETUP] Encoders ESP32Encoder iniciados. "
    "M1=(GPIO%d,GPIO%d) M2=(GPIO%d,GPIO%d) M3=(GPIO%d,GPIO%d)\n",
    enc1_A, enc1_B, enc2_A, enc2_B, enc3_A, enc3_B
  );

  // ---------------- TEMPORIZADORES ----------------

  instanteControleAnterior = millis();
  instanteTelemetriaAnterior = millis();
  instanteDebugAnterior = millis();

  DEBUG_PRINTLN("[SETUP] Inicialização concluída.");
  DEBUG_PRINTLN(
    "[SETUP] Aguardando JSON do Raspberry Pi..."
  );

  enviarMensagem(
    "{\"estado\":\"inicializado\","
    "\"placa\":\"ESP32-WROOM\","
    "\"comunicacao\":\"usb\","
    "\"baud\":115200,"
    "\"rpm_maximo\":180,"
    "\"garfo_curso_pulsos\":7200}"
  );
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  unsigned long instanteAtual = millis();

  atualizarGiroscopio();
  receberDadosRaspberry();
  verificarTimeout(instanteAtual);
  executarControle(instanteAtual);
  enviarTelemetria(instanteAtual);
  atualizarLedEstado();
  imprimirDebugControle(instanteAtual);
}
