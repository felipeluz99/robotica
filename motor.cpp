#include <Arduino.h>
#include <ArduinoJson.h>
#include <math.h>

// =====================================================
// CONFIGURAÇÕES DE DEBUG
// =====================================================

// Coloque false para remover os prints de diagnóstico.
const bool DEBUG_ATIVO = true;

// Mostra cada JSON completo recebido.
const bool DEBUG_JSON_RECEBIDO = false;

// Mostra cada byte recebido pela UART.
// Deixe false normalmente, pois gera muitos prints.
const bool DEBUG_BYTES_UART = false;

// Mostra periodicamente RPM, setpoints, PWM, erros e pulsos.
const bool DEBUG_CONTROLE = true;

// Intervalo entre relatórios periódicos.
const unsigned long INTERVALO_DEBUG_MS = 1500;

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

// USB: monitor serial e debug.
const uint32_t BAUD_USB = 115200;

// UART entre Raspberry Pi e ESP32.
const uint32_t BAUD_RASPBERRY = 115200;

// UART2 do ESP32.
const int PINO_RX_RASPBERRY = 16;
const int PINO_TX_RASPBERRY = 17;

HardwareSerial SerialRaspberry(2);

// Tempo sem comandos antes da parada automática.
const unsigned long TIMEOUT_COMANDO_MS = 1000;

// =====================================================
// ALIMENTAÇÃO DOS ENCODERS POR GPIO
// =====================================================

// GPIO32 mantido em HIGH.
const int pinoVccEncoder = 32;

// GPIO4 mantido em LOW.
const int pinoGndEncoder = 4;

const unsigned long TEMPO_ESTABILIZACAO_ENCODER_MS = 100;

// =====================================================
// MOTOR 1
// =====================================================

const int pinDirecaoA_M1 = 13;
const int pinDirecaoB_M1 = 14;
const int pinoEnable_M1  = 25;

const int enc1_A = 34;
const int enc1_B = 35;

volatile long posAtualM1 = 0;
long oldPosM1 = 0;

float rotacoesM1 = 0.0f;
float setpointM1 = 0.0f;

int pwmM1 = 0;
int sentidoAnteriorM1 = 0;

// =====================================================
// MOTOR 2
// =====================================================

const int pinDirecaoA_M2 = 26;
const int pinDirecaoB_M2 = 27;
const int pinoEnable_M2  = 33;

// GPIO36 aparece como VP.
// GPIO39 aparece como VN.
const int enc2_A = 36;
const int enc2_B = 39;

volatile long posAtualM2 = 0;
long oldPosM2 = 0;

float rotacoesM2 = 0.0f;
float setpointM2 = 0.0f;

int pwmM2 = 0;
int sentidoAnteriorM2 = 0;

// Últimos deltas para diagnóstico.
long ultimoDeltaPulsosM1 = 0;
long ultimoDeltaPulsosM2 = 0;

// =====================================================
// CONFIGURAÇÕES DOS MOTORES E ENCODERS
// =====================================================

// Ajustar conforme a resolução real do encoder.
const float PULSOS_POR_REVOLUCAO = 720.0f;

// RPM para comando máximo.
const float RPM_MAXIMO = 180.0f;

// Inverta caso o motor físico esteja no sentido contrário.
const bool INVERTER_MOTOR_1 = false;
const bool INVERTER_MOTOR_2 = false;

// Inverta caso o encoder informe sinal contrário.
const bool INVERTER_ENCODER_1 = false;
const bool INVERTER_ENCODER_2 = false;

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
  1.50f,  // Kp
  0.80f,  // Ki
  0.02f,  // Kd
  0.0f,
  0.0f,
  false
};

ControladorPID pidM2 = {
  1.50f,
  0.80f,
  0.02f,
  0.0f,
  0.0f,
  false
};

const float LIMITE_INTEGRAL = 300.0f;

// =====================================================
// INTERRUPÇÕES DOS ENCODERS
// =====================================================

// Não coloque Serial.print dentro das interrupções.

void IRAM_ATTR lerEncoder1() {
  if (digitalRead(enc1_A) == digitalRead(enc1_B)) {
    posAtualM1++;
  } else {
    posAtualM1--;
  }
}

void IRAM_ATTR lerEncoder2() {
  if (digitalRead(enc2_A) == digitalRead(enc2_B)) {
    posAtualM2++;
  } else {
    posAtualM2--;
  }
}

// =====================================================
// MENSAGENS PARA RASPBERRY
// =====================================================

// Envia resposta para o Raspberry e replica na USB.
void enviarMensagem(const char *mensagem) {
  SerialRaspberry.println(mensagem);

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
// PID
// =====================================================

void resetarPID(ControladorPID &pid) {
  pid.integral = 0.0f;
  pid.medidaAnterior = 0.0f;
  pid.iniciado = false;
}

float calcularPID(
  ControladorPID &pid,
  float setpoint,
  float medida,
  float dt
) {
  if (setpoint < 0.5f || dt <= 0.0f) {
    resetarPID(pid);
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
  ControladorPID &pid,
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

    resetarPID(pid);
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

    resetarPID(pid);
  }

  sentidoAnterior = sentidoAtual;

  float setpointAbsoluto =
    fabsf(setpoint);

  float rpmNoSentidoSolicitado =
    rpmMedido * static_cast<float>(sentidoAtual);

  float intensidadePWM = calcularPID(
    pid,
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

  if (
    throttle != throttleOriginal ||
    yaw != yawOriginal
  ) {
    DEBUGF(
      "[COMANDO] Valores limitados: throttle %.3f -> %.3f, "
      "yaw %.3f -> %.3f\n",
      throttleOriginal,
      throttle,
      yawOriginal,
      yaw
    );
  }

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
  StaticJsonDocument<128> documento;
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

    SerialRaspberry.print(
      "{\"erro\":\"json_invalido\",\"detalhe\":\""
    );

    SerialRaspberry.print(erroJson.c_str());
    SerialRaspberry.println("\"}");

    return false;
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

  atualizarSetpoints();

  ultimoComandoRecebido = millis();
  comandoRecebido = true;
  totalJsonValidos++;

  DEBUGF(
    "[JSON OK] Comando válido número %lu processado.\n",
    totalJsonValidos
  );

  SerialRaspberry.print(
    "{\"ack\":true,\"throttle\":"
  );

  SerialRaspberry.print(throttle, 3);

  SerialRaspberry.print(",\"yaw\":");
  SerialRaspberry.print(yaw, 3);

  SerialRaspberry.println("}");

  return true;
}

// =====================================================
// RECEPÇÃO UART
// =====================================================

void receberDadosRaspberry() {
  static char buffer[128];

  static size_t indice = 0;
  static int profundidadeChaves = 0;
  static bool recebendoJson = false;

  while (SerialRaspberry.available() > 0) {
    char caractere =
      static_cast<char>(SerialRaspberry.read());

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

  setpointM1 = 0.0f;
  setpointM2 = 0.0f;

  pwmM1 = 0;
  pwmM2 = 0;

  sentidoAnteriorM1 = 0;
  sentidoAnteriorM2 = 0;

  resetarPID(pidM1);
  resetarPID(pidM2);

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

    pararMotores();

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

  long posicaoM1;
  long posicaoM2;

  noInterrupts();

  posicaoM1 = posAtualM1;
  posicaoM2 = posAtualM2;

  interrupts();

  long deltaPulsosM1 =
    posicaoM1 - oldPosM1;

  long deltaPulsosM2 =
    posicaoM2 - oldPosM2;

  ultimoDeltaPulsosM1 = deltaPulsosM1;
  ultimoDeltaPulsosM2 = deltaPulsosM2;

  oldPosM1 = posicaoM1;
  oldPosM2 = posicaoM2;

  rotacoesM1 =
    (deltaPulsosM1 * 60000.0f) /
    (PULSOS_POR_REVOLUCAO * intervalo);

  rotacoesM2 =
    (deltaPulsosM2 * 60000.0f) /
    (PULSOS_POR_REVOLUCAO * intervalo);

  if (INVERTER_ENCODER_1) {
    rotacoesM1 = -rotacoesM1;
  }

  if (INVERTER_ENCODER_2) {
    rotacoesM2 = -rotacoesM2;
  }

  float saidaM1 = controlarMotor(
    "M1",
    pidM1,
    setpointM1,
    rotacoesM1,
    dt,
    sentidoAnteriorM1
  );

  float saidaM2 = controlarMotor(
    "M2",
    pidM2,
    setpointM2,
    rotacoesM2,
    dt,
    sentidoAnteriorM2
  );

  pwmM1 =
    static_cast<int>(roundf(saidaM1));

  pwmM2 =
    static_cast<int>(roundf(saidaM2));

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

  escreverTelemetria(SerialRaspberry);
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
    "Raspberry RX ESP32: GPIO%d\n",
    PINO_RX_RASPBERRY
  );

  DEBUGF(
    "Raspberry TX ESP32: GPIO%d\n",
    PINO_TX_RASPBERRY
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

  DEBUG_PRINTLN();
  DEBUG_PRINTLN("======================================");
  DEBUG_PRINTLN("      INICIALIZAÇÃO DO ESP32");
  DEBUG_PRINTLN("======================================");

  DEBUGF(
    "[SETUP] Monitor USB iniciado em %lu baud.\n",
    BAUD_USB
  );

  // ---------------- UART RASPBERRY ----------------

  DEBUG_PRINTLN(
    "[SETUP] Inicializando UART2 do Raspberry..."
  );

  SerialRaspberry.begin(
    BAUD_RASPBERRY,
    SERIAL_8N1,
    PINO_RX_RASPBERRY,
    PINO_TX_RASPBERRY
  );

  DEBUGF(
    "[SETUP] UART Raspberry: baud=%lu RX=%d TX=%d\n",
    BAUD_RASPBERRY,
    PINO_RX_RASPBERRY,
    PINO_TX_RASPBERRY
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

  digitalWrite(pinDirecaoA_M1, LOW);
  digitalWrite(pinDirecaoB_M1, LOW);
  analogWrite(pinoEnable_M1, 0);

  digitalWrite(pinDirecaoA_M2, LOW);
  digitalWrite(pinDirecaoB_M2, LOW);
  analogWrite(pinoEnable_M2, 0);

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

  pinMode(enc1_A, INPUT);
  pinMode(enc1_B, INPUT);

  pinMode(enc2_A, INPUT);
  pinMode(enc2_B, INPUT);

  DEBUGF(
    "[SETUP] Estado inicial M1 Encoder A=%d B=%d\n",
    digitalRead(enc1_A),
    digitalRead(enc1_B)
  );

  DEBUGF(
    "[SETUP] Estado inicial M2 Encoder A=%d B=%d\n",
    digitalRead(enc2_A),
    digitalRead(enc2_B)
  );

  noInterrupts();

  posAtualM1 = 0;
  posAtualM2 = 0;

  oldPosM1 = 0;
  oldPosM2 = 0;

  interrupts();

  DEBUG_PRINTLN(
    "[SETUP] Contadores dos encoders zerados."
  );

  attachInterrupt(
    digitalPinToInterrupt(enc1_A),
    lerEncoder1,
    CHANGE
  );

  DEBUGF(
    "[SETUP] Interrupção do encoder M1 anexada ao GPIO%d.\n",
    enc1_A
  );

  attachInterrupt(
    digitalPinToInterrupt(enc2_A),
    lerEncoder2,
    CHANGE
  );

  DEBUGF(
    "[SETUP] Interrupção do encoder M2 anexada ao GPIO%d.\n",
    enc2_A
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
    "\"raspberry_baud\":115200,"
    "\"raspberry_rx_gpio\":16,"
    "\"raspberry_tx_gpio\":17,"
    "\"rpm_maximo\":180,"
    "\"encoder_vcc\":\"GPIO32_HIGH\","
    "\"encoder_gnd\":\"GPIO4_LOW\","
    "\"encoder_m2_a\":\"GPIO36_VP\","
    "\"encoder_m2_b\":\"GPIO39_VN\"}"
  );
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  unsigned long instanteAtual = millis();

  receberDadosRaspberry();
  verificarTimeout(instanteAtual);
  executarControle(instanteAtual);
  enviarTelemetria(instanteAtual);
  imprimirDebugControle(instanteAtual);
}
