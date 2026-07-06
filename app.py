
import glob
import json
import math
import threading
import time
from enum import Enum

import cv2
import numpy as np
import serial
from flask import Flask, render_template
from flask_socketio import SocketIO

try:
    from pupil_apriltags import Detector
except ImportError:
    Detector = None


# ============================================================
# CONFIGURAÇÕES GERAIS
# ============================================================

BAUD_ESP = 115200
CAMERA_ID = 0
LARGURA = 640
ALTURA = 480
FPS = 10
QUALIDADE_JPEG = 90

DETECTAR_APRILTAGS = True
TAMANHO_TAG_M = 0.048
FAMILIA_TAG = "tag25h9"
APRILTAG_TELEMETRY_INTERVAL = 0.20

# ============================================================
# PARÂMETROS INICIAIS DA BUSCA
# Todos estes valores são de bancada e precisam ser calibrados.
# ============================================================

# A arena terá três posições longitudinais, com uma tag em cada lado.
NUM_ESTACOES = 3

# Avanço aproximado entre duas fileiras de tags. Como ainda não temos
# diâmetro de roda e relação pulsos/cm confirmados, nesta primeira versão
# o avanço é temporizado. Ajuste até o robô percorrer aproximadamente 20 cm.
TEMPO_AVANCO_20CM_S = 1.20
THROTTLE_BUSCA = 0.24

# Giros fechados por giroscópio.
KP_GIRO = 0.012
YAW_GIRO_MAX = 0.42
YAW_GIRO_MIN = 0.16
TOLERANCIA_GIRO_GRAUS = 3.0
AMOSTRAS_GIRO_ESTAVEL = 4

# Em alguns chassis, o sinal de yaw pode estar invertido.
# Troque para -1.0 se o robô girar para o lado oposto.
SINAL_YAW_GIRO = 1.0

# Tempo parado olhando para cada lado antes de concluir que o alvo não está ali.
TEMPO_OBSERVACAO_LADO_S = 0.90

# Controle visual final.
DISTANCIA_FINAL_TAG_MM = 150.0
TOLERANCIA_X_MM = 20.0
TOLERANCIA_Z_MM = 25.0
KP_X_VISUAL = 0.0035
KP_Z_VISUAL = 0.0030
YAW_VISUAL_MAX = 0.30
THROTTLE_VISUAL_MAX = 0.22
AMOSTRAS_ALINHADO = 6

# Troque o sinal caso a correção visual vá para o lado errado.
SINAL_YAW_VISUAL = 1.0
SINAL_THROTTLE_VISUAL = 1.0

# A pose angular da AprilTag costuma ser mais ruidosa sem calibração de câmera.
# Por isso, nesta primeira versão o alinhamento final usa centralização X + distância Z.
USAR_ANGULO_DA_TAG = False
TOLERANCIA_ANGULO_TAG_GRAUS = 8.0
KP_ANGULO_TAG = 0.010

IDADE_MAXIMA_DETECCAO_S = 0.55
TAG_RETENCAO_INTERFACE_S = 1.00
TEMPO_PERDA_TAG_S = 0.60
TEMPO_RECUPERACAO_TAG_S = 3.0
AMPLITUDE_RECUPERACAO_GRAUS = 18.0

INTERVALO_LOOP_AUTONOMO_S = 0.05
INTERVALO_STATUS_S = 0.20

# Reenvio de comando manual pelo backend.
# O ESP32 tem timeout de 1 s; então o app.py passa a repetir o último
# comando manual em frequência fixa, em vez de depender apenas dos eventos
# ``move`` do joystick no navegador/celular.
INTERVALO_REENVIO_MANUAL_S = 0.10
TEMPO_MAX_COMANDO_MANUAL_SEM_ATUALIZACAO_S = 3.0


# ============================================================
# CONEXÃO SERIAL COM O ESP32
# ============================================================

serial_lock = threading.Lock()
ser = None
_ultimo_envio = 0.0

# Último comando manual recebido do navegador.
# O loop de reenvio usa estes valores para manter o ESP32 alimentado
# com throttle/yaw enquanto o operador segura o joystick parado.
manual_lock = threading.Lock()
ultimo_comando_manual = {
    "throttle": 0.0,
    "yaw": 0.0,
    "fork": 0.0,
}
instante_ultimo_comando_manual = 0.0


def estado_atual_para_esp():
    """Retorna o estado lógico atual para debug no ESP32/LED.

    A função tolera chamadas antes da criação do controlador.
    """
    try:
        status = controlador.status()
    except NameError:
        return {
            "mode": "MANUAL",
            "state": "IDLE",
            "phase": "PARADO",
            "target_tag": None,
        }

    return {
        "mode": status.get("mode", "MANUAL"),
        "state": status.get("state", "IDLE"),
        "phase": status.get("phase", "PARADO"),
        "target_tag": status.get("target_tag"),
    }


def anexar_estado_e_garfo(dados):
    """Anexa modo/estado/fase ao JSON enviado ao ESP32."""
    comando = dict(dados)
    estado = estado_atual_para_esp()

    comando.setdefault("mode", estado["mode"])
    comando.setdefault("state", estado["state"])
    comando.setdefault("phase", estado["phase"])

    if estado["target_tag"] is not None:
        comando.setdefault("target_tag", estado["target_tag"])

    comando.setdefault("fork", 0.0)
    return comando


def abrir_serial_esp(baud=BAUD_ESP):
    portas = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    if not portas:
        print("AVISO: nenhum ESP32 encontrado em /dev/ttyUSB* ou /dev/ttyACM*.")
        return None

    porta = portas[0]
    print(f"Abrindo ESP32 em {porta} @ {baud} baud")

    try:
        conexao = serial.Serial(porta, baud, timeout=0.2)
        time.sleep(2)
        conexao.reset_input_buffer()
        return conexao
    except Exception as erro:
        print("Erro ao abrir ESP32:", erro)
        return None


def envio_esp(dados, forcar=False):
    """Envia JSON ao ESP32. Em emergência, use forcar=True."""
    global _ultimo_envio

    agora = time.monotonic()
    if not forcar and agora - _ultimo_envio < 0.045:
        return False

    _ultimo_envio = agora
    dados = anexar_estado_e_garfo(dados)
    msg = json.dumps(dados, separators=(",", ":")) + "\n"

    if ser is None:
        return False

    try:
        with serial_lock:
            ser.write(msg.encode("utf-8"))
        return True
    except Exception as erro:
        print("Erro ao escrever na serial do ESP:", erro)
        return False


def parar_robo():
    envio_esp({"throttle": 0.0, "yaw": 0.0, "fork": 0.0}, forcar=True)


# ============================================================
# FLASK + SOCKET.IO
# ============================================================

app = Flask(__name__)
app.config["SECRET_KEY"] = "secret"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")


@app.route("/")
def index():
    return render_template("index.html")


# ============================================================
# TELEMETRIA / VISÃO COMPARTILHADAS ENTRE THREADS
# ============================================================

telemetria_lock = threading.Lock()
telemetria_esp = {
    "gyro_available": False,
    "driver": None,
    "error": None,
    "address": None,
    "sda": None,
    "scl": None,
    "roll_deg": None,
    "pitch_deg": None,
    "yaw_deg": None,
    "gx_dps": None,
    "gy_dps": None,
    "gz_dps": None,
    "temp_c": None,
    "last_update": 0.0,
}

visao_lock = threading.Lock()
ultimas_tags = []
memoria_tags = {}
instante_ultimo_frame_tags = 0.0


def atualizar_telemetria_local(dados):
    if not isinstance(dados, dict):
        return

    gyro = dados.get("gyro")
    # ACKs e mensagens de inicialização também são JSON, mas não carregam
    # a telemetria completa do giroscópio. Eles não devem invalidar o último yaw.
    if not isinstance(gyro, dict):
        return

    with telemetria_lock:
        telemetria_esp["gyro_available"] = bool(gyro.get("available", False))
        telemetria_esp["driver"] = gyro.get("driver")
        telemetria_esp["error"] = gyro.get("error")
        telemetria_esp["address"] = gyro.get("address")
        telemetria_esp["sda"] = gyro.get("sda")
        telemetria_esp["scl"] = gyro.get("scl")
        telemetria_esp["i2c_config"] = gyro.get("i2c_config")

        for chave in (
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "gx_dps",
            "gy_dps",
            "gz_dps",
            "temp_c",
        ):
            if gyro.get(chave) is not None:
                try:
                    telemetria_esp[chave] = float(gyro[chave])
                except (TypeError, ValueError):
                    pass

        telemetria_esp["last_update"] = time.monotonic()


def obter_yaw_atual():
    with telemetria_lock:
        disponivel = telemetria_esp["gyro_available"]
        yaw = telemetria_esp["yaw_deg"]
        idade = time.monotonic() - telemetria_esp["last_update"]

    if not disponivel or yaw is None or idade > 1.0:
        return None
    return yaw


def obter_tag_alvo(tag_id):
    """
    Retorna a tag alvo com pequena tolerância temporal.

    A detecção por AprilTag pode falhar em frames isolados, mesmo com a tag
    fisicamente parada em frente à câmera. Por isso, o controle pode usar a
    última pose válida por IDADE_MAXIMA_DETECCAO_S segundos. Depois disso, a
    tag é considerada perdida para navegação.
    """
    agora = time.monotonic()
    with visao_lock:
        tags = list(ultimas_tags)

    for tag in tags:
        if tag.get("id") != tag_id:
            continue

        ultima_vez = tag.get("last_seen_monotonic")
        if ultima_vez is None:
            return tag

        if agora - ultima_vez <= IDADE_MAXIMA_DETECCAO_S:
            return tag

    return None


# ============================================================
# MÁQUINA DE ESTADOS
# ============================================================

class EstadoMissao(str, Enum):
    IDLE = "IDLE"
    BUSCA = "BUSCA"
    ENCONTRADO = "ENCONTRADO"


class FaseBusca(str, Enum):
    AGUARDANDO_GIRO = "AGUARDANDO_GIRO"
    GIRAR_ESQUERDA = "GIRAR_ESQUERDA"
    OBSERVAR_ESQUERDA = "OBSERVAR_ESQUERDA"
    GIRAR_DIREITA = "GIRAR_DIREITA"
    OBSERVAR_DIREITA = "OBSERVAR_DIREITA"
    RETORNAR_FRENTE = "RETORNAR_FRENTE"
    AVANCAR = "AVANCAR"
    GIRAR_RETORNO = "GIRAR_RETORNO"
    ALINHAR = "ALINHAR"
    RECUPERAR_TAG = "RECUPERAR_TAG"
    PARADO = "PARADO"


def normalizar_angulo(angulo):
    return (angulo + 180.0) % 360.0 - 180.0


def erro_angular(alvo, atual):
    return normalizar_angulo(alvo - atual)


def limitar(valor, minimo, maximo):
    return max(minimo, min(maximo, valor))


class ControladorMissao:
    def __init__(self):
        self.lock = threading.RLock()
        self.modo = "MANUAL"
        self.estado = EstadoMissao.IDLE
        self.fase = FaseBusca.PARADO
        self.tag_alvo = None
        self.mensagem = "Informe uma AprilTag e ative o modo AUTO."

        self.heading_frente = None
        self.heading_desejado = None
        self.heading_recuperacao = None
        self.fase_iniciada = time.monotonic()
        self.estacao = 0
        self.passagem = 0
        self.giro_estavel = 0
        self.alinhamento_estavel = 0
        self.ultimo_instante_tag = 0.0
        self.sentido_recuperacao = 1
        self.ultimo_status = 0.0

    def _mudar_fase(self, fase, mensagem=None):
        self.fase = fase
        self.fase_iniciada = time.monotonic()
        self.giro_estavel = 0
        if mensagem is not None:
            self.mensagem = mensagem

    def definir_tag(self, tag_id):
        with self.lock:
            if self.modo != "MANUAL":
                return False, "A tag só pode ser alterada no modo MANUAL."

            try:
                tag_id = int(tag_id)
            except (TypeError, ValueError):
                return False, "Digite um número de AprilTag válido."

            if tag_id < 0:
                return False, "O ID da AprilTag não pode ser negativo."

            self.tag_alvo = tag_id
            self.estado = EstadoMissao.IDLE
            self._mudar_fase(FaseBusca.PARADO)
            self.mensagem = f"AprilTag {tag_id} definida. Posicione o robô e ative AUTO."
            return True, self.mensagem

    def alterar_modo(self, novo_modo):
        novo_modo = str(novo_modo).upper()
        if novo_modo not in ("MANUAL", "AUTO"):
            return False, "Modo inválido."

        with self.lock:
            if novo_modo == "MANUAL":
                estava_encontrado = self.estado == EstadoMissao.ENCONTRADO
                self.modo = "MANUAL"
                self.estado = EstadoMissao.IDLE
                self._mudar_fase(FaseBusca.PARADO)
                self.heading_frente = None
                self.heading_desejado = None
                self.alinhamento_estavel = 0
                parar_robo()

                if estava_encontrado:
                    self.tag_alvo = None
                    self.mensagem = (
                        "Alvo anterior concluído. Levante o garfo, informe a próxima tag "
                        "e ative AUTO novamente."
                    )
                else:
                    self.mensagem = "Modo MANUAL. Controle liberado para o operador."
                return True, self.mensagem

            if self.tag_alvo is None:
                return False, "Informe primeiro o número da AprilTag."

            self.modo = "AUTO"
            self.estado = EstadoMissao.BUSCA
            self.estacao = 0
            self.passagem = 0
            self.alinhamento_estavel = 0
            self.heading_frente = None
            self.heading_desejado = None
            self.heading_recuperacao = None
            self._mudar_fase(
                FaseBusca.AGUARDANDO_GIRO,
                f"Buscando a AprilTag {self.tag_alvo}. Aguardando giroscópio.",
            )
            parar_robo()
            return True, self.mensagem

    def status(self):
        with self.lock:
            yaw = obter_yaw_atual()
            with telemetria_lock:
                roll = telemetria_esp.get("roll_deg")
                pitch = telemetria_esp.get("pitch_deg")
                gx = telemetria_esp.get("gx_dps")
                gy = telemetria_esp.get("gy_dps")
                gz = telemetria_esp.get("gz_dps")
                driver = telemetria_esp.get("driver")
                gyro_error = telemetria_esp.get("error")
                gyro_address = telemetria_esp.get("address")
                gyro_sda = telemetria_esp.get("sda")
                gyro_scl = telemetria_esp.get("scl")
                gyro_i2c_config = telemetria_esp.get("i2c_config")

            agora = time.monotonic()
            with visao_lock:
                tags_visiveis = list(ultimas_tags)
                idade_tags = (
                    agora - instante_ultimo_frame_tags
                    if instante_ultimo_frame_tags > 0
                    else None
                )

            ids_tags_visiveis = [tag.get("id") for tag in tags_visiveis if tag.get("id") is not None]
            tag_alvo_info = None

            if self.tag_alvo is not None:
                for tag in tags_visiveis:
                    if tag.get("id") == self.tag_alvo:
                        tag_alvo_info = tag
                        break

            alvo_detectado = tag_alvo_info is not None

            return {
                "mode": self.modo,
                "state": self.estado.value,
                "phase": self.fase.value,
                "target_tag": self.tag_alvo,
                "detected_tag_ids": ids_tags_visiveis,
                "detected_tag_count": len(tags_visiveis),
                "target_tag_detected": alvo_detectado,
                "target_tag_info": tag_alvo_info,
                "apriltag_detector_ok": apriltag_disponivel,
                "apriltag_error": apriltag_mensagem_erro,
                "apriltag_age_s": None if idade_tags is None else round(idade_tags, 2),
                "message": self.mensagem,
                "station": self.estacao + 1 if self.estado == EstadoMissao.BUSCA else None,
                "pass": self.passagem + 1 if self.estado == EstadoMissao.BUSCA else None,
                "gyro_driver": driver,
                "gyro_error": gyro_error,
                "gyro_address": gyro_address,
                "gyro_sda": gyro_sda,
                "gyro_scl": gyro_scl,
                "gyro_i2c_config": gyro_i2c_config,
                "gyro_roll_deg": None if roll is None else round(roll, 1),
                "gyro_pitch_deg": None if pitch is None else round(pitch, 1),
                "gyro_yaw_deg": None if yaw is None else round(yaw, 1),
                "gyro_gx_dps": None if gx is None else round(gx, 2),
                "gyro_gy_dps": None if gy is None else round(gy, 2),
                "gyro_gz_dps": None if gz is None else round(gz, 2),
                "gyro_ok": bool(telemetria_esp.get("gyro_available")) and yaw is not None,
            }

    def _comando_giro(self, yaw_atual, heading_alvo):
        erro = erro_angular(heading_alvo, yaw_atual)

        if abs(erro) <= TOLERANCIA_GIRO_GRAUS:
            self.giro_estavel += 1
            return 0.0, True

        self.giro_estavel = 0
        comando = limitar(KP_GIRO * erro, -YAW_GIRO_MAX, YAW_GIRO_MAX)
        if 0 < abs(comando) < YAW_GIRO_MIN:
            comando = math.copysign(YAW_GIRO_MIN, comando)

        return SINAL_YAW_GIRO * comando, False

    def _entrar_alinhamento(self, yaw_atual):
        self.heading_recuperacao = yaw_atual
        self.ultimo_instante_tag = time.monotonic()
        self.alinhamento_estavel = 0
        self._mudar_fase(
            FaseBusca.ALINHAR,
            f"AprilTag {self.tag_alvo} detectada. Fazendo alinhamento final.",
        )

    def _finalizar_encontrado(self):
        self.estado = EstadoMissao.ENCONTRADO
        self._mudar_fase(FaseBusca.PARADO)
        self.mensagem = (
            f"AprilTag {self.tag_alvo} encontrada e alinhada. "
            "Mude para MANUAL antes de levantar o garfo."
        )
        parar_robo()

    def _passo_alinhamento(self, tag, yaw_atual):
        agora = time.monotonic()

        if tag is None:
            if agora - self.ultimo_instante_tag <= TEMPO_PERDA_TAG_S:
                return {"throttle": 0.0, "yaw": 0.0}

            self.heading_recuperacao = yaw_atual
            self.sentido_recuperacao = 1
            self._mudar_fase(
                FaseBusca.RECUPERAR_TAG,
                "A tag saiu da imagem. Fazendo uma varredura curta para recuperá-la.",
            )
            return {"throttle": 0.0, "yaw": 0.0}

        self.ultimo_instante_tag = agora

        x_mm = float(tag.get("x_mm", 0.0))
        z_mm = float(tag.get("z_mm", 0.0))
        angulo_tag = float(tag.get("giro_horizontal_graus", 0.0))

        erro_z = z_mm - DISTANCIA_FINAL_TAG_MM
        termo_angulo = KP_ANGULO_TAG * angulo_tag if USAR_ANGULO_DA_TAG else 0.0

        yaw_cmd = SINAL_YAW_VISUAL * limitar(
            KP_X_VISUAL * x_mm + termo_angulo,
            -YAW_VISUAL_MAX,
            YAW_VISUAL_MAX,
        )

        throttle_cmd = SINAL_THROTTLE_VISUAL * limitar(
            KP_Z_VISUAL * erro_z,
            -THROTTLE_VISUAL_MAX,
            THROTTLE_VISUAL_MAX,
        )

        # Primeiro centraliza; evita avançar com grande erro lateral.
        if abs(x_mm) > 70.0:
            throttle_cmd = 0.0

        alinhado_x = abs(x_mm) <= TOLERANCIA_X_MM
        alinhado_z = abs(erro_z) <= TOLERANCIA_Z_MM
        alinhado_angulo = (
            not USAR_ANGULO_DA_TAG
            or abs(angulo_tag) <= TOLERANCIA_ANGULO_TAG_GRAUS
        )

        if alinhado_x and alinhado_z and alinhado_angulo:
            self.alinhamento_estavel += 1
            throttle_cmd = 0.0
            yaw_cmd = 0.0
        else:
            self.alinhamento_estavel = 0

        self.mensagem = (
            f"Alinhando tag {self.tag_alvo}: X={x_mm:.0f} mm, "
            f"Z={z_mm:.0f} mm."
        )

        if self.alinhamento_estavel >= AMOSTRAS_ALINHADO:
            self._finalizar_encontrado()
            return {"throttle": 0.0, "yaw": 0.0}

        return {"throttle": throttle_cmd, "yaw": yaw_cmd}

    def _passo_recuperacao(self, tag, yaw_atual):
        agora = time.monotonic()

        if tag is not None:
            self._entrar_alinhamento(yaw_atual)
            return {"throttle": 0.0, "yaw": 0.0}

        if agora - self.fase_iniciada > TEMPO_RECUPERACAO_TAG_S:
            # Sem pose absoluta, a opção mais segura é assumir a orientação atual
            # como novo eixo longitudinal e reiniciar a varredura.
            self.heading_frente = yaw_atual
            self.heading_desejado = normalizar_angulo(self.heading_frente + 90.0)
            self._mudar_fase(
                FaseBusca.GIRAR_ESQUERDA,
                "Tag não recuperada. Reiniciando a busca a partir da posição atual.",
            )
            return {"throttle": 0.0, "yaw": 0.0}

        centro = self.heading_recuperacao if self.heading_recuperacao is not None else yaw_atual
        tempo = agora - self.fase_iniciada
        # Alterna o alvo a cada 0,7 s para varrer para os dois lados.
        sentido = 1 if int(tempo / 0.7) % 2 == 0 else -1
        alvo = normalizar_angulo(centro + sentido * AMPLITUDE_RECUPERACAO_GRAUS)
        yaw_cmd, _ = self._comando_giro(yaw_atual, alvo)
        return {"throttle": 0.0, "yaw": yaw_cmd}

    def passo(self):
        with self.lock:
            if self.modo != "AUTO" or self.estado != EstadoMissao.BUSCA:
                return

            yaw_atual = obter_yaw_atual()
            if yaw_atual is None:
                self.mensagem = (
                    "Sem telemetria válida do giroscópio. O robô permanece parado."
                )
                parar_robo()
                return

            tag = obter_tag_alvo(self.tag_alvo)

            if self.fase not in (FaseBusca.ALINHAR, FaseBusca.RECUPERAR_TAG) and tag is not None:
                self._entrar_alinhamento(yaw_atual)

            if self.fase == FaseBusca.ALINHAR:
                comando = self._passo_alinhamento(tag, yaw_atual)
                envio_esp(comando)
                return

            if self.fase == FaseBusca.RECUPERAR_TAG:
                comando = self._passo_recuperacao(tag, yaw_atual)
                envio_esp(comando)
                return

            if self.fase == FaseBusca.AGUARDANDO_GIRO:
                self.heading_frente = yaw_atual
                self.heading_desejado = normalizar_angulo(self.heading_frente + 90.0)
                self._mudar_fase(
                    FaseBusca.GIRAR_ESQUERDA,
                    "Girando 90° para procurar a tag no lado esquerdo.",
                )
                parar_robo()
                return

            if self.fase == FaseBusca.GIRAR_ESQUERDA:
                yaw_cmd, chegou = self._comando_giro(yaw_atual, self.heading_desejado)
                envio_esp({"throttle": 0.0, "yaw": yaw_cmd})
                if chegou and self.giro_estavel >= AMOSTRAS_GIRO_ESTAVEL:
                    parar_robo()
                    self._mudar_fase(
                        FaseBusca.OBSERVAR_ESQUERDA,
                        "Observando o lado esquerdo.",
                    )
                return

            if self.fase == FaseBusca.OBSERVAR_ESQUERDA:
                parar_robo()
                if time.monotonic() - self.fase_iniciada >= TEMPO_OBSERVACAO_LADO_S:
                    self.heading_desejado = normalizar_angulo(self.heading_frente - 90.0)
                    self._mudar_fase(
                        FaseBusca.GIRAR_DIREITA,
                        "Girando 180° para procurar no lado direito.",
                    )
                return

            if self.fase == FaseBusca.GIRAR_DIREITA:
                yaw_cmd, chegou = self._comando_giro(yaw_atual, self.heading_desejado)
                envio_esp({"throttle": 0.0, "yaw": yaw_cmd})
                if chegou and self.giro_estavel >= AMOSTRAS_GIRO_ESTAVEL:
                    parar_robo()
                    self._mudar_fase(
                        FaseBusca.OBSERVAR_DIREITA,
                        "Observando o lado direito.",
                    )
                return

            if self.fase == FaseBusca.OBSERVAR_DIREITA:
                parar_robo()
                if time.monotonic() - self.fase_iniciada >= TEMPO_OBSERVACAO_LADO_S:
                    self.heading_desejado = self.heading_frente
                    self._mudar_fase(
                        FaseBusca.RETORNAR_FRENTE,
                        "Retornando 90° para o eixo central.",
                    )
                return

            if self.fase == FaseBusca.RETORNAR_FRENTE:
                yaw_cmd, chegou = self._comando_giro(yaw_atual, self.heading_desejado)
                envio_esp({"throttle": 0.0, "yaw": yaw_cmd})
                if chegou and self.giro_estavel >= AMOSTRAS_GIRO_ESTAVEL:
                    parar_robo()
                    self._mudar_fase(
                        FaseBusca.AVANCAR,
                        "Avançando aproximadamente 20 cm até a próxima fileira.",
                    )
                return

            if self.fase == FaseBusca.AVANCAR:
                decorrido = time.monotonic() - self.fase_iniciada
                if decorrido < TEMPO_AVANCO_20CM_S:
                    envio_esp({"throttle": THROTTLE_BUSCA, "yaw": 0.0})
                    return

                parar_robo()
                self.estacao += 1

                if self.estacao < NUM_ESTACOES:
                    self.heading_desejado = normalizar_angulo(self.heading_frente + 90.0)
                    self._mudar_fase(
                        FaseBusca.GIRAR_ESQUERDA,
                        f"Fileira {self.estacao + 1}: procurando primeiro à esquerda.",
                    )
                else:
                    self.heading_desejado = normalizar_angulo(self.heading_frente + 180.0)
                    self._mudar_fase(
                        FaseBusca.GIRAR_RETORNO,
                        "Fim das três fileiras. Girando 180° para repetir a busca no retorno.",
                    )
                return

            if self.fase == FaseBusca.GIRAR_RETORNO:
                yaw_cmd, chegou = self._comando_giro(yaw_atual, self.heading_desejado)
                envio_esp({"throttle": 0.0, "yaw": yaw_cmd})
                if chegou and self.giro_estavel >= AMOSTRAS_GIRO_ESTAVEL:
                    parar_robo()
                    self.heading_frente = self.heading_desejado
                    self.estacao = 0
                    self.passagem += 1
                    self.heading_desejado = normalizar_angulo(self.heading_frente + 90.0)
                    self._mudar_fase(
                        FaseBusca.GIRAR_ESQUERDA,
                        "Iniciando nova passagem no sentido contrário.",
                    )
                return

    def emitir_status_se_necessario(self, forcar=False):
        agora = time.monotonic()
        if not forcar and agora - self.ultimo_status < INTERVALO_STATUS_S:
            return
        self.ultimo_status = agora
        socketio.emit("status", self.status())


controlador = ControladorMissao()


def atualizar_comando_manual_local(comando):
    """Guarda o último comando manual para reenvio periódico ao ESP32."""
    global instante_ultimo_comando_manual

    with manual_lock:
        ultimo_comando_manual["throttle"] = max(-1.0, min(1.0, float(comando.get("throttle", 0.0))))
        ultimo_comando_manual["yaw"] = max(-1.0, min(1.0, float(comando.get("yaw", 0.0))))
        ultimo_comando_manual["fork"] = max(-1.0, min(1.0, float(comando.get("fork", 0.0))))
        instante_ultimo_comando_manual = time.monotonic()


def zerar_comando_manual_local():
    """Para o comando manual local e envia parada forçada ao ESP32."""
    atualizar_comando_manual_local({
        "throttle": 0.0,
        "yaw": 0.0,
        "fork": 0.0,
    })
    parar_robo()


def loop_reenvio_manual():
    """
    Reenvia o último comando manual em taxa fixa.

    Isso evita a falha em que o joystick fica fisicamente segurado no celular,
    mas o navegador para de emitir eventos ``move``. Sem este reenvio, o ESP32
    fica mais de 1 s sem JSON válido e ativa o timeout dos motores.
    """
    while True:
        try:
            modo = controlador.status().get("mode", "MANUAL")
            agora = time.monotonic()

            with manual_lock:
                idade = agora - instante_ultimo_comando_manual
                comando = dict(ultimo_comando_manual)

            if modo == "MANUAL" and instante_ultimo_comando_manual > 0:
                if idade <= TEMPO_MAX_COMANDO_MANUAL_SEM_ATUALIZACAO_S:
                    envio_esp(comando)
                elif (
                    abs(comando.get("throttle", 0.0)) > 0.001
                    or abs(comando.get("yaw", 0.0)) > 0.001
                    or abs(comando.get("fork", 0.0)) > 0.001
                ):
                    # Segurança: se o navegador sumir sem mandar evento end, para após alguns segundos.
                    atualizar_comando_manual_local({
                        "throttle": 0.0,
                        "yaw": 0.0,
                        "fork": 0.0,
                    })
                    parar_robo()
        except Exception as erro:
            print("Erro no loop de reenvio manual:", erro)

        time.sleep(INTERVALO_REENVIO_MANUAL_S)


# ============================================================
# SOCKET.IO
# ============================================================

@socketio.on("connect")
def handle_connect():
    print("Cliente Socket.IO conectado")
    controlador.emitir_status_se_necessario(forcar=True)


@socketio.on("disconnect")
def handle_disconnect():
    print("Cliente Socket.IO desconectado")
    zerar_comando_manual_local()


@socketio.on("set_target")
def handle_set_target(data):
    sucesso, mensagem = controlador.definir_tag(data.get("tag_id"))
    socketio.emit("target_result", {"ok": sucesso, "message": mensagem})
    controlador.emitir_status_se_necessario(forcar=True)


@socketio.on("mode")
def handle_mode(data):
    novo_modo = str(data.get("mode", "MANUAL")).upper()
    if novo_modo == "AUTO":
        atualizar_comando_manual_local({
            "throttle": 0.0,
            "yaw": 0.0,
            "fork": 0.0,
        })

    sucesso, mensagem = controlador.alterar_modo(novo_modo)
    socketio.emit("mode_result", {"ok": sucesso, "message": mensagem})
    controlador.emitir_status_se_necessario(forcar=True)




@socketio.on("reset_yaw")
def handle_reset_yaw(data=None):
    """Zera o yaw integrado do MPU6050 no ESP32."""
    envio_esp({"throttle": 0.0, "yaw": 0.0, "reset_yaw": True})
    socketio.emit("gyro_command_result", {"ok": True, "message": "Yaw zerado no ESP32."})


@socketio.on("calibrate_gyro")
def handle_calibrate_gyro(data=None):
    """Recalibra o bias do giroscópio. O robô deve estar parado."""
    envio_esp({"throttle": 0.0, "yaw": 0.0, "calibrate_gyro": True, "reset_yaw": True})
    socketio.emit("gyro_command_result", {
        "ok": True,
        "message": "Calibração do giroscópio enviada. Mantenha o robô parado por alguns segundos."
    })


@socketio.on("control")
def handle_control(data):
    # Segurança no backend: ignora joystick se o modo não for MANUAL.
    if controlador.status()["mode"] != "MANUAL":
        return

    comando = {
        "throttle": max(-1.0, min(1.0, float(data.get("throttle", 0.0)))),
        "yaw": max(-1.0, min(1.0, float(data.get("yaw", 0.0)))),
        "fork": max(-1.0, min(1.0, float(data.get("fork", 0.0)))),
    }

    atualizar_comando_manual_local(comando)

    # Envia imediatamente para reduzir latência; o loop_reenvio_manual mantém vivo.
    envio_esp(comando, forcar=True)

# ============================================================
# LEITURA DE TELEMETRIA DO ESP32
# ============================================================

LER_TELEMETRIA = True


def ler_telemetria_esp():
    while True:
        if ser is None:
            time.sleep(1.0)
            continue

        try:
            # PySerial permite leitura e escrita em threads distintas. Não seguramos
            # o lock de escrita durante readline(), pois isso atrasaria o controle.
            linha = ser.readline().decode(errors="ignore").strip()

            if not linha or not linha.startswith("{"):
                continue

            try:
                dados_esp = json.loads(linha)
            except json.JSONDecodeError:
                continue

            atualizar_telemetria_local(dados_esp)
            dados_esp["app_status"] = controlador.status()
            socketio.emit("telemetry", dados_esp)

        except Exception as erro:
            print("Erro lendo telemetria do ESP:", erro)
            time.sleep(0.1)


# ============================================================
# DETECÇÃO APRILTAG
# ============================================================

detector_tags = None
parametros_camera = None
apriltag_disponivel = False
apriltag_mensagem_erro = ""
ultimo_envio_apriltags = 0.0


def matriz_rotacao_para_angulos_euler(matriz_rotacao):
    sy = math.sqrt(matriz_rotacao[0, 0] ** 2 + matriz_rotacao[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        rotacao_lateral = math.atan2(matriz_rotacao[2, 1], matriz_rotacao[2, 2])
        inclinacao_vertical = math.atan2(-matriz_rotacao[2, 0], sy)
        giro_horizontal = math.atan2(matriz_rotacao[1, 0], matriz_rotacao[0, 0])
    else:
        rotacao_lateral = math.atan2(-matriz_rotacao[1, 2], matriz_rotacao[1, 1])
        inclinacao_vertical = math.atan2(-matriz_rotacao[2, 0], sy)
        giro_horizontal = 0.0

    return (
        math.degrees(inclinacao_vertical),
        math.degrees(giro_horizontal),
        math.degrees(rotacao_lateral),
    )


def desenhar_eixos(imagem, parametros_camera_local, matriz_rotacao, vetor_translacao):
    fx, fy, cx, cy = parametros_camera_local
    matriz_camera = np.array(
        [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32
    )
    coeficientes_distorcao = np.zeros((4, 1))
    eixo = TAMANHO_TAG_M
    pontos_eixos = np.float32(
        [[0, 0, 0], [eixo, 0, 0], [0, eixo, 0], [0, 0, eixo]]
    )

    vetor_rotacao, _ = cv2.Rodrigues(matriz_rotacao)
    pontos_imagem, _ = cv2.projectPoints(
        pontos_eixos,
        vetor_rotacao,
        vetor_translacao.astype(np.float32),
        matriz_camera,
        coeficientes_distorcao,
    )
    pontos = pontos_imagem.reshape(-1, 2).astype(int)
    origem = tuple(pontos[0])
    cv2.line(imagem, origem, tuple(pontos[1]), (0, 0, 255), 2)
    cv2.line(imagem, origem, tuple(pontos[2]), (0, 255, 0), 2)
    cv2.line(imagem, origem, tuple(pontos[3]), (255, 0, 0), 2)


def processar_tag(imagem, tag, parametros_camera_local):
    """
    Extrai apenas os dados necessários da AprilTag.
    A imagem não é desenhada nem codificada porque não há mais streaming
    de vídeo; isso reduz custo de CPU e melhora a taxa de detecção.
    """
    id_tag = int(tag.tag_id)
    matriz_rotacao = tag.pose_R
    vetor_translacao = tag.pose_t

    # pupil_apriltags fornece Z positivo para frente da câmera.
    x_mm = float(vetor_translacao[0][0] * 1000.0)
    y_mm = float(vetor_translacao[1][0] * 1000.0)
    z_mm = float(vetor_translacao[2][0] * 1000.0)

    inclinacao_vertical, giro_horizontal, rotacao_lateral = (
        matriz_rotacao_para_angulos_euler(matriz_rotacao)
    )

    familia_tag = (
        tag.tag_family.decode("utf-8")
        if isinstance(tag.tag_family, bytes)
        else str(tag.tag_family)
    )

    return {
        "id": id_tag,
        "familia": familia_tag,
        "x_mm": round(x_mm, 1),
        "y_mm": round(y_mm, 1),
        "z_mm": round(z_mm, 1),
        "giro_horizontal_graus": round(float(giro_horizontal), 2),
        "inclinacao_vertical_graus": round(float(inclinacao_vertical), 2),
        "rotacao_lateral_graus": round(float(rotacao_lateral), 2),
        "center_x": int(tag.center[0]),
        "center_y": int(tag.center[1]),
    }

def inicializar_detector_apriltags(largura_camera, altura_camera):
    global detector_tags, parametros_camera
    global apriltag_disponivel, apriltag_mensagem_erro

    if not DETECTAR_APRILTAGS:
        apriltag_mensagem_erro = "Detecção de AprilTags desativada."
        return

    if Detector is None:
        apriltag_mensagem_erro = (
            "pupil_apriltags não instalado. Rode: pip install pupil-apriltags"
        )
        print("AVISO:", apriltag_mensagem_erro)
        return

    # Estimativa inicial. Substitua pelos parâmetros reais após calibrar a câmera.
    focal_estimado = largura_camera * 1.0
    parametros_camera = [
        focal_estimado,
        focal_estimado,
        largura_camera / 2.0,
        altura_camera / 2.0,
    ]

    detector_tags = Detector(
        families=FAMILIA_TAG,
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.8,
        refine_edges=1,
    )
    apriltag_disponivel = True
    apriltag_mensagem_erro = ""
    print(f"AprilTag ativo: família={FAMILIA_TAG}, tamanho={TAMANHO_TAG_M} m")


def atualizar_memoria_apriltags(processadas):
    """
    Suaviza a exibição e o controle da AprilTag.

    Sem esta memória temporal, um único frame sem detecção fazia a interface
    alternar rapidamente entre "ID X" e "Nenhuma". Agora a tag permanece
    visível por TAG_RETENCAO_INTERFACE_S segundos após a última detecção real.
    """
    global ultimas_tags, memoria_tags, instante_ultimo_frame_tags

    agora = time.monotonic()

    for tag in processadas:
        tag_id = tag.get("id")
        if tag_id is None:
            continue

        tag_mem = dict(tag)
        tag_mem["last_seen_monotonic"] = agora
        tag_mem["last_seen_age_s"] = 0.0
        tag_mem["retained"] = False
        memoria_tags[tag_id] = tag_mem

    tags_estaveis = []
    for tag_id, tag_mem in list(memoria_tags.items()):
        idade = agora - float(tag_mem.get("last_seen_monotonic", agora))

        if idade > TAG_RETENCAO_INTERFACE_S:
            memoria_tags.pop(tag_id, None)
            continue

        tag_saida = dict(tag_mem)
        tag_saida["last_seen_age_s"] = round(idade, 2)
        tag_saida["retained"] = idade > 0.0
        tags_estaveis.append(tag_saida)

    tags_estaveis.sort(key=lambda item: item.get("id", -1))
    ultimas_tags = tags_estaveis
    instante_ultimo_frame_tags = agora

    return tags_estaveis


def detectar_e_desenhar_apriltags(frame):
    global ultimas_tags, memoria_tags, instante_ultimo_frame_tags

    if not apriltag_disponivel or detector_tags is None or parametros_camera is None:
        with visao_lock:
            ultimas_tags = []
            memoria_tags = {}
            instante_ultimo_frame_tags = time.monotonic()
        return []

    imagem_cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Equalização simples ajuda quando a iluminação oscila ou a tag está
    # com contraste baixo. Não substitui boa iluminação/foco, mas reduz
    # falhas intermitentes em frames isolados.
    imagem_cinza = cv2.equalizeHist(imagem_cinza)

    tags_detectadas = detector_tags.detect(
        imagem_cinza,
        estimate_tag_pose=True,
        camera_params=parametros_camera,
        tag_size=TAMANHO_TAG_M,
    )

    processadas = []
    for tag in tags_detectadas:
        try:
            processadas.append(processar_tag(frame, tag, parametros_camera))
        except Exception as erro:
            print("Erro ao processar AprilTag:", erro)

    with visao_lock:
        tags_estaveis = atualizar_memoria_apriltags(processadas)

    return tags_estaveis


def emitir_apriltags_para_interface(tags):
    global ultimo_envio_apriltags
    agora = time.monotonic()
    if agora - ultimo_envio_apriltags < APRILTAG_TELEMETRY_INTERVAL:
        return
    ultimo_envio_apriltags = agora

    status = controlador.status()
    tags_estaveis = status.get("target_tag_info")
    with visao_lock:
        todas_tags = list(ultimas_tags)

    socketio.emit("apriltags", {
        "count": len(todas_tags),
        "tags": todas_tags,
        "ids": status.get("detected_tag_ids", []),
        "target_tag": status.get("target_tag"),
        "target_tag_detected": status.get("target_tag_detected", False),
        "target_tag_info": tags_estaveis,
        "detector_ok": status.get("apriltag_detector_ok", False),
        "error": status.get("apriltag_error", ""),
        "age_s": status.get("apriltag_age_s"),
        "retention_s": TAG_RETENCAO_INTERFACE_S,
        "control_hold_s": IDADE_MAXIMA_DETECCAO_S,
    })


# ============================================================
# CÂMERA + APRILTAG SEM STREAM DE VÍDEO
# ============================================================

def loop_camera_apriltags():
    """
    Captura frames apenas para detectar AprilTags.
    Não envia vídeo por WebSocket e não codifica JPEG, para deixar
    o processamento da tag mais leve na Raspberry.
    """
    camera = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, LARGURA)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTURA)

    if not camera.isOpened():
        print("ERRO: não foi possível abrir a câmera.")
        return

    largura_real = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
    altura_real = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Câmera aberta para AprilTags: {largura_real}x{altura_real} sem streaming de vídeo")
    inicializar_detector_apriltags(largura_real, altura_real)

    intervalo = 1.0 / max(1, FPS)

    while True:
        inicio = time.monotonic()
        sucesso, frame = camera.read()

        if sucesso:
            tags = detectar_e_desenhar_apriltags(frame)
            emitir_apriltags_para_interface(tags)
        else:
            time.sleep(0.1)

        tempo_gasto = time.monotonic() - inicio
        espera = intervalo - tempo_gasto
        if espera > 0:
            time.sleep(espera)


# ============================================================
# LOOP AUTÔNOMO
# ============================================================

def loop_autonomo():
    while True:
        try:
            controlador.passo()
            controlador.emitir_status_se_necessario()
        except Exception as erro:
            print("Erro no loop autônomo:", erro)
            parar_robo()
        time.sleep(INTERVALO_LOOP_AUTONOMO_S)


# ============================================================
# INÍCIO
# ============================================================

if __name__ == "__main__":
    ser = abrir_serial_esp()

    thread_camera = threading.Thread(
        target=loop_camera_apriltags, daemon=True
    )
    thread_camera.start()

    if LER_TELEMETRIA:
        thread_telemetria = threading.Thread(
            target=ler_telemetria_esp, daemon=True
        )
        thread_telemetria.start()

    thread_manual = threading.Thread(target=loop_reenvio_manual, daemon=True)
    thread_manual.start()

    thread_auto = threading.Thread(target=loop_autonomo, daemon=True)
    thread_auto.start()

    socketio.run(
        app,
        host="0.0.0.0",
        port=5000,
        debug=False,
        use_reloader=False,
    )
