import asyncio
import math
import threading
import time

import cv2
import numpy as np
import websockets
from flask import Flask, render_template
from flask_socketio import SocketIO

try:
    from pupil_apriltags import Detector
except ImportError:
    Detector = None


# ============================================================
# FLASK + SOCKET.IO
# ============================================================
# Flask entrega a página web.
# Socket.IO recebe os controles dos joysticks e envia status/telemetria.
# WebSocket puro, em outra porta, transmite o vídeo da câmera.
# ============================================================

app = Flask(__name__)
app.config["SECRET_KEY"] = "secret"

socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

modo = "MANUAL"


@app.route("/")
def index():
    return render_template("index.html")


@socketio.on("connect")
def handle_connect():
    print("Cliente Socket.IO conectado")
    socketio.emit("status", {"mode": modo})


@socketio.on("disconnect")
def handle_disconnect():
    print("Cliente Socket.IO desconectado")


@socketio.on("control")
def handle_control(data):
    """
    Recebe os dados dos dois joysticks.

    Exemplo:
    {
        "throttle": 0.7,
        "yaw": -0.2
    }
    """
    print("Controle recebido:", data)

    # Aqui, futuramente, você pode enviar esses valores para Arduino,
    # ESP32, Raspberry, ponte H, controle de motor etc.


@socketio.on("mode")
def handle_mode(data):
    global modo

    modo = data.get("mode", modo)
    print(f"Modo alterado para: {modo}")
    socketio.emit("status", {"mode": modo})


# ============================================================
# CONFIGURAÇÕES DA CÂMERA / VÍDEO
# ============================================================

CAMERA_ID = 0
VIDEO_WS_HOST = "0.0.0.0"
VIDEO_WS_PORT = 8765

LARGURA = 640 
ALTURA = 480
FPS = 8
QUALIDADE_JPEG = 95

video_clientes = set()


# ============================================================
# CONFIGURAÇÕES DA DETECÇÃO APRILTAG
# Baseadas no arquivo camera.py enviado.
# ============================================================

DETECTAR_APRILTAGS = True
TAMANHO_TAG = 0.048
FAMILIA_TAG = "tag25h9"
APRILTAG_TELEMETRY_INTERVAL = 0.20

ultimo_envio_apriltags = 0.0
detector_tags = None
parametros_camera = None
apriltag_disponivel = False
apriltag_mensagem_erro = ""


# ============================================================
# FUNÇÕES DE APRILTAG — ADAPTADAS DO camera.py
# ============================================================

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
        giro_horizontal = 0

    inclinacao_vertical = math.degrees(inclinacao_vertical)
    giro_horizontal = math.degrees(giro_horizontal)
    rotacao_lateral = math.degrees(rotacao_lateral)

    return inclinacao_vertical, giro_horizontal, rotacao_lateral


def desenhar_borda_tag(imagem, tag):
    pontos = np.array(tag.corners, dtype=np.int32)
    cv2.polylines(imagem, [pontos], True, (0, 255, 0), 2)


def desenhar_centro_tag(imagem, tag):
    centro_x = int(tag.center[0])
    centro_y = int(tag.center[1])
    cv2.circle(imagem, (centro_x, centro_y), 5, (0, 0, 255), -1)


def desenhar_texto(imagem, texto, x, y, cor=(0, 0, 255)):
    cv2.putText(
        imagem,
        texto,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        cor,
        2,
        cv2.LINE_AA
    )


def desenhar_eixos(imagem, parametros_camera_local, matriz_rotacao, vetor_translacao, tamanho_tag):
    fx, fy, cx, cy = parametros_camera_local

    matriz_camera = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ], dtype=np.float32)

    coeficientes_distorcao = np.zeros((4, 1))
    comprimento_eixo = tamanho_tag

    pontos_eixos = np.float32([
        [0, 0, 0],
        [comprimento_eixo, 0, 0],
        [0, comprimento_eixo, 0],
        [0, 0, -comprimento_eixo]
    ])

    vetor_rotacao, _ = cv2.Rodrigues(matriz_rotacao)
    vetor_translacao = vetor_translacao.astype(np.float32)

    pontos_imagem, _ = cv2.projectPoints(
        pontos_eixos,
        vetor_rotacao,
        vetor_translacao,
        matriz_camera,
        coeficientes_distorcao
    )

    pontos_imagem = pontos_imagem.reshape(-1, 2).astype(int)

    origem = tuple(pontos_imagem[0])
    eixo_x = tuple(pontos_imagem[1])
    eixo_y = tuple(pontos_imagem[2])
    eixo_z = tuple(pontos_imagem[3])

    cv2.line(imagem, origem, eixo_x, (0, 0, 255), 3)
    cv2.line(imagem, origem, eixo_y, (0, 255, 0), 3)
    cv2.line(imagem, origem, eixo_z, (255, 0, 0), 3)


def processar_tag(imagem, tag, parametros_camera_local):
    """
    Desenha a tag no frame e devolve os dados calculados para a interface web.
    """
    desenhar_borda_tag(imagem, tag)
    desenhar_centro_tag(imagem, tag)

    matriz_rotacao = tag.pose_R
    vetor_translacao = tag.pose_t

    x = vetor_translacao[0][0] * 1000.0
    y = vetor_translacao[1][0] * 1000.0
    z = vetor_translacao[2][0] * 1000.0

    z_calibrado = z * 0.94

    inclinacao_vertical, giro_horizontal, rotacao_lateral = matriz_rotacao_para_angulos_euler(
        matriz_rotacao
    )

    rad_inclinacao_vertical = math.radians(inclinacao_vertical)
    rad_giro_horizontal = math.radians(giro_horizontal)

    motor_x = x - z * math.sin(rad_giro_horizontal)
    motor_y = y + z * math.sin(rad_inclinacao_vertical)
    motor_z = z * math.cos(rad_giro_horizontal) - x * math.sin(rad_giro_horizontal)

    familia_tag = tag.tag_family.decode("utf-8") if isinstance(tag.tag_family, bytes) else str(tag.tag_family)
    id_tag = int(tag.tag_id)

    x_texto = int(tag.corners[0][0])
    y_texto = int(tag.corners[0][1]) - 10

    # Evita escrever texto fora do topo da imagem.
    y_texto = max(y_texto, 25)

    desenhar_texto(
        imagem,
        f"ID:{id_tag} | X:{x:.0f} Y:{y:.0f} Z:{z_calibrado:.0f} mm",
        x_texto,
        y_texto
    )


    desenhar_eixos(
        imagem,
        parametros_camera_local,
        matriz_rotacao,
        vetor_translacao,
        TAMANHO_TAG
    )

    print(
        f"Tag detectada | família={familia_tag} | ID={id_tag} | "
        f"Motor XYZ=({int(motor_x)}, {int(motor_y)}, {int(motor_z)})"
    )

    return {
        "id": id_tag,
        "familia": familia_tag,
        "x_mm": round(float(x), 1),
        "y_mm": round(float(y), 1),
        "z_mm": round(float(z), 1),
        "z_calibrado_mm": round(float(z_calibrado), 1),
        "motor_x_mm": round(float(motor_x), 1),
        "motor_y_mm": round(float(motor_y), 1),
        "motor_z_mm": round(float(motor_z), 1),
        "giro_horizontal_graus": round(float(giro_horizontal), 2),
        "inclinacao_vertical_graus": round(float(inclinacao_vertical), 2),
        "rotacao_lateral_graus": round(float(rotacao_lateral), 2),
        "center_x": int(tag.center[0]),
        "center_y": int(tag.center[1])
    }


def inicializar_detector_apriltags(largura_camera, altura_camera):
    """
    Inicializa o detector pupil_apriltags e calcula parâmetros estimados da câmera.
    """
    global detector_tags
    global parametros_camera
    global apriltag_disponivel
    global apriltag_mensagem_erro

    if not DETECTAR_APRILTAGS:
        apriltag_mensagem_erro = "Detecção de AprilTags desativada."
        apriltag_disponivel = False
        return

    if Detector is None:
        apriltag_mensagem_erro = "pupil_apriltags não instalado. Rode: py -m pip install pupil-apriltags"
        apriltag_disponivel = False
        print("AVISO:", apriltag_mensagem_erro)
        return

    focal_estimado = largura_camera * 1.0

    parametros_camera = [
        focal_estimado,
        focal_estimado,
        largura_camera / 2,
        altura_camera / 2
    ]

    detector_tags = Detector(
        families=FAMILIA_TAG,
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.8,
        refine_edges=1
    )

    apriltag_disponivel = True
    apriltag_mensagem_erro = ""

    print(f"Detector AprilTag ativado: família={FAMILIA_TAG}, tamanho={TAMANHO_TAG} m")
    print(f"Parâmetros estimados da câmera: {parametros_camera}")


def detectar_e_desenhar_apriltags(frame):
    """
    Detecta AprilTags com pupil_apriltags, desenha no frame e retorna dados JSON.
    """
    if not DETECTAR_APRILTAGS:
        return []

    if not apriltag_disponivel or detector_tags is None or parametros_camera is None:
        cv2.putText(
            frame,
            "AprilTag OFF",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
            cv2.LINE_AA
        )

        if apriltag_mensagem_erro:
            cv2.putText(
                frame,
                apriltag_mensagem_erro[:70],
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 255),
                1,
                cv2.LINE_AA
            )

        return []

    imagem_cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    tags_detectadas = detector_tags.detect(
        imagem_cinza,
        estimate_tag_pose=True,
        camera_params=parametros_camera,
        tag_size=TAMANHO_TAG
    )

    if len(tags_detectadas) == 0:
        cv2.putText(
            frame,
            "Nenhuma tag encontrada",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
            cv2.LINE_AA
        )
        return []

    tags_processadas = []

    for tag in tags_detectadas:
        try:
            tags_processadas.append(processar_tag(frame, tag, parametros_camera))
        except Exception as erro:
            print("Erro ao processar AprilTag:", erro)

    return tags_processadas


def emitir_apriltags_para_interface(tags):
    global ultimo_envio_apriltags

    agora = time.time()
    if agora - ultimo_envio_apriltags < APRILTAG_TELEMETRY_INTERVAL:
        return

    ultimo_envio_apriltags = agora

    socketio.emit("apriltags", {
        "count": len(tags),
        "tags": tags
    })


# ============================================================
# WEBSOCKET PURO PARA TRANSMISSÃO DO VÍDEO
# ============================================================

async def conectar_cliente_video(websocket, *args):
    print("Cliente de vídeo conectado")
    video_clientes.add(websocket)

    try:
        await websocket.wait_closed()
    finally:
        video_clientes.discard(websocket)
        print("Cliente de vídeo desconectado")


async def transmitir_camera():
    camera = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, LARGURA)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTURA)

    if not camera.isOpened():
        print("ERRO: não foi possível abrir a câmera.")
        print("Tente mudar CAMERA_ID para 1 ou 2 se tiver mais de uma câmera.")
        return

    largura_real = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
    altura_real = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"Câmera aberta com sucesso: {largura_real}x{altura_real}")

    inicializar_detector_apriltags(largura_real, altura_real)

    while True:
        sucesso, frame = camera.read()

        if not sucesso:
            print("Erro ao capturar frame da câmera.")
            await asyncio.sleep(0.1)
            continue

        tags = detectar_e_desenhar_apriltags(frame)
        emitir_apriltags_para_interface(tags)

        sucesso, buffer = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), QUALIDADE_JPEG]
        )

        if sucesso and video_clientes:
            imagem_bytes = buffer.tobytes()
            clientes_atuais = list(video_clientes)

            resultados = await asyncio.gather(
                *[cliente.send(imagem_bytes) for cliente in clientes_atuais],
                return_exceptions=True
            )

            for cliente, resultado in zip(clientes_atuais, resultados):
                if isinstance(resultado, Exception):
                    video_clientes.discard(cliente)

        await asyncio.sleep(1 / FPS)


async def servidor_video():
    async with websockets.serve(
        conectar_cliente_video,
        VIDEO_WS_HOST,
        VIDEO_WS_PORT,
        max_size=None
    ):
        print(f"WebSocket de vídeo rodando em ws://0.0.0.0:{VIDEO_WS_PORT}")
        await transmitir_camera()
        

def pid_eixo_x():
    return 

def pid_eixo_z():
    return 
    
    
def iniciar_servidor_video_em_thread():
    asyncio.run(servidor_video())


# ============================================================
# INÍCIO DO PROGRAMA
# ============================================================

if __name__ == "__main__":
    thread_video = threading.Thread(
        target=iniciar_servidor_video_em_thread,
        daemon=True
    )
    thread_video.start()

    socketio.run(
        app,
        host="0.0.0.0",
        port=5000,
        debug=False,
        use_reloader=False
    )
