

import pybullet as p
import pybullet_data
import numpy as np
import time
import pickle
import json
import os
import math
import random
import traceback
import io
import base64
import matplotlib.pyplot as plt 

# --- CONFIGURAÇÃO DE MAPA E VISUAL ---
random.seed(42) # Mude este número para gerar um layout diferente
MAP_SIZE = 100           
MAP_METERS = 20.0        
MAP_FILENAME = "mapa_v27_cheio.pkl"
SIM_STEP = 1.0 / 240.0

# FÍSICA
FORCE_HUSKY = 15000      
BASE_SPEED = 25.0        
TURN_SPEED = 12.0        

# SENSORES
SENSOR_ANGLES = [-0.6, -0.3, 0.0, 0.3, 0.6]  
SENSOR_MAX_RANGE = 6.0   
OBSTACLE_THRESHOLD = 2.5 

# MQTT
MQTT_BROKER = "localhost"
try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False

# --- FUNÇÕES ---

def world_to_cell(pos):
    x, y = pos[0], pos[1]
    i = int(((x + MAP_METERS/2) / MAP_METERS) * MAP_SIZE)
    j = int(((y + MAP_METERS/2) / MAP_METERS) * MAP_SIZE)
    return max(0, min(MAP_SIZE - 1, i)), max(0, min(MAP_SIZE - 1, j))

def cell_to_world(i, j):
    x = (i / MAP_SIZE) * MAP_METERS - (MAP_METERS/2)
    y = (j / MAP_SIZE) * MAP_METERS - (MAP_METERS/2)
    return [x, y]

def carregar_memoria():
    mapa = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.int32)
    existe = False
    if os.path.exists(MAP_FILENAME):
        try:
            with open(MAP_FILENAME, "rb") as f: mapa = pickle.load(f)
            existe = True
        except: pass
    return mapa, existe

def gerar_plano_de_limpeza(mapa):
    waypoints = []
    step = 5 
    for j in range(0, MAP_SIZE, step):
        linha_range = range(0, MAP_SIZE, step) if (j//step)%2==0 else range(MAP_SIZE-1, -1, -step)
        for i in linha_range:
            if mapa[i, j] == 50: 
                seguro = True
                for dx in [-2, 0, 2]:
                    for dy in [-2, 0, 2]:
                        if mapa[min(max(i+dx,0),99), min(max(j+dy,0),99)] >= 100: seguro = False
                if seguro:
                    wx, wy = cell_to_world(i, j)
                    waypoints.append([wx, wy])
    return waypoints

class RoboComunicador:
    def __init__(self):
        self.client = None
        if MQTT_AVAILABLE:
            try:
                self.client = mqtt.Client()
                self.client.connect(MQTT_BROKER, 1883, 60)
                self.client.loop_start()
            except: self.client = None
    
    def enviar_telemetria(self, dados):
        if self.client: self.client.publish("aspirador/telemetria", json.dumps(dados))

    def enviar_imagem(self, mapa, trajetoria):
        if not self.client: return
        try:
            plt.figure(figsize=(5,5))
            plt.imshow(mapa.T, origin='lower', cmap='hot', interpolation='nearest', vmin=0, vmax=100)
            
            xs, ys = [], []
            for x, y in trajetoria:
                i, j = world_to_cell([x, y])
                xs.append(i); ys.append(j)
            plt.plot(xs, ys, 'c-', linewidth=2)
            plt.axis('off') 
            
            buf = io.BytesIO()
            plt.savefig(buf, format='png', bbox_inches='tight', pad_inches=0)
            
            # --- SALVA NO PC TAMBÉM (OPCIONAL) ---
            plt.savefig("ultimo_mapa.png", bbox_inches='tight', pad_inches=0)
            # -------------------------------------

            plt.close()
            buf.seek(0)
            img_str = base64.b64encode(buf.getvalue()).decode('utf-8')
            self.client.publish("aspirador/mapa_imagem", img_str)
        except: pass

def ler_sensores(robot_id):
    try:
        pos, orn = p.getBasePositionAndOrientation(robot_id)
        yaw = p.getEulerFromQuaternion(orn)[2]
        origins, targets = [], []
        for angle in SENSOR_ANGLES:
            start_x = pos[0] + 0.4 * math.cos(yaw)
            start_y = pos[1] + 0.4 * math.sin(yaw)
            end_x = pos[0] + SENSOR_MAX_RANGE * math.cos(yaw + angle)
            end_y = pos[1] + SENSOR_MAX_RANGE * math.sin(yaw + angle)
            origins.append([start_x, start_y, pos[2]+0.2])
            targets.append([end_x, end_y, pos[2]+0.2])
        
        results = p.rayTestBatch(origins, targets)
        distancias, pontos = [], []
        for i, res in enumerate(results):
            hit = res[2]
            distancias.append(hit * SENSOR_MAX_RANGE)
            color = [1, 0, 0] if hit < 1.0 else [0, 1, 0]
            p.addUserDebugLine(origins[i], res[3] if hit < 1.0 else targets[i], color, lifeTime=0.1)
            if hit < 1.0: pontos.append(res[3])
        return distancias, pontos, pos, yaw
    except: return [], [], [0,0,0], 0

def criar_ambiente_arena():
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.8)
    p.loadURDF("plane.urdf", globalScaling=3.0) 
    
    def wall(pos, dim):
        c = p.createCollisionShape(p.GEOM_BOX, halfExtents=dim)
        v = p.createVisualShape(p.GEOM_BOX, halfExtents=dim, rgbaColor=[0.4,0.4,0.4,1])
        p.createMultiBody(0, c, v, basePosition=pos)

    wall([0, 10, 1], [10.2, 0.2, 1])
    wall([0, -10, 1], [10.2, 0.2, 1])
    wall([10, 0, 1], [0.2, 10.2, 1])
    wall([-10, 0, 1], [0.2, 10.2, 1])
    
    # --- AQUI VOCÊ CONTROLA A QUANTIDADE ---
    print("Gerando Arena LOTADA...")
    quantidade_obstaculos = 20  # <--- MUDE AQUI PARA MAIS OU MENOS (Ex: 5, 20, 50)

    for i in range(quantidade_obstaculos):   
        while True:
            pos_x = random.uniform(-8, 8)
            pos_y = random.uniform(-8, 8)
            # Garante que não nasça em cima do robô
            if math.sqrt((pos_x - (-8))**2 + (pos_y - (-8))**2) > 3.0: break
        
        if random.random() > 0.5:
            c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.4,0.4,0.5])
            v = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.4,0.4,0.5], rgbaColor=[random.random(),0.5,0.5,1])
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=c, baseVisualShapeIndex=v, basePosition=[pos_x, pos_y, 0.5])
        else:
            c = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.3, height=1.0)
            v = p.createVisualShape(p.GEOM_CYLINDER, radius=0.3, length=1.0, rgbaColor=[0.5,random.random(),0.5,1])
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=c, baseVisualShapeIndex=v, basePosition=[pos_x, pos_y, 0.5])
            
    p.loadURDF("duck_vhacd.urdf", [0, 0, 0.5], globalScaling=3.0, useFixedBase=True)
    print("Arena Pronta.")

def calcular_controle(robot_pos, robot_yaw, target_pos):
    dx = target_pos[0] - robot_pos[0]
    dy = target_pos[1] - robot_pos[1]
    dist = math.sqrt(dx**2 + dy**2)
    ang = math.atan2(dy, dx) - robot_yaw
    while ang > 3.14: ang -= 6.28
    while ang < -3.14: ang += 6.28
    turn = ang * 5.0 
    fwd = BASE_SPEED
    if abs(ang) > 0.5: fwd = BASE_SPEED * 0.6 
    return fwd - turn, fwd + turn, dist

def main():
    try:
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(18, 0, -89, [0,0,0])
        criar_ambiente_arena()
        
        robot = p.loadURDF("husky/husky.urdf", [-8, -8, 0.1], [0,0,0,1])
        p.changeDynamics(robot, -1, activationState=p.ACTIVATION_STATE_DISABLE_SLEEPING)

        mapa, memoria_existe = carregar_memoria()
        rota_inteligente = []
        if memoria_existe: rota_inteligente = gerar_plano_de_limpeza(mapa)
        modo = "AUTO" if (memoria_existe and len(rota_inteligente) > 10) else "EXPLORAR"

        comunicador = RoboComunicador()
        trajetoria_atual = []
        passos = 0
        energia = 0.0
        area_coberta = 0
        wp_index = 0
        tempo_re = 0
        tempo_giro = 0
        last_pos_check = [0,0,0]
        stuck_counter = 0
        
        while p.isConnected():
            dists, obstaculos, pos, yaw = ler_sensores(robot)
            if not dists: continue
            min_dist = min(dists)
            
            ix, iy = world_to_cell(pos)
            if mapa[ix, iy] == 0: 
                mapa[ix, iy] = 50 
                area_coberta += 1
            trajetoria_atual.append([pos[0], pos[1]])
            
            for obs in obstaculos:
                ox, oy = world_to_cell(obs)
                if 0 <= ox < MAP_SIZE and 0 <= oy < MAP_SIZE: mapa[ox, oy] = 100 
            
            vl, vr = 0, 0
            if passos % 30 == 0: 
                delta = math.sqrt((pos[0]-last_pos_check[0])**2 + (pos[1]-last_pos_check[1])**2)
                last_pos_check = pos
                if delta < 0.1: stuck_counter += 1
                else: stuck_counter = 0
                if stuck_counter > 2: modo, tempo_re, stuck_counter = "ESCAPE", 40, 0

            if modo == "ESCAPE":
                vl, vr = -15.0, -20.0 
                tempo_re -= 1
                if tempo_re <= 0: modo = "EXPLORAR"
            elif min_dist < OBSTACLE_THRESHOLD:
                if modo in ["AUTO", "EXPLORAR"]: modo, tempo_re = "RE", 20 
            elif modo == "RE":
                vl, vr = -10.0, -10.0 
                tempo_re -= 1
                if tempo_re <= 0: modo, tempo_giro = "GIRAR", random.randint(15, 40)
            elif modo == "GIRAR":
                vl, vr = -TURN_SPEED, TURN_SPEED
                tempo_giro -= 1
                if tempo_giro <= 0: modo = "AUTO" if len(rota_inteligente) > 0 else "EXPLORAR"
            elif modo == "EXPLORAR":
                vl, vr = BASE_SPEED, BASE_SPEED
                if random.random() < 0.05: modo, tempo_giro = "GIRAR", 20
            elif modo == "AUTO":
                if wp_index < len(rota_inteligente):
                    alvo = rota_inteligente[wp_index]
                    p.addUserDebugLine([pos[0],pos[1],0.5], [alvo[0],alvo[1],0.5], [0,1,0], 2.0)
                    vl, vr, dist = calcular_controle(pos, yaw, alvo)
                    if dist < 1.2: wp_index += 1
                else: vl, vr = 0, 0 

            for j, v in zip([2,4,3,5], [vl,vl,vr,vr]):
                p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, targetVelocity=v, force=FORCE_HUSKY)
            
            passos += 1
            energia += abs(vl) + abs(vr)
            
            if passos % 50 == 0:
                dados = {
                    "modo": modo,
                    "area_m2": round(area_coberta * 0.04, 2),
                    "energia": int(energia),
                    "progresso_rota": f"{wp_index}/{len(rota_inteligente)}" if modo == "AUTO" else "N/A",
                    "posicao": {"x": float(pos[0]), "y": float(pos[1])}
                }
                comunicador.enviar_telemetria(dados)
            
            if passos % 200 == 0:
                comunicador.enviar_imagem(mapa, trajetoria_atual)

            p.stepSimulation()
            time.sleep(SIM_STEP)
            
    except Exception: print(traceback.format_exc())
    finally:
        try:
            with open(MAP_FILENAME, "wb") as f: pickle.dump(mapa, f)
        except: pass
        try: p.disconnect()
        except: pass

if __name__ == "__main__":
    main()
