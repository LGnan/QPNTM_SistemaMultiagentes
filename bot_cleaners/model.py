from mesa.model import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector
from random import choice
import numpy as np
import heapq

from .AgenteMover import (
    AgenteMover,
    
)

from .AgenteRecoger import (
    AgenteRecoger,
    
)



from .Agentes import (
    Celda,
    EstanteriaChica,
    EstacionCarga,
    EstanteriaGrande,
    Cinta,
    Cinta2,
)


class Habitacion(Model):
    def __init__(
        self,
        M: int,
        N: int,
        num_agentes: int = 4,
        rate_packages: int =10,
        porc_celdas_sucias: float = 0.6,
        porc_muebles: float = 0.1,
        modo_pos_inicial: str = "Fija",
        step_counter=0,
    ):
        self.datacollector = DataCollector(
            model_reporters={
                "Grid": get_grid,
                "Cargas": get_cargas,
                "CeldasSucias": get_sucias,
            },
        )

        self.num_agentes = num_agentes
        self.porc_celdas_sucias = porc_celdas_sucias
        self.porc_muebles = porc_muebles
        self.step_counter = step_counter
        self.rate_packages = rate_packages
        self.id_counter = 1
        self.grid = MultiGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)

        posiciones_disponibles = [pos for _, pos in self.grid.coord_iter()]

        # Posiciones de cinta
        posiciones_cintas = []
        for y in [14, 9]:
            for x in [28, 27, 26, 25, 24]:
                posiciones_cintas.append((x, y))

        posiciones_cintas2 = []
        for y in [14, 9]:
            for x in [1, 2, 3]:
                posiciones_cintas2.append((x, y))

        posiciones_estanteriasG = []
        valores_x = [8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]
        valores_y = [22, 21, 12, 11, 2, 1]
        for y in valores_y:
            for x in valores_x:
                posiciones_estanteriasG.append((x, y))

        posiciones_dispCacaPipi = []
        v_x1 = [23, 23, 24, 25, 26, 27, 28]
        v_y1 = [9, 5, 6, 7]
        for y in v_y1:
            for x in v_x1:
                posiciones_dispCacaPipi.append((x, y))
        v_i = [(23, 14), (23, 9)]
        #
        for coord in v_i:
            posiciones_dispCacaPipi.append(coord)

        for id, pos in enumerate(posiciones_cintas):
            cinta = Cinta(int(f"6200{id}") + 1, self)
            self.grid.place_agent(cinta, pos)
            self.schedule.add(cinta)
            posiciones_disponibles.remove(pos)

        for id, pos in enumerate(posiciones_cintas2):
            cinta2 = Cinta2(int(f"8200{id}") + 1, self)
            self.grid.place_agent(cinta2, pos)
            self.schedule.add(cinta2)
            posiciones_disponibles.remove(pos)

        for id, pos in enumerate(posiciones_estanteriasG):
            estanteria = EstanteriaGrande(int(f"7100{id}") + 1, self)
            self.grid.place_agent(estanteria, pos)
            self.schedule.add(estanteria)
            posiciones_disponibles.remove(pos)

        # Posicionamiento de muebless
        num_muebles = int(M * N * porc_muebles)

        estanterias_chicas_pos = []
        for y in [18, 15, 8, 5]:
            for x in range(8, 19):
                estanterias_chicas_pos.append((x, y))

        for id, pos in enumerate(estanterias_chicas_pos):
            estanteria_chica = EstanteriaChica(int(f"1000{id}") + 1, self)
            self.grid.place_agent(estanteria_chica, pos)
            self.schedule.add(estanteria_chica)
            posiciones_disponibles.remove(pos)

        posiciones_estaciones = [(8, 0), (18, 0), (8, 23), (18, 23)]
        for id, pos in enumerate(posiciones_estaciones):
            estacion = EstacionCarga(int(f"5000{id}") + 1, self)
            self.grid.place_agent(estacion, pos)
            self.schedule.add(estacion)  # Añadir la estación al schedules
            if pos in posiciones_disponibles:
                posiciones_disponibles.remove(pos)

        print(f"{len(posiciones_estaciones)} estaciones de carga creadas.")

        # posiciones_dispCacaPipi = [pos for _, *pos in self.grid.coord_iter() if condicion(pos)]

        self.datacollector.collect(self)

        self.schedule.step()

        
        self.num_celdas_sucias = int(4 * 10 * porc_celdas_sucias)


        new_dirty_cell = Celda(3000, self)
        new_dirty_cell.sucia = True  # Suponiendo que el atributo para suciedad se llama 'sucia'
        self.schedule.add(new_dirty_cell)
        self.grid.place_agent(new_dirty_cell, (23, 14))
        new_dirty_cell = Celda(4000, self)
        new_dirty_cell.sucia = True  # Suponiendo que el atributo para suciedad se llama 'sucia'
        self.schedule.add(new_dirty_cell)
        self.grid.place_agent(new_dirty_cell, (23, 9))
        

        # Posicionamiento de agentes robot
        if modo_pos_inicial == "Aleatoria":
            pos_inicial_robots = self.random.sample(
                posiciones_disponibles, k=num_agentes
            )
        else:  # 'Fija'
            pos_inicial_robots = [(21, 14), (21
            
            , 9)] 

        for id in range(min(num_agentes, len(pos_inicial_robots))):
            print(f"Length of pos_inicial_robots: {len(pos_inicial_robots)}")  # Debug line
            print(f"Current id: {id}")
            robot = AgenteMover(id, self)
            self.grid.place_agent(robot, pos_inicial_robots[id])
            self.schedule.add(robot)
        num_agentes2= num_agentes*2
        if modo_pos_inicial == "Aleatoria":
            pos_inicial_robots = self.random.sample(
                posiciones_disponibles, k=num_agentes2
            )
        else:  # 'Fija'
            pos_inicial_robots = [(6, 14), (6, 9)] 
        
        for id in range( min(num_agentes2, len(pos_inicial_robots))):
            robot = AgenteRecoger(id+num_agentes, self)
            self.grid.place_agent(robot, pos_inicial_robots[id])
            self.schedule.add(robot)

        self.datacollector = DataCollector(
            model_reporters={
                "Grid": get_grid,
                "Cargas": get_cargas,
                "CeldasSucias": get_sucias,
            },
        )

    def mover_robots_a_estaciones(self):
        estaciones = self.get_estaciones()
        for robot in self.schedule.agents:
            if isinstance(robot, AgenteMover):
                estacion_cercana = min(
                    estaciones,
                    key=lambda pos: robot.distancia_euclidiana(robot.pos, pos),
                )
                robot.mover_hacia_estacion(estacion_cercana)

    
    def step(self):
        self.datacollector.collect(self)

        self.schedule.step()

         # Suponiendo que la ubicación específica es (x, y)
        x, y = 23,14   # Reemplaza con las coordenadas deseadas
        x2, y2 = 23,9
        self.step_counter += 1

        cajas_cinta1 = []
        cajas_cinta2 = []

        if self.step_counter%self.rate_packages == 0:

            new_dirty_cell = Celda(3001+self.id_counter, self)# Usar un ID fijo
        
            new_dirty_cell.sucia = True
            self.schedule.add(new_dirty_cell)
            self.grid.place_agent(new_dirty_cell, (x, y))
            
            new_dirty_cell = Celda(4001+self.id_counter, self) # Usar otro ID fijo

            new_dirty_cell.sucia = True
            self.schedule.add(new_dirty_cell)
            self.grid.place_agent(new_dirty_cell, (x2, y2))
            self.id_counter+=1


    def todoLimpio(self):
        for content, x, y in self.grid.coord_iter():
            for obj in content:
                if isinstance(obj, Celda) and obj.sucia:
                    return False
        return True


def get_grid(model: Model) -> np.ndarray:
    grid = np.zeros((model.grid.width, model.grid.height))
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        x, y = pos
        for obj in cell_content:
            if isinstance(obj, AgenteMover):
                grid[x][y] = 2
            elif isinstance(obj, Celda):
                grid[x][y] = int(obj.sucia)
    return grid


def get_cargas(model: Model):
    return [
        (agent.unique_id, agent.carga)
        for agent in model.schedule.agents
        if isinstance(agent, AgenteMover)
    ]


def get_sucias(model: Model) -> int:
    sum_sucias = 0
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        for obj in cell_content:
            if isinstance(obj, Celda) and obj.sucia:
                sum_sucias += 1
    return sum_sucias / model.porc_celdas_sucias


def get_movimientos(agent: Agent) -> dict:
    if isinstance(agent, AgenteMover):
        return {agent.unique_id: agent.movimientos}
