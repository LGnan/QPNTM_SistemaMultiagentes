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

from .Agentes import (
    Celda,
    EstanteriaChica,
    EstacionCarga,
    EstanteriaGrande,
    Cinta,
    Cinta2
)

class AgenteRecoger(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.color = choice(["Gray"])
        self.sig_pos = None
        self.movimientos = 0
        self.carga = 100
        self.previous_pos = None
        self.tiempo_en_estacion = 0
        self.enCarga = False
        
    def distancia_euclidiana(self, pos1, pos2):
        x1, y1 = pos1
        x2, y2 = pos2
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    
    def distancia_manhattan(self, pos1, pos2):
        if pos2 is None:
            return float("inf")
        x1, y1 = pos1
        x2, y2 = pos2
        return abs(x2 - x1) + abs(y2 - y1)    
    
    def mover(self, cinta_pos):
        if cinta_pos is None:
            return
        path = self.a_star(self.pos, cinta_pos)

        if path:
            next_pos = path[0]
            self.model.grid.move_agent(self, next_pos)
            self.pos = next_pos
            self.sig_pos = next_pos

    def estacion_carga_mas_cercana(self):
        estaciones = [
            agent.pos
            for agent in self.model.schedule.agents
            if isinstance(agent, EstacionCarga)
        ]

        if not estaciones:
            return None  # si no hay estaciones d carga

        # calcula la distancia entre los puntos del robot actual y la de cada posicion d carga (lista) y agarra el minimo4
        estacion_cercana = min(
            estaciones, key=lambda pos: self.distancia_euclidiana(self.pos, pos)
        )

        return estacion_cercana

    def cinta2_mas_cercana(self):
        cintas = [
            agent.pos
            for agent in self.model.schedule.agents
            if isinstance(agent, Cinta2)
        ]

        if not cintas:
            return None

        cinta_cercana = min(
            cintas, key=lambda pos: self.distancia_euclidiana(self.pos, pos)
        )

        return cinta_cercana

    def a_star(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}

        while open_list:
            current_cost, current = heapq.heappop(open_list)
            if current == goal:
                path = []
                while current in came_from:
                    path.insert(0, current)
                    current = came_from[current]
                return path

            for next_cell in self.model.grid.get_neighborhood(
                current, moore=True, include_center=False
            ):
                if any(
                    isinstance(agent, AgenteMover)
                    for agent in self.model.grid.get_cell_list_contents([next_cell])
                ):
                    continue

                if any(
                    isinstance(agent, EstanteriaChica) and agent.enUso == True
                    for agent in self.model.grid.get_cell_list_contents([next_cell])
                ):
                    continue

                if any(
                    isinstance(agent, EstanteriaGrande)
                    for agent in self.model.grid.get_cell_list_contents([next_cell])
                ):
                    continue

                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score.get(next_cell, float("inf")):
                    came_from[next_cell] = current
                    g_score[next_cell] = tentative_g_score
                    f_score = tentative_g_score + self.distancia_manhattan(
                        next_cell, goal
                    )
                    heapq.heappush(open_list, (f_score, next_cell))

        return None
    
    def step(self):
            
        cinta_cerca = self.cinta2_mas_cercana()
        estacion_pos = self.estacion_carga_mas_cercana()

        if any(
            isinstance(agent, EstacionCarga)
            for agent in self.model.grid.get_cell_list_contents([self.pos])
        ):
            self.carga += 50
            self.carga = min(100, self.carga)

            self.tiempo_en_estacion += 1

            if self.tiempo_en_estacion >= 2 and self.carga <= 100:
                if self.previous_pos is not None:
                        self.mover(cinta_cerca)
            return

        if self.carga <= 40:
            estacion_pos = self.estacion_carga_mas_cercana()
            if self.pos == estacion_pos:
                self.carga = 100

            else:
                
                self.mover(estacion_pos)  # se tira el astar4

