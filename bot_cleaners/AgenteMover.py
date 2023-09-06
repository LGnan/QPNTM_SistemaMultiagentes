from mesa.model import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector
from random import choice
import numpy as np
import heapq

from .Agentes import (
    Celda,
    EstanteriaChica,
    EstacionCarga,
    EstanteriaGrande,
    Cinta,
    
)

class AgenteMover(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.color = choice(["Magenta", "Red", "Green", "Blue", "Black"])
        self.sig_pos = None
        self.movimientos = 0
        self.carga = 100
        self.previous_pos = None
        self.tiempo_en_estacion = 0
        self.enCarga = False
        self.idCaja = None
        self.cajaEnCarga = None

    def mover(self, cinta_pos):
        if cinta_pos is None:
            return
        path = self.a_star(self.pos, cinta_pos)

        if path:
            next_pos = path[0]
            self.model.grid.move_agent(self, next_pos)
            self.pos = next_pos
            self.sig_pos = next_pos

    def distancia_manhattan(self, pos1, pos2):
        if pos2 is None:
            return float("inf")
        x1, y1 = pos1
        x2, y2 = pos2
        return abs(x2 - x1) + abs(y2 - y1)

    def distancia_euclidiana(self, pos1, pos2):
        x1, y1 = pos1
        x2, y2 = pos2
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def estanteria_grande_mas_cercana(self):
        estanterias = [
            agent.pos
            for agent in self.model.schedule.agents
            if isinstance(agent, EstanteriaGrande)
        ]

        if not estanterias:
            return None

        estanteria_cercana = min(
            estanterias, key=lambda pos: self.distancia_euclidiana(self.pos, pos)
        )

        return estanteria_cercana

    def cinta_mas_cercana(self):
        cintas = [
            agent.pos
            for agent in self.model.schedule.agents
            if isinstance(agent, Cinta)
        ]

        if not cintas:
            return None

        cinta_cercana = min(
            cintas, key=lambda pos: self.distancia_euclidiana(self.pos, pos)
        )

        return cinta_cercana

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

    def mueble_mas_cercano(self):
        muebles = [
            agent.pos
            for agent in self.model.schedule.agents
            # if not self.enUso == True 
            if isinstance(agent, EstanteriaChica) and not agent.enUso
        ]

        if not muebles:
            return None  # si no hay estaciones d carga

        mueble_cercano = min(
            muebles, key=lambda pos: self.distancia_euclidiana(self.pos, pos)
        )

        return mueble_cercano

    def cagada_mas_cercana(self):
        cagadas_pos = [
            agent.pos
            for agent in self.model.schedule.agents
            if isinstance(agent, Celda) and agent.sucia
        ]

        if not cagadas_pos:
            return None

        cagada_cercana = min(
            cagadas_pos, key=lambda pos: self.distancia_manhattan(self.pos, pos)
        )

        return cagada_cercana

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

    def mover_hacia_estacion(self, estacion_pos):
        path = self.a_star(self.pos, estacion_pos)

        if path:
            next_pos = path[0]
            self.model.grid.move_agent(self, next_pos)
            self.pos = next_pos
            self.sig_pos = next_pos

    def mover_hacia_mueble(self, mueble_pos):
        if mueble_pos is None:
            return
        path = self.a_star(self.pos, mueble_pos)

        # # print("este es el path dl puto agente", path)

        if path:
            next_pos = path[0]
            self.model.grid.move_agent(self, next_pos)
            self.pos = next_pos
            self.sig_pos = next_pos

    def mover_hacia_mueble_caja(self, mueble_pos, caja):
        if mueble_pos is None:
            return
        path = self.a_star(self.pos, mueble_pos)

        # print("este es el path d la caja", path)

        if path:
            next_pos = path[0]
            self.model.grid.move_agent(caja, next_pos)  # Mueve solo la caja a la siguiente posición
            caja.pos = next_pos  # Actualiza la posición de la caja
            # print(caja.pos, "soy la caja y m estoy moviendo alaverga")




    def seleccionar_nueva_pos(self, lista_de_vecinos):
        self.sig_pos = self.random.choice(lista_de_vecinos).pos

    @staticmethod
    def buscar_celdas_sucia(lista_de_vecinos):
        return [
            vecino
            for vecino in lista_de_vecinos
            if isinstance(vecino, Celda) and vecino.sucia
        ]

    @staticmethod
    def buscar_muebles(lista_de_vecinos):
        return [
            vecino for vecino in lista_de_vecinos if isinstance(vecino, EstanteriaChica)
        ]

    def step(self):
        vecinos = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False
        )
        vecinos = [
            vecino
            for vecino in vecinos
            if not isinstance(vecino, (EstanteriaChica, AgenteMover, EstacionCarga))
        ]

        vecinos2 = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False
        )
        vecinos2 = [
            vecino
            for vecino in vecinos
            if not isinstance(vecino, (Celda, AgenteMover, EstacionCarga))
        ]

        muebles = self.buscar_muebles(vecinos2)
        celdas_sucias = self.buscar_celdas_sucia(vecinos)
        cagada_pos = self.cagada_mas_cercana()
        estacion_pos = self.estacion_carga_mas_cercana()
        mueble_cercano_pos = self.mueble_mas_cercano()

        

        # Verificar si el robot está llevando una carga
        if self.enCarga == True:
            
            self.mover_hacia_mueble_caja(mueble_cercano_pos,self.cajaEnCarga)
            self.mover_hacia_mueble(mueble_cercano_pos)

            
            
            # print(self.model.grid.get_cell_list_contents([self.pos]), "penenenenenenennenenenen")
            if isinstance(
                self.model.grid.get_cell_list_contents([self.pos])[0], EstanteriaChica
            ):
                
                self.enCarga = False
                estanteria_chica = self.model.grid.get_cell_list_contents([self.pos])[0]
                estanteria_chica.enUso = True
                self.cajaEnCarga = self.model.grid.get_cell_list_contents([self.pos])[0]

                
                # print("id caja de la estanteria", estanteria_chica.idCaja)
                
                estanteria_chica.idCaja = self.idCaja
                # print("idCaja estanteriaCh:", estanteria_chica.idCaja)
                # print(estanteria_chica.enUso, "estanteria en uso True/False")
                # print(estanteria_chica, "estanteria_chica")

            return

        if any(
            isinstance(agent, EstacionCarga)
            for agent in self.model.grid.get_cell_list_contents([self.pos])
        ):
            self.carga += 50
            self.carga = min(100, self.carga)

            self.tiempo_en_estacion += 1

            if self.tiempo_en_estacion >= 2 and self.carga <= 100:
                if self.previous_pos is not None:
                        self.mover(cagada_pos)
            return

        if self.carga <= 40:
            estacion_pos = self.estacion_carga_mas_cercana()
            if self.pos == estacion_pos:
                self.carga = 100

            else:
                
                self.mover(estacion_pos)  # se tira el astar4

        elif len(celdas_sucias) == 0:
            self.mover(cagada_pos)
            if cagada_pos == None:
                self.mover(estacion_pos)
            

        else:
                # print(self.model.grid.get_cell_list_contents([(18,15)]), "penenenenenenennenenenen")
                self.mover(cagada_pos)
                # print("pos", self.pos, "cagadapos", cagada_pos)
                #dtermna si estoy parado en la celda sucia igual q arriba con lo d los muebles chiquitos (eliminamos el limpiar celda)
                celda_actual = self.model.grid.get_cell_list_contents([self.pos])[0]
                if isinstance(celda_actual, Celda) and celda_actual.sucia:
                    if isinstance(
                    self.model.grid.get_cell_list_contents([self.pos])[0], Celda
                    ):
                        self.idCaja = celda_actual.unique_id
                        celda_sucia = self.model.grid.get_cell_list_contents([self.pos])[0]
                        celda_sucia.sucia = False
                        self.cajaEnCarga = celda_sucia
                        self.mover_hacia_mueble_caja(mueble_cercano_pos,celda_sucia) ## aqui hacemos q la celda sucia se mueva a esa posicion
            
            
                        #esto tenemos que moverlo junto con el agente para que no se quede la celda sucia en esa posicion.#important
                        # print("estableciendo en true", celda_actual.sucia)
                        self.enCarga = True

        # Salir del método step

    def advance(self):
        if self.sig_pos is not None:
            if self.carga == 100:
                self.previous_pos = self.pos
                self.movimientos = 0
            if self.pos != self.sig_pos:
                self.movimientos += 1
            if self.carga > 0:
                self.carga -= 1
                self.model.grid.move_agent(self, self.sig_pos)