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
        self.idCaja = None
        self.cajaEnCarga = None
        self.cajaEscogida = False
        self.idEscogida = None
        self.x = 4
        self.x2 = 4
        self.despachar_caja = False
        
        
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
            # print("Destino es None")
            return

        # print(f"Calculando path desde {self.pos} hasta {cinta_pos}")
        path = self.a_star(self.pos, cinta_pos)

        if path:
            # print(f"Path encontrado: {path}")
            next_pos = path[0]
            self.model.grid.move_agent(self, next_pos)
            self.pos = next_pos
            self.sig_pos = next_pos
            # print(f"Me he movido a {self.pos}")
        # else:
            # print("No se encontró path")

    def mover_hacia_mueble_cinta(self, cinta_pos, caja):
        if cinta_pos is None:
            return
        path = self.a_star(self.pos, cinta_pos)

        # print("este es el path d la caja", path)

        if path:
            next_pos = path[0]
            self.model.grid.move_agent(caja, next_pos)  # Mueve solo la caja a la siguiente posición
            caja.pos = next_pos  # Actualiza la posición de la caja
            # print(caja.pos, "soy la caja y m estoy moviendo alaverga")
            #despachar_caja()
            


    def estanteria_chica_mas_cercana(self,cajas,caja_escogida_bool):

        if caja_escogida_bool == False:
            self.idEscogida = np.random.choice(cajas)
        estanterias = [
            agent
            for agent in self.model.schedule.agents
            if isinstance(agent, EstanteriaChica)
        ]

        estanteria_id = [estanteria for estanteria in estanterias if estanteria.idCaja == self.idEscogida]

        if estanteria_id == None or estanteria_id == []:
            self.idEscogida = np.random.choice(cajas)
            estanteria_id = [estanteria for estanteria in estanterias if estanteria.idCaja == self.idEscogida]
            return estanteria_id[0].pos

        # estanteria_cercana = min(
        #     estanterias_en_uso,
        #     key=lambda estanteria: self.distancia_euclidiana(self.pos, estanteria.pos)
        # )
        return estanteria_id[0].pos

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
                    isinstance(agent, AgenteRecoger)
                    for agent in self.model.grid.get_cell_list_contents([next_cell])
                ):
                    continue

                if any(
                    isinstance(agent, AgenteMover)
                    for agent in self.model.grid.get_cell_list_contents([next_cell])
                ):
                    continue

                if any(
                    isinstance(agent, EstanteriaChica) and agent.enUso == False
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
                    f_score = tentative_g_score + self.distancia_euclidiana(
                        next_cell, goal
                    )
                    heapq.heappush(open_list, (f_score, next_cell))

        return None
    
    def step(self):

    
        if self.despachar_caja == True:
            # print("ME LA METIO EL DESPACHAR CAJA")
            ### LEEER IMPORTANTE
            
            
            #### EN caso de que se truene y vayan a la caja que está en la cfinta final


            #### quitar el self.cajaEnCarga.sucia = True (es meramente estético para el mesa)
            #(4,9) (4,14)
            if self.cajaEnCarga.pos == (3,9) or self.x < 4:
                # print("NO BANDA MEMAMAELPITPXDXDXDX",self.x)
                self.cajaEnCarga.model.grid.move_agent(self.cajaEnCarga, (self.x, 9))
                self.x-=1
                if self.x < 3:
                    self.cajaEnCarga.sucia = True  
            if self.cajaEnCarga.pos == (3,14) or self.x2 < 4:
                print(" penenenennenenenenenenennenene SI BANDA MEMAMAELPITPXDXDXDX2",self.x2)
                self.cajaEnCarga.model.grid.move_agent(self.cajaEnCarga, (self.x2, 14))
                self.x2-=1
                if self.x2 < 3:
                    self.cajaEnCarga.sucia = True  
                
            # print(self.cajaEnCarga.pos)
            if self.cajaEnCarga.pos == (0,14):
                self.despachar_caja = False  
                self.cajaEnCarga.sucia = False 
            if self.cajaEnCarga.pos == (0,9):
                self.despachar_caja = False
                self.cajaEnCarga.sucia = False 
                

        estanterias = [
            agent
            for agent in self.model.schedule.agents
            if isinstance(agent, EstanteriaChica)
        ]

        estanterias_en_uso = [estanteria.idCaja for estanteria in estanterias if estanteria.enUso == True]
        
        
        if estanterias_en_uso == []:
            estanterias_en_uso = None
            estanteria_en_uso = None

        # print(estanterias_en_uso , "MENSAJE DE ESTANTERIAS EN USO")
        cinta_cerca = self.cinta2_mas_cercana()
        # print(cinta_cerca, "aqi esta la cinta mas cercana")

        estacion_pos = self.estacion_carga_mas_cercana()
        
        if estanterias_en_uso != [] and estanterias_en_uso != None:
            estanteria_en_uso = self.estanteria_chica_mas_cercana(estanterias_en_uso,self.cajaEscogida)
            self.cajaEscogida = True
            # print(estanteria_en_uso, "estanteria en uso")

        if self.enCarga == True and self.carga >= 40:
            self.mover(cinta_cerca)
            self.mover_hacia_mueble_cinta(cinta_cerca,self.cajaEnCarga)

        if any(
            isinstance(agent, EstacionCarga)
            for agent in self.model.grid.get_cell_list_contents([self.pos])
        ):
            self.carga += 50
            self.carga = min(100, self.carga)
            self.tiempo_en_estacion += 1

            if self.tiempo_en_estacion >= 2 and self.carga <= 100:
                if self.previous_pos is not None:
                        self.mover(estanteria_en_uso)
            return
        
        elif self.carga <= 40:
            estacion_pos = self.estacion_carga_mas_cercana()
            if self.pos == estacion_pos:
                self.carga = 100
            else:
                self.mover(estacion_pos)
                # print(f"Me estoy moviendo hacia la estacion d carga {estanteria_en_uso}")

        
        elif estanteria_en_uso is not None and self.enCarga == False:
                self.mover(estanteria_en_uso)
                # print(f"Me deberia estar moviendo hacia la estantería en uso en {estanteria_en_uso}")

                if isinstance(
                self.model.grid.get_cell_list_contents([self.pos])[0], EstanteriaChica
                ):
                    self.enCarga = True
                    # print(self.enCarga, "en carga cacacaca")
                    estanteria_chica = self.model.grid.get_cell_list_contents([self.pos])[0]
                    caja = self.model.grid.get_cell_list_contents([self.pos])[1]
                    self.cajaEnCarga = caja
                    estanteria_chica.enUso = False
                    # print(estanteria_chica.idCaja, "aqui esta la caja en estanteria chicalol")
                    self.idCaja = estanteria_chica.idCaja 
                    print("idCaja recogido:", self.idCaja)
                    # self.mover_hacia_mueble_cinta(cinta_cerca,self.cajaEnCarga)

        elif isinstance(
                self.model.grid.get_cell_list_contents([self.pos])[0], Cinta2
                ):
                    self.x = 4
                    self.x2 = 4
                    self.enCarga = False
                    self.cajaEscogida = False
                    self.despachar_caja = True

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
