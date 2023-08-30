from mesa.model import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector
from random import choice
import numpy as np
import heapq


class EstacionCarga(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Celda(Agent):
    def __init__(self, unique_id, model, suciedad: bool = False):
        super().__init__(unique_id, model)
        self.sucia = suciedad


class EstanteriaChica(Agent):  # estantería chica
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.enUso = False
        self.idCaja = None


class EstanteriaGrande(Agent):  # estantería grande
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class Cinta(Agent):  # estantería cinta
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class Cinta2(Agent):  # estantería cinta
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

