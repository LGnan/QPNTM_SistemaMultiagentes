from mesa.model import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector
import numpy as np
import heapq


class EstacionCarga(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class Celda(Agent):
    def __init__(self, unique_id, model, suciedad: bool = False):
        super().__init__(unique_id, model)
        self.sucia = suciedad


class Mueble(Agent):  # estantería chica
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class EstanteriaGrande(Agent):  # estantería grande
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class Cinta(Agent):  # estantería cinta
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class RobotLimpieza(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.sig_pos = None
        self.movimientos = 0
        self.carga = 100
        self.previous_pos = None
        self.tiempo_en_estacion = 0
        self.enCarga = False

    def distancia_manhattan(self, pos1, pos2):
        if pos2 is None:
            # Handle the None case
            return float("inf")  # or some other value
        x1, y1 = pos1
        x2, y2 = pos2
        return abs(x2 - x1) + abs(y2 - y1)

    def distancia_euclidiana(self, pos1, pos2):
        x1, y1 = pos1
        x2, y2 = pos2
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def estacion_carga_mas_cercana(self):
        # Crea lista d todas las estaciones q hay
        estaciones = [
            agent.pos
            for agent in self.model.schedule.agents
            if isinstance(agent, EstacionCarga)
        ]

        if not estaciones:
            print("\ No hay estaciones de carga en el modelo.")
            return None  # si no hay estaciones d carga

        # calcula la distancia entre los puntos del robot actual y la de cada posicion d carga (lista) y agarra el minimo4
        estacion_cercana = min(
            estaciones, key=lambda pos: self.distancia_euclidiana(self.pos, pos)
        )

        return estacion_cercana

    def mueble_mas_cercano(self):
        # Crea lista d todas las estaciones q hay
        muebles = [
            agent.pos
            for agent in self.model.schedule.agents
            if isinstance(agent, Mueble)
        ]

        if not muebles:
            print("\ No hay estaciones de carga en el modelo.")
            return None  # si no hay estaciones d carga

        # calcula la distancia entre los puntos del robot actual y la de cada posicion d carga (lista) y agarra el minimo4
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
            print("No hay cagadas en el modelo.")
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
                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score.get(next_cell, float("inf")):
                    came_from[next_cell] = current
                    g_score[next_cell] = tentative_g_score
                    f_score = tentative_g_score + self.distancia_euclidiana(
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

        if path:
            next_pos = path[0]
            self.model.grid.move_agent(self, next_pos)
            self.pos = next_pos
            self.sig_pos = next_pos

    def limpiar_una_celda(self, lista_de_celdas_sucias):
        celda_a_limpiar = self.random.choice(lista_de_celdas_sucias)
        celda_a_limpiar.sucia = False
        self.enCarga = True

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
        return [vecino for vecino in lista_de_vecinos if isinstance(vecino, Mueble)]

    def step(self):
        vecinos = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False
        )
        vecinos = [
            vecino
            for vecino in vecinos
            if not isinstance(vecino, (Mueble, RobotLimpieza, EstacionCarga))
        ]

        vecinos2 = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False
        )
        vecinos2 = [
            vecino
            for vecino in vecinos
            if not isinstance(vecino, (Celda, RobotLimpieza, EstacionCarga))
        ]

        muebles = self.buscar_muebles(vecinos2)
        celdas_sucias = self.buscar_celdas_sucia(vecinos)
        cagada_pos = self.cagada_mas_cercana()
        estacion_pos = self.estacion_carga_mas_cercana()
        mueble_cercano_pos = self.mueble_mas_cercano()

        # Verificar si el robot está llevando una carga
        if self.enCarga == True:
            self.mover_hacia_mueble(mueble_cercano_pos)

            if self.pos == mueble_cercano_pos:
                self.enCarga = False

            return

        print("posicionn", self.pos, "mueble cercano", mueble_cercano_pos)

        if any(
            isinstance(agent, EstacionCarga)
            for agent in self.model.grid.get_cell_list_contents([self.pos])
        ):
            self.carga += 50
            self.carga = min(100, self.carga)

            self.tiempo_en_estacion += 1

            if self.tiempo_en_estacion >= 2 and self.carga <= 100:
                if self.previous_pos is not None:
                    self.mover_hacia_estacion(self.previous_pos)
                self.tiempo_en_estacion = 0
            return

        if self.carga <= 40:
            estacion_pos = self.estacion_carga_mas_cercana()
            if self.pos == estacion_pos:
                self.carga = 100

            else:
                self.mover_hacia_estacion(estacion_pos)  # se tira el astar4

        elif len(celdas_sucias) == 0:
            if cagada_pos == None:
                self.mover_hacia_estacion(estacion_pos)
            else:
                # Mover hacia paquete
                self.mover_hacia_estacion(cagada_pos)

        else:
            self.limpiar_una_celda(celdas_sucias)

        # Salir del método step

    def advance(self):
        if self.sig_pos is not None:  # Añadir esta comprobación
            if self.carga == 100:
                self.previous_pos = self.pos
                self.movimientos = 0
            if self.pos != self.sig_pos:
                self.movimientos += 1
            if self.carga > 0:
                self.carga -= 1
                self.model.grid.move_agent(self, self.sig_pos)


class Habitacion(Model):
    def __init__(
        self,
        M: int,
        N: int,
        num_agentes: int = 5,
        porc_celdas_sucias: float = 0.6,
        porc_muebles: float = 0.1,
        modo_pos_inicial: str = "Fija",
    ):
        self.num_agentes = num_agentes
        self.porc_celdas_sucias = porc_celdas_sucias
        self.porc_muebles = porc_muebles

        self.grid = MultiGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)

        posiciones_disponibles = [pos for _, pos in self.grid.coord_iter()]

        # Posicionamiento de muebless
        num_muebles = int(M * N * porc_muebles)
        posiciones_muebles = [
            # Cintas
            (28, 14),
            (27, 14),
            (25, 14),
            (24, 14),
            (28, 9),
            (27, 9),
            (25, 9),
            (24, 9),
            (1, 14),
            (2, 14),
            (3, 14),
            (1, 9),
            (2, 9),
            (3, 9),
            # EstanteríasChicas
            (8, 18),
            (9, 18),
            (10, 18),
            (11, 18),
            (12, 18),
            (13, 18),
            (14, 18),
            (15, 18),
            (16, 18),
            (17, 18),
            (18, 18),
            (8, 15),
            (9, 15),
            (10, 15),
            (11, 15),
            (12, 15),
            (13, 15),
            (14, 15),
            (15, 15),
            (16, 15),
            (17, 15),
            (18, 15),
            (8, 8),
            (9, 8),
            (10, 8),
            (11, 8),
            (12, 8),
            (13, 8),
            (14, 8),
            (15, 8),
            (16, 8),
            (17, 8),
            (18, 8),
            (8, 5),
            (9, 5),
            (10, 5),
            (11, 5),
            (12, 5),
            (13, 5),
            (14, 5),
            (15, 5),
            (16, 5),
            (17, 5),
            (18, 5),
            # Estanterías Grandes
            (8, 22),
            (9, 22),
            (10, 22),
            (11, 22),
            (12, 22),
            (13, 22),
            (14, 22),
            (15, 22),
            (16, 22),
            (17, 22),
            (18, 22),
            (8, 21),
            (9, 21),
            (10, 21),
            (11, 21),
            (12, 21),
            (13, 21),
            (14, 21),
            (15, 21),
            (16, 21),
            (17, 21),
            (18, 21),
            (8, 12),
            (9, 12),
            (10, 12),
            (11, 12),
            (12, 12),
            (13, 12),
            (14, 12),
            (15, 12),
            (16, 12),
            (17, 12),
            (18, 12),
            (8, 11),
            (9, 11),
            (10, 11),
            (11, 11),
            (12, 11),
            (13, 11),
            (14, 11),
            (15, 11),
            (16, 11),
            (17, 11),
            (18, 11),
            (8, 2),
            (9, 2),
            (10, 2),
            (11, 2),
            (12, 2),
            (13, 2),
            (14, 2),
            (15, 2),
            (16, 2),
            (17, 2),
            (18, 2),
            (8, 1),
            (9, 1),
            (10, 1),
            (11, 1),
            (12, 1),
            (13, 1),
            (14, 1),
            (15, 1),
            (16, 1),
            (17, 1),
            (18, 1),
        ]
        # EC = [(8,18),(9,18),(10,18),(11,18),(12,18),(13,18),(14,18),(15,18),(16,18),(17,18),(18,18), (8,15),(9,15),(10,15),(11,15),(12,15),(13,15),(14,15),(15,15),(16,15),(17,15),(18,15), (8,8), (9,8), (10,8), (11,8), (12,8), (13,8), (14,8), (15,8), (16,8), (17,8), (18,8), (8,5), (9,5), (10,5), (11,5), (12,5), (13,5), (14,5), (15,5), (16,5), (17,5), (18,5)]
        # EG = [(8,22),(9,22),(10,22),(11,22),(12,22),(13,22),(14,22),(15,22),(16,22),(17,22),(18,22), (8,21),(9,21),(10,21),(11,21),(12,21),(13,21),(14,21),(15,21),(16,21),(17,21),(18,21), (8,12), (9,12), (10,12), (11,12), (12,12), (13,12), (14,12), (15,12), (16,12), (17,12), (18,12), (8,11), (9,11), (10,11), (11,11), (12,11), (13,11), (14,11), (15,11), (16,11), (17,11), (18,11), (8,2), (9,2), (10,2), (11,2), (12,2), (13,2), (14,2),(15,2), (16,2), (17,2), (18,2), (8,1), (9,1), (10,1), (11,1), (12,1), (13,1), (14,1),(15,1), (16,1), (17,1), (18,1),]
        # Cinta = [(1, 14), (2, 14), (3, 14),(1,9), (2,9), (3, 9)]

        for id, pos in enumerate(posiciones_muebles):
            mueble = Mueble(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(mueble, pos)
            self.schedule.add(mueble)  # Añadir la estación al schedules
            posiciones_disponibles.remove(pos)

        # Posicionar las estaciones en las esquinas 346765
        posiciones_estaciones = [(0, 0), (0, N - 1), (M - 1, 0), (M - 1, N - 1)]
        for id, pos in enumerate(posiciones_estaciones):
            estacion = EstacionCarga(int(f"5000{id}") + 1, self)
            self.grid.place_agent(estacion, pos)
            self.schedule.add(estacion)  # Añadir la estación al schedules
            if pos in posiciones_disponibles:
                posiciones_disponibles.remove(pos)

        print(f"{len(posiciones_estaciones)} estaciones de carga creadas.")

        # Posicionamiento de celdas sucias
        def condicion(pos):
            fila, columna = pos
            return 14 <= fila <= 18 and 6 <= columna <= 16

        # 14 - 18 en x
        # 6 - 16 en y

        # posiciones_dispCacaPipi = [pos for _, *pos in self.grid.coord_iter() if condicion(pos)]
        self.num_celdas_sucias = int(4 * 10 * porc_celdas_sucias)

        posiciones_dispCacaPipi = [
            # (15, 0),
            # (15, 1),
            # (15, 2),
            # (15, 3),
            # (15, 4),
            # (15, 5),
            # (16, 6),
            # (16, 7),
            # (16, 8),
            # (16, 9),
            # (16, 10),
            # (16, 11),
            # (16, 12),
            # (16, 13),
            # (16, 14),
            # (16, 15),
            # (16, 16),
            # (17, 6),
            # (17, 7),
            # (17, 8),
            # (17, 9),
            # (17, 10),
            # (17, 11),
            # (17, 12),
            # (17, 13),
            # (17, 14),
            # (17, 15),
            # (17, 16),
            (22, 7),
            (23, 7),
            (24, 7),
            (25, 7),
            (26, 7),
            (27, 7),
            (28, 7),
            (22, 6),
            (23, 6),
            (24, 6),
            (25, 6),
            (26, 6),
            (27, 6),
            (28, 6),
            (22, 5),
            (23, 5),
            (24, 5),
            (25, 5),
            (26, 5),
            (27, 5),
            (28, 5),
            (22, 4),
            (23, 4),
            (24, 4),
            (25, 4),
            (26, 4),
            (27, 4),
            (28, 4),
            (23, 14),
            (23, 9),
        ]

        posiciones_celdas_sucias = self.random.sample(
            posiciones_dispCacaPipi, k=self.num_celdas_sucias
        )

        for id, pos in enumerate(posiciones_disponibles):
            suciedad = pos in posiciones_dispCacaPipi
            celda = Celda(int(f"{num_agentes}{id}") + 1, self, suciedad)
            self.schedule.add(celda)
            self.grid.place_agent(celda, pos)

        # Posicionamiento de agentes robot
        if modo_pos_inicial == "Aleatoria":
            pos_inicial_robots = self.random.sample(
                posiciones_disponibles, k=num_agentes
            )
        else:  # 'Fija'
            pos_inicial_robots = [(0, N - 1)] * num_agentes

        for id in range(num_agentes):
            robot = RobotLimpieza(id, self)
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
            if isinstance(robot, RobotLimpieza):
                estacion_cercana = min(
                    estaciones,
                    key=lambda pos: robot.distancia_euclidiana(robot.pos, pos),
                )
                robot.mover_hacia_estacion(estacion_cercana)

    def step(self):
        self.datacollector.collect(self)

        self.schedule.step()

    def todoLimpio(self):
        for content, x, y in self.grid.coord_iter():
            for obj in content:
                if isinstance(obj, Celda) and obj.sucia:
                    return False
        return True


def get_grid(model: Model) -> np.ndarray:
    """
    Método para la obtención de la grid y representarla en un notebook
    :param model: Modelo (entorno)
    :return: grid
    """
    grid = np.zeros((model.grid.width, model.grid.height))
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        x, y = pos
        for obj in cell_content:
            if isinstance(obj, RobotLimpieza):
                grid[x][y] = 2
            elif isinstance(obj, Celda):
                grid[x][y] = int(obj.sucia)
    return grid


def get_cargas(model: Model):
    return [
        (agent.unique_id, agent.carga)
        for agent in model.schedule.agents
        if isinstance(agent, RobotLimpieza)
    ]


def get_sucias(model: Model) -> int:
    """
    Método para determinar el número total de celdas sucias
    :param model: Modelo Mesa
    :return: número de celdas sucias
    """
    sum_sucias = 0
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        for obj in cell_content:
            if isinstance(obj, Celda) and obj.sucia:
                sum_sucias += 1
    return sum_sucias / model.num_celdas_sucias


def get_movimientos(agent: Agent) -> dict:
    if isinstance(agent, RobotLimpieza):
        return {agent.unique_id: agent.movimientos}
    # else:
    #    return 0
