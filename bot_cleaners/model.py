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


class EstanteriaChica(Agent):  # estantería chica
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
            # Handle the None case
            return float("inf")  # or some other value
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
            print("No hay estanterías grandes en el modelo.")
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
            print("No hay cintas en el modelo.")
            return None

        cinta_cercana = min(
            cintas, key=lambda pos: self.distancia_euclidiana(self.pos, pos)
        )

        return cinta_cercana

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
            if isinstance(agent, EstanteriaChica)
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
            if not isinstance(vecino, (EstanteriaChica, RobotLimpieza, EstacionCarga))
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
                    self.mover(self.previous_pos)
                self.tiempo_en_estacion = 0
            return

        if self.carga <= 40:
            estacion_pos = self.estacion_carga_mas_cercana()
            if self.pos == estacion_pos:
                self.carga = 100

            else:
                self.mover(estacion_pos)  # se tira el astar4

        elif len(celdas_sucias) == 0:
            if cagada_pos == None:
                self.mover(estacion_pos)
            else:
                # Mover hacia paquete
                self.mover(cagada_pos)

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

        # Posiciones de cintas
        posiciones_cintas = []
        for y in [14, 9]:
            for x in [28, 27, 25, 24]:
                posiciones_cintas.append((x, y))
            for x in [1, 2, 3]:
                posiciones_cintas.append((x, y))

        posiciones_estanteriasG = []
        valores_x = [8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]
        valores_y = [22, 21, 12, 11, 2, 1]
        for y in valores_y:
            for x in valores_x:
                posiciones_estanteriasG.append((x, y))

        posiciones_dispCacaPipi = [
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

        for id, pos in enumerate(posiciones_cintas):
            cinta = Cinta(int(f"{num_agentes}200{id}") + 1, self)
            self.grid.place_agent(cinta, pos)
            self.schedule.add(cinta)
            posiciones_disponibles.remove(pos)

        for id, pos in enumerate(posiciones_estanteriasG):
            estanteria = EstanteriaGrande(int(f"{num_agentes}100{id}") + 1, self)
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
            estanteria_chica = EstanteriaChica(int(f"{num_agentes}00{id}") + 1, self)
            self.grid.place_agent(estanteria_chica, pos)
            self.schedule.add(estanteria_chica)
            posiciones_disponibles.remove(pos)

        posiciones_estaciones = [(0, 0), (0, N - 1), (M - 1, 0), (M - 1, N - 1)]
        for id, pos in enumerate(posiciones_estaciones):
            estacion = EstacionCarga(int(f"5000{id}") + 1, self)
            self.grid.place_agent(estacion, pos)
            self.schedule.add(estacion)  # Añadir la estación al schedules
            if pos in posiciones_disponibles:
                posiciones_disponibles.remove(pos)

        print(f"{len(posiciones_estaciones)} estaciones de carga creadas.")

        # posiciones_dispCacaPipi = [pos for _, *pos in self.grid.coord_iter() if condicion(pos)]
        self.num_celdas_sucias = int(4 * 10 * porc_celdas_sucias)

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
