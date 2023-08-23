from mesa import Agent, Model
from mesa.time import RandomActivation
from mesa.space import MultiGrid
from mesa.datacollection import DataCollector
import random


class Producto(Agent):
    def _init_(self, unique_id, model):
        super()._init_(unique_id, model)
        self.almacenado = False
        self.empaquetado = False  # New attribute


class Estanteria(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class AgenteRecolector(Agent):
    def _init_(self, unique_id, model):
        super()._init_(unique_id, model)
        self.bateria = 100
        self.en_carga = False

    def find_nearest_charging_station(self):
        all_stations = [
            (x, y)
            for x in range(self.model.grid.width)
            for y in range(self.model.grid.height)
            for obj in self.model.grid.get_cell_list_contents([(x, y)])
            if isinstance(obj, EstacionCarga)
        ]
        if all_stations:
            return min(all_stations, key=lambda pos: self.distance_to(pos))
        else:
            return None

    def move_towards(self, target_pos):
        possible_steps = self.model.grid.get_neighborhood(
            self.pos, moore=True, include_center=False
        )
        return min(possible_steps, key=lambda pos: self.distance_to(target_pos, pos))

    def distance_to(self, target_pos, from_pos=None):
        if from_pos is None:
            from_pos = self.pos
        x1, y1 = from_pos
        x2, y2 = target_pos
        return abs(x1 - x2) + abs(y1 - y2)

    def step(self):
        if self.bateria <= 20:
            self.en_carga = True

        if self.en_carga:
            nearest_station = self.find_nearest_charging_station()
            if nearest_station is None:
                print("No charging stations found.")
                return

            next_move = self.move_towards(nearest_station)
            self.model.grid.move_agent(self, next_move)

            if self.pos == nearest_station:
                self.bateria += 10  # Charging speed
                if self.bateria >= 100:
                    self.bateria = 100
                    self.en_carga = False
            return

        vecinos_posibles = self.model.grid.get_neighborhood(
            self.pos, moore=True, include_center=False
        )
        nueva_pos = random.choice(vecinos_posibles)
        self.model.grid.move_agent(self, nueva_pos)

        vecinos = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False
        )

        producto_cercano = [obj for obj in vecinos if isinstance(obj, Producto)]

        if len(producto_cercano) > 0:
            producto = producto_cercano[0]
            producto.almacenado = True
            self.model.grid.move_agent(
                producto, (random.randint(0, 5), random.randint(0, 5))
            )  # Move to packaging area

        self.bateria -= 1


class AgenteEmpaquetador(Agent):
    def _init_(self, unique_id, model):
        super()._init_(unique_id, model)

    def step(self):
        new_position = (random.randint(0, 5), random.randint(0, 5))
        self.model.grid.move_agent(self, new_position)

        vecinos = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False
        )

        productos_almacenados = [
            obj for obj in vecinos if isinstance(obj, Producto) and obj.almacenado
        ]

        for producto in productos_almacenados:
            producto.empaquetado = True
            print(f"Product {producto.unique_id} has been packaged.")
            self.model.grid.remove_agent(producto)
            self.model.schedule.remove(producto)


class EstacionCarga(Agent):
    def _init_(self, unique_id, model):
        super()._init_(unique_id, model)


class Almacen(Model):
    def _init_(self, N):
        self.num_agentes = N
        self.grid = MultiGrid(20, 20, True)
        self.schedule = RandomActivation(self)

        for i in range(self.num_agentes):
            agente = AgenteRecolector(i, self)
            x = random.randrange(self.grid.width)
            y = random.randrange(self.grid.height)
            self.grid.place_agent(agente, (x, y))
            self.schedule.add(agente)

        for i in range(10):
            producto = Producto(i + 100, self)
            x = random.randrange(self.grid.width)
            y = random.randrange(self.grid.height)
            self.grid.place_agent(producto, (x, y))
            self.schedule.add(producto)

        for i in range(5):
            estacion = EstacionCarga(i + 200, self)
            x = random.randrange(self.grid.width)
            y = random.randrange(self.grid.height)
            self.grid.place_agent(estacion, (x, y))
            self.schedule.add(estacion)

        for i in range(5):  # Let's say there are 5 packaging agents
            agente = AgenteEmpaquetador(i + 300, self)
            x = random.randrange(self.grid.width)
            y = random.randrange(self.grid.height)
            self.grid.place_agent(agente, (x, y))
            self.schedule.add(agente)

    def step(self):
        self.schedule.step()
