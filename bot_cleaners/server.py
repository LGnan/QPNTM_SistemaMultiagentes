import mesa
from mesa.visualization.modules import CanvasGrid, ChartModule
from mesa.visualization.ModularVisualization import ModularServer
from .model import (
    Almacen,
    Producto,
    AgenteRecolector,
    AgenteEmpaquetador,
    EstacionCarga,
)


MAX_NUMBER_ROBOTS = 20


def agent_portrayal(agent):
    if isinstance(agent, AgenteRecolector):
        return {
            "Shape": "circle",
            "Filled": "true",
            "Color": "blue",
            "Layer": 1,
            "r": 0.8,  # Make the circle larger
            "text": str(agent.bateria) + "%",
            "text_color": "white",
            "text_anchor": "middle",
            "Font": "20px Arial",  # You can adjust the font size here
        }
    elif isinstance(agent, Producto):
        return {
            "Shape": "rect",
            "Filled": "true",
            "Color": "red",
            "Layer": 0,
            "w": 0.8,
            "h": 0.8,
        }
    elif isinstance(agent, EstacionCarga):
        return {
            "Shape": "rect",
            "Filled": "true",
            "Color": "green",
            "Layer": 0,
            "w": 0.8,
            "h": 0.8,
        }
    elif isinstance(agent, AgenteEmpaquetador):
        return {
            "Shape": "circle",
            "Filled": "true",
            "Color": "yellow",
            "Layer": 1,
            "r": 0.5,
        }


grid = CanvasGrid(agent_portrayal, 20, 20, 400, 400)

model_params = {
    "N": mesa.visualization.Slider(
        "Numero de Robots",
        1,
        1,
        MAX_NUMBER_ROBOTS,
        1,
        description="Escoge cuantos robots deseas implementar en el modelo",
    )
}

server = ModularServer(Almacen, [grid], "Simulacion de Almacen", model_params)
server.port = 8521  # O cualquier otro puerto que desees
