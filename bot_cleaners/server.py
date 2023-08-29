import mesa

from .model import (
    Habitacion,
    AgenteMover,
    Celda,
    Mueble,
    EstacionCarga,
    EstanteriaGrande,
    Cinta,
)

MAX_NUMBER_ROBOTS = 20


def agent_portrayal(agent):
    if isinstance(agent, AgenteMover):
        return {
            "Shape": "circle",
            "Filled": "false",
            "Color": agent.color,  #pa q tengan diferente qolor
            "Layer": 1,
            "r": 1.0,
            "text": f"{agent.carga}",
            "text_color": "white",
        }
    elif isinstance(agent, EstanteriaChica):
        color = "black" if not agent.enUso else "blue" 
        text = "" if not agent.enUso else "📦" 
        return {
            "Shape": "rect",
            "Filled": "true",
            "Color": color,
            "Layer": 0,
            "w": 0.9,
            "h": 0.9,  
            "text" : text
        }

    elif isinstance(agent, EstanteriaGrande):
        return {
            "Shape": "rect",
            "Filled": "true",
            "Color": "red",
            "Layer": 0,
            "w": 0.9,
            "h": 0.9,
        }

    elif isinstance(agent, Cinta):
        return {
            "Shape": "rect",
            "Filled": "true",
            "Color": "blue",
            "Layer": 0,
            "w": 0.9,
            "h": 0.9,
        }
    elif isinstance(agent, EstacionCarga):
        return {
            "Shape": "rect",
            "Filled": "true",
            "Color": "green",
            "Layer": 0,
            "w": 0.9,
            "h": 0.9,
            "text": "⚡",  # Emoji d batería
            "text_color": "white",
        }

    elif isinstance(agent, Celda):
        portrayal = {
            "Shape": "rect",
            "Filled": "true",
            "Layer": 0,
            "w": 0.9,
            "h": 0.9,
            "text_color": "Black",
        }
        if agent.sucia:
            portrayal["Color"] = "#ccbeaf"
            portrayal["text"] = "📦"
        else:
            portrayal["Color"] = "white"
            portrayal["text"] = ""
        return portrayal


grid = mesa.visualization.CanvasGrid(agent_portrayal, 32, 24, 500, 500)
chart_celdas = mesa.visualization.ChartModule(
    [{"Label": "CeldasSucias", "Color": "#36A2EB", "label": "Celdas Sucias"}],
    50,
    200,
    data_collector_name="datacollector",
)

model_params = {
    "num_agentes": mesa.visualization.Slider(
        "Número de Robots",
        5,
        2,
        MAX_NUMBER_ROBOTS,
        1,
        description="Escoge cuántos robots deseas implementar en el modelo",
    ),
    "porc_celdas_sucias": mesa.visualization.Slider(
        "Porcentaje de Celdas Sucias",
        0.3,
        0.0,
        0.75,
        0.05,
        description="Selecciona el porcentaje de celdas sucias",
    ),
    "porc_muebles": mesa.visualization.Slider(
        "Porcentaje de Muebles",
        0.1,
        0.0,
        0.25,
        0.01,
        description="Selecciona el porcentaje de muebles",
    ),
    "modo_pos_inicial": mesa.visualization.Choice(
        "Posición Inicial de los Robots",
        "Aleatoria",
        ["Fija", "Aleatoria"],
        "Seleciona la forma se posicionan los robots",
    ),
    "M": 32,
    "N": 24,
}

server = mesa.visualization.ModularServer(
    Habitacion, [grid, chart_celdas], "botCleaner", model_params, 9108
)