from flask import Flask, jsonify, request
import traceback  # Import traceback for better debugging

app = Flask(__name__)

from bot_cleaners.AgenteMover import AgenteMover
from bot_cleaners.AgenteRecoger import AgenteRecoger
from bot_cleaners.Agentes import Celda
from bot_cleaners.model import Habitacion

model = None

@app.route('/initialize', methods=['POST'])
def initialize_model():
    global model
    config = request.json
    print(f"Initializing model with configuration: {config}")  

    try:
        # Desempaqueta los parámetros del JSON de configuración
        # M = 32, N = 24
        M = config.get('M')  
        N = config.get('N')  
        num_agentes = config.get('num_agentes')  
        rate_packages = config.get('rate_packages')
        num_agentesRecoger = config.get('num_agentesRecoger') 
        
        model = Habitacion(
            M=M,
            N=N,
            num_agentes=num_agentes,
            rate_packages=rate_packages,
            num_agentesRecoger = num_agentesRecoger
        )

        return jsonify({"message": "Model initialized"}), 200  # Successfully initialized
    except Exception as e:
        traceback.print_exc()
        return jsonify({"error": f"Initialization failed due to {str(e)}"}), 500  # Internal Server Error

@app.route('/step', methods=['GET'])
def step_model():
    global model
    if model is None:
        return jsonify({"error": "Model not initialized"}), 400  # Bad Request
    try:
        model.step()
        return jsonify({"status": "Model stepped"}), 200  # OK
    except Exception as e:
        traceback.print_exc()
        return jsonify({"error": f"Step failed due to {str(e)}"}), 500  # Internal Server Error

@app.route('/agent_data', methods=['GET'])
def get_agent_data():
    global model
    if model is None:
        return jsonify({"error": "Model not initialized"}), 400  # Bad Request
    
    try:
        agent_data = []
        for agent in model.schedule.agents:
            if isinstance(agent, AgenteMover):
                agent_data.append({
                    "type": "AgenteMover",
                    "id": agent.unique_id,
                    "pos": agent.pos,
                    "enCarga": agent.enCarga,
                })
        return jsonify(agent_data), 200  # OK
    except Exception as e:
        traceback.print_exc()
        return jsonify({"error": f"Fetching agent data failed due to {str(e)}"}), 500  # Internal Server Error
    

@app.route('/caja_data', methods=['GET'])
def get_caja_data():
    global model
    if model is None:
        return jsonify({"error": "Model not initialized"}), 400  # Bad Request
    
    try:
        agent_data_cajas = []
        for caja in model.schedule.agents:
            if isinstance(caja, Celda):
                agent_data_cajas.append({
                    "type": "Celda",
                    "id": caja.unique_id,
                    "pos": caja.pos,
                })
        return jsonify(agent_data_cajas), 200  # OK
    except Exception as e:
        traceback.print_exc()
        return jsonify({"error": f"Fetching agent data failed due to {str(e)}"}), 500  # Internal Server Error

@app.route('/agent_recoger', methods=['GET'])
def get_agent_recoger():
    global model
    if model is None:
        return jsonify({"error": "Model not initialized"}), 400  # Bad Request
    
    try:
        agent_data = []
        for agent in model.schedule.agents:
            if isinstance(agent, AgenteRecoger):
                agent_data.append({
                    "type": "AgenteRecoger",
                    "id": agent.unique_id,
                    "pos": agent.pos,
                    "enCarga": agent.enCarga,
                })
        return jsonify(agent_data), 200  # OK
    except Exception as e:
        traceback.print_exc()
        return jsonify({"error": f"Fetching agent data failed due to {str(e)}"}), 500  # Internal Server Error
    


if __name__ == '__main__':
    app.run(debug=True)