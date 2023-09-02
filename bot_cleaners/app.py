from flask import Flask, jsonify, request
import traceback  # Import traceback for better debugging

app = Flask(__name__)

from bot_cleaners.AgenteMover import AgenteMover
from bot_cleaners.AgenteRecoger import AgenteRecoger
from bot_cleaners.model import Habitacion

model = None

@app.route('/initialize', methods=['POST'])
def initialize_model():
    global model
    config = request.json
    print(f"Initializing model with configuration: {config}")  
    print(f"M: {config['M']}, N: {config['N']}")

    try:
        model = Habitacion(**config)
        return jsonify({"message": "Model initialized"}), 200  # Successfully initialized
    except Exception as e:  # General Exception to catch all errors
        traceback.print_exc()  # This will print the stack trace
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
                })
            elif isinstance(agent, AgenteRecoger):
                agent_data.append({
                    "type": "AgenteRecoger",
                    "id": agent.unique_id,
                    "pos": agent.pos,
                    "carga": agent.carga,
                    "enCarga": agent.enCarga,
                })
        return jsonify(agent_data), 200  # OK
    except Exception as e:
        traceback.print_exc()
        return jsonify({"error": f"Fetching agent data failed due to {str(e)}"}), 500  # Internal Server Error

if __name__ == '__main__':
    app.run(debug=True)