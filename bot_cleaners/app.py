from flask import Flask, jsonify, request
app = Flask(__name__)


model = None

@app.route('/initialize', methods=['POST'])
def initialize_model():
    global model
    config = request.json
    model = Habitacion(**config) 
    return jsonify({"status": "Model initialized"})

@app.route('/step', methods=['GET'])
def step_model():
    global model
    if model is None:
        return jsonify({"error": "Model not initialized"}), 400
    model.step()
    return jsonify({"status": "Model stepped"})

@app.route('/agent_data', methods=['GET'])
def get_agent_data():
    global model
    if model is None:
        return jsonify({"error": "Model not initialized"}), 400
    
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
    
    return jsonify(agent_data)

if __name__ == '__main__':
    print("Iniciando servidor en http://127.0.0.1:5000/")
    app.run(debug=True)