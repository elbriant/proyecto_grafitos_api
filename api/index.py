from flask import Flask, request, jsonify
import heapq
import math
import sqlite3
from contextlib import closing
import os

app = Flask(__name__)

# Configuración mejorada de la base de datos
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATABASE = os.path.join(BASE_DIR, '..', 'database', 'dateBase.db')

def get_db():
    """Obtiene conexión a la base de datos con verificación de existencia"""
    if not os.path.exists(DATABASE):
        raise FileNotFoundError(f"Archivo de base de datos no encontrado en: {DATABASE}")
    return sqlite3.connect(DATABASE)

def init_db():
    """Inicializa la base de datos (si es necesario)"""
    with closing(get_db()) as db:
        with app.open_resource('schema.sql', mode='r') as f:
            db.cursor().executescript(f.read())
        db.commit()

def query_db(query, args=(), one=False):
    """Ejecuta consultas a la base de datos de forma segura"""
    with closing(get_db()) as db:
        db.row_factory = sqlite3.Row  # Para obtener resultados como diccionarios
        cur = db.execute(query, args)
        rv = cur.fetchall()
        cur.close()
        return (rv[0] if rv else None) if one else rv

def get_campus_graph():
    """Obtiene el grafo del campus desde la base de datos con manejo de errores"""
    try:
        nodes = query_db("SELECT idNodos, latitud, longitud FROM nodos")
        if not nodes:
            raise ValueError("No se encontraron nodos en la base de datos")
            
        graph = {}
        
        for node in nodes:
            node_id, lat, lon = node
            neighbors_query = """  
            SELECT idCaminos, distancia
            FROM caminos 
            WHERE ruta = ?
            """
            neighbors = query_db(neighbors_query, [node_id])
            
            graph[node_id] = {
                "coords": (lat, lon),
                "neighbors": {dest: dist for dest, dist in neighbors}
            }
        
        return graph
        
    except sqlite3.Error as e:
        raise Exception(f"Error al leer la base de datos: {str(e)}")

def haversine(lat1, lon1, lat2, lon2):
    """Calcula la distancia en metros entre dos coordenadas geográficas"""
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    return 6371 * 1000 * 2 * math.asin(math.sqrt(a))

def a_star(graph, start, goal):
    """Implementación del algoritmo A*"""
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = haversine(*graph[start]["coords"], *graph[goal]["coords"])

    open_set_hash = {start}

    while open_set:
        current = heapq.heappop(open_set)[1]
        open_set_hash.remove(current)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, g_score[goal]

        for neighbor, distance in graph[current]["neighbors"].items():
            tentative_g_score = g_score[current] + distance

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + haversine(*graph[neighbor]["coords"], *graph[goal]["coords"])
                if neighbor not in open_set_hash:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    open_set_hash.add(neighbor)

    return None, float('inf')

def dijkstra(graph, start, goal):
    """Implementación del algoritmo Dijkstra"""
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    previous_nodes = {node: None for node in graph}
    unvisited = set(graph.keys())
    
    while unvisited:
        current = min(unvisited, key=lambda node: distances[node])
        
        if current == goal or distances[current] == float('inf'):
            break
            
        for neighbor, distance in graph[current]["neighbors"].items():
            alt = distances[current] + distance
            if alt < distances[neighbor]:
                distances[neighbor] = alt
                previous_nodes[neighbor] = current
                
        unvisited.remove(current)
    
    path = []
    current = goal
    while previous_nodes[current] is not None:
        path.insert(0, current)
        current = previous_nodes[current]
    if path:
        path.insert(0, start)
    
    return path if path else None, distances[goal]

@app.route('/astar', methods=['GET'])
def astar_endpoint():
    """Endpoint para el algoritmo A*"""
    start = request.args.get('start')
    goal = request.args.get('goal')

    if not start or not goal:
        return jsonify({'error': 'Se requieren los parámetros start y goal'}), 400

    campus_graph = get_campus_graph()
    
    if start not in campus_graph or goal not in campus_graph:
        return jsonify({'error': 'Nodo de inicio o fin no encontrado en el grafo'}), 404

    path, distance = a_star(campus_graph, start, goal)

    if path is None:
        return jsonify({'error': 'No se encontró un camino entre los nodos'}), 404

    path_coords = [{"node": node, "coords": campus_graph[node]["coords"]} for node in path]

    return jsonify({
        'algorithm': 'A*',
        'path': path,
        'path_coords': path_coords,
        'distance': distance,
        'units': 'metros'
    })

@app.route('/dijkstra', methods=['GET'])
def dijkstra_endpoint():
    """Endpoint para el algoritmo Dijkstra"""
    start = request.args.get('start')
    goal = request.args.get('goal')

    if not start or not goal:
        return jsonify({'error': 'Se requieren los parámetros start y goal'}), 400

    campus_graph = get_campus_graph()
    
    if start not in campus_graph or goal not in campus_graph:
        return jsonify({'error': 'Nodo de inicio o fin no encontrado en el grafo'}), 404

    path, distance = dijkstra(campus_graph, start, goal)

    if path is None:
        return jsonify({'error': 'No se encontró un camino entre los nodos'}), 404

    path_coords = [{"node": node, "coords": campus_graph[node]["coords"]} for node in path]

    return jsonify({
        'algorithm': 'Dijkstra',
        'path': path,
        'path_coords': path_coords,
        'distance': distance,
        'units': 'metros'
    })

if __name__ == '__main__':
    app.run(debug=True)