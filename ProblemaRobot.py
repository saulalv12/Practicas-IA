import heapq
import math
from collections import deque

graph = {
    '1,1': {'coords': (1, 1), 'neighbors': ['1,2', '2,1']},
    '1,2': {'coords': (1, 2), 'neighbors': ['1,1', '1,3']},
    '1,3': {'coords': (1, 3), 'neighbors': ['1,2', '1,4', '2,3']},
    '1,4': {'coords': (1, 4), 'neighbors': ['1,3', '2,4']},
    '2,1': {'coords': (2, 1), 'neighbors': ['1,1']},
    '2,3': {'coords': (2, 3), 'neighbors': ['2,4', '1,3']},
    '2,4': {'coords': (2, 4), 'neighbors': ['2,3', '1,4', '3,4']},
    '3,4': {'coords': (3, 4), 'neighbors': ['2,4', '4,4']},
    '4,4': {'coords': (4, 4), 'neighbors': ['4,3', '3,4']}
}

# Función para calcular la distancia euclidiana entre dos nodos
def euclidean_distance(node1, node2):
    x1, y1 = graph[node1]['coords']
    x2, y2 = graph[node2]['coords']
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Implementación del algoritmo de búsqueda por anchura (BFS)
def bfs(graph, start, goal):
    visited = set()
    queue = deque([(start, [start], 0)])

    while queue:
        node, path, depth = queue.popleft()

        if node == goal:
            print(f"Se encontró el nodo objetivo: {node}")
            print(f"Ruta más corta: {path}")
            print(f"Número de niveles (profundidad): {depth}")
            return path

        if node not in visited:
            visited.add(node)
            print(f"Nodo visitado: {node} en el nivel {depth}")

            for neighbor in graph[node]['neighbors']:
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor], depth + 1))
    
    print("No se encontró una ruta al nodo objetivo")
    return None

# Implementación del algoritmo de búsqueda en profundidad (DFS)
def dfs(graph, start, goal):
    visited = set()
    stack = [start]
    parent = {start: None}

    while stack:
        node = stack.pop()

        if node == goal:
            print(f"Se encontró el nodo objetivo: {node}")
            print(f"Nodos por explorar: {stack}")
            path = []
            while node is not None:
                path.append(node)
                node = parent[node]
            path.reverse()
            print(f"Ruta encontrada: {path}")
            return path
        
        if node not in visited:
            visited.add(node)
            print(f"Nodo visitado: {node}")
            print(f"Nodos por explorar: {stack}")

            for neighbor in reversed(graph[node]['neighbors']):
                if neighbor not in visited:
                    stack.append(neighbor)
                    parent[neighbor] = node
    
    print("No se encontró una ruta al nodo objetivo")
    return None

# Implementación del algoritmo de búsqueda Primero el Mejor 
def bfis(graph, start, goal):
    visited = set()
    priority_queue = []
    heapq.heappush(priority_queue, (euclidean_distance(start, goal), start))
    parent = {start: None}

    while priority_queue:
        h, node = heapq.heappop(priority_queue)

        if node == goal:
            print(f"Se encontró el nodo objetivo: {node}")
            path = []
            while node is not None:
                path.append(node)
                node = parent[node]
            path.reverse()
            print(f"Ruta encontrada: {path}")
            return path
        
        if node not in visited:
            visited.add(node)
            print(f"Nodo visitado: {node} con heurística {h}")

            for neighbor in graph[node]['neighbors']:
                if neighbor not in visited:
                    heapq.heappush(priority_queue, (euclidean_distance(neighbor, goal), neighbor))
                    parent[neighbor] = node
    
    print("No se encontró una ruta al nodo objetivo")
    return None

# Ejemplo de uso
if __name__ == "__main__":
    print("**Búsqueda por Anchura**")
    bfs(graph, '1,1', '4,4')
 
    print("\n**Búsqueda en Profundidad**")
    dfs(graph, '1,1', '4,4')

    print("\n**Búsqueda Primero el Mejor**")
    bfis(graph, '1,1', '4,4')