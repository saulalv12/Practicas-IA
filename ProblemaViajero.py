from collections import deque
import heapq

# Definición del grafo
graph = {
    'A': ['B', 'C'],
    'B': ['A', 'D'],
    'C': ['A', 'D', 'F'],
    'D': ['B', 'C', 'E'],
    'E': ['D', 'F'],
    'F': ['C', 'E', 'G'],
    'G': ['F']
}

# Definición de la heurística
heuristic = {
    'A': 15,
    'B': 10,
    'C': 13,
    'D': 8,
    'E': 6,
    'F': 3,
    'G': 0
}

# Implementación del algoritmo de búsqueda por anchura (BFS)
def bfs(graph, start, goal):
    visited = set()  # Conjunto para almacenar los nodos visitados
    queue = deque([(start, [start], 0)])  # Cola para explorar los nodos, almacena el nodo, la ruta y la profundidad

    while queue:
        node, path, depth = queue.popleft()  # Sacar el nodo, la ruta y la profundidad de la cola

        if node == goal:
            print(f"Se encontró el nodo objetivo: {node}")
            print(f"Ruta más corta: {path}")
            print(f"Número de niveles (profundidad): {depth}")
            return path

        if node not in visited:
            visited.add(node)  # Marcar el nodo como visitado
            print(f"Nodo visitado: {node} en el nivel {depth}")

            # Añadir los vecinos no visitados a la cola
            for neighbor in graph[node]:
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor], depth + 1))
    
    print("No se encontró una ruta al nodo objetivo")
    return None

# Implementación del algoritmo de búsqueda en profundidad (DFS) 
def dfs(graph, start, goal):
    visited = set()  # Conjunto para almacenar los nodos visitados
    stack = [start]  # Pila que almacena solo nodos
    parent = {start: None}  # Diccionario para rastrear la ruta

    while stack:
        node = stack.pop()  # Sacar el nodo de la pila (simula un "desapilado")
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

            # Agregar los vecinos no visitados a la pila
            for neighbor in reversed(graph[node]):
                if neighbor not in visited:
                    stack.append(neighbor)
                    parent[neighbor] = node  # Guardar el padre del vecino para reconstruir la ruta
    
    print("No se encontró una ruta al nodo objetivo")
    return None

# Implementación del algoritmo de búsqueda Primero el Mejor 
def bfis(graph, start, goal, heuristic):
    visited = set()  # Conjunto para almacenar los nodos visitados
    priority_queue = []  # Cola de prioridad para seleccionar el mejor nodo
    heapq.heappush(priority_queue, (heuristic[start], start))  # Añadir el nodo inicial a la cola

    parent = {start: None}  # Para reconstruir la ruta

    while priority_queue:
        # Extraer el nodo con la menor heurística
        h, node = heapq.heappop(priority_queue)

        if node == goal:
            print(f"Se encontró el nodo objetivo: {node}")
            # Reconstruir la ruta desde el nodo objetivo al nodo inicial
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
            
            # Agregar los vecinos a la cola de prioridad
            for neighbor in graph[node]:
                if neighbor not in visited:
                    heapq.heappush(priority_queue, (heuristic[neighbor], neighbor))
                    parent[neighbor] = node
    
    print("No se encontró una ruta al nodo objetivo")
    return None


if __name__ == "__main__":
    print("**Búsqueda por Anchura**")
    bfs(graph, 'A', 'G')
    
    print("\n**Búsqueda en Profundidad**")
    dfs(graph, 'A', 'G')

    print("\n**Búsqueda Primero el Mejor**")
    bfis(graph, 'A', 'G', heuristic)