from collections import deque
import tree_hanoi
import hanoi_states
import heapq  # Para manejar la frontera como una cola de prioridad


def breadth_first_tree_search(problem: hanoi_states.ProblemHanoi):
    """
    Realiza una búsqueda en anchura para encontrar una solución a un problema de Hanoi.
    Esta función no chequea si un estado se visito, por lo que puede entrar en Loop infinitos muy fácilmente. No
    usarla con más de 3 discos.

    Parameters:
        problem (hanoi_states.ProblemHanoi): El problema de la Torre de Hanoi a resolver.

    Returns:
        tree_hanoi.NodeHanoi: El nodo que contiene la solución encontrada.
    """
    frontier = deque([tree_hanoi.NodeHanoi(problem.initial)])  # Creamos una cola FIFO con el nodo inicial
    while frontier:
        node = frontier.popleft()  # Extraemos el primer nodo de la cola
        if problem.goal_test(node.state):  # Comprobamos si hemos alcanzado el estado objetivo
            return node
        frontier.extend(node.expand(problem))  # Agregamos a la cola todos los nodos sucesores del nodo actual

    return None


def breadth_first_graph_search(problem: hanoi_states.ProblemHanoi, display: bool = False):
    """
    Realiza una búsqueda en anchura para encontrar una solución a un problema de Hanoi. Pero ahora si recuerda si ya
    paso por un estado e ignora seguir buscando en ese nodo para evitar recursividad.

    Parameters:
        problem (hanoi_states.ProblemHanoi): El problema de la Torre de Hanoi a resolver.
        display (bool, optional): Muestra un mensaje de cuantos caminos se expandieron y cuantos quedaron sin expandir.
                                  Por defecto es False.

    Returns:
        tree_hanoi.NodeHanoi: El nodo que contiene la solución encontrada.
    """

    frontier = deque([tree_hanoi.NodeHanoi(problem.initial)])  # Creamos una cola FIFO con el nodo inicial

    explored = set()  # Este set nos permite ver si ya exploramos un estado para evitar repetir indefinidamente
    while frontier:
        node = frontier.popleft()  # Extraemos el primer nodo de la cola

        # Agregamos nodo al set. Esto evita guardar duplicados, porque set nunca tiene elementos repetidos, esto sirve
        # porque heredamos el método __eq__ en tree_hanoi.NodeHanoi de aima.Node
        explored.add(node.state)

        if problem.goal_test(node.state):  # Comprobamos si hemos alcanzado el estado objetivo
            if display:
                print(len(explored), "caminos se expandieron y", len(frontier), "caminos quedaron en la frontera")
            return node
        # Agregamos a la cola todos los nodos sucesores del nodo actual que no haya visitados
        frontier.extend(child for child in node.expand(problem)
                        if child.state not in explored and child not in frontier)

    return None

def a_star_search(problem_hanoi, display: bool = False):
    """
    Método de búsqueda A* para resolver el problema de las Torres de Hanoi.

    Parameters:
        problem_hanoi (ProblemHanoi): Instancia del problema de las Torres de Hanoi.

    Returns:
        StatesHanoi: El estado objetivo alcanzado al resolver el problema.
    """
    # Estado inicial y configuración de la frontera
    initial_state = problem_hanoi.initial
    goal_state = problem_hanoi.goal
    frontier = []  # Cola de prioridad
    heapq.heappush(frontier, (0, tree_hanoi.NodeHanoi(initial_state)))  # Inserta el nodo inicial con costo 0
    explored = set()  # Conjunto de estados ya explorados

    # Diccionario para rastrear el camino hacia el objetivo
    path_to_goal = []  # Lista para almacenar los movimientos hacia el objetivo

    while frontier:
        # Extraemos el nodo con el menor valor de f(n) = g(n) + h(n)
        _, current_node = heapq.heappop(frontier)

        # Si el estado actual es el objetivo, hemos encontrado la solución
        if problem_hanoi.goal_test(current_node.state):
            if display:
                print("Secuencia de movimientos para resolver el problema:")
                for move in path_to_goal:
                    print(f"Movimiento: Disco {move['disk']} de la varilla {move['peg_start']} a la varilla {move['peg_end']}")
                print(len(explored), "caminos se expandieron y", len(frontier), "caminos quedaron en la frontera")
            return current_node

        # Agregar el estado actual al conjunto de explorados
        explored.add(current_node.state)

        # Imprime el nodo actual
        #print(f"Nodo actual: {current_node.state}")

        # Lista para almacenar las fronteras
        fronteras = []

        for action in problem_hanoi.actions(current_node.state):
            child_node = current_node.child_node(problem_hanoi, action)
            child_state = child_node.state

            # Si el hijo no ha sido explorado, calcular f(n) = g(n) + h(n)
            if child_state not in explored:


                # g(n): costo acumulado hasta el nodo hijo
                g_cost = current_node.path_cost + action.cost
                # h(n): heurística - número de discos fuera de la posición final
                h_cost = heuristic_hanoi(child_state, goal_state)
                f_cost = g_cost + h_cost

                # Almacenar el nodo en la frontera con su costo total
                heapq.heappush(frontier, (f_cost, child_node))

    return None  # Si no se encuentra una solución

def heuristic_hanoi_backup(state, goal_state):
    """
    Heurística para A* que cuenta el número de discos fuera de la posición final.

    Parameters:
        state (StatesHanoi): Estado actual.
        goal_state (StatesHanoi): Estado objetivo.

    Returns:
        int: Número de discos fuera de su posición final.
    """
    # Contar discos fuera de la posición final
    misplaced_disks = 0
    for rod_index in range(3):
        if rod_index != 2:  # Solo la tercera varilla debe contener los discos en orden en el estado objetivo
            misplaced_disks += len(state.rods[rod_index])
    return misplaced_disks



def heuristic_hanoi_b2(state, goal_state):
    """
    Heurística para A* que cuenta el número de discos fuera de la posición final.

    Parameters:
        state (StatesHanoi): Estado actual.
        goal_state (StatesHanoi): Estado objetivo.

    Returns:
        int: Número de discos fuera de su posición final.
    """
    misplaced_disks = 0

    # Contar discos en las varillas incorrectas (primera y segunda varilla)
    for rod_index in range(2):  # Solo las dos primeras varillas
        misplaced_disks += len(state.rods[rod_index])

    # Contar discos mal ubicados en la tercera varilla (varilla objetivo)
    # Los discos deben estar en orden descendente para estar bien colocados.
    correct_disks = goal_state.rods[2]
    for i, disk in enumerate(state.rods[2]):
        # Comparar con la posición en el estado objetivo
        if i < len(correct_disks) and disk != correct_disks[i]:
            misplaced_disks += 1

    return misplaced_disks

def heuristic_hanoib3(state, goal_state):
    """
    Heurística mejorada para A* que estima el número de movimientos necesarios
    para alcanzar el estado final en la tercera varilla.

    Parameters:
        state (StatesHanoi): Estado actual.
        goal_state (StatesHanoi): Estado objetivo.

    Returns:
        int: Estimación de movimientos restantes para alcanzar el estado objetivo.
    """
    estimated_cost = 0
    goal_disks = goal_state.rods[2]  # Discos en la tercera varilla en el estado final

    # Paso 1: Penalización por discos en las varillas incorrectas (primera y segunda)
    for rod_index in range(2):
        estimated_cost += len(state.rods[rod_index])  # Cada disco en varillas incorrectas suma un costo

    # Paso 2: Penalización por discos mal ubicados en la tercera varilla
    # Contamos desde el disco superior en la tercera varilla y verificamos el orden
    for i, disk in enumerate(state.rods[2]):
        if i < len(goal_disks):
            if disk != goal_disks[i]:
                # Si el disco no está en el orden correcto, penalizar
                estimated_cost += 2  # Penalización mayor para discos fuera de orden en la tercera varilla
        else:
            # Discos adicionales en la tercera varilla que no pertenecen al objetivo
            estimated_cost += 1

    return estimated_cost

def heuristic_hanoib4(state, goal_state):
    """
    Heurística para A* que estima el número de movimientos necesarios
    para alcanzar el estado final en la tercera varilla, sin preocuparse
    por el orden en la misma varilla (ya que se respetan las reglas).

    Parameters:
        state (StatesHanoi): Estado actual.
        goal_state (StatesHanoi): Estado objetivo.

    Returns:
        int: Estimación de movimientos restantes para alcanzar el estado objetivo.
    """
    # Calcular el número de discos que faltan en la tercera varilla
    # El objetivo es tener todos los discos en la tercera varilla.
    total_disks = sum(len(rod) for rod in state.rods)
    disks_in_third_rod = len(state.rods[2])

    # La heurística será el número de discos que aún faltan en la tercera varilla
    estimated_cost = total_disks - disks_in_third_rod

    return estimated_cost

def heuristic_hanoi(state, goal_state):
    """
    Heurística para A* que cuenta el número de discos fuera de la posición final en comparación con el estado objetivo.

    Parameters:
        state (StatesHanoi): Estado actual.
        goal_state (StatesHanoi): Estado objetivo.

    Returns:
        int: Número de discos fuera de su posición final.
    """
    misplaced_disks = 0

    # Paso 1: Contar discos en las varillas incorrectas (primera y segunda)
    for rod_index in range(2):
        misplaced_disks += len(state.rods[rod_index])

    # Paso 2: Comparar los discos en la tercera varilla con el goal_state
    for i, disk in enumerate(state.rods[2]):
        if i < len(goal_state.rods[2]):
            # Si el disco en la posición `i` no coincide con el estado objetivo
            if disk != goal_state.rods[2][i]:
                misplaced_disks += 1
        else:
            # Discos adicionales que no deberían estar en la tercera varilla
            misplaced_disks += 1

    return misplaced_disks


