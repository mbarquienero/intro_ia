import tracemalloc
import time
import timeit
from hanoi_states import StatesHanoi, ProblemHanoi
from tree_hanoi import NodeHanoi
from search import (  # Importa las funciones de búsqueda del módulo search
    breadth_first_tree_search,
    breadth_first_graph_search,
    a_star_search
)

def main():
    """
    Función principal que resuelve el problema de la Torre de Hanoi y genera los JSON para el simulador.
    """
    # Define el estado inicial y el estado objetivo del problema
    #initial_state = StatesHanoi([5,4,3,2,1], [], [], max_disks=5)
    #goal_state = StatesHanoi([], [], [5,4,3,2,1], max_disks=5)
    initial_state = StatesHanoi([3,2,1], [], [], max_disks=3)
    goal_state = StatesHanoi([], [], [3,2,1], max_disks=3)

    # Crea una instancia del problema de la Torre de Hanoi
    problem_hanoi = ProblemHanoi(initial=initial_state, goal=goal_state)

    # Para medir tiempo consumido
    start_time = time.perf_counter()
    # Para medir memoria consumida (usamos el pico de memoria)
    tracemalloc.start()

    # Métodos no informados

    # Resuelve el problema utilizando búsqueda en anchura
    # Esta forma de búsqueda es muy ineficiente, por lo que si deseas probarlo, usa 3 discos o si querés esperar
    # un poco más, 4 discos, pero 5 discos no finaliza nunca.
    #last_node = breadth_first_tree_search(problem_hanoi)

    # Resuelve el problema utilizando búsqueda en anchura, pero con memoria que recuerda caminos ya recorridos.
    #last_node = breadth_first_graph_search(problem_hanoi, display=True)
    last_node = a_star_search(problem_hanoi, display=True)

    _, memory_peak = tracemalloc.get_traced_memory()
    memory_peak /= 1024*1024
    tracemalloc.stop()

    end_time = time.perf_counter()
    elapsed_time = end_time - start_time

    #=========================================================
    """def run_a_star_search():
        a_star_search(problem_hanoi, display=False)

    repetitions = 1

    # Calcula el tiempo total de ejecución
    total_time = timeit.timeit(run_a_star_search, number=repetitions)

    # Calcula el promedio
    average_time = total_time / repetitions
    print(f"Tiempo promedio: {average_time:.5f} segundos")"""
    #=========================================================

    if isinstance(last_node, NodeHanoi):
        # Imprime la longitud del camino de la solución encontrada
        print(f'Longitud del camino de la solución: {last_node.state.accumulated_cost}')

        # Genera los JSON para el simulador
        last_node.generate_solution_for_simulator()

    else:
        print(last_node)
        print("No se encuentra solución")

    # Imprime las métricas medidas
    print(f"Tiempo que demoró: {elapsed_time} [s]", )
    print(f"Maxima memoria ocupada: {round(memory_peak, 2)} [MB]", )


# Sección de ejecución del programa
if __name__ == "__main__":
    main()
