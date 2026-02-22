from algorithms.problems import SearchProblem
import algorithms.utils as utils
from world.game import Directions
from algorithms.heuristics import nullHeuristic


def tinyHouseSearch(problem: SearchProblem):
    """
    Returns a sequence of moves that solves tinyHouse. For any other building, the
    sequence of moves will be incorrect, so only use this for tinyHouse.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """

    stack = (
        utils.Stack()
    )  # stack para DFS ya que es un LIFO ENTONCES EXPLORMOS PUES EL MAS PROFUNDO
    visitado = (
        set()
    )  # conjunto para guardar los estados ya visitados y no volver a visitarlos

    estado_inicio = problem.getStartState()
    stack.push(
        (estado_inicio, [])
    )  # metemos el estado inicial a la pila junto con un path vacío

    while not stack.isEmpty():
        estado_actual, path = (
            stack.pop()
        )  # sacamos el estado actual y el path para llegar a el de la pila

        if problem.isGoalState(
            estado_actual
        ):  # si el estado actual es un estado objetivo, retornamos el path para llegar a él
            return path

        if (
            estado_actual in visitado
        ):  # si el estado actual ya fue visitado, lo saltamos para evitar ciclos
            continue

        visitado.add(
            estado_actual
        )  # si el estado actual no fue visitado, lo marcamos como visitado

        for sucesor, accion, cost in problem.getSuccessors(estado_actual):
            stack.push(
                (sucesor, path + [accion])
            )  # metemos cada sucesor del estado actual a la pila junto con el path actualizado para llegar a ese sucesor (el path actual más la acción necesaria para llegar al sucesor)


def breadthFirstSearch(problem: SearchProblem):
    """
    Search the shallowest nodes in the search tree first.
    """
    queue = (
        utils.Queue()
    )  # queue para BFS ya que es un FIFO entones exploramos a lo ancho
    visitado = (
        set()
    )  # Conjunto para guardar los estados ya visitados y no volver a visitarlos
    estado_inicio = problem.getStartState()
    queue.push(
        (estado_inicio, [])
    )  # Metemos el estado inicial a la cola junto con un path vacío
    while not queue.isEmpty():
        estado_actual, path = (
            queue.pop()
        )  # Sacamos el estado actual y el path para llegar a el de la cola
        if problem.isGoalState(
            estado_actual
        ):  # Si el estado actual es un estado objetivo, retornamos el path para llegar a él
            return path
        if (
            estado_actual in visitado
        ):  # Si el estado actual ya fue visitado, lo saltamos para evitar ciclos
            continue
        visitado.add(
            estado_actual
        )  # Si el estado actual no fue visitado, lo marcamos como visitado
        for sucesor, accion, cost in problem.getSuccessors(estado_actual):
            queue.push(
                (sucesor, path + [accion])
            )  # Metemos cada sucesor del estado actual a la cola junto con el path actualizado para llegar a ese sucesor (el path actual más la acción necesaria para llegar al sucesor)


def uniformCostSearch(problem: SearchProblem):

    cordi = problem.getStartState()  # (x, y)
    # Inicializar cosas
    pq = utils.PriorityQueue()
    bcost = {cordi: 0}
    # Primer elemento de la pq (donde empieza), path vacío, costo 0
    pq.push((cordi, [], 0), 0)

    while not pq.isEmpty():
        # Sacar de la pq
        state, path, cost = pq.pop()
        if problem.isGoalState(state):
            return path

        # Reviso el mapa de costos a vr si toca actualizar
        oldcost = bcost[state]
        # Si el costo actual es mayor q el viejo no actualizo y me salto la iteración
        if cost > oldcost:
            continue

        # Sino, sacarle los sucesores para volver a iterar
        for succ, action, stp_cost in problem.getSuccessors(state):
            # El nuevo costo es el costo del state actual + el de la accion q toma ir a succ
            newcost = cost + stp_cost
            # se le suma al path para llegar al state actual la acción necesaria para llegar a él
            newpath = path + [action]
            # Si el state no está en el mapa O si está pero el costo que tiene es mayor que el que acabo de calcular
            # Acabo de encontrar un camino mejor, actualizo el valor en el dict
            if succ not in bcost or bcost[succ] > newcost:
                bcost[succ] = newcost

            # Meto cada sucesor a la pq
            pq.push((succ, newpath, newcost), newcost)

    # si terminó todo y no encontró la fokin goal, retorna una lista vacía (sin acciones)
    return []


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    # PriorityQueue ordena los estados según f(n) = g(n) + h(n)
    frontier = utils.PriorityQueue()

    # estado inicial del problema
    start = problem.getStartState()

    # cada elemento de la frontier es:
    # (estado_actual, lista_de_acciones_para_llegar, costo_acumulado_real)
    frontier.push((start, [], 0), heuristic(start, problem))

    # visited guarda el menor costo conocido para cada estado
    visited = {}

    # loop de exploración
    while not frontier.isEmpty():

        # se obtiene el estado más tentador(/util?) usando f(n)
        state, actions, cost_so_far = frontier.pop()

        # si se llega al objetivo retorna el camino completo
        if problem.isGoalState(state):
            return actions

        # si ya fue alcanzado antes con menor costo entonces paq explorarlo otra vez
        if state in visited and visited[state] <= cost_so_far:
            continue

        # se registra el mejor costo conocido para este estado
        visited[state] = cost_so_far

        # obtenemos los estados vecinos accesibles desde el estado actual
        successors = problem.getSuccessors(state)

        # exploramos cada sucesor
        for successor, action, step_cost in successors:

            # nuevo costo acumulado real
            new_cost = cost_so_far + step_cost

            # actualizamos la lista de acciones realizadas
            new_actions = actions + [action]

            # prioridad A*
            # f(n) = g(n) + h(n)
            priority = new_cost + heuristic(successor, problem)

            # insertamos el nuevo estado en la frontier
            frontier.push((successor, new_actions, new_cost), priority)

    return []


# Abbreviations (you can use them for the -f option in main.py)
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
