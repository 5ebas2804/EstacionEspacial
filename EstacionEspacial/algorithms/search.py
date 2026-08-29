from algorithms.problems import SearchProblem
import algorithms.utils as utils
from world.game import Directions
from algorithms.heuristics import nullHeuristic


def tinyDiagnosticSearch(problem: SearchProblem):
    """
    Returns a hard-coded sequence of moves for the tinyDiagnostic layout.
    For any other station layout, the sequence of moves will be incorrect.
    """
    s = Directions.SOUTH
    e = Directions.EAST
    return [s, e, s, e, e, e, e, s, e, e, s, s, e, s, s, e, s, e, e, e, e, e, e, e]


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
    # TODO: Add your code here
    stack = utils.Stack()
    nodo_inicial = problem.getStartState(), []
    stack.push(nodo_inicial)
    camino_visitado = set()
    while stack.isEmpty() is not True:
        estado, acciones = stack.pop()
        if estado in camino_visitado:
            continue
        if problem.isGoalState(estado):
            return acciones
        camino_visitado.add(estado)
        for sucesor, direccion, costo in problem.getSuccessors(estado):
            if sucesor not in camino_visitado:
                nuevo_nodo = sucesor, acciones + [direccion]
                stack.push(nuevo_nodo)
        
    return []


def breadthFirstSearch(problem: SearchProblem):
    """
    Search the shallowest nodes in the search tree first.
    """
    cola = utils.Queue()
    inicio = problem.getStartState()
    cola.push((inicio, []))
    visitados = utils.Counter()

    while not cola.isEmpty():
        actual, acciones = cola.pop()
        if problem.isGoalState(actual):
            return acciones

        for sucesor, accion, _ in problem.getSuccessors(actual):
            if sucesor not in visitados:
                visitados[sucesor] = 1
                cola.push((sucesor, acciones + [accion]))

    return []

def uniformCostSearch(problem: SearchProblem):
    """
    Search the node of least total cost first.
    """
    cola_prioridad = utils.PriorityQueue()
    estado_inicial = problem.getStartState()
    cola_prioridad.push((estado_inicial,[],0),0)
    visitados = utils.Counter()
    while not cola_prioridad.isEmpty():
        estado_actual, acciones, costo_actual = cola_prioridad.pop()
        
        if problem.isGoalState(estado_actual):
            return acciones
        if estado_actual not in visitados:
            visitados[estado_actual] = 1
            for sucesor, accion, costo_paso in problem.getSuccessors(estado_actual):
                nuevo_costo = costo_actual + costo_paso
                nuevas_acciones = acciones +[accion]
                cola_prioridad.push((sucesor, nuevas_acciones, nuevo_costo), nuevo_costo)
    return []


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    cola_prioridad = utils.PriorityQueue()
    estado_inicial = problem.getStartState()
    
    h_inicial = heuristic(estado_inicial, problem) # la prioridad es f(n) = g(n) + h(n)
    cola_prioridad.push((estado_inicial, [], 0), 0 + h_inicial) # en la cola guardams: (estado, acciones, costo_acumulado), prioridad con heursitica
    
    # Conjunto para registrar estados ya expandidos
    visitados = utils.Counter()

    while not cola_prioridad.isEmpty():
        estado_actual, acciones, costo_actual = cola_prioridad.pop()

        if problem.isGoalState(estado_actual):
            return acciones

        if estado_actual not in visitados:
            visitados[estado_actual] = 1

            for sucesor, accion, costo_paso in problem.getSuccessors(estado_actual):
                if sucesor not in visitados:
                    nuevo_costo = costo_actual + costo_paso ### g(n)
                    nuevas_acciones = acciones + [accion]
                    h_sucesor = heuristic(sucesor, problem) ### h(n)
                    f_val = nuevo_costo + h_sucesor ### f(n) = g(n) h(n)
                    
                    cola_prioridad.push((sucesor, nuevas_acciones, nuevo_costo), f_val)
        
    return []
        
        
    


# Abbreviations (you can use them for the -f option in main.py)
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
