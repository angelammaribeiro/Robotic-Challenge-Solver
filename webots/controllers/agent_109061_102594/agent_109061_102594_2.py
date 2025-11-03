"""sample2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
from collections import deque
from gridMap import GridMap
from intersection import Intersection

sensor_angles = [
    math.pi/2,    # ps0
    math.pi/4,    # ps1
    0,            # ps2
    -math.pi/4,   # ps3
    -math.pi/2,   # ps4
    -3*math.pi/4, # ps5
    math.pi,      # ps6
    3*math.pi/4   # ps7
]

THRESHOLD_DISTANCE = 120.0  # Distância para detetar obstáculos

# Estados do robô
STATE_EXPLORING = "exploring"
STATE_BACKTRACKING = "backtracking"
STATE_NAVIGATING_TO_NODE = "navigating_to_node"

def is_edge_explored(edges, from_id, to_id):
    """
    Verifica se uma aresta entre dois nós já foi explorada.
    """
    for edge in edges:
        if edge['from'] == from_id and edge['to'] == to_id:
            return edge.get('explored', False)
    return False

def get_edge(edges, from_id, to_id):
    """
    Retorna a aresta entre dois nós, se existir.
    """
    for edge in edges:
        if edge['from'] == from_id and edge['to'] == to_id:
            return edge
    return None

def all_edges_explored_from_node(edges, node, intersections):
    """
    Verifica se todas as arestas saindo de um nó foram exploradas.
    """
    # Para cada direção do nó
    for idx, (direction, target_cell) in enumerate(node.heading_options):
        if not node.visited_flags[idx]:
            # Direção não visitada = aresta não explorada
            return False
    return True

def bfs_path(grid, start, goal):
    """
    Encontra o caminho mais curto entre start e goal usando BFS.
    Retorna uma lista de células (i, j) do caminho.
    """
    from collections import deque
    
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0
    
    # Verifica se start e goal são válidos
    if not (0 <= start[0] < rows and 0 <= start[1] < cols):
        return []
    if not (0 <= goal[0] < rows and 0 <= goal[1] < cols):
        return []
    
    queue = deque([(start, [start])])
    visited = {start}
    
    while queue:
        (i, j), path = queue.popleft()
        
        if (i, j) == goal:
            return path
        
        # Explora vizinhos (cima, baixo, esquerda, direita)
        for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            ni, nj = i + di, j + dj
            
            # Verifica limites e se a célula é livre (1) ou checkpoint (3)
            if (0 <= ni < rows and 0 <= nj < cols and 
                (ni, nj) not in visited and 
                grid[ni][nj] in [1, 3]):  # Livre ou checkpoint
                
                visited.add((ni, nj))
                queue.append(((ni, nj), path + [(ni, nj)]))
    
    return []  # Nenhum caminho encontrado

def run_robot(robot):

    # Initialize
    grid_map = GridMap()

    intersections = []
    node_id_counter = 0
    stack = deque()  # Pilha para DFS usando deque (mais eficiente)
    edges = []  # Lista de arestas (caminhos entre nós)
    
    # Estado de exploração
    current_state = STATE_EXPLORING
    current_node = None
    target_position = None
    current_path = []  # Caminho atual sendo percorrido
    path_to_follow = []  # Caminho a seguir durante navegação

    # Initialize timestep and speed
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28

    # Initialize motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')

    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # Initialize sensors
    # Initialize GPS
    gps = robot.getDevice('gps')
    gps.enable(timestep)

    # Initialize distance sensors
    num_dist_sensors = 8  # Number of distance sensors
    dist_sensors = [robot.getDevice('ps' + str(x)) for x in range(num_dist_sensors)]
    for sensor in dist_sensors:
        sensor.enable(timestep)

    # Initialize camera
    camera = robot.getDevice('camera')
    camera.enable(timestep)

    # Initialize compass
    compass = robot.getDevice('compass')
    compass.enable(timestep)

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read sensor values
        gps_values = gps.getValues()  # Get GPS coordinates [x, y, z]

        if grid_map.x0 is None:
            grid_map.initialize_origin(gps_values[0], gps_values[2])

        compass_values = compass.getValues()  # Get compass values
        dist_sensor_values = [sensor.getValue() for sensor in dist_sensors]  # Get distance sensor values

        localization = (gps_values[0], gps_values[2])  # Use x and z for localization

        yaw = math.atan2(compass_values[0], compass_values[2])

        i, j = grid_map.world_to_grid(localization[0], localization[1])
        grid_map.set_cell_state(i, j, 1)  # Marca célula como livre
        current_orientation = yaw #keeps track of orientation
        
        # Regista a célula atual no caminho
        if not current_path or current_path[-1] != (i, j):
            current_path.append((i, j))
        
        # Print sensor information
        print("GPS coordinates [x, y, z]:", gps_values)
        print("Compass values:", compass_values)
        print("Distance sensors:", dist_sensor_values)
        print("Custom data:", robot.getCustomData())

        # Chama a função para projetar dados dos sensores no grid
        project_sensor_data_to_grid(grid_map, dist_sensors, localization[0], localization[1], yaw)

        # Reseta heading_options a cada timestep
        heading_options = []
        
        if grid_map.get_cell_state(i-1, j) == 1:
            heading_options.append(('N', (i-1, j)))
        if grid_map.get_cell_state(i+1, j) == 1:
            heading_options.append(('S', (i+1, j)))
        if grid_map.get_cell_state(i, j-1) == 1:
            heading_options.append(('W', (i, j-1)))
        if grid_map.get_cell_state(i, j+1) == 1:
            heading_options.append(('E', (i, j+1)))

        # Verifica se é uma interseção (3+ direções) e se não está já registada
        is_already_registered = any(node.map_coord == (i, j) for node in intersections)
        
        if len(heading_options) >= 3 and not is_already_registered:
            intersection = Intersection(
                id=node_id_counter,
                map_coord=(i, j),
                world_pos=(localization[0], localization[1]),
                heading_options=heading_options
            )
            intersections.append(intersection)
            stack.append(intersection)  # Adiciona à pilha para explorar depois (DFS)
            grid_map.set_cell_state(i, j, 3)  # Marca célula como CHECKPOINT_1
            node_id_counter += 1
            print(f"Nova interseção criada: ID {intersection.id} em {(i, j)}")
            print(f"Interseções na pilha: {len(stack)}")
            
            # Se não estamos a explorar um nó, usa este como nó atual
            if current_node is None:
                current_node = intersection
            
            # Se vínhamos de outro nó, guarda a aresta
            elif current_node != intersection:
                edge = {
                    'from': current_node.id,
                    'to': intersection.id,
                    'path': current_path.copy(),
                    'explored': True  # Marca como explorada ao criar (caminho percorrido)
                }
                edges.append(edge)
                print(f"Aresta criada e explorada: Nó {current_node.id} -> Nó {intersection.id} (caminho com {len(current_path)} células)")
                
                # Guarda também aresta inversa para backtracking
                edge_back = {
                    'from': intersection.id,
                    'to': current_node.id,
                    'path': current_path[::-1],  # Caminho invertido
                    'explored': True  # Também explorada (mesmo caminho, direção inversa)
                }
                edges.append(edge_back)
                
                current_node = intersection
                current_path = [(i, j)]  # Reinicia o caminho
        
        # Lógica DFS com backtracking
        if current_state == STATE_EXPLORING:
            # Verifica se estamos numa interseção
            current_intersection = None
            for node in intersections:
                if node.map_coord == (i, j):
                    current_intersection = node
                    break
            
            if current_intersection:
                # Procura uma direção não visitada
                has_unvisited = False
                for idx, visited in enumerate(current_intersection.visited_flags):
                    if not visited:
                        # Marca como visitada
                        current_intersection.visited_flags[idx] = True
                        direction, target_cell = current_intersection.heading_options[idx]
                        print(f"Explorando direção {direction} da interseção {current_intersection.id}")

                        # Guarda a célula destino para navegação
                        # (Aqui poderias implementar lógica para mover o robô até target_cell)
                        
                        # Adiciona o nó de volta à pilha se ainda tiver direções não visitadas
                        if not all(current_intersection.visited_flags):
                            if current_intersection not in stack:
                                stack.append(current_intersection)
                        
                        has_unvisited = True
                        break
                
                # Se todas as direções foram visitadas, fazer backtrack
                if not has_unvisited:
                    print(f"Todas as direções exploradas na interseção {current_intersection.id}")
                    if stack:
                        # Remove da pilha se estava lá
                        if current_intersection in stack:
                            stack.remove(current_intersection)
                        
                        # Volta para o próximo nó da pilha
                        next_node = stack[-1]
                        print(f"Backtracking para interseção {next_node.id}")
                        
                        # Procura caminho guardado (aresta)
                        edge_found = None
                        for edge in edges:
                            if edge['from'] == current_intersection.id and edge['to'] == next_node.id:
                                edge_found = edge
                                break
                        
                        if edge_found:
                            print(f"Usando caminho guardado ({len(edge_found['path'])} células)")
                            path_to_follow = edge_found['path']
                        else:
                            # Calcula caminho com BFS
                            print("Calculando caminho com BFS...")
                            path_to_follow = bfs_path(grid_map.grid, current_intersection.map_coord, next_node.map_coord)
                            if path_to_follow:
                                print(f"Caminho BFS encontrado ({len(path_to_follow)} células)")
                        
                        current_state = STATE_BACKTRACKING
                        target_position = next_node.world_pos
                        current_node = next_node
                        current_path = []  # Reinicia o caminho
        
        # Verifica se a exploração está completa
        if len(intersections) > 0:
            all_explored = all(all_edges_explored_from_node(edges, node, intersections) 
                             for node in intersections)
            if all_explored and len(stack) == 0:
                print("=" * 50)
                print("EXPLORAÇÃO COMPLETA!")
                print(f"Total de interseções: {len(intersections)}")
                print(f"Total de arestas: {len(edges)}")
                print("=" * 50)

        # Set motor velocities
        right_motor.setVelocity(0.25 * max_speed)
        left_motor.setVelocity(0.25 * max_speed)


    # Enter here exit cleanup code.

def project_sensor_data_to_grid(grid_map, dist_sensors, x, z, yaw):
    for idx, sensor in enumerate(dist_sensors):
        d = sensor.getValue()
        angle = yaw + sensor_angles[idx]
        x_obst = x + d * math.cos(angle)
        z_obst = z + d * math.sin(angle)
        i, j = grid_map.world_to_grid(x_obst, z_obst)
        if d < THRESHOLD_DISTANCE:
            grid_map.set_cell_state(i, j, 2)  # Marca como OCCUPIED
        else:
            grid_map.set_cell_state(i, j, 1)  # Marca como FREE

if __name__ == '__main__' :
    my_robot = Robot()
    run_robot(my_robot)
