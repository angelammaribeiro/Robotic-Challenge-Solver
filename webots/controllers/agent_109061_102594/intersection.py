
class Intersection : 
    def __init__(self, id, map_coord, world_pos, heading_options):
        self.id = id  # Identificador único da interseção
        self.map_coord = map_coord  # Coordenadas na grade (i, j)
        self.world_pos = world_pos  # Posição no mundo real (x, z)
        self.heading_options = heading_options  # Opções de direção disponíveis (N, E, S, W)
        self.visited_flags = [False] * len(heading_options)  # Flags para marcar direções visitadas