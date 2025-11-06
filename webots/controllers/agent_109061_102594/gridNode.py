class GridNode:
    __slots__ = ("grid_coord", "world_pos", "neighbors", "visited")

    def __init__(self, grid_coord, world_pos, neighbors=None, visited=False):
        self.grid_coord = grid_coord  # (i, j)
        self.world_pos = world_pos    # (x, y)
        self.neighbors = list(neighbors) if neighbors else []
        self.visited = visited

    def add_neighbor(self, neighbor_node):
        self.neighbors.append(neighbor_node)

    def remove_neighbor(self, neighbor_node):
        self.neighbors.remove(neighbor_node)
        
    def mark_visited(self):
        self.visited = True

    def is_visited(self):
        return self.visited
    
    def __repr__(self):
        return f"GridNode(grid_coord={self.grid_coord}, world_pos={self.world_pos}, neighbors={self.neighbors}, visited={self.visited})"