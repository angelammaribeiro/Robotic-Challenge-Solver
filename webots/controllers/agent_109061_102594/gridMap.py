# Estados possíveis das células do grid
UNKNOWN = 0
FREE = 1
OCCUPIED = 2
CHECKPOINT_1 = 3
CHECKPOINT_2 = 4
# Adicione mais checkpoints conforme necessário

class GridMap:
    def __init__(self, largura_m=14, altura_m=7.0, res=0.15):
        self.res = res
        self.largura_m = largura_m
        self.altura_m = altura_m
        self.cols = int(largura_m / res)
        self.rows = int(altura_m / res)
        self.grid = [[UNKNOWN for _ in range(self.cols)] for _ in range(self.rows)]
        self.x0 = None
        self.z0 = None

    def initialize_origin(self, x, z):
        self.x0 = x
        self.z0 = z

    def world_to_grid(self, x, z):
        if self.x0 is None or self.z0 is None:
            raise ValueError("Origem não inicializada!")
        i = round((x - self.x0) / self.res)
        j = round((z - self.z0) / self.res)
        i = max(0, min(self.rows - 1, i))
        j = max(0, min(self.cols - 1, j))
        return i, j

    def set_cell_state(self, i, j, state):
        """Atualiza o estado da célula (i, j)"""
        self.grid[i][j] = state

    def get_cell_state(self, i, j):
        """Retorna o estado da célula (i, j)"""
        return self.grid[i][j]