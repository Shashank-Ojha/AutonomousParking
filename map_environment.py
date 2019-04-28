MARGIN = 5

class Map_Environment(object):
    @staticmethod
    def read_file(path):
      with open(path, "rt") as f:
          return f.read()

    def parse_map(self, map_file):
      contents = self.read_file(map_file)
      data = contents.split()
      num_rows = int(data[0])
      num_cols = int(data[1])
      data = data[2:]
      map_env = []
      for i in range(num_rows):
          col = []
          for j in range(num_cols):
              col.append(int(data[num_cols * i + j]))
          map_env.append(col)
      return map_env
    


    def __init__(self, file, grid_width, grid_height):
        self.margin = MARGIN
        self.grid_width = grid_width
        self.grid_height = grid_height

        self.map = self.parse_map(file)
        self.rows = len(self.map)
        self.cols = len(self.map[0])
        self.cell_width = (self.grid_width - (2*self.margin)) / self.cols
        self.cell_height = (self.grid_height - (2*self.margin)) / self.rows



