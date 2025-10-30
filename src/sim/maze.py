import random
from typing import List, Tuple
from src.core.grid import LLA


class Maze:
    def __init__(self, num_lon=20, num_lat=20, step=0.02):
        self.num_lon = num_lon
        self.num_lat = num_lat
        self.step = step
        self.grid = [[1 for _ in range(num_lon)] for _ in range(num_lat)]
        self.start = (2, 2)
        self.end = (num_lon-3, num_lat-3)
        self._generate_maze()
        self.lla_grid = self._to_lla()

    # ---------------- Maze 生成 ----------------
    def _generate_maze(self):
        """使用 DFS 挖通法生成可达迷宫"""
        stack = [self.start]
        dirs = [(2, 0), (-2, 0), (0, 2), (0, -2)]
        self.grid[self.start[1]][self.start[0]] = 0

        while stack:
            x, y = stack[-1]
            random.shuffle(dirs)
            moved = False
            for dx, dy in dirs:
                nx, ny = x + dx, y + dy
                if 2 <= nx < self.num_lon -2  and 2 <= ny < self.num_lat - 2 and self.grid[ny][nx] == 1:
                    # 挖通路径
                    self.grid[y + dy // 2][x + dx // 2] = 0
                    self.grid[ny][nx] = 0
                    stack.append((nx, ny))
                    moved = True
                    break
            if not moved:
                stack.pop()

        # 保证终点可达
        self.grid[self.end[1]][self.end[0]] = 0

    def ensure_end_reachable(self):
        ex, ey = self.end
        if self.grid[ey][ex] == 1:
            neighbors = [(ex + dx, ey + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]]
            for nx, ny in neighbors:
                if 0 <= nx < self.num_lon and 0 <= ny < self.num_lat:
                    if self.grid[ny][nx] == 0:
                        self.grid[ey][ex] = 0
                        break
            else:
                nx, ny = neighbors[0]
                self.grid[ny][nx] = 0
                self.grid[ey][ex] = 0

    # ---------------- 辅助函数 ----------------
    def moveable(self, pos: Tuple[int, int]) -> bool:
        x, y = pos
        if 0 <= x < self.num_lon and 0 <= y < self.num_lat:
            return self.grid[y][x] == 0
        return False

    def _to_lla(self) -> List[List[LLA]]:
        """将 grid 转换为 LLA（alt>0表示障碍）"""
        lla_grid = []
        for i in range(self.num_lat):
            row = []
            for j in range(self.num_lon):
                lon = j * self.step
                lat = i * self.step
                alt = 1.0 if self.grid[i][j] == 1 else -5
                row.append(LLA(lon, lat, alt))
            lla_grid.append(row)
        return lla_grid

    def print_grid(self):
        """左下角为(0,0)，右上角为(num_lon-1, num_lat-1)"""
        for i in range(self.num_lat - 1, -1, -1):
            row = []
            for j in range(self.num_lon):
                pos = (j, i)
                if pos == self.start:
                    row.append("S" if self.moveable(pos) else "s")
                elif pos == self.end:
                    row.append("E" if self.moveable(pos) else "e")
                elif self.moveable(pos):
                    row.append("_")
                else:
                    row.append("X")
            print(" ".join(row))
        print()



