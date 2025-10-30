from typing import List
from src.core.grid import LLA
from .maze import Maze


def query_area(lon: float, lat: float, maze: Maze, range_blocks: int = 3) -> List[LLA]:
    """
    返回以查询点所在块为中心的 range_blocks × range_blocks 块（一维数组）。
    每块包含 block_size × block_size 点。
    排序：从左到右，从下到上。
    range_blocks 必须是奇数，如 3、5、7
    """
    assert range_blocks % 2 == 1, "range_blocks 必须为奇数"

    block_size = 2
    num_blocks_x = maze.num_lon // block_size
    num_blocks_y = maze.num_lat // block_size

    block_x = int(lon / (block_size * maze.step))
    block_y = int(lat / (block_size * maze.step))

    half_range = range_blocks // 2
    block_x = max(half_range, min(num_blocks_x - 1 - half_range, block_x))
    block_y = max(half_range, min(num_blocks_y - 1 - half_range, block_y))

    llas = []
    for by in range(block_y - half_range, block_y + half_range + 1):
        for bx in range(block_x - half_range, block_x + half_range + 1):
            for dy in range(block_size):
                for dx in range(block_size):
                    gx = bx * block_size + dx
                    gy = by * block_size + dy
                    if 0 <= gx < maze.num_lon and 0 <= gy < maze.num_lat:
                        llas.append(maze.lla_grid[gy][gx])
    llas.sort(key=lambda x: (x.lon, x.lat))
    return llas



