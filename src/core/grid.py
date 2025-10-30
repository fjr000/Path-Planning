import math
from dataclasses import dataclass
from typing import List, Tuple
from collections import  defaultdict


@dataclass
class LLA:
    lon: float
    lat: float
    alt: float

    def __repr__(self):
        return f"LLA(lon={self.lon:.4f}, lat={self.lat:.4f}, alt={self.alt:.2f})"


def lon_is_valid(lon):
    return -180<=lon<=180


def lat_is_valid(lat):
    return -90<=lat<=90


def distance(lon1: float, lat1: float, lon2: float, lat2: float) -> float:
    """近似计算两点间地表距离（单位：km）"""
    R = 6371.0  # 地球半径 km
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    return 2 * R * math.asin(math.sqrt(a))


def clamp(x: int, low: int, high: int) -> int:
    return max(low, min(x, high))


class Grid:
    def __init__(self, thred = -10):
        self.dir_8D = [
            (0, 1), (1, 1), (1, 0), (1, -1),
            (0, -1), (-1, -1), (-1, 0), (-1, 1)
        ]
        self.thred = thred
        self.min_lon = math.inf
        self.max_lon = -math.inf
        self.min_lat = math.inf
        self.max_lat = -math.inf
        self.gap_lon = 3e-3
        self.gap_lat = 3e-3
        self.num_lon = 0
        self.num_lat = 0
        self.start: Tuple[int, int] = (0, 0)
        self.end: Tuple[int, int] = (0, 0)
        self.altitude: List[List[float]] = []

    # 经纬高有效性检测
    def lon_is_valid(self, lon: float) -> bool:
        return lon_is_valid(lon)

    def lat_is_valid(self, lat: float) -> bool:
        return lat_is_valid(lat)

    def alt_is_valid(self, alt: float) -> bool:
        return alt > -32767

    # 网格有效性判定
    def is_valid(self, a: Tuple[int, int]) -> bool:
        return 0 <= a[0] < self.num_lon and 0 <= a[1] < self.num_lat

    def is_obstacle(self, a: Tuple[int, int]) -> bool:
        return self.altitude[a[0]][a[1]] > self.thred

    def moveable(self, a: Tuple[int, int]) -> bool:
        return self.is_valid(a) and not self.is_obstacle(a)

    def is_in_grid(self, lla:LLA):
        return self.min_lon <= lla.lon <=self.max_lon and self.min_lat <= lla.lat <= self.max_lat

    def data_init(self, data: List[LLA], init_data: List[LLA]):
        self.min_lon = math.inf
        self.min_lat = math.inf
        self.max_lon = -math.inf
        self.max_lat = -math.inf

        sq = math.sqrt(len(data))
        self.num_lon = self.num_lat = math.ceil(sq)

        cur_gap_lat = 0
        if len(data) > 1 and self.num_lat > 1:
            cur_gap_lat = (data[-1].lat - data[0].lat) / (self.num_lat - 1) * 0.9

        init_data[:] = data.copy()
        pre_lla = data[0]

        for idx, pos in enumerate(init_data):
            flag = 0
            if self.lon_is_valid(pos.lon):
                self.min_lon = min(self.min_lon, pos.lon)
                self.max_lon = max(self.max_lon, pos.lon)
            else:
                flag = 1

            if self.lat_is_valid(pos.lat):
                self.min_lat = min(self.min_lat, pos.lat)
                self.max_lat = max(self.max_lat, pos.lat)
            else:
                flag = 1

            if not self.alt_is_valid(pos.alt):
                flag = 1

            if flag:
                for i in range(len(init_data)):
                    for new_idx in [idx + i, idx - i]:
                        if 0 <= new_idx < len(init_data):
                            if not self.lon_is_valid(pos.lon) and self.lon_is_valid(init_data[new_idx].lon):
                                pos.lon = init_data[new_idx].lon
                            if not self.alt_is_valid(pos.alt) and self.alt_is_valid(init_data[new_idx].alt):
                                pos.alt = init_data[new_idx].alt
                            if self.lon_is_valid(pos.lon) and self.alt_is_valid(pos.alt):
                                break
                    else:
                        continue
                    break
                pos.lat = pre_lla.lat + cur_gap_lat
            pre_lla = pos

    def get_index(self, lla: LLA, if_clamp = True) -> Tuple[int, int]:
        diff_lon = lla.lon - self.min_lon
        diff_lat = lla.lat - self.min_lat
        x = round(diff_lon / self.gap_lon)
        y = round(diff_lat / self.gap_lat)
        if if_clamp:
            x = clamp(x, 0, self.num_lon - 1)
            y = clamp(y, 0, self.num_lat - 1)
        return (x, y)

    def index_to_lla(self, idx: Tuple[int, int]) -> LLA:
        lon_idx = clamp(idx[0], 0, self.num_lon - 1)
        lat_idx = clamp(idx[1], 0, self.num_lat - 1)
        return LLA(
            lon_idx * self.gap_lon + self.min_lon,
            lat_idx * self.gap_lat + self.min_lat,
            self.altitude[lon_idx][lat_idx]
        )

    def init(self, data: List[LLA]):
        if not data:
            return False
        init_data: List[LLA] = []
        self.data_init(data, init_data)

        len_gap_lon = distance(self.min_lon, self.min_lat, self.max_lon, self.min_lat)
        len_gap_lat = distance(self.min_lon, self.min_lat, self.min_lon, self.max_lat)

        self.gap_lon = 0
        self.gap_lat = 0
        if self.num_lat > 1:
            len_gap_lat /= (self.num_lat - 1)
            self.gap_lat = (self.max_lat - self.min_lat) / (self.num_lat - 1)
        if self.num_lon > 1:
            len_gap_lon /= (self.num_lon - 1)
            self.gap_lon = (self.max_lon - self.min_lon) / (self.num_lon - 1)

        self.altitude = [[0.0 for _ in range(self.num_lat)] for _ in range(self.num_lon)]
        curGap = len_gap_lon * 0.5 + len_gap_lat * 0.5

        idx = 0
        for i in range(self.num_lon):
            for j in range(self.num_lat):
                dist = distance(init_data[idx].lon, init_data[idx].lat,
                                self.min_lon + i * self.gap_lon,
                                self.min_lat + j * self.gap_lat)
                count = -1
                new_idx = idx
                min_gap = math.inf
                min_idx = idx
                center_lon = self.min_lon + i * self.gap_lon
                center_lat = self.min_lat + j * self.gap_lat

                while dist >= curGap *0.8 and count < len(init_data) - 1:
                    new_idx = (idx + count + 1) % len(init_data)
                    dist = distance(init_data[new_idx].lon, init_data[new_idx].lat,
                                    center_lon, center_lat)
                    if dist < min_gap:
                        min_idx = new_idx
                        min_gap = dist
                    count += 1

                if count == len(init_data):
                    idx = min_idx
                    curGap = min_gap * 0.8
                else:
                    idx = new_idx
                self.altitude[i][j] = init_data[idx].alt
        return True

    # 将数据按块划分
    def _build_blocks(self, data: List['LLA'], block_size: int = 5):
        block_dict = defaultdict(list)
        for p in data:
            bx = round((p.lon - self.min_lon) / (self.gap_lon * block_size))
            by = round((p.lat - self.min_lat) / (self.gap_lat * block_size))
            block_dict[(bx, by)].append(p)
        return block_dict

    def _find_nearest_in_blocks(self, lon, lat, block_dict, bx, by, max_search=3):
        """只在附近块中找最近点"""
        best_p, best_d = None, float('inf')
        for r in range(1, max_search + 1):
            found = False
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    pts = block_dict.get((bx + dx, by + dy))
                    if not pts:
                        continue
                    for p in pts:
                        d = distance(lon, lat, p.lon, p.lat)
                        if d < best_d:
                            best_d = d
                            best_p = p
                            found = True
            if found:
                break
        return best_p, best_d

    def init2(self, data: List['LLA'], block_size=5):
        if not data:
            return False

        init_data: List[LLA] = []
        self.data_init(data, init_data)

        len_gap_lon = distance(self.min_lon, self.min_lat, self.max_lon, self.min_lat)
        len_gap_lat = distance(self.min_lon, self.min_lat, self.min_lon, self.max_lat)

        self.gap_lon = 0
        self.gap_lat = 0
        if self.num_lat > 1:
            len_gap_lat /= (self.num_lat - 1)
            self.gap_lat = (self.max_lat - self.min_lat) / (self.num_lat - 1)
        if self.num_lon > 1:
            len_gap_lon /= (self.num_lon - 1)
            self.gap_lon = (self.max_lon - self.min_lon) / (self.num_lon - 1)

        self.altitude = [[0.0 for _ in range(self.num_lat)] for _ in range(self.num_lon)]

        block_dict = self._build_blocks(data, block_size)

        for i in range(self.num_lon):
            for j in range(self.num_lat):
                lon = self.min_lon + i * self.gap_lon
                lat = self.min_lat + j * self.gap_lat

                bx = round((lon - self.min_lon) / (self.gap_lon * block_size))
                by = round((lat - self.min_lat) / (self.gap_lat * block_size))

                nearest, dist = self._find_nearest_in_blocks(lon, lat, block_dict, bx, by)
                if nearest:
                    self.altitude[i][j] = nearest.alt
                else:
                    self.altitude[i][j] = 9.999999  # 没找到点时默认0
        return True

    def print_grid(self):
        for i in range(self.num_lat - 1, -1, -1):
            row = []
            for j in range(self.num_lon):
                if (j, i) == self.start:
                    row.append("S" if self.moveable(self.start) else "s")
                elif (j, i) == self.end:
                    row.append("E" if self.moveable(self.end) else "e")
                elif self.moveable((j, i)):
                    row.append("_")
                else:
                    row.append("X")
            print(" ".join(row))
        print()


# 示例使用
if __name__ == "__main__":
    # 创建假数据
    data = [LLA(lon, lat, alt) for lon, lat, alt in zip(
        [100 + i * 0.01 for i in range(8)],
        [30 + i * 0.01 for i in range(8)],
        [i for i in range(8)]
    )]

    grid = Grid()
    grid.init(data)
    grid.start = (0, 0)
    grid.end = (2, 2)
    grid.print_grid()



