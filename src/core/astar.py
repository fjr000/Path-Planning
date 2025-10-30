import heapq
from typing import Dict, Tuple, List
from .grid import *
MAXMAX = 10**9


@dataclass
class Point2D:
    x: float = 0.0
    y: float = 0.0


def lla_to_ned(ori: 'LLA', ter: 'LLA') -> Point2D:
    """
    简化版的 LLA -> NED（仅用于判断方位的符号/象限）
    返回 (X, Y) 分量（单位与 distance() 一致），其中
    X ≈ 东向距离（lon 方向），Y ≈ 北向距离（lat 方向）
    """
    x = distance(ori.lon, ori.lat, ter.lon, ori.lat)
    if ter.lon < ori.lon:
        x = -x
    y = distance(ori.lon, ori.lat, ori.lon, ter.lat)
    if ter.lat < ori.lat:
        y = -y
    return Point2D(x, y)


class AStar(Grid):
    """A* 搜索（8 邻域），继承 Grid。"""
    def heuristic8d_idx(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """基于网格索引的 8-连通 启发式（使用 gap_lon/gap_lat 作为尺度）"""
        len_lon = abs(a[0] - b[0]) * self.gap_lon
        len_lat = abs(a[1] - b[1]) * self.gap_lat
        return (math.sqrt(2) - 2) * min(len_lon, len_lat) + len_lon + len_lat

    def heuristic8d_lla(self, a: 'LLA', b: 'LLA') -> float:
        """基于 LLA 的 8D 启发式（通过 distance 在经纬方向上投影）"""
        len_lon = distance(a.lon, a.lat, b.lon, a.lat)
        len_lat = distance(a.lon, a.lat, a.lon, b.lat)
        return (math.sqrt(2) - 2) * min(len_lon, len_lat) + len_lon + len_lat

    def heuristic4d_lla(self, a: 'LLA', b: 'LLA') -> float:
        len_lon = distance(a.lon, a.lat, b.lon, a.lat)
        len_lat = distance(a.lon, a.lat, a.lon, b.lat)
        return len_lon + len_lat

    # --- 设置起终点（支持索引或 LLA） ---
    def set_start_idx(self, idx: Tuple[int, int]):
        x = clamp(idx[0], 0, max(0, self.num_lon - 1))
        y = clamp(idx[1], 0, max(0, self.num_lat - 1))
        self.start = (x, y)

    def set_start(self, a: 'LLA'):
        self.set_start_idx(self.get_index(a))

    def set_end_idx(self, idx: Tuple[int, int]):
        x = clamp(idx[0], 0, max(0, self.num_lon - 1))
        y = clamp(idx[1], 0, max(0, self.num_lat - 1))
        self.end = (x, y)

    def set_end(self, a: 'LLA'):
        self.set_end_idx(self.get_index(a))

    def get_terminal_bound(self, ori, ter):
        # 计算边界点，若ter不可直接到达，则通过启发式规则得到其他可能边界点

        pq = []
        ori_idx = self.get_index(ori)
        ter_idx = self.get_index(ter)

        if self.is_in_grid(ter):
            pq.append((self.heuristic8d_idx(ori_idx, ter_idx),self.get_index(ter)))

        # 入堆
        def pq_append(pos):
            ter_idx = self.get_index(ter, if_clamp=False)
            if self.moveable(pos):
                g = self.heuristic8d_idx(ori_idx, pos)
                h = self.heuristic8d_idx(ter_idx, pos)
                f= g+h
                pq.append((f, pos))

        # 四边界点入堆
        for lon in range(self.num_lon):
            pos = (lon,0)
            pq_append(pos)

        for lon in range(self.num_lon):
            pos = (lon,self.num_lat-1)
            pq_append(pos)

        for lat in range(1,self.num_lat-1):
            pos = (0,lat)
            pq_append(pos)

        for lat in range(1,self.num_lat-1):
            pos = (self.num_lon-1,lat)
            pq_append(pos)

        # 建堆
        heapq.heapify(pq)
        # 合并同质化点（四方向线性合并），减少A*算法运行次数
        visited = set()
        while pq:
            _, pos = heapq.heappop(pq)
            if pos not in visited:
                lon, lat = pos
                visited.add(pos)
                # 处理经度边界（东西边界）：沿纬度方向（上下）扩展
                if lon == 0 or lon == self.num_lon - 1:
                    # 向下（lat 减小）
                    for l_lat in range(lat-1, -1, -1):
                        nxt_pos = (lon, l_lat)
                        if self.moveable(nxt_pos)and (nxt_pos not in visited):
                            visited.add(nxt_pos)
                        else:
                            break
                    # 向上（lat 增大）
                    for r_lat in range(lat + 1, self.num_lat):
                        nxt_pos = (lon, r_lat)
                        if self.moveable(nxt_pos)and (nxt_pos not in visited):
                            visited.add(nxt_pos)
                        else:
                            break

                # 处理纬度边界（南北边界）：沿经度方向（左右）扩展
                if lat == 0 or lat == self.num_lat - 1:
                    # 向左（lon 减小）
                    for l_lon in range(lon-1, -1, -1):
                        nxt_pos = (l_lon, lat)
                        if self.moveable(nxt_pos) and (nxt_pos not in visited):
                            visited.add(nxt_pos)
                        else:
                            break
                    # 向右（lon 增大）
                    for r_lon in range(lon + 1, self.num_lon):
                        nxt_pos = (r_lon, lat)
                        if self.moveable(nxt_pos)and (nxt_pos not in visited):
                            visited.add(nxt_pos)
                        else:
                            break

                yield pos

    # --- 终点重置（尝试寻找可行的临近格子）---
    def terminal_reset(self, ori: 'LLA', ter: 'LLA', change_direct: bool = False) -> Tuple[Tuple[int, int], bool]:
        """
        将 ter 映射为网格索引，如果不可通行，则沿边缘/次优方向搜索可通行格子。
        返回 (ter_idx, flag)。
        """
        top = 0
        right = 0
        ter_pos = lla_to_ned(ori, ter)
        if ter_pos.x > 0:
            top = 1
        if ter_pos.y > 0:
            right = 1
        if change_direct:
            top = 1 - top
            right = 1 - right

        ori_idx = self.get_index(ori)
        real_ter_idx = self.get_index(ter)
        ter_idx = real_ter_idx
        min_dist = MAXMAX
        flag = self.moveable(ter_idx)

        # 如果目标不可通行，优先在相应边界方向搜索第一个可通行点
        if not flag:
            x0, y0 = real_ter_idx
            # 尝试在横/纵边界方向查找（与原逻辑保持一致）
            # 优先沿 x 方向（行）搜索
            if y0 == 0 or y0 == self.num_lat - 1:
                if right:
                    rng = range(x0, self.num_lon)
                else:
                    rng = range(x0, -1, -1)
                for i in rng:
                    if self.moveable((i, y0)):
                        dist = abs(i - x0)
                        if dist < min_dist:
                            ter_idx = (i, y0)
                            min_dist = dist
                            flag = True
                            break
            # 尝试沿 y 方向（列）搜索
            if not flag and (x0 == 0 or x0 == self.num_lon - 1):
                if top:
                    rng = range(y0, self.num_lat)
                else:
                    rng = range(y0, -1, -1)
                for j in rng:
                    if self.moveable((x0, j)):
                        dist = abs(j - y0)
                        if dist < min_dist:
                            ter_idx = (x0, j)
                            min_dist = dist
                            flag = True
                            break

        # 如果仍不可通行，尝试反向方向的同类搜索（与原代码重复两次逻辑保持一致）
        if not flag:
            x0, y0 = real_ter_idx
            if y0 == 0 or y0 == self.num_lat - 1:
                if right:
                    rng = range(x0, -1, -1)
                else:
                    rng = range(x0, self.num_lon)
                for i in rng:
                    if self.moveable((i, y0)):
                        dist = abs(i - x0)
                        if dist < min_dist:
                            ter_idx = (i, y0)
                            min_dist = dist
                            flag = True
                            break

            if not flag and (x0 == 0 or x0 == self.num_lon - 1):
                if top:
                    rng = range(y0, -1, -1)
                else:
                    rng = range(y0, self.num_lat)
                for j in rng:
                    if self.moveable((x0, j)):
                        dist = abs(j - y0)
                        if dist < min_dist:
                            ter_idx = (x0, j)
                            min_dist = dist
                            flag = True
                            break

        return ter_idx, flag

    # --- 直线可行性检查 ---
    def straight_check(self, ori: 'LLA', ter: 'LLA', ori_idx: Tuple[int, int], ter_idx: Tuple[int, int]) -> bool:
        """
        在 ori->ter 直线方向上以若干采样点检测网格是否可通行（返回 True 表示整条直线可通）
        """
        diff_lon = ter.lon - ori.lon
        diff_lat = ter.lat - ori.lat
        sample_num = max(abs(ori_idx[0] - ter_idx[0]) + abs(ori_idx[1] - ter_idx[1]), 20)
        step_lon = diff_lon / sample_num
        step_lat = diff_lat / sample_num

        for k in range(1, sample_num + 1):
            sample_lla = LLA(ori.lon + step_lon * k, ori.lat + step_lat * k, 0.0)
            idx = self.get_index(sample_lla)
            if not self.moveable(idx):
                return False
        return True

    # --- A* 路径规划（8 邻域）---
    def path_plan(self) -> Tuple[List[Tuple[int, int]], bool]:
        """
        返回 (path_list, success)，path_list 为索引对列表（从 start 到 end）。
        若失败返回 ([], False)。
        """
        if not self.altitude:
            return [], False

        start = self.start
        end = self.end

        # 优先队列项： (f, counter, x, y)
        open_heap: List[Tuple[float, int, int, int]] = []
        counter = 0

        start_idx = start[0] * self.num_lat + start[1]
        # g_costs: idx -> g
        g_costs: Dict[int, float] = {start_idx: 0.0}
        # parent: idx -> parent_idx
        parent: Dict[int, int] = {start_idx: start_idx}

        start_f = self.heuristic8d_idx(start, end)
        heapq.heappush(open_heap, (start_f, counter, start[0], start[1]))
        counter += 1
        closed: set = set()

        while open_heap:
            f, _, cx, cy = heapq.heappop(open_heap)
            cur_idx = cx * self.num_lat + cy

            # 已经扩展过则跳过
            if cur_idx in closed:
                continue

            # 目标到达
            if (cx, cy) == end:
                break

            closed.add(cur_idx)

            # 遍历 8 邻域
            for dx, dy in self.dir_8D:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < self.num_lon and 0 <= ny < self.num_lat):
                    continue
                n_idx = nx * self.num_lat + ny
                if n_idx in closed:
                    continue
                if not self.moveable((nx, ny)):
                    continue

                # 代价：当前 g + cost(cur->next)
                step_cost = self.heuristic8d_idx((cx, cy), (nx, ny))
                tentative_g = g_costs.get(cur_idx, math.inf) + step_cost

                # 如果不是 open 或者找到更优 g
                if tentative_g < g_costs.get(n_idx, math.inf):
                    g_costs[n_idx] = tentative_g
                    parent[n_idx] = cur_idx
                    h = self.heuristic8d_idx((nx, ny), end)
                    heapq.heappush(open_heap, (tentative_g + h, counter, nx, ny))
                    counter += 1

        # 回溯路径
        end_idx = end[0] * self.num_lat + end[1]
        if end_idx not in parent:
            return [], False

        path_idx_list: List[Tuple[int, int]] = []
        cur = end_idx
        while True:
            x = cur // self.num_lat
            y = cur % self.num_lat
            path_idx_list.append((x, y))
            if cur == parent[cur]:
                break
            cur = parent[cur]
            if len(path_idx_list) > self.num_lon * self.num_lat + 5:
                # 保护性中断（防止死循环）
                return [], False

        path_idx_list.reverse()
        return path_idx_list, len(path_idx_list) > 1

    def search(self) -> Tuple[List['LLA'], bool]:
        """
        执行 PathPlan 并返回 LLA 路径与是否成功。
        """
        path_idx, ok = self.path_plan()
        if not ok:
            return [], False
        res: List[LLA] = [self.index_to_lla(p) for p in path_idx]
        return res, True


if __name__ == "__main__":
    data = [LLA(lon, lat, alt) for lon, lat, alt in zip(
        [100 + i * 0.01 for i in range(8)],
        [30 + i * 0.01 for i in range(8)],
        [i for i in range(8)]
    )]

    astar = AStar()
    astar.init(data)
    astar.start = (0, 0)
    astar.end = (5, 5)
    astar.print_grid()

    path = astar.path_plan()
    print("路径坐标索引：", path)
    print("路径对应经纬高：")
    for p in path:
        print(astar.index_to_lla(p))



