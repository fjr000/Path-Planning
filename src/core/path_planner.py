import time
from typing import Optional, List
from .astar import AStar
from .grid import LLA, distance


def is_colinear(p1, p2, p3, tol=1e-6):
    """判断三点是否共线"""
    dx1, dy1 = p2.lon - p1.lon, p2.lat - p1.lat
    dx2, dy2 = p3.lon - p2.lon, p3.lat - p2.lat
    cross = dx1 * dy2 - dy1 * dx2
    return abs(cross) < tol


def merge_trajectories_smart(trajectory_segments, tol=0.0001):
    """
    合并多段轨迹：
    1. 去掉重复点与过近点；
    2. 合并成一条连续轨迹；
    3. 共线点只保留首尾两点。
    """
    if not trajectory_segments:
        return []

    merged = []
    for seg in trajectory_segments:
        if not seg:
            continue
        if not merged:
            merged.extend(seg)
        else:
            if merged[-1] == seg[0]:
                merged.extend(seg[1:])
            else:
                merged.extend(seg)

    filtered = [merged[0]]
    for p in merged[1:]:
        if distance(filtered[-1].lon, filtered[-1].lat, p.lon, p.lat) < tol:
            continue
        filtered.append(p)

    result = [filtered[0]]
    for i in range(1, len(filtered) - 1):
        if is_colinear(filtered[i - 1], filtered[i], filtered[i + 1]):
            continue
        result.append(filtered[i])
    result.append(filtered[-1])

    return result


def merge_trajectory(traj_list, dist_thresh=0.00001):
    """
    traj_list: [[LLA,...], [LLA,...], ...]
    dist_thresh: 距离小于此值认为是相近点，可以合并
    """
    merged_traj = []

    for traj in traj_list:
        if not traj:
            continue
        new_traj = [traj[0]]

        for i in range(1, len(traj)-1):
            prev, curr, nex = new_traj[-1], traj[i], traj[i+1]
            if distance(prev.lon,prev.lat, curr.lon, curr.lat) < dist_thresh:
                continue
            if is_colinear(prev, curr, nex):
                continue
            new_traj.append(curr)

        new_traj.append(traj[-1])
        merged_traj.append(new_traj)

    final_traj = []
    for traj in merged_traj:
        if not final_traj:
            final_traj.append(traj)
            continue
        last_traj = final_traj[-1]
        if distance(last_traj[-1].lon, last_traj[-1].lat, traj[0].lon, traj[0].lat) < dist_thresh:
            last_traj.extend(traj[1:])
        else:
            final_traj.append(traj)

    return final_traj


class PathPlan:
    def __init__(self, query_func):
        self._query_func = query_func
        self._AStar = AStar()
        self.visited_ori=set()

    def _update_grid(self, lla:LLA):
        query_data = self._query_func(lla)
        res = self._AStar.init(query_data)
        return res

    def PathPlan(self, ori:LLA, ter:LLA, thred:int):
        self._AStar.thred = thred
        cur_ori = ori
        self._update_grid(cur_ori)
        self._AStar.set_start(cur_ori)
        self._AStar.set_end(ter)
        new_ter_idx, _ = self._AStar.terminal_reset(cur_ori, ter)
        self._AStar.set_end_idx(new_ter_idx)
        print(f"cur_ori:{cur_ori}")
        paths = []
        path, ok = self._AStar.search()
        if ok:
            paths.append(path)
            cur_ori = paths[-1][-1]
        else:
            return [], ok

        while not self._AStar.is_in_grid(ter):
            self._update_grid(cur_ori)
            self._AStar.set_start(cur_ori)
            self._AStar.set_end(ter)
            new_ter_idx, _ = self._AStar.terminal_reset(cur_ori, ter)
            self._AStar.set_end_idx(new_ter_idx)
            self._AStar.print_grid()
            print(f"cur_ori:{cur_ori}, cur_ter:{ter}")

            path, ok = self._AStar.search()
            if ok:
                paths.append(path)
                cur_ori = paths[-1][-1]
            else:
                return [], ok
        return merge_trajectory(paths), ok

    def PathPlanPair(self, ori: LLA, ter: LLA, thred: float):
        """
        分块贪心路径规划。
        thred: 海拔高于 thred 认定为障碍
        """
        self._AStar.thred = thred
        cur_ori = ori
        paths = []
        self.visited_ori.add((cur_ori.lon, cur_ori.lat))

        def local_search(start: LLA, end: LLA):
            st = time.time()
            res = self._update_grid(start)
            if not res:
                print(f"高程信息缺失，查询点：{start}")
                return [], False, start
            ed = time.time()
            self._AStar.set_start(start)
            self._AStar.set_end(end)

            for i, new_ter_idx in enumerate(self._AStar.get_terminal_bound(start, end)):
                print(f"[LocalSearch] Try {i+1}: cur_ori={start}, cur_ter=:{self._AStar.index_to_lla(new_ter_idx)}")
                self._AStar.set_end_idx(new_ter_idx)
                path, ok = self._AStar.search()
                if ok and path:
                    next_point = path[-1]
                    ed2 = time.time()
                    return path, True, next_point
            ed2 = time.time()
            return [], False, start

        first_path, ok, cur_ori = local_search(cur_ori, ter)
        if not ok:
            print("初始局部区域内无法规划路径。")
            return [], False
        paths.append(first_path)

        while True:
            if (cur_ori.lon, cur_ori.lat) in self.visited_ori:
                print("贪心规划出现重复，搜索停止。需要全局搜索。")
                return [], False

            self.visited_ori.add((cur_ori.lon, cur_ori.lat))

            if self._AStar.get_index(cur_ori, if_clamp=False) == self._AStar.get_index(ter, if_clamp=False):
                break

            path, ok, new_ori = local_search(cur_ori, ter)
            if not ok:
                print("当前网格内无法继续前进，停止规划。")
                break

            paths.append(path)
            cur_ori = new_ori

            if self._AStar.get_index(cur_ori, if_clamp=False) == self._AStar.get_index(ter, if_clamp=False):
                break
        merge_path = merge_trajectories_smart(paths)

        for x in merge_path:
            x.alt = min(0.0,max(x.alt, thred))

        return merge_path, ok



