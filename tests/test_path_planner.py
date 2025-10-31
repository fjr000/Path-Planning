import asyncio
from src.sim.maze import Maze
from src.sim.area_query import query_area
from src.core.grid import LLA
from src.core.path_planner import PathPlan


async def main():
    num_lon = 20
    num_lat = 20
    step = 0.02
    maze = Maze(num_lon,num_lat,step=step)
    maze.print_grid()

    def func(lla:LLA):
        return query_area(lla.lon, lla.lat, maze,5)

    planning = PathPlan(func)

    ori = LLA(maze.start[0]*step, maze.start[1]*step, -5)
    ter = LLA(maze.end[0]*step,maze.end[0]*step,-6)
    paths, ok = await planning.PathPlanPair(ori, ter, 0)

    if ok:
        print(paths)
    else:
        print("Faild!")


if __name__ == "__main__":
    asyncio.run(main())



