from fastapi import FastAPI, Query, HTTPException
from src.core.grid import LLA, distance, lon_is_valid, lat_is_valid
from src.core.path_planner import PathPlan
from src.services.query import AsyncQueryHelper
import uvicorn
import json
import logging
import os
import asyncio
from typing import Optional
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import RedirectResponse


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)

CONFIG_PATH = "config/config.json"


def load_config(path: str):
    if not os.path.exists(path):
        raise FileNotFoundError(f"配置文件未找到：{path}")
    with open(path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    return cfg


config = load_config(CONFIG_PATH)
# 允许通过环境变量覆盖配置
SERVER_HOST = os.getenv("SERVER_HOST", config.get("server_host", "0.0.0.0"))
SERVER_PORT = int(os.getenv("SERVER_PORT", config.get("server_port", 8025)))
QUERY_HOST = os.getenv("QUERY_HOST", config.get("query_host", "192.168.3.12"))
QUERY_PORT = int(os.getenv("QUERY_PORT", config.get("query_port", 5555)))
QUERY_REQUEST = os.getenv("QUERY_REQUEST", config.get("query_request", "free/tinder/v3/box2/query"))
TILE_URL = os.getenv("TILE_URL", config.get("tile_url", "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"))

# 全局共享的查询助手实例（带缓存）
_global_query_helper: Optional[AsyncQueryHelper] = None

def get_query_helper() -> AsyncQueryHelper:
    """获取全局共享的查询助手实例（延迟初始化，支持跨请求缓存）"""
    global _global_query_helper
    if _global_query_helper is None:
        # 缓存精度：0.005度 ≈ 500米，相近的点会使用同一个缓存条目
        # 可根据需要调整为 0.005-0.01（500米-1公里）
        cache_precision = 0.005  # 约500米精度
        _global_query_helper = AsyncQueryHelper(
            QUERY_HOST, 
            QUERY_PORT, 
            QUERY_REQUEST,
            cache_size=1000,      # 缓存1000条查询结果
            cache_ttl=300,        # TTL 5分钟
            cache_precision=cache_precision  # 缓存精度：0.005度
        )
        logging.info(f"[Cache] 初始化全局查询助手，缓存配置: size=1000, ttl=300s, precision={cache_precision}度(≈{cache_precision*111:.0f}米)")
    return _global_query_helper


app = FastAPI(
    title="Route Planning Service",
    description="基于固定高度的路径规划HTTP服务",
    version="1.1.0"
)

# 开发用 CORS（允许本地文件或任意源调用 API）
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 挂载前端静态页面
app.mount("/web", StaticFiles(directory="web", html=True), name="web")

@app.get("/", include_in_schema=False)
async def index():
    return RedirectResponse(url="/web/")

@app.get("/web-config", summary="前端配置", tags=["Utils"])
async def web_config():
    return {"tile_url": TILE_URL}


@app.get("/query-alt", summary="查询点的代表性高程", tags=["Utils"])
async def query_alt(
        lon: float = Query(..., description="经度"),
        lat: float = Query(..., description="纬度"),
):
    """返回以传入点为中心查询得到的代表性高程（最近点）。"""
    try:
        QH = get_query_helper()  # 使用全局共享实例（带缓存）
        llas = await QH.query(lon, lat)
        if not llas:
            return {"status": "failed", "message": "该点附近无高程数据", "lon": lon, "lat": lat}

        def nearest_alt():
            best = min(llas, key=lambda p: distance(lon, lat, p.lon, p.lat))
            return {"lon": lon, "lat": lat, "query_alt": best.alt}

        res = nearest_alt()
        return {"status": "success", **res}
    except Exception as e:
        logging.error(f"query-alt error: {e}")
        return {"status": "failed", "message": str(e), "lon": lon, "lat": lat}


@app.get("/path-planning", summary="执行路径规划", tags=["Route"])
async def get_path(
        lon1: float = Query(..., description="起点经度"),
        lat1: float = Query(..., description="起点纬度"),
        lon2: float = Query(..., description="终点经度"),
        lat2: float = Query(..., description="终点纬度"),
        alt: float = Query(0, description="高度（米）")
):
    logging.info(f"Request: origin=({lon1}, {lat1}), target=({lon2}, {lat2}), alt={alt}")

    # 参数合法性详细校验
    invalid_fields = []
    if not lon_is_valid(lon1):
        invalid_fields.append({"field": "lon1", "value": lon1, "expect": "[-180, 180]"})
    if not lon_is_valid(lon2):
        invalid_fields.append({"field": "lon2", "value": lon2, "expect": "[-180, 180]"})
    if not lat_is_valid(lat1):
        invalid_fields.append({"field": "lat1", "value": lat1, "expect": "[-90, 90]"})
    if not lat_is_valid(lat2):
        invalid_fields.append({"field": "lat2", "value": lat2, "expect": "[-90, 90]"})
    if invalid_fields:
        return {
            "status": "failed",
            "error": "invalid_parameters",
            "message": "经纬度参数不合法",
            "invalid": invalid_fields
        }

    dist_km = distance(lon1, lat1, lon2, lat2)
    max_dist_km = 50.0
    if dist_km >= max_dist_km:
        return {
            "status": "failed",
            "error": "distance_too_long",
            "message": "两点规划距离过长",
            "distance_km": round(dist_km, 3),
            "limit_km": max_dist_km,
            "origin": {"lon": lon1, "lat": lat1},
            "target": {"lon": lon2, "lat": lat2}
        }

    try:
        QH = get_query_helper()  # 使用全局共享实例（带缓存）
        planning = PathPlan(QH.query_fn)
        ori = LLA(lon1, lat1, alt)
        ter = LLA(lon2, lat2, alt)
        # 将前端 alt 同步为 A* 的障碍阈值 thred，用于后续所有可行性判断
        planning._AStar.thred = alt

        # 先查询起点并构建网格
        local_data = await QH.query_fn(ori)
        if not local_data:
            return {
                "status": "failed",
                "error": "no_elevation_data_origin",
                "message": "起点附近缺少高程/可通行数据",
                "origin": {"lon": lon1, "lat": lat1}
            }
        
        # 构建起点网格
        planning._AStar.init(local_data)
        
        # 尝试从网格获取终点高程（如果终点在起点网格内）
        ter_data = None
        if planning._AStar.is_in_grid(ter):
            # 终点在网格内，直接从网格获取高程
            try:
                ter_idx = planning._AStar.get_index(ter)
                ter_alt = planning._AStar.altitude[ter_idx[0]][ter_idx[1]]
                # 构造返回格式（模拟查询结果，包含该点）
                ter_data = [LLA(ter.lon, ter.lat, ter_alt)]
                logging.debug(f"[GridCache] 终点从网格获取高程: {ter_alt:.2f}")
            except Exception as e:
                logging.warning(f"[GridCache] 从网格获取终点高程失败: {e}，回退到查询接口")
                ter_data = None
        
        # 如果终点不在网格内或获取失败，调用查询接口
        if ter_data is None:
            try:
                ter_data = await QH.query(ter.lon, ter.lat)
            except Exception as e:
                logging.error(f"终点查询异常: {e}")
                ter_data = None

        # 尝试终点附近是否可取到数据（作为提示）
        end_hint = {}
        if not ter_data:
            end_hint["no_elevation_data_target"] = True

        # 计算起点/终点查询到的代表性高程（取最近点）
        def nearest_alt(llas, lon, lat):
            if not llas:
                return None
            best = min(llas, key=lambda p: distance(lon, lat, p.lon, p.lat))
            return best.alt
        def nearest_point(llas, lon, lat):
            if not llas:
                return None
            best = min(llas, key=lambda p: distance(lon, lat, p.lon, p.lat))
            return {"lon": best.lon, "lat": best.lat, "alt": best.alt}
        origin_query_alt = nearest_alt(local_data, lon1, lat1)
        target_query_alt = nearest_alt(ter_data or [], lon2, lat2)
        origin_query_pt = nearest_point(local_data, lon1, lat1)
        target_query_pt = nearest_point(ter_data or [], lon2, lat2)

        # 网格已在之前初始化，直接使用
        start_idx = planning._AStar.get_index(ori)
        if not planning._AStar.moveable(start_idx):
            # 起点格的高程信息
            try:
                origin_cell_alt = planning._AStar.altitude[start_idx[0]][start_idx[1]]
            except Exception:
                origin_cell_alt = None
            return {
                "status": "failed",
                "error": "origin_blocked",
                "message": "起点所在网格为障碍，不可通，起点查询到的代表性高程为：" + str(origin_cell_alt),
                "origin": {"lon": lon1, "lat": lat1},
                "origin_cell_alt": origin_cell_alt,
                "thred": planning._AStar.thred
            }

        # 终点局部可达性提示
        if planning._AStar.is_in_grid(ter):
            end_idx = planning._AStar.get_index(ter)
            if not planning._AStar.moveable(end_idx):
                end_hint["end_blocked_local"] = True
        else:
            end_hint["end_out_of_local_grid"] = True

        # 直接调用异步方法，不需要 run_in_threadpool（因为已经是异步的）
        path, ok = await planning.PathPlanPair(ori, ter, alt)
    except Exception as e:
        logging.error(f"路径规划异常: {e}")
        return {
            "status": "failed",
            "error": "exception",
            "message": str(e)
        }

    if ok:
        logging.info(f"[SUCCESS] 规划成功, path length={len(path)}")

        def lla_to_dict(lla: LLA):
            return {"lon": lla.lon, "lat": lla.lat, "alt": lla.alt}
        origin_dict = {"lon": lon1, "lat": lat1, "alt": alt, "query_alt": origin_query_alt}
        target_dict = {"lon": lon2, "lat": lat2, "alt": alt, "query_alt": target_query_alt}
        core_path = [lla_to_dict(p) for p in path]
        full_path = [origin_dict] + core_path + [target_dict]
        return {
            "status": "success",
            "origin": origin_dict,
            "target": target_dict,
            "path": full_path
        }
    else:
        logging.warning(f"[FAILED] 规划失败: origin=({lon1},{lat1}), target=({lon2},{lat2})")
        return {
            "status": "failed",
            "error": "unreachable",
            "message": "终点不可达或当前数据条件下无法规划路径，终点查询到的代表性高程为：" + str(target_query_alt),
            "origin": {"lon": lon1, "lat": lat1, "alt": alt, "query_alt": origin_query_alt},
            "target": {"lon": lon2, "lat": lat2, "alt": alt, "query_alt": target_query_alt},
            **(end_hint if 'end_hint' in locals() else {})
        }


if __name__ == "__main__":
    logging.info(f"启动服务: host={SERVER_HOST}, port={SERVER_PORT}")
    logging.info(f"Query host={QUERY_HOST}:{QUERY_PORT}")
    uvicorn.run(app, host=SERVER_HOST, port=SERVER_PORT)



