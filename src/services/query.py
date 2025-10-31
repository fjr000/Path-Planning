import requests
import httpx
import logging
import json
import asyncio
from typing import Optional, List
from cachetools import TTLCache
from src.core.grid import LLA


class QueryHelper:
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8025,
        request_path: str = "free/tinder/v3/box2/query",
        timeout: float = 5.0
    ):
        self.host = host
        self.port = port
        self.server = f"http://{host}:{port}/"
        self.request_path = request_path
        self.timeout = timeout

    def query(self, lon: float, lat: float, size: int = 3) -> Optional[List[LLA]]:
        url = f"{self.server}{self.request_path}?lon={lon}&lat={lat}&size={size}"
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            data_json = response.json()
            data_list = data_json.get("data", [])
            if not data_list:
                return None
            lla_data = [LLA(item["lon"], item["lat"], item.get("alt", 0)) for item in data_list]
            return lla_data
        except requests.RequestException as e:
            print(f"[QueryHelper] HTTP 请求错误: {e}")
            return None
        except json.JSONDecodeError as e:
            print(f"[QueryHelper] JSON 解析错误: {e}")
            return None
        except KeyError as e:
            print(f"[QueryHelper] 返回数据缺少字段: {e}")
            return None

    def query_fn(self, lla: LLA) -> Optional[List[LLA]]:
        return self.query(lla.lon, lla.lat)


class AsyncQueryHelper:
    def __init__(
        self, 
        host="127.0.0.1", 
        port=8025, 
        request="free/tinder/v3/box2/query", 
        timeout=5.0,
        cache_size=1000,
        cache_ttl=300,
        cache_precision=0.005
    ):
        self.host = host
        self.port = port
        self.server = f"http://{host}:{port}/"
        self.request = request
        self.timeout = timeout
        
        # 查询结果缓存：使用 TTL + LRU 策略
        # cache_size: 最多缓存条目数
        # cache_ttl: 缓存过期时间（秒），默认5分钟
        # cache_precision: 缓存精度（度），默认0.005度（约500米）
        #   0.005度 ≈ 500米，0.01度 ≈ 1公里
        self.cache_precision = cache_precision
        self._cache = TTLCache(maxsize=cache_size, ttl=cache_ttl)
        self._cache_lock = asyncio.Lock()
        self._hit_count = 0
        self._miss_count = 0

    def _make_cache_key(self, lon: float, lat: float, size: int = 3):
        """
        生成缓存键，按照指定精度四舍五入。
        默认精度0.005度 ≈ 500米，相近的点会使用同一个缓存条目。
        """
        # 根据精度计算小数位数（例如0.005需要3位小数，0.01需要2位小数）
        # 使用更安全的方式：直接除以精度后四舍五入，再乘以精度
        lon_rounded = round(lon / self.cache_precision) * self.cache_precision
        lat_rounded = round(lat / self.cache_precision) * self.cache_precision
        return (lon_rounded, lat_rounded, size)

    async def query(self, lon: float, lat: float, size: int = 3):
        """
        查询高程数据，带缓存支持。
        缓存键按照指定精度（默认0.005度≈500米）进行四舍五入，提高缓存命中率。
        """
        # 生成缓存键（按精度四舍五入）
        cache_key = self._make_cache_key(lon, lat, size)
        
        # 尝试从缓存获取
        async with self._cache_lock:
            if cache_key in self._cache:
                self._hit_count += 1
                logging.debug(f"[QueryCache] HIT: ({lon:.6f}, {lat:.6f})")
                return self._cache[cache_key]
        
        # 缓存未命中，执行查询
        self._miss_count += 1
        logging.debug(f"[QueryCache] MISS: ({lon:.6f}, {lat:.6f})")
        
        url = f"{self.server}{self.request}?lon={lon}&lat={lat}&size={size}"
        try:
            async with httpx.AsyncClient(timeout=self.timeout) as client:
                resp = await client.get(url)
                resp.raise_for_status()
                data_json = resp.json()
                data_list = data_json.get("data", [])
                if not data_list:
                    # 查询结果为空也缓存（避免重复查询无效点）
                    result = None
                else:
                    result = [LLA(item["lon"], item["lat"], item.get("alt", 0)) for item in data_list]
                
                # 存入缓存
                async with self._cache_lock:
                    self._cache[cache_key] = result
                
                return result
        except Exception as e:
            logging.error(f"[QueryHelper] 异步查询失败: {e}")
            return None

    async def query_fn(self, lla: LLA):
        return await self.query(lla.lon, lla.lat)
    
    def get_cache_stats(self):
        """获取缓存统计信息（用于监控）"""
        async def _get_stats():
            async with self._cache_lock:
                total = self._hit_count + self._miss_count
                hit_rate = self._hit_count / total if total > 0 else 0.0
                return {
                    "cache_size": len(self._cache),
                    "hit_count": self._hit_count,
                    "miss_count": self._miss_count,
                    "hit_rate": hit_rate
                }
        return _get_stats()


def query_func(lon, lat, size = 3):
    server = "http://192.168.3.12:5555/"
    request = server + f"free/tinder/v3/box2/query?lon={lon}&lat={lat}&size={size}"
    response = requests.get(request)
    data =response.content
    parse_data = json.loads(data)
    data = parse_data['data']
    if data:
        lla_data = [LLA(x['lon'], x['lat'], x['alt']) for x in data]
        return lla_data
    return None


def query_fn(lla:LLA):
    return query_func(lla.lon,lla.lat,3)



