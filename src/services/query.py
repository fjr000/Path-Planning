import requests
import httpx
import logging
import json
from typing import Optional, List
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
    def __init__(self, host="127.0.0.1", port=8025, request="free/tinder/v3/box2/query", timeout=5.0):
        self.host = host
        self.port = port
        self.server = f"http://{host}:{port}/"
        self.request = request
        self.timeout = timeout

    async def query(self, lon: float, lat: float, size: int = 3):
        url = f"{self.server}{self.request}?lon={lon}&lat={lat}&size={size}"
        try:
            async with httpx.AsyncClient(timeout=self.timeout) as client:
                resp = await client.get(url)
                resp.raise_for_status()
                data_json = resp.json()
                data_list = data_json.get("data", [])
                if not data_list:
                    return None
                return [LLA(item["lon"], item["lat"], item.get("alt", 0)) for item in data_list]
        except Exception as e:
            logging.error(f"[QueryHelper] 异步查询失败: {e}")
            return None

    async def query_fn(self, lla: LLA):
        return await self.query(lla.lon, lla.lat)


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



