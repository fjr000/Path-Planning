# Path-Planning
基于高程查询的路径规划服务

## 快速开始

1) Docker 启动（推荐）

```
docker\docker_restart.bat
```

启动后访问：

- 前端界面: `http://127.0.0.1:8025/`
- 接口文档: `http://127.0.0.1:8025/docs`

2) 本地启动

```
python -m pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
python -m src.services.http_service
```

3) Docker Compose 启动（可配置覆盖）

```
docker compose up -d --build
```

可在 `docker-compose.yml` 中通过 environment 配置以下变量来覆盖 `config/config.json`：

- `SERVER_HOST`, `SERVER_PORT`
- `QUERY_HOST`, `QUERY_PORT`, `QUERY_REQUEST`
- `TILE_URL`


## 配置

`config/config.json`

```
{
  "query_host": "192.168.3.12",
  "query_port": 5555,
  "query_request": "free/tinder/v3/box2/query",
  "server_host": "0.0.0.0",
  "server_port": 8025,
  "tile_url": "http://192.168.3.100:8035/map/image/tiles/{z}/{x}/{-y}.jpg"
}
```

- `tile_url`: 前端底图瓦片地址
- `query_*`: 外部高程查询服务


## 接口文档

### 1) 路径规划

- 路由: `GET /path-planning`
- 描述: 执行固定“障碍阈值=alt”的分块贪心+A*路径规划（返回 path 为包含原始起点与原始终点的整体轨迹）
- 参数:
  - `lon1`(float): 起点经度
  - `lat1`(float): 起点纬度
  - `lon2`(float): 终点经度
  - `lat2`(float): 终点纬度
  - `alt`(float, 可负): 障碍阈值 thred（网格中 altitude > alt 视为障碍）

成功响应 200：

```
{
  "status": "success",
  "origin": { "lon": 121.52, "lat": 25.29, "alt": 0.0, "query_alt": -8.32 },
  "target": { "lon": 121.53, "lat": 25.30, "alt": 0.0, "query_alt": -9.11 },
  "path": [
    { "lon": 121.52, "lat": 25.29, "alt": 0.0, "query_alt": -8.32 },
    { "lon": 121.5201, "lat": 25.2901, "alt": -9.00 },
    { "lon": 121.5210, "lat": 25.2910, "alt": -9.12 },
    { "lon": 121.53, "lat": 25.30, "alt": 0.0, "query_alt": -9.11 }
  ]
}
```

失败响应（统一 200，`status=failed`，附明确原因）：

- 参数非法

```
{ "status":"failed", "error":"invalid_parameters", "message":"经纬度参数不合法", "invalid":[{"field":"lon1","value":..., "expect":"[-180, 180]"}] }
```

- 距离超限

```
{ "status":"failed", "error":"distance_too_long", "message":"两点规划距离过长", "distance_km": 62.3, "limit_km": 50.0, "origin":{...}, "target":{...} }
```

- 起点附近无数据

```
{ "status":"failed", "error":"no_elevation_data_origin", "message":"起点附近缺少高程/可通行数据", "origin":{...} }
```

- 起点为障碍（包含起点格高程与阈值）

```
{ "status":"failed", "error":"origin_blocked", "message":"起点所在网格为障碍，不可通，起点查询到的代表性高程为：-3.19", "origin":{...}, "origin_cell_alt": -3.19, "thred": 0 }
```

- 终点不可达（含提示）

```
{ "status":"failed", "error":"unreachable", "message":"终点不可达或当前数据条件下无法规划路径，终点查询到的代表性高程为：-7.55", "origin":{...}, "target":{...}, "end_blocked_local": true }
```

- 异常

```
{ "status":"failed", "error":"exception", "message":"错误信息" }
```

示例（curl）：

```
curl "http://127.0.0.1:8025/path-planning?lon1=121.523978&lat1=25.296777&lon2=121.524965&lat2=25.291694&alt=0"
```


### 2) 查询点代表性高程

- 路由: `GET /query-alt`
- 描述: 对指定经纬度执行一次高程数据查询，返回“最近点”的高程
- 参数:
  - `lon`(float): 经度
  - `lat`(float): 纬度

成功响应 200：

```
{ "status":"success", "lon": 121.523978, "lat": 25.296777, "query_alt": -8.12 }
```

失败响应（统一 200）：

```
{ "status":"failed", "message":"该点附近无高程数据", "lon": 121.52, "lat": 25.29 }
```

示例（curl）：

```
curl "http://127.0.0.1:8025/query-alt?lon=121.523978&lat=25.296777"
```
