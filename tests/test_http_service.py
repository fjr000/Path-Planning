import asyncio
import httpx


async def request_path(i):
    async with httpx.AsyncClient() as client:
        resp = await client.get("http://127.0.0.1:8025/path-planning?lon1=121.32&lat1=25.17&lon2=121.75&lat2=25.26&alt=-10")
        print(f"Request {i}: {resp.status_code}, path length={len(resp.json().get('path', []))}")


async def main():
    await asyncio.gather(*(request_path(i) for i in range(5)))


if __name__ == "__main__":
    asyncio.run(main())



