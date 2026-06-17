"""UR5 Web Simulator 진입점.

실행:  cd backend && uv run main.py
문서:  http://localhost:8500/docs (Swagger)

포트 8500 — 8000 은 synex 개발 서버가 점유하므로 회피.
앱 본체는 app/main.py 의 `app`. (reload 용으로 import 문자열로 넘긴다)
"""
import uvicorn

HOST = "127.0.0.1"
PORT = 8500

if __name__ == "__main__":
    uvicorn.run("app.main:app", host=HOST, port=PORT, reload=True)
