"""
산출물 경로 헬퍼 — 데모가 만드는 gif/png/npz 를 outputs/ 한곳에 모은다.

  from paths import out
  anim.save(out("demo.gif"))     # → pythonscript/outputs/demo.gif

outputs/ 는 .gitignore 처리되며, import 시 자동 생성된다(없으면).
CWD 와 무관하게 항상 이 파일 옆 outputs/ 를 가리킨다.
"""

import os

OUTDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "outputs")
os.makedirs(OUTDIR, exist_ok=True)


def out(name):
    """산출물 파일의 outputs/ 내 전체 경로."""
    return os.path.join(OUTDIR, name)
