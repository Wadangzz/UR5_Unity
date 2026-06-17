# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

UR5 robot **web simulator**: a FastAPI backend (Python, `uv`) exposing the in-house
`robot_math` screw-theory kinematics/dynamics library, plus a React + react-three-fiber
frontend that renders the real UR5 URDF meshes and replays control simulations.

> The repo name and root `README.md` describe the **original Unity project** (the `main`
> branch still holds it). Active work is on the **`develop`** branch, which is the web app —
> Unity and the Qt GUI were removed. Trust `DEVLOG.md` over `README.md` for current state.

## Commands

Backend (`cd backend`, requires Python 3.12+ and [`uv`](https://docs.astral.sh/uv/)):
```bash
uv sync                         # install deps (first time)
uv run main.py                  # serve on http://localhost:8500/docs  (Swagger)
uv run python demos/<name>.py   # run a standalone control/viz demo (matplotlib, no browser)
uv run ruff check .             # lint   (ruff config in pyproject.toml: 80 cols, single quotes)
uv run ruff format .            # format
```
Port is **8500** (8000 is taken by another local dev server). There is no pytest suite —
validation has been done ad-hoc via FastAPI `TestClient` in the REPL.

Frontend (`cd frontend`, pnpm):
```bash
pnpm install
pnpm dev        # Vite dev server; proxies /api and /meshes → :8500 (run backend too)
pnpm build      # tsc -b && vite build
pnpm lint       # eslint
pnpm format     # prettier (with tailwind class sorting)
```
Both servers must run together: the frontend has no data of its own; Vite proxies REST
(`/api`) and static URDF/mesh files (`/meshes`) to the backend.

## Architecture

**Core principle: heavy compute (simulation) runs once on the backend; the browser only
replays the resulting θ(t).** Never call REST per animation frame — joint-slider jogging
does forward kinematics in the browser (urdf-loader). See `backend/WEB_DESIGN.md`.

### Backend layering (`backend/app/`)
Request flow is **routers → controllers → core/sim**:
- `core/` — the reusable physics library, framework-agnostic:
  - `robot_math.py` — `SO3` / `SE3` / `Kinematics` / `Dynamics` split along Lie group/algebra
    structure. `Dynamics` is RNE-based (`inverse_dynamics`, `mass_matrix`, `gravity_forces`,
    `coriolis_forces`, `forward_dynamics`) and takes `(Mlist, Glist, Slist)` as arguments, so
    it is not UR5-specific.
  - `ur5_model.py` — UR5 Modern-Robotics parameters (MLIST/GLIST/SLIST, SI units) + FK/IK/jacobians.
  - `robots.py`, `paths.py` — kinematic model helpers / trajectory generation (quintic).
- `sim.py` — the simulation **engine**: closes the control loop with `scipy.solve_ivp`
  (`forward_dynamics` as the plant). Implements all four controllers in one `torque()
  closure: **PD+gravity-comp, PID, Computed Torque, Impedance** (Cartesian: `τ = Jbᵀ(K·V − D·Jb·θ̇) + g`,
  body twist `V = log(T⁻¹T_d)`). Uses the implicit **Radau** solver (the closed loop is stiff;
  explicit RK45 takes tens of seconds). Returns `settle_time` / `steady_state_error` / `diverged`
  by detecting steady-state (velocity below tolerance) with a divergence safety net.
- `realism.py` — builds separate **plant vs. controller** models (payload, mass scaling) t`o
  simulate model uncertainty; disturbance is an end-effector force applied after motion ends.
- `routers/` (`robot`, `kinematics`, `programs`, `simulate`) — FastAPI endpoints under `/api`.
- `controllers/` — business logic between routers and core (e.g. resolve a saved program's
  poses → IK → joint waypoints, then call `sim.run_simulation`).
- `models.py` (SQLModel `Pose`), `schemas.py` (pydantic request/response), `db.py` (SQLite),
  `meshes_setup.py` (copies UR5 URDF+meshes from `robot_descriptions` into `app/meshes/` and
  mounts them as static; idempotent, gitignored).

Key endpoints: `GET /api/robot`, `POST /api/ik`, `POST /api/fk` (returns manipulability `w`
and `sigma_min`), `GET/POST/DELETE /api/programs/...`, `POST /api/run` (the simulation).

### Frontend (`frontend/app/`)
Vite + React 19 + react-three-fiber v9 + drei v10 + urdf-loader. Source lives in `app/`
(not `src/`), entry `main.web.tsx` / `root.tsx`, path alias `@` → `app/`, shadcn-style UI in
`components/ui/`. Main components: `RobotView` (urdf-loader mesh render in a **Z-up scene** so
world axes match robot axes; replays θ(t) via `requestAnimationFrame`), `ControlPanel`
(controller + gain sliders, ζ/ωn badges), `PoseEditor` (live Cartesian↔joint IK/FK), `ProgramList`
(teach/run/delete poses), `Plots` (recharts error/torque). `app/api.ts` is the axios client and
also holds the **controller spec as a frontend const** (`CONTROLLERS`) — the gain keys
(`kp`/`ki`/`kd`) are the contract with the backend `torque()` function; there is no `/controllers` API.

## Gotchas (from DEVLOG)

- Work branch is **`develop`**; it is not pushed.
- **DB schema changes**: SQLModel `create_all` does not ALTER. After changing `Pose` or any model,
  delete `backend/robot.db` and restart.
- The UR5 **zero/home pose is a singularity**, so a `sigma_min = 0` warning at startup is expected.
- Singularity handling uses **DLS IK** (damps when `sigma_min < 0.04`); manipulability
  `w = √det(JJᵀ)` and `sigma_min` are shown live.
- Gitignored runtime artifacts: `backend/robot.db`, `outputs/`, `*.npz`, `backend/app/meshes/`,
  `node_modules`, `dist`.
