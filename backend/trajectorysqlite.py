import sqlite3

DB_PATH = "trajectory.db"


def init_db():
    """
    Program 테이블이 없으면 생성한다 (멱등). 모듈 import 시 자동 호출되어
    DB 파일이 없는 새 환경에서도 첫 실행이 깨지지 않도록 보장한다.
    """
    with sqlite3.connect(DB_PATH) as conn:
        conn.execute('''
            CREATE TABLE IF NOT EXISTS Program (
                id         INTEGER PRIMARY KEY AUTOINCREMENT,
                program_id TEXT NOT NULL,
                X  REAL, Y  REAL, Z  REAL,
                qx REAL, qy REAL, qz REAL, qw REAL
            )
        ''')
        conn.commit()


def save_pose_to_db(program_id, pose):
    with sqlite3.connect(DB_PATH) as conn:
        cursor = conn.cursor()
        cursor.execute('''
            INSERT INTO Program (program_id, X, Y, Z, qx, qy, qz, qw)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        ''', (program_id, *pose))
        conn.commit()


def load_poses_from_db(program_id):
    with sqlite3.connect(DB_PATH) as conn:
        cursor = conn.cursor()
        cursor.execute('''
            SELECT X, Y, Z, qx, qy, qz, qw FROM Program
            WHERE program_id = ?
            ORDER BY id
        ''', (program_id,))
        poses = cursor.fetchall()
    return [list(p) for p in poses]


def reset_poses_from_db(program_id):
    with sqlite3.connect(DB_PATH) as conn:
        cursor = conn.cursor()
        cursor.execute('''
            DELETE FROM Program
            WHERE program_id = ?
        ''', (program_id,))
        conn.commit()


# 모듈 import 시 테이블 보장
init_db()
