import sqlite3

def save_pose_to_db(program_id,pose):
    with sqlite3.connect("trajectory.db") as conn:
        cursor = conn.cursor()
        cursor.execute(f'''
            INSERT INTO Program (program_id, X, Y, Z, qx, qy, qz, qw)
            VALUES (? ,?, ?, ?, ?, ?, ?, ?)
        ''', (program_id, *pose))
        conn.commit()

def load_poses_from_db(program_id):
    with sqlite3.connect("trajectory.db") as conn:
        cursor = conn.cursor()
        cursor.execute('''
            SELECT x, y, z, qx, qy, qz, qw FROM Program
            WHERE program_id = ?
            ORDER BY id
        ''', (program_id,))
        poses = cursor.fetchall()    
    return [list(p) for p in poses]

def reset_poses_from_db(program_id):
    with sqlite3.connect("trajectory.db") as conn:
        cursor = conn.cursor()
        cursor.execute('''
            DELETE FROM Program
            WHERE program_id = ?
        ''', (program_id,))
        conn.commit() 
