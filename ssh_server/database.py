import os
import sqlite3
from typing import List, Tuple

DB_PATH = os.path.join(os.path.dirname(__file__), "users.db")


class Database:
    """Simple SQLite-based user and container registry."""

    def __init__(self, path: str = DB_PATH):
        self.conn = sqlite3.connect(path)
        self._create()

    def _create(self) -> None:
        cur = self.conn.cursor()
        cur.execute(
            """CREATE TABLE IF NOT EXISTS users (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT,
            email TEXT UNIQUE,
            password TEXT
        )"""
        )
        cur.execute(
            """CREATE TABLE IF NOT EXISTS containers (
            id TEXT PRIMARY KEY,
            user_id INTEGER,
            docker_id TEXT,
            port INTEGER,
            password TEXT,
            image_tag TEXT DEFAULT 'linuxserver/openssh-server',
            ssh_ready BOOLEAN DEFAULT 1,
            funnel_port INTEGER,
            FOREIGN KEY(user_id) REFERENCES users(id)
        )"""
        )
        # Silent ALTERs for backwards compatibility
        for col, ddl in [
            ("image_tag", "TEXT DEFAULT 'linuxserver/openssh-server'"),
            ("ssh_ready", "BOOLEAN DEFAULT 1"),
            ("funnel_port", "INTEGER")
        ]:
            try:
                cur.execute(f"ALTER TABLE containers ADD COLUMN {col} {ddl}")
            except sqlite3.OperationalError:
                pass
        self.conn.commit()

    # -------------------- user helpers ------------------------------------ #
    def add_user(self, name: str, email: str, password: str) -> bool:
        try:
            self.conn.execute(
                "INSERT INTO users (name, email, password) VALUES (?, ?, ?)",
                (name, email, password),
            )
            self.conn.commit()
            return True
        except sqlite3.IntegrityError:
            return False

    def verify_user(self, email: str, password: str) -> int | None:
        cur = self.conn.cursor()
        cur.execute("SELECT id FROM users WHERE email=? AND password=?",
                    (email, password))
        row = cur.fetchone()
        return row[0] if row else None

    # -------------------- container helpers ------------------------------- #
    def add_container(
        self, cid: str, user_id: int, docker_id: str, port: int, password: str,
        image_tag: str = "linuxserver/openssh-server", ssh_ready: bool = True,
        funnel_port: int | None = None
    ) -> None:
        self.conn.execute(
            "INSERT INTO containers (id, user_id, docker_id, port, password, "
            "image_tag, ssh_ready, funnel_port) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
            (cid, user_id, docker_id, port, password,
             image_tag, ssh_ready, funnel_port),
        )
        self.conn.commit()

    def list_containers(self, user_id: int) -> List[
        Tuple[str, str, int, str, str, bool, int | None]]:
        cur = self.conn.cursor()
        cur.execute(
            "SELECT id, docker_id, port, password, COALESCE(image_tag, "
            "'linuxserver/openssh-server'), COALESCE(ssh_ready, 1), "
            "funnel_port FROM containers WHERE user_id=?",
            (user_id,),
        )
        return cur.fetchall()

    def delete_container(self, user_id: int, cid: str) -> None:
        self.conn.execute(
            "DELETE FROM containers WHERE user_id=? AND id=?",
            (user_id, cid),
        )
        self.conn.commit()
