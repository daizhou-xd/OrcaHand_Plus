"""
3D URDF hand viewer using yourdfpy + pyglet 2.x (shader-based OpenGL).

Runs in a background thread alongside the tkinter GUI.
Receives servo position updates and renders the hand in real time.
"""

import os
import re
import sys
import time
import threading
import tempfile
import logging
from ctypes import c_float, c_int, sizeof, cast, pointer, POINTER

import numpy as np

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Path resolution
# ---------------------------------------------------------------------------

_ORCAHAND_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir, os.pardir, os.pardir)
)


def _resolve_urdf_path(right_or_left: str = "right") -> str:
    urdf_rel = f"orcahand_description/v1/models/urdf/orcahand_{right_or_left}.urdf"
    urdf_path = os.path.join(_ORCAHAND_ROOT, urdf_rel)
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    with open(urdf_path, encoding="utf-8") as fh:
        text = fh.read()

    def _replacer(m: re.Match) -> str:
        uri = m.group(0)
        rel = uri.replace("package://orcahand_description/", "")
        return os.path.join(_ORCAHAND_ROOT, "orcahand_description", rel).replace("\\", "/")

    text = re.sub(r"package://orcahand_description/[^\"\s]+", _replacer, text)

    tmp = os.path.join(tempfile.gettempdir(), f"orcahand_{right_or_left}_resolved.urdf")
    with open(tmp, "w", encoding="utf-8") as fh:
        fh.write(text)
    return tmp


# ---------------------------------------------------------------------------
# Servo ID -> actuated-joint cfg index
# ---------------------------------------------------------------------------

SERVO_TO_CFG: dict[int, int] = {
    1: 3, 2: 4, 3: 2, 4: 1, 5: 11, 6: 13, 7: 12,
    8: 9, 9: 10, 10: 15, 11: 16, 12: 14, 13: 8,
    14: 5, 15: 6, 16: 7, 17: 0,
}

SERVO_POS_RANGE = 4095

_joint_limits: dict[int, tuple[float, float]] = {}


def _load_robot(right_or_left: str = "right"):
    from yourdfpy import URDF

    urdf_tmp = _resolve_urdf_path(right_or_left)
    robot = URDF.load(urdf_tmp)

    global _joint_limits
    _joint_limits.clear()
    for i, jname in enumerate(robot.actuated_joint_names):
        j = robot.joint_map.get(jname)
        if j is not None and j.limit is not None:
            _joint_limits[i] = (float(j.limit.lower), float(j.limit.upper))
        else:
            _joint_limits[i] = (-1.57, 1.57)
    return robot


def servo_positions_to_cfg(positions: dict[int, int]) -> np.ndarray:
    cfg = np.zeros(17, dtype=np.float64)
    for sid, pos in positions.items():
        cfg_idx = SERVO_TO_CFG.get(sid)
        if cfg_idx is None:
            continue
        lower, upper = _joint_limits.get(cfg_idx, (-1.57, 1.57))
        t = np.clip(float(pos) / SERVO_POS_RANGE, 0.0, 1.0)
        cfg[cfg_idx] = lower + t * (upper - lower)
    return cfg


# ---------------------------------------------------------------------------
# GLSL shaders for simple diffuse lighting
# ---------------------------------------------------------------------------

_VERTEX_SHADER = """
#version 330 core

uniform mat4 u_viewproj;
uniform mat4 u_model;

in vec3 a_position;
in vec3 a_normal;
in vec4 a_color;

out vec4 v_color;

const vec3 light_dir = normalize(vec3(0.6, 0.8, 1.2));
const float ambient = 0.20;
const float diffuse = 0.80;

void main() {
    vec4 world_pos = u_model * vec4(a_position, 1.0);
    gl_Position = u_viewproj * world_pos;

    mat3 normal_mat = mat3(transpose(inverse(u_model)));
    vec3 n = normalize(normal_mat * a_normal);
    float ndotl = max(dot(n, light_dir), 0.0);
    float brightness = ambient + diffuse * ndotl;
    v_color = vec4(a_color.rgb * brightness, a_color.a);
}
"""

_FRAGMENT_SHADER = """
#version 330 core

in vec4 v_color;
out vec4 frag_color;

void main() {
    frag_color = v_color;
}
"""


# ---------------------------------------------------------------------------
# Helper: build view/projection matrices with numpy
# ---------------------------------------------------------------------------

def _perspective(fov_y_deg: float, aspect: float, near: float, far: float) -> np.ndarray:
    f = 1.0 / np.tan(np.radians(fov_y_deg) / 2.0)
    d = near - far
    m = np.zeros((4, 4), dtype=np.float32)
    m[0, 0] = f / aspect
    m[1, 1] = f
    m[2, 2] = (far + near) / d
    m[2, 3] = (2.0 * far * near) / d
    m[3, 2] = -1.0
    return m


def _lookat(eye, center, up) -> np.ndarray:
    eye = np.array(eye, dtype=np.float32)
    center = np.array(center, dtype=np.float32)
    up = np.array(up, dtype=np.float32)

    f = center - eye
    f /= np.linalg.norm(f)
    s = np.cross(f, up)
    s /= np.linalg.norm(s)
    u = np.cross(s, f)

    m = np.eye(4, dtype=np.float32)
    m[0, 0:3] = s
    m[1, 0:3] = u
    m[2, 0:3] = -f
    m[0, 3] = -np.dot(s, eye)
    m[1, 3] = -np.dot(u, eye)
    m[2, 3] = np.dot(f, eye)
    return m


# ---------------------------------------------------------------------------
# Pyglet ShaderProgram wrapper
# ---------------------------------------------------------------------------

class _LitShader:
    """Compiles and manages the diffuse-lighting shader."""

    def __init__(self):
        from pyglet.graphics.shader import Shader, ShaderProgram

        vs = Shader(_VERTEX_SHADER, "vertex")
        fs = Shader(_FRAGMENT_SHADER, "fragment")
        self._program = ShaderProgram(vs, fs)

    @property
    def program(self):
        return self._program

    def set_viewproj(self, m: np.ndarray):
        self._program["u_viewproj"] = m.astype(np.float32).T.flatten().tolist()

    def set_model(self, m: np.ndarray):
        self._program["u_model"] = m.astype(np.float32).T.flatten().tolist()

    def use(self):
        self._program.use()


# ---------------------------------------------------------------------------
# Mesh data stored per URDF geometry
# ---------------------------------------------------------------------------

class _MeshDrawData:
    """Pre-loaded GPU buffers for one URDF visual geometry."""

    __slots__ = (
        "geom_name", "index_count", "vao", "vbo_pos", "vbo_nrm",
        "vbo_col", "vbo_idx",
    )

    def __init__(self):
        self.geom_name = ""
        self.index_count = 0
        self.vao = 0
        self.vbo_pos = 0
        self.vbo_nrm = 0
        self.vbo_col = 0
        self.vbo_idx = 0


# ---------------------------------------------------------------------------
# The 3D viewer
# ---------------------------------------------------------------------------

class OrcaHandViewer3D:
    """pyglet window rendering the URDF hand model with real-time joint sync.

    Runs its own thread so the tkinter mainloop stays responsive.
    """

    def __init__(
        self,
        urdf_type: str = "right",
        width: int = 520,
        height: int = 460,
        title: str = "OrcaHand 3D",
    ):
        self._urdf_type = urdf_type
        self._width = width
        self._height = height
        self._title = title

        self._lock = threading.Lock()
        self._latest_cfg = np.zeros(17, dtype=np.float64)
        self._cfg_dirty = True
        self._running = False
        self._window = None
        self._thread: threading.Thread | None = None

        self._scene = None
        self._robot = None
        self._shader: _LitShader | None = None
        self._mesh_data: list[_MeshDrawData] = []
        self._viewproj = np.eye(4, dtype=np.float32)

    # ---- public API ---------------------------------------------------------

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True, name="urdf3d")
        self._thread.start()

    def update_servo_positions(self, positions: dict[int, int]):
        with self._lock:
            self._latest_cfg = servo_positions_to_cfg(positions)
            self._cfg_dirty = True

    def close(self):
        self._running = False

    def is_running(self) -> bool:
        return self._running

    # ---- render thread ------------------------------------------------------

    def _run(self):
        import pyglet
        from pyglet import gl

        # Load URDF
        try:
            self._robot = _load_robot(self._urdf_type)
        except Exception as exc:
            logger.error(f"Failed to load URDF: {exc}")
            self._running = False
            return
        self._scene = self._robot.scene

        # Create window with a core profile context
        try:
            gl_config = pyglet.gl.Config(
                double_buffer=True,
                depth_size=24,
                sample_buffers=1,
                samples=4,
                major_version=3,
                minor_version=3,
            )
            self._window = pyglet.window.Window(
                width=self._width,
                height=self._height,
                caption=self._title,
                resizable=True,
                config=gl_config,
            )
        except Exception:
            self._window = pyglet.window.Window(
                width=self._width,
                height=self._height,
                caption=self._title,
                resizable=True,
            )

        self._window.set_minimum_size(240, 200)

        # Compile shader and upload meshes
        self._shader = _LitShader()
        self._upload_meshes()
        self._update_viewproj(self._width, self._height)

        gl.glClearColor(0.18, 0.18, 0.20, 1.0)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDepthFunc(gl.GL_LESS)
        gl.glEnable(gl.GL_MULTISAMPLE)

        @self._window.event
        def on_resize(w, h):
            gl.glViewport(0, 0, w, h)
            self._update_viewproj(w, h)

        last_cfg = np.zeros(17)

        while self._running:
            with self._lock:
                if self._cfg_dirty:
                    cfg = self._latest_cfg.copy()
                    self._cfg_dirty = False
                else:
                    cfg = last_cfg

            if not np.array_equal(cfg, last_cfg):
                try:
                    self._robot.update_cfg(cfg)
                except Exception:
                    pass
                last_cfg = cfg

            self._window.switch_to()
            self._window.dispatch_events()
            self._render()
            self._window.flip()

            pyglet.clock.tick()
            time.sleep(0.001)

        # Cleanup GPU resources
        self._delete_meshes()

    # ---- rendering ----------------------------------------------------------

    def _update_viewproj(self, w: int, h: int):
        aspect = w / max(h, 1)
        proj = _perspective(45.0, aspect, 0.005, 3.0)

        # Camera: look at the hand from front-right-top
        eye = (0.12, -0.18, 0.14)
        center = (0.01, 0.0, 0.04)
        up = (0.0, 0.0, 1.0)
        view = _lookat(eye, center, up)

        # Extra rotation for nicer viewing angle
        rot = np.array([
            [1, 0, 0, 0],
            [0, np.cos(np.radians(-15)), -np.sin(np.radians(-15)), 0],
            [0, np.sin(np.radians(-15)), np.cos(np.radians(-15)), 0],
            [0, 0, 0, 1],
        ], dtype=np.float32)

        self._viewproj = proj @ rot @ view

    def _render(self):
        from pyglet import gl

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        if self._shader is None:
            return
        self._shader.use()
        self._shader.set_viewproj(self._viewproj)

        for md in self._mesh_data:
            # Get world transform from scene graph
            try:
                tf = self._scene.graph.get(md.geom_name)[0]
            except Exception:
                tf = np.eye(4, dtype=np.float32)

            self._shader.set_model(tf)

            # Bind VAO and draw
            gl.glBindVertexArray(md.vao)
            gl.glDrawElements(
                gl.GL_TRIANGLES,
                md.index_count,
                gl.GL_UNSIGNED_INT,
                None,
            )
            gl.glBindVertexArray(0)

    # ---- GPU mesh upload ----------------------------------------------------

    def _upload_meshes(self):
        from pyglet import gl

        geom_names = sorted(self._scene.geometry.keys())
        for gname in geom_names:
            geom = self._scene.geometry[gname]
            if not hasattr(geom, "vertices") or len(geom.vertices) == 0:
                continue
            if not hasattr(geom, "faces") or len(geom.faces) == 0:
                continue

            verts = np.array(geom.vertices, dtype=np.float32)
            faces = np.array(geom.faces, dtype=np.uint32)

            # Per-vertex data (expand faces)
            v_verts = verts[faces.flatten()].astype(np.float32)
            idx_count = len(faces) * 3

            # Face normals
            v0 = verts[faces[:, 0]]
            v1 = verts[faces[:, 1]]
            v2 = verts[faces[:, 2]]
            fn = np.cross(v1 - v0, v2 - v0)
            ln = np.linalg.norm(fn, axis=1, keepdims=True)
            ln[ln < 1e-10] = 1.0
            fn /= ln
            v_nrm = np.repeat(fn, 3, axis=0).astype(np.float32)

            # Color per vertex
            color = _color_for_geometry_name(gname)
            c = np.tile(np.array(color + (1.0,), dtype=np.float32), (idx_count, 1))

            # Generate buffer objects
            vao = gl.GLuint()
            gl.glGenVertexArrays(1, vao)
            gl.glBindVertexArray(vao)

            vbo_pos = gl.GLuint()
            gl.glGenBuffers(1, vbo_pos)
            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, vbo_pos)
            gl.glBufferData(gl.GL_ARRAY_BUFFER, v_verts.nbytes,
                           v_verts.ctypes.data_as(POINTER(gl.GLfloat)),
                           gl.GL_STATIC_DRAW)
            gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, 0, None)
            gl.glEnableVertexAttribArray(0)

            vbo_nrm = gl.GLuint()
            gl.glGenBuffers(1, vbo_nrm)
            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, vbo_nrm)
            gl.glBufferData(gl.GL_ARRAY_BUFFER, v_nrm.nbytes,
                           v_nrm.ctypes.data_as(POINTER(gl.GLfloat)),
                           gl.GL_STATIC_DRAW)
            gl.glVertexAttribPointer(1, 3, gl.GL_FLOAT, gl.GL_FALSE, 0, None)
            gl.glEnableVertexAttribArray(1)

            vbo_col = gl.GLuint()
            gl.glGenBuffers(1, vbo_col)
            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, vbo_col)
            gl.glBufferData(gl.GL_ARRAY_BUFFER, c.nbytes,
                           c.ctypes.data_as(POINTER(gl.GLfloat)),
                           gl.GL_STATIC_DRAW)
            gl.glVertexAttribPointer(2, 4, gl.GL_FLOAT, gl.GL_FALSE, 0, None)
            gl.glEnableVertexAttribArray(2)

            vbo_idx = gl.GLuint()
            gl.glGenBuffers(1, vbo_idx)
            gl.glBindBuffer(gl.GL_ELEMENT_ARRAY_BUFFER, vbo_idx)
            indices = np.arange(idx_count, dtype=np.uint32)
            gl.glBufferData(gl.GL_ELEMENT_ARRAY_BUFFER, indices.nbytes,
                           indices.ctypes.data_as(POINTER(gl.GLuint)),
                           gl.GL_STATIC_DRAW)

            gl.glBindVertexArray(0)

            md = _MeshDrawData()
            md.geom_name = gname
            md.index_count = idx_count
            md.vao = vao
            md.vbo_pos = vbo_pos
            md.vbo_nrm = vbo_nrm
            md.vbo_col = vbo_col
            md.vbo_idx = vbo_idx
            self._mesh_data.append(md)

    def _delete_meshes(self):
        from pyglet import gl

        for md in self._mesh_data:
            try:
                gl.glDeleteVertexArrays(1, gl.GLuint(md.vao))
                gl.glDeleteBuffers(1, gl.GLuint(md.vbo_pos))
                gl.glDeleteBuffers(1, gl.GLuint(md.vbo_nrm))
                gl.glDeleteBuffers(1, gl.GLuint(md.vbo_col))
                gl.glDeleteBuffers(1, gl.GLuint(md.vbo_idx))
            except Exception:
                pass
        self._mesh_data.clear()


# ---------------------------------------------------------------------------
# Per-geometry colour
# ---------------------------------------------------------------------------

def _color_for_geometry_name(name: str) -> tuple[float, float, float]:
    n = name.lower()
    if "tower" in n:
        return (0.35, 0.35, 0.38)
    if "palm" in n:
        return (0.55, 0.50, 0.45) if "skin" not in n else (0.82, 0.71, 0.62)
    if "thumb" in n:
        return (0.65, 0.60, 0.55) if "skin" not in n else (0.85, 0.72, 0.60)
    if "index" in n:
        return (0.60, 0.55, 0.50) if "skin" not in n else (0.82, 0.70, 0.58)
    if "middle" in n:
        return (0.62, 0.56, 0.51) if "skin" not in n else (0.83, 0.71, 0.59)
    if "ring" in n:
        return (0.58, 0.53, 0.48) if "skin" not in n else (0.81, 0.70, 0.58)
    if "pinky" in n:
        return (0.56, 0.51, 0.46) if "skin" not in n else (0.80, 0.69, 0.57)
    return (0.50, 0.48, 0.45)
