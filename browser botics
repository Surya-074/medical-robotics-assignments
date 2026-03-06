import browserbotics as bb
import time
import math
import random

# ═══════════════════════════════════════════════════════════════
#   BROWSERBOTICS — NURSE ROBOT v7
#   - Open room (no ceiling, open front side)
#   - Robot walks in ALL directions (X, Y, diagonal)
#   - Robot FIRST scans/analyzes patient, then acts
#   - Smart navigation with waypoints
#   - Curtain open/close based on patient state
# ═══════════════════════════════════════════════════════════════

TICK = 0.022

# ── Room: open on FRONT side (+Y), no ceiling ────────────────
ROOM_W =  7.0   # X
ROOM_D =  8.0   # Y  (open at +Y end)
ROOM_H =  3.0   # Z  (walls go up, no ceiling)
hw = ROOM_W / 2   # 3.5
hd = ROOM_D / 2   # 4.0

# ── Key world positions ───────────────────────────────────────
NURSE_STATION_X = -3.0
NURSE_STATION_Y = -3.5   # near back-left corner (nurse start)

BED_CX = 1.6
BED_CY = 0.4

# Patient head resting on pillow
P_HEAD_X = BED_CX
P_HEAD_Y = BED_CY + 1.05
P_HEAD_Z = 0.72

# Patient wrist (SpO2 + glucose check)
P_WRIST_X = BED_CX - 0.30
P_WRIST_Y = BED_CY + 0.10
P_WRIST_Z = 0.57

# Window on RIGHT wall (+X wall)
WIN_X   =  hw - 0.05
WIN_CY  =  0.50
WIN_Z   =  1.60

# TV on BACK wall (-Y wall)
TV_X =  0.0
TV_Y = -hd + 0.10
TV_Z =  1.55

# Equipment cart
CART_X = -0.20
CART_Y =  0.90

# Nurse waypoints (positions visited during routine)
WP_STATION  = (NURSE_STATION_X,  NURSE_STATION_Y)
WP_SCAN     = (BED_CX - 1.10,   BED_CY - 0.30)   # scan position
WP_BEDSIDE  = (BED_CX - 0.75,   BED_CY + 0.70)   # beside patient
WP_FOOT     = (BED_CX,           BED_CY - 1.40)   # foot of bed
WP_WINDOW   = (WIN_X  - 0.60,   WIN_CY)           # curtain position
WP_MONITOR  = (-2.80,            BED_CY + 0.40)   # vitals monitor
WP_CART     = (CART_X - 0.30,   CART_Y)           # equipment cart

# ── Math helpers ──────────────────────────────────────────────
def ease(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)

def lerp(a, b, t):
    return a + (b - a) * t

def norm_angle(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def face_yaw(fx, fy, tx, ty):
    return math.atan2(ty - fy, tx - fx)

def dist2(ax, ay, bx, by):
    return math.sqrt((ax-bx)**2 + (ay-by)**2)

def place(bid, x, y, z, rx=0, ry=0, rz=0):
    bb.resetBasePose(bid, [x, y, z],
                     bb.getQuaternionFromEuler([rx, ry, rz]))

# ═══════════════════════════════════════════════════════════════
#   OPEN CLINIC ROOM  (no ceiling, open front wall +Y)
# ═══════════════════════════════════════════════════════════════
def build_room():
    bb.addGroundPlane()

    # ── Floor tiles ───────────────────────────────────────────
    for ix in range(-4, 4):
        for iy in range(-4, 5):
            c = '#e8e4dc' if (ix+iy) % 2 == 0 else '#f2ede5'
            bb.createBody('box', halfExtent=[0.494, 0.494, 0.014],
                          position=[ix+0.5, iy+0.5, 0.014],
                          color=c, mass=0)

    # ── LEFT WALL  (-X) ───────────────────────────────────────
    bb.createBody('box', halfExtent=[0.06, hd, ROOM_H/2],
                  position=[-hw, 0, ROOM_H/2], color='#f0ebe3', mass=0)

    # ── BACK WALL  (-Y) ───────────────────────────────────────
    bb.createBody('box', halfExtent=[hw, 0.06, ROOM_H/2],
                  position=[0, -hd, ROOM_H/2], color='#ede8e0', mass=0)

    # ── RIGHT WALL (+X) — with window gap ────────────────────
    # Below window
    bb.createBody('box', halfExtent=[0.06, hd, (WIN_Z - 0.68)/2],
                  position=[hw, 0, (WIN_Z - 0.68)/2],
                  color='#f0ebe3', mass=0)
    # Above window
    top_z_start = WIN_Z + 0.68
    top_z_half  = (ROOM_H - top_z_start) / 2
    bb.createBody('box', halfExtent=[0.06, hd, top_z_half],
                  position=[hw, 0, top_z_start + top_z_half],
                  color='#f0ebe3', mass=0)
    # Left of window (toward back)
    bb.createBody('box', halfExtent=[0.06, hd/2 - 1.05, ROOM_H/2],
                  position=[hw, -hd/2 - 0.55, ROOM_H/2],
                  color='#f0ebe3', mass=0)
    # Right of window (toward front)
    bb.createBody('box', halfExtent=[0.06, hd/2 - 1.05, ROOM_H/2],
                  position=[hw, hd/2 + 0.55, ROOM_H/2],
                  color='#f0ebe3', mass=0)

    # ── NO FRONT WALL — room is OPEN at +Y ───────────────────
    # Just a floor border strip to mark the opening
    bb.createBody('box', halfExtent=[hw, 0.04, 0.06],
                  position=[0, hd, 0.06], color='#c8bfb0', mass=0)

    # ── Skirting boards (3 walls only) ───────────────────────
    for pos, ext in [
        ([-hw, 0,   0.07], [0.02, hd,  0.07]),
        ([0,  -hd,  0.07], [hw,  0.02, 0.07]),
        ([hw,  0,   0.07], [0.02, hd,  0.07]),
    ]:
        bb.createBody('box', halfExtent=ext, position=pos,
                      color='#c0b8ac', mass=0)

    # ── Ceiling light rail (no ceiling, just a light bar) ────
    bb.createBody('box', halfExtent=[0.55, 0.16, 0.04],
                  position=[0, 0, ROOM_H - 0.04], color='#fffde7', mass=0)
    bb.createBody('box', halfExtent=[0.60, 0.20, 0.03],
                  position=[0, 0, ROOM_H], color='#e0ddd8', mass=0)
    # Suspended light cable
    bb.createBody('box', halfExtent=[0.008, 0.008, 0.35],
                  position=[0, 0, ROOM_H + 0.35], color='#9e9e9e', mass=0)

    # ── Door on LEFT wall ─────────────────────────────────────
    bb.createBody('box', halfExtent=[0.05, 0.46, 1.10],
                  position=[-hw+0.03, -2.20, 1.10], color='#c8a87a', mass=0)
    for pos, ext in [
        ([-hw, -2.20, 2.24], [0.08, 0.50, 0.055]),
        ([-hw, -2.68, 1.10], [0.08, 0.04, 1.12]),
        ([-hw, -1.72, 1.10], [0.08, 0.04, 1.12]),
    ]:
        bb.createBody('box', halfExtent=ext, position=pos,
                      color='#a07850', mass=0)
    bb.createBody('sphere', radius=0.038,
                  position=[-hw+0.09, -1.78, 1.05], color='#d4af37', mass=0)

    print("[ROOM] Open clinic room built ✔")


# ═══════════════════════════════════════════════════════════════
#   WINDOW + ANIMATED CURTAINS
# ═══════════════════════════════════════════════════════════════
class Window:
    def __init__(self):
        self.cl = None   # left curtain panel
        self.cr = None   # right curtain panel
        self.is_open = False
        self._build()

    def _build(self):
        # Glass pane
        bb.createBody('box', halfExtent=[0.04, 0.92, 0.66],
                      position=[WIN_X, WIN_CY, WIN_Z],
                      color='#aed6f1', mass=0)
        # Frame
        for pos, ext in [
            ([WIN_X, WIN_CY,        WIN_Z + 0.68], [0.055, 0.95, 0.038]),
            ([WIN_X, WIN_CY,        WIN_Z - 0.68], [0.055, 0.95, 0.038]),
            ([WIN_X, WIN_CY - 0.93, WIN_Z],        [0.055, 0.038, 0.70]),
            ([WIN_X, WIN_CY + 0.93, WIN_Z],        [0.055, 0.038, 0.70]),
            ([WIN_X, WIN_CY,        WIN_Z],        [0.038, 0.020, 0.70]),
            ([WIN_X, WIN_CY,        WIN_Z],        [0.038, 0.92,  0.020]),
        ]:
            bb.createBody('box', halfExtent=ext, position=pos,
                          color='white', mass=0)

        # Curtain rod
        bb.createBody('box', halfExtent=[0.014, 1.08, 0.014],
                      position=[WIN_X - 0.10, WIN_CY, WIN_Z + 0.84],
                      color='#8b7355', mass=0)
        for dy in [-1.10, 1.10]:
            bb.createBody('sphere', radius=0.028,
                          position=[WIN_X - 0.10, WIN_CY + dy, WIN_Z + 0.84],
                          color='#d4af37', mass=0)

        # Curtain panels — start CLOSED
        self.cl = bb.createBody('box',
            halfExtent=[0.042, 0.46, 0.66],
            position=[WIN_X - 0.07, WIN_CY - 0.24, WIN_Z],
            color='#e8c49a', mass=0)
        self.cr = bb.createBody('box',
            halfExtent=[0.042, 0.46, 0.66],
            position=[WIN_X - 0.07, WIN_CY + 0.24, WIN_Z],
            color='#e8c49a', mass=0)

        # Curtain texture stripes
        for dz in [-0.28, 0.0, 0.28]:
            for side, dy in [(-0.24, -1), (0.24, +1)]:
                bb.createBody('box', halfExtent=[0.005, 0.44, 0.012],
                              position=[WIN_X - 0.025, WIN_CY + side, WIN_Z + dz],
                              color='#c8a06a', mass=0)

        print("[WINDOW] Built ✔")

    def animate_open(self, steps=50):
        if self.is_open:
            return
        ly_s = WIN_CY - 0.24;  ly_e = WIN_CY - 0.84
        ry_s = WIN_CY + 0.24;  ry_e = WIN_CY + 0.84
        for i in range(steps + 1):
            t = ease(i / steps)
            place(self.cl, WIN_X - 0.07, lerp(ly_s, ly_e, t), WIN_Z)
            place(self.cr, WIN_X - 0.07, lerp(ry_s, ry_e, t), WIN_Z)
            time.sleep(TICK * 1.6)
        self.is_open = True

    def animate_close(self, steps=50):
        if not self.is_open:
            return
        ly_s = WIN_CY - 0.84;  ly_e = WIN_CY - 0.24
        ry_s = WIN_CY + 0.84;  ry_e = WIN_CY + 0.24
        for i in range(steps + 1):
            t = ease(i / steps)
            place(self.cl, WIN_X - 0.07, lerp(ly_s, ly_e, t), WIN_Z)
            place(self.cr, WIN_X - 0.07, lerp(ry_s, ry_e, t), WIN_Z)
            time.sleep(TICK * 1.6)
        self.is_open = False


# ═══════════════════════════════════════════════════════════════
#   TV
# ═══════════════════════════════════════════════════════════════
def build_tv():
    bb.createBody('box', halfExtent=[0.60, 0.06, 0.36],
                  position=[TV_X, TV_Y + 0.04, TV_Z], color='#212121', mass=0)
    bb.createBody('box', halfExtent=[0.55, 0.04, 0.32],
                  position=[TV_X, TV_Y + 0.06, TV_Z], color='#1a237e', mass=0)
    bb.createBody('box', halfExtent=[0.50, 0.03, 0.28],
                  position=[TV_X, TV_Y + 0.08, TV_Z], color='#00bcd4', mass=0)
    bb.createBody('box', halfExtent=[0.04, 0.06, 0.34],
                  position=[TV_X, TV_Y + 0.05, TV_Z - 0.70], color='#424242', mass=0)
    bb.createBody('box', halfExtent=[0.18, 0.10, 0.024],
                  position=[TV_X, TV_Y + 0.05, TV_Z - 1.04], color='#424242', mass=0)
    print("[TV] Built ✔")


# ═══════════════════════════════════════════════════════════════
#   BED + EQUIPMENT
# ═══════════════════════════════════════════════════════════════
def build_bed():
    bx, by = BED_CX, BED_CY

    # Frame
    bb.createBody('box', halfExtent=[0.56, 1.14, 0.22],
                  position=[bx, by, 0.22], color='#e0e0e0', mass=0)
    bb.createBody('box', halfExtent=[0.53, 1.08, 0.10],
                  position=[bx, by, 0.44], color='#fafafa', mass=0)
    # Pillow
    bb.createBody('box', halfExtent=[0.22, 0.24, 0.075],
                  position=[bx, by + 1.02, 0.575], color='white', mass=0)
    # Blanket
    bb.createBody('box', halfExtent=[0.50, 0.68, 0.055],
                  position=[bx, by - 0.18, 0.505], color='#b3d9f7', mass=0)
    bb.createBody('box', halfExtent=[0.50, 0.055, 0.06],
                  position=[bx, by + 0.44, 0.502], color='#82b9e8', mass=0)
    # Headboard
    bb.createBody('box', halfExtent=[0.57, 0.065, 0.44],
                  position=[bx, by + 1.56, 0.66], color='#bdbdbd', mass=0)
    # Footboard
    bb.createBody('box', halfExtent=[0.57, 0.065, 0.28],
                  position=[bx, by - 1.18, 0.50], color='#bdbdbd', mass=0)
    # Legs
    for lx, ly in [(bx-0.50, by-1.12), (bx+0.50, by-1.12),
                   (bx-0.50, by+1.50), (bx+0.50, by+1.50)]:
        bb.createBody('box', halfExtent=[0.04, 0.04, 0.22],
                      position=[lx, ly, 0.11], color='#9e9e9e', mass=0)

    # ── Equipment cart ────────────────────────────────────────
    bb.createBody('box', halfExtent=[0.22, 0.32, 0.40],
                  position=[CART_X, CART_Y, 0.40], color='#eeeeee', mass=0)
    bb.createBody('box', halfExtent=[0.24, 0.34, 0.025],
                  position=[CART_X, CART_Y, 0.83], color='#f5f5f5', mass=0)
    bb.createBody('box', halfExtent=[0.20, 0.28, 0.015],
                  position=[CART_X, CART_Y, 0.86], color='#c8d8e8', mass=0)
    # SpO2 meter (green screen)
    bb.createBody('box', halfExtent=[0.065, 0.080, 0.055],
                  position=[CART_X - 0.06, CART_Y - 0.08, 0.93],
                  color='#37474f', mass=0)
    bb.createBody('box', halfExtent=[0.055, 0.068, 0.045],
                  position=[CART_X - 0.06, CART_Y - 0.08, 0.934],
                  color='#4caf50', mass=0)
    bb.createBody('sphere', radius=0.024,
                  position=[CART_X - 0.06, CART_Y - 0.17, 0.916],
                  color='#00e676', mass=0)
    # Glucose meter (yellow screen)
    bb.createBody('box', halfExtent=[0.055, 0.065, 0.048],
                  position=[CART_X + 0.08, CART_Y - 0.08, 0.926],
                  color='#1a237e', mass=0)
    bb.createBody('box', halfExtent=[0.045, 0.055, 0.038],
                  position=[CART_X + 0.08, CART_Y - 0.08, 0.930],
                  color='#ffeb3b', mass=0)
    bb.createBody('sphere', radius=0.024,
                  position=[CART_X + 0.08, CART_Y - 0.17, 0.914],
                  color='#ffc107', mass=0)
    # Pills tray
    for dx, col in [(-0.06, '#f06292'), (0.02, '#81d4fa'), (0.10, '#aed581')]:
        bb.createBody('sphere', radius=0.016,
                      position=[CART_X + dx, CART_Y + 0.12, 0.888],
                      color=col, mass=0)

    # ── IV Stand ──────────────────────────────────────────────
    bb.createBody('box', halfExtent=[0.024, 0.024, 0.96],
                  position=[bx + 0.84, by + 1.12, 0.96], color='#bdbdbd', mass=0)
    bb.createBody('box', halfExtent=[0.22, 0.024, 0.012],
                  position=[bx + 0.84, by + 1.12, 1.92], color='#bdbdbd', mass=0)
    bb.createBody('box', halfExtent=[0.09, 0.04, 0.15],
                  position=[bx + 0.84, by + 1.12, 1.75], color='#b3e5fc', mass=0)
    bb.createBody('box', halfExtent=[0.005, 0.005, 0.48],
                  position=[bx + 0.72, by + 1.02, 1.16], color='#e0e0e0', mass=0)
    bb.createBody('box', halfExtent=[0.24, 0.06, 0.025],
                  position=[bx + 0.84, by + 1.12, 0.025], color='#9e9e9e', mass=0)

    # ── Wall-mounted vitals monitor ───────────────────────────
    bb.createBody('box', halfExtent=[0.28, 0.08, 0.20],
                  position=[-hw + 0.30, by + 0.50, 1.60], color='#263238', mass=0)
    bb.createBody('box', halfExtent=[0.24, 0.05, 0.17],
                  position=[-hw + 0.30, by + 0.55, 1.60], color='#1b5e20', mass=0)
    bb.createBody('box', halfExtent=[0.22, 0.03, 0.005],
                  position=[-hw + 0.30, by + 0.57, 1.62], color='#00e676', mass=0)
    bb.createBody('box', halfExtent=[0.03, 0.14, 0.03],
                  position=[-hw + 0.30, by + 0.37, 1.60], color='#546e7a', mass=0)

    # ── Scan marker on floor (analysis zone) ──────────────────
    bb.createBody('box', halfExtent=[0.90, 1.40, 0.004],
                  position=[BED_CX, BED_CY, 0.004], color='#e3f2fd', mass=0)
    # Corner markers
    for sx, sy in [(-0.85, -1.35), (0.85, -1.35),
                   (-0.85,  1.35), (0.85,  1.35)]:
        bb.createBody('box', halfExtent=[0.06, 0.06, 0.012],
                      position=[BED_CX + sx, BED_CY + sy, 0.012],
                      color='#1565c0', mass=0)

    print("[BED] Built ✔")


# ═══════════════════════════════════════════════════════════════
#   PATIENT
# ═══════════════════════════════════════════════════════════════
def build_patient():
    sk = '#f5cba7'
    bx, by = BED_CX, BED_CY
    # Torso
    bb.createBody('box', halfExtent=[0.21, 0.42, 0.12],
                  position=[bx, by - 0.18, 0.59], color='#e3f2fd', mass=0)
    bb.createBody('box', halfExtent=[0.27, 0.11, 0.09],
                  position=[bx, by + 0.30, 0.58], color='#e3f2fd', mass=0)
    # Head
    bb.createBody('sphere', radius=0.178,
                  position=[P_HEAD_X, P_HEAD_Y, P_HEAD_Z], color=sk, mass=0)
    bb.createBody('sphere', radius=0.180,
                  position=[P_HEAD_X, P_HEAD_Y + 0.02, P_HEAD_Z + 0.08],
                  color='#5d4037', mass=0)
    # Eyes closed
    for ex in [bx - 0.08, bx + 0.08]:
        bb.createBody('box', halfExtent=[0.036, 0.008, 0.009],
                      position=[ex, P_HEAD_Y - 0.12, P_HEAD_Z + 0.02],
                      color='#4a3728', mass=0)
    bb.createBody('sphere', radius=0.022,
                  position=[bx, P_HEAD_Y - 0.14, P_HEAD_Z - 0.02],
                  color='#f0b27a', mass=0)
    bb.createBody('box', halfExtent=[0.042, 0.008, 0.009],
                  position=[bx, P_HEAD_Y - 0.15, P_HEAD_Z - 0.05],
                  color='#e59866', mass=0)
    for ex in [bx - 0.19, bx + 0.19]:
        bb.createBody('sphere', radius=0.042,
                      position=[ex, P_HEAD_Y - 0.02, P_HEAD_Z],
                      color=sk, mass=0)
    # Arm with SpO2 sensor
    bb.createBody('box', halfExtent=[0.062, 0.28, 0.052],
                  position=[bx - 0.30, by + 0.42, 0.577], color=sk, mass=0)
    bb.createBody('sphere', radius=0.067,
                  position=[P_WRIST_X, P_WRIST_Y, P_WRIST_Z], color=sk, mass=0)
    bb.createBody('box', halfExtent=[0.040, 0.055, 0.025],
                  position=[P_WRIST_X, P_WRIST_Y, P_WRIST_Z],
                  color='#e53935', mass=0)
    bb.createBody('box', halfExtent=[0.065, 0.024, 0.055],
                  position=[P_WRIST_X, P_WRIST_Y + 0.12, P_WRIST_Z],
                  color='#fff9c4', mass=0)

    print("[PATIENT] Built ✔")


# ═══════════════════════════════════════════════════════════════
#   SCAN BEAM (visual analysis effect)
# ═══════════════════════════════════════════════════════════════
class ScanBeam:
    def __init__(self):
        self.beam = bb.createBody('box',
            halfExtent=[0.88, 0.008, 0.004],
            position=[BED_CX, BED_CY - 1.30, 0.006],
            color='#00e5ff', mass=0)
        self.hide()

    def scan_animate(self, steps=60):
        """Sweep scan beam from foot to head of bed."""
        y_start = BED_CY - 1.30
        y_end   = BED_CY + 1.30
        for i in range(steps + 1):
            t = i / steps
            y = lerp(y_start, y_end, t)
            # Pulse opacity via color alternation
            col = '#00e5ff' if int(t * 20) % 2 == 0 else '#80f0ff'
            bb.resetBasePose(self.beam, [BED_CX, y, 0.006],
                             bb.getQuaternionFromEuler([0, 0, 0]))
            time.sleep(TICK * 1.4)
        self.hide()

    def hide(self):
        bb.resetBasePose(self.beam, [BED_CX, BED_CY, -2],
                         bb.getQuaternionFromEuler([0, 0, 0]))


# ═══════════════════════════════════════════════════════════════
#   NURSE ROBOT CLASS
# ═══════════════════════════════════════════════════════════════
class NurseRobot:
    ARM0_LX =  0.26
    ARM0_LY =  0.00
    ARM0_LZ =  0.30

    def __init__(self):
        self.x   = NURSE_STATION_X
        self.y   = NURSE_STATION_Y
        self.z   = 0.0
        self.yaw = 0.0
        self.arm       = 0.0
        self.larm      = 0.0
        self.nod       = 0.0
        self.wt        = 0.0
        self.arm_tgt   = (self.x + 1, self.y, 0.70)
        self.ids = {}
        self.hand_wx = 0.0
        self.hand_wy = 0.0
        self.hand_wz = 0.0

    def _mk(self, k, shape, **kw):
        kw['mass'] = 0
        self.ids[k] = bb.createBody(shape, **kw)

    def spawn(self):
        d = [0, 0, -9]
        # Uniform
        self._mk('torso', 'box',    halfExtent=[0.155,0.105,0.260], position=d, color='#ffffff')
        self._mk('pants', 'box',    halfExtent=[0.145,0.095,0.155], position=d, color='#bbdefb')
        self._mk('apron', 'box',    halfExtent=[0.100,0.020,0.220], position=d, color='#e3f2fd')
        self._mk('crossH','box',    halfExtent=[0.045,0.008,0.012], position=d, color='#e53935')
        self._mk('crossV','box',    halfExtent=[0.012,0.008,0.045], position=d, color='#e53935')
        self._mk('tag',   'box',    halfExtent=[0.040,0.008,0.018], position=d, color='#1565c0')
        # Neck
        self._mk('neck',  'box',    halfExtent=[0.052,0.052,0.065], position=d, color='#f5cba7')
        # Head & face
        self._mk('head',  'sphere', radius=0.155,                   position=d, color='#f5cba7')
        self._mk('ewl',   'sphere', radius=0.050,                   position=d, color='white')
        self._mk('ewr',   'sphere', radius=0.050,                   position=d, color='white')
        self._mk('el',    'sphere', radius=0.036,                   position=d, color='#1a237e')
        self._mk('er',    'sphere', radius=0.036,                   position=d, color='#1a237e')
        self._mk('ebl',   'box',    halfExtent=[0.038,0.006,0.008], position=d, color='#5d4037')
        self._mk('ebr',   'box',    halfExtent=[0.038,0.006,0.008], position=d, color='#5d4037')
        self._mk('nose',  'sphere', radius=0.020,                   position=d, color='#f0b27a')
        self._mk('mth',   'box',    halfExtent=[0.048,0.007,0.010], position=d, color='#e57373')
        self._mk('earl',  'sphere', radius=0.036,                   position=d, color='#f5cba7')
        self._mk('earr',  'sphere', radius=0.036,                   position=d, color='#f5cba7')
        # Nurse cap
        self._mk('cap',   'box',    halfExtent=[0.145,0.105,0.038], position=d, color='white')
        self._mk('cap2',  'box',    halfExtent=[0.100,0.075,0.055], position=d, color='white')
        self._mk('capst', 'box',    halfExtent=[0.148,0.107,0.010], position=d, color='#1565c0')
        self._mk('capcH', 'box',    halfExtent=[0.030,0.006,0.008], position=d, color='#e53935')
        self._mk('capcV', 'box',    halfExtent=[0.008,0.006,0.030], position=d, color='#e53935')
        # Left arm + clipboard
        self._mk('lau',   'box',    halfExtent=[0.048,0.048,0.155], position=d, color='#ffffff')
        self._mk('lal',   'box',    halfExtent=[0.040,0.040,0.125], position=d, color='#f5cba7')
        self._mk('lah',   'sphere', radius=0.050,                   position=d, color='#f5cba7')
        self._mk('clip',  'box',    halfExtent=[0.075,0.015,0.100], position=d, color='#8d6e63')
        self._mk('paper', 'box',    halfExtent=[0.065,0.010,0.088], position=d, color='white')
        self._mk('ln1',   'box',    halfExtent=[0.050,0.005,0.006], position=d, color='#bdbdbd')
        self._mk('ln2',   'box',    halfExtent=[0.050,0.005,0.006], position=d, color='#bdbdbd')
        self._mk('ln3',   'box',    halfExtent=[0.040,0.005,0.006], position=d, color='#90caf9')
        # Right arm + probe
        self._mk('rau',   'box',    halfExtent=[0.048,0.048,0.155], position=d, color='#ffffff')
        self._mk('ral',   'box',    halfExtent=[0.040,0.040,0.125], position=d, color='#f5cba7')
        self._mk('rah',   'sphere', radius=0.050,                   position=d, color='#f5cba7')
        self._mk('probe', 'box',    halfExtent=[0.022,0.022,0.075], position=d, color='#37474f')
        self._mk('probeL','sphere', radius=0.028,                   position=d, color='#00e676')
        # Legs & shoes
        self._mk('llt',   'box',    halfExtent=[0.058,0.058,0.160], position=d, color='#bbdefb')
        self._mk('lls',   'box',    halfExtent=[0.048,0.048,0.120], position=d, color='#bbdefb')
        self._mk('llf',   'box',    halfExtent=[0.070,0.042,0.030], position=d, color='white')
        self._mk('rlt',   'box',    halfExtent=[0.058,0.058,0.160], position=d, color='#bbdefb')
        self._mk('rls',   'box',    halfExtent=[0.048,0.048,0.120], position=d, color='#bbdefb')
        self._mk('rlf',   'box',    halfExtent=[0.070,0.042,0.030], position=d, color='white')

        self.update()
        print("[NURSE] Spawned ✔")

    def _w(self, lx, ly, lz):
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        return (self.x + cy*lx - sy*ly,
                self.y + sy*lx + cy*ly,
                self.z + lz)

    def _p(self, key, lx, ly, lz, rx=0, ry=0):
        wx, wy, wz = self._w(lx, ly, lz)
        place(self.ids[key], wx, wy, wz, rx, ry, self.yaw)

    def update(self):
        a   = self.arm
        la  = self.larm
        nd  = self.nod
        wt  = self.wt

        LL = math.sin(wt) * 0.065
        RL = math.sin(wt + math.pi) * 0.065

        # Body
        self._p('torso', 0, 0,      0.740)
        self._p('pants', 0, 0,      0.388)
        self._p('apron', 0,-0.108,  0.730)
        self._p('crossH',0,-0.118,  0.840)
        self._p('crossV',0,-0.118,  0.840)
        self._p('tag',   0,-0.118,  0.800)

        # Neck + head
        self._p('neck',  0, 0, 1.018)
        hx,hy,hz = self._w(0, 0, 1.210)
        place(self.ids['head'], hx, hy, hz, nd, 0, self.yaw)

        # Cap
        cx,cy_,cz = self._w(0, 0, 1.390)
        place(self.ids['cap'],  cx, cy_, cz,        nd, 0, self.yaw)
        place(self.ids['cap2'], cx, cy_, cz + 0.05, nd, 0, self.yaw)
        place(self.ids['capst'],cx, cy_, cz - 0.02, nd, 0, self.yaw)
        place(self.ids['capcH'],cx, cy_, cz + 0.042,nd, 0, self.yaw)
        place(self.ids['capcV'],cx, cy_, cz + 0.042,nd, 0, self.yaw)

        # Face (front = local -Y)
        self._p('ewl', -0.062, -0.125, 1.228, nd)
        self._p('ewr',  0.062, -0.125, 1.228, nd)
        self._p('el',  -0.062, -0.138, 1.228, nd)
        self._p('er',   0.062, -0.138, 1.228, nd)
        self._p('ebl', -0.062, -0.138, 1.252, nd)
        self._p('ebr',  0.062, -0.138, 1.252, nd)
        self._p('nose', 0,     -0.148, 1.196, nd)
        self._p('mth',  0,     -0.145, 1.168, nd)
        self._p('earl',-0.194,  0.010, 1.210, nd)
        self._p('earr', 0.194,  0.010, 1.210, nd)

        # Left arm + clipboard
        wzh = 0.330 + la * 0.280
        lly = -la * 0.150
        self._p('lau', -0.228, 0,   0.810 + la*0.120)
        self._p('lal', -0.228, lly, 0.510 + la*0.200)
        self._p('lah', -0.228, lly, wzh)
        clx,cly,clz = self._w(-0.228, lly - 0.016, wzh + 0.005)
        place(self.ids['clip'],  clx, cly, clz, 0, 0, self.yaw)
        place(self.ids['paper'], clx, cly, clz, 0, 0, self.yaw)
        place(self.ids['ln1'],   clx, cly, clz+0.040, 0, 0, self.yaw)
        place(self.ids['ln2'],   clx, cly, clz+0.010, 0, 0, self.yaw)
        place(self.ids['ln3'],   clx, cly, clz-0.020, 0, 0, self.yaw)

        # Right arm — IK to arm_target
        tx, ty, tz = self.arm_tgt
        cy_r, sy_r = math.cos(self.yaw), math.sin(self.yaw)
        dm_x = tx - self.x
        dm_y = ty - self.y
        dz   = tz - self.z
        lx1  =  cy_r*dm_x + sy_r*dm_y
        ly1  = -sy_r*dm_x + cy_r*dm_y
        lz1  =  dz

        lx0, ly0, lz0 = self.ARM0_LX, self.ARM0_LY, self.ARM0_LZ
        t_  = ease(a)
        hlx = lerp(lx0, lx1, t_)
        hly = lerp(ly0, ly1, t_)
        hlz = lerp(lz0, lz1, t_)

        self._p('rau', 0.228, lerp(0, ly1*0.28, t_),
                lerp(0.810, lz1*0.72, t_), -t_*0.85)
        self._p('ral', lerp(0.228,hlx*0.82,t_),
                lerp(0,hly*0.62,t_),
                lerp(0.510,hlz*0.78,t_))
        hwx,hwy,hwz = self._w(hlx, hly, hlz)
        place(self.ids['rah'],    hwx, hwy, hwz, 0, 0, self.yaw)
        place(self.ids['probe'],  hwx, hwy, hwz, 0, 0, self.yaw)
        place(self.ids['probeL'], hwx, hwy, hwz+0.082, 0, 0, self.yaw)
        self.hand_wx, self.hand_wy, self.hand_wz = hwx, hwy, hwz

        # Legs
        self._p('llt', -0.10, LL*0.5,  0.268)
        self._p('lls', -0.10, LL*0.25, 0.088)
        self._p('llf', -0.10, 0,       0.020)
        self._p('rlt',  0.10, RL*0.5,  0.268)
        self._p('rls',  0.10, RL*0.25, 0.088)
        self._p('rlf',  0.10, 0,       0.020)

    def set_target(self, tx, ty, tz):
        self.arm_tgt = (tx, ty, tz)

    def set_pose(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw
        self.update()


# ═══════════════════════════════════════════════════════════════
#   MULTI-DIRECTIONAL NAVIGATION
#   walk_to moves in X AND Y simultaneously (true diagonal walk)
# ═══════════════════════════════════════════════════════════════
def walk_to(nurse, tx, ty, spd=1.65):
    """
    Walk nurse from current position to (tx,ty).
    Turns to face direction first, then walks directly.
    Supports any direction: +X, -X, +Y, -Y, diagonal.
    """
    sx, sy = nurse.x, nurse.y
    dx, dy = tx - sx, ty - sy
    d = math.sqrt(dx*dx + dy*dy)
    if d < 0.025:
        return

    # Compute walk direction yaw
    target_yaw = math.atan2(dy, dx)

    # Smooth turn toward destination
    turn_to(nurse, target_yaw, steps=22)

    # Walk with leg animation
    steps = max(1, int(d / (spd * TICK)))
    for i in range(steps + 1):
        t = ease(i / steps)
        nurse.x = lerp(sx, tx, t)
        nurse.y = lerp(sy, ty, t)
        nurse.wt += 0.19
        nurse.update()
        time.sleep(TICK)

    nurse.x, nurse.y = tx, ty
    nurse.wt = 0.0
    nurse.update()


def turn_to(nurse, target_yaw, steps=28):
    """Smooth rotation to any yaw angle."""
    start = nurse.yaw
    diff  = norm_angle(target_yaw - start)
    if abs(diff) < 0.008:
        return
    for i in range(steps + 1):
        nurse.yaw = start + diff * ease(i / steps)
        nurse.update()
        time.sleep(TICK)


def move_arm(nurse, target, steps=40):
    start = nurse.arm
    for i in range(steps + 1):
        nurse.arm = lerp(start, target, ease(i / steps))
        nurse.update()
        time.sleep(TICK)


def move_larm(nurse, target, steps=32):
    start = nurse.larm
    for i in range(steps + 1):
        nurse.larm = lerp(start, target, ease(i / steps))
        nurse.update()
        time.sleep(TICK)


def do_nod(nurse, amp=0.28, reps=2):
    steps = 20
    for _ in range(reps):
        for i in range(steps + 1):
            nurse.nod = amp * math.sin(math.pi * i / steps)
            nurse.update()
            time.sleep(TICK)
    nurse.nod = 0.0
    nurse.update()


def do_wave(nurse, reps=3):
    for _ in range(reps):
        move_arm(nurse, 0.72, steps=14)
        time.sleep(0.07)
        move_arm(nurse, 0.44, steps=11)
        time.sleep(0.06)
    move_arm(nurse, 0.0, steps=20)


def do_spin(nurse, rounds=1, steps=56):
    start = nurse.yaw
    for i in range(steps + 1):
        nurse.yaw = start + rounds * 2 * math.pi * ease(i / steps)
        nurse.update()
        time.sleep(TICK)
    nurse.yaw = start
    nurse.update()


# ═══════════════════════════════════════════════════════════════
#   HUD
# ═══════════════════════════════════════════════════════════════
_sid = [None]; _rid = [None]; _vit = [None]; _ana = [None]

def status(msg, col='#a5d6a7'):
    if _sid[0]: bb.removeDebugObject(_sid[0])
    _sid[0] = bb.createDebugText(f"▶  {msg}", (0, -4.6, 0.50),
        bb.getQuaternionFromEuler([0,0,0]), color=col, size=0.20)
    print(f"   >> {msg}")

def set_run(n):
    if _rid[0]: bb.removeDebugObject(_rid[0])
    _rid[0] = bb.createDebugText(
        f"NURSE-BOT v7  |  Smart Clinic 204  |  Run #{n}",
        (0, 0, 3.55), bb.getQuaternionFromEuler([0,0,0]),
        color='#00e5ff', size=0.22)

def show_analysis(msg, col='#80deea'):
    if _ana[0]: bb.removeDebugObject(_ana[0])
    _ana[0] = bb.createDebugText(f"🔍  {msg}", (0, 0, 3.20),
        bb.getQuaternionFromEuler([0,0,0]), color=col, size=0.19)

def show_vitals(o2, glucose):
    if _vit[0]: bb.removeDebugObject(_vit[0])
    oc = '#00e676' if o2 >= 95 else '#ff5252'
    gc = '#aed581' if 70 <= glucose <= 140 else '#ff9800'
    _vit[0] = bb.createDebugText(
        f"SpO2: {o2}%   |   Glucose: {glucose} mg/dL",
        (0, 0, 2.90), bb.getQuaternionFromEuler([0,0,0]),
        color=oc, size=0.20)
    print(f"   [VITALS] SpO2={o2}%  Glucose={glucose} mg/dL")

def clear_hud():
    for ref in [_ana, _vit]:
        if ref[0]:
            bb.removeDebugObject(ref[0])
            ref[0] = None


# ═══════════════════════════════════════════════════════════════
#   MAIN SEQUENCE — 12 phases, all directions
# ═══════════════════════════════════════════════════════════════
def run_sequence(nurse, window, scan, run_no):
    set_run(run_no)
    clear_hud()
    nurse.arm  = 0.0
    nurse.larm = 0.0
    nurse.nod  = 0.0
    nurse.wt   = 0.0
    nurse.set_target(nurse.x + 1.0, nurse.y, 0.70)
    nurse.set_pose(NURSE_STATION_X, NURSE_STATION_Y, 0.0)

    o2_val = random.randint(93, 99)
    gl_val = random.randint(82, 148)

    # ── PHASE 1: Walk from station to scan position ───────────
    # Direction: +X and +Y diagonal
    status("Walking to patient scan position 🚶 (diagonal)", '#80deea')
    walk_to(nurse, *WP_SCAN)

    # Face the bed for scan
    turn_to(nurse, face_yaw(nurse.x, nurse.y, BED_CX, BED_CY), steps=24)
    time.sleep(0.20)

    # ── PHASE 2: Analyze / scan patient (sweep beam) ──────────
    status("ANALYZING patient position... 🔍", '#80deea')
    show_analysis("Scanning patient... body position detected")
    move_larm(nurse, 0.50, steps=18)   # raise clipboard for recording

    # Sweep scan beam across bed
    scan.scan_animate(steps=60)
    time.sleep(0.25)

    show_analysis(f"✔ Analysis complete — SpO2 est. {o2_val}%  |  Glucose est. {gl_val} mg/dL")
    do_nod(nurse, amp=0.18, reps=1)
    time.sleep(0.35)

    # ── PHASE 3: Walk to foot of bed (-Y direction) ───────────
    # Direction: +X, -Y
    status("Moving to foot of bed to check posture 🦶", '#b3e5fc')
    walk_to(nurse, *WP_FOOT)
    turn_to(nurse, face_yaw(nurse.x, nurse.y, BED_CX, BED_CY), steps=20)
    do_nod(nurse, amp=0.14, reps=1)
    time.sleep(0.25)

    # ── PHASE 4: Walk to vitals monitor on LEFT wall (-X) ─────
    # Direction: -X and +Y — crossing the room
    status("Checking vitals monitor on wall 📊", '#ffe082')
    walk_to(nurse, *WP_MONITOR)
    turn_to(nurse, face_yaw(nurse.x, nurse.y, -hw+0.30, BED_CY+0.50), steps=22)

    # Reach arm toward monitor
    nurse.set_target(-hw+0.30, BED_CY+0.55, 1.60)
    move_arm(nurse, 0.85, steps=32)
    time.sleep(0.40)
    show_analysis("Wall monitor: ECG normal ✔  HR: 72 BPM")
    do_nod(nurse, amp=0.14, reps=2)
    move_arm(nurse, 0.0, steps=22)
    time.sleep(0.25)

    # ── PHASE 5: Walk to equipment cart (+X, +Y) ──────────────
    # Direction: +X and +Y diagonal
    status("Walking to equipment cart for sensors 🏃", '#80deea')
    walk_to(nurse, *WP_CART)
    turn_to(nurse, face_yaw(nurse.x, nurse.y, CART_X, CART_Y), steps=20)

    # Pick up O2 sensor
    nurse.set_target(CART_X - 0.06, CART_Y - 0.08, 0.94)
    move_arm(nurse, 0.90, steps=28)
    time.sleep(0.35)
    move_arm(nurse, 0.0,  steps=20)
    time.sleep(0.22)

    # ── PHASE 6: Walk DIAGONALLY to bedside (+X, +Y) ──────────
    status("Approaching patient bedside with probe 🩺", '#80deea')
    walk_to(nurse, *WP_BEDSIDE)
    turn_to(nurse, face_yaw(nurse.x, nurse.y, P_HEAD_X, P_HEAD_Y), steps=26)
    time.sleep(0.25)

    # Greet patient
    status("Hello! I am your nurse robot. Checking vitals now 👋", '#fff9c4')
    do_nod(nurse, amp=0.22, reps=2)
    time.sleep(0.25)

    # ── PHASE 7: Open curtains (walk to window = +X direction) ─
    status("Opening curtains for natural light 🌅", '#ffe082')
    walk_to(nurse, *WP_WINDOW)
    turn_to(nurse, face_yaw(nurse.x, nurse.y, WIN_X, WIN_CY), steps=20)

    nurse.set_target(WIN_X - 0.18, WIN_CY - 0.70, WIN_Z + 0.48)
    move_arm(nurse, 0.88, steps=28)
    time.sleep(0.22)
    window.animate_open(steps=48)
    move_arm(nurse, 0.0, steps=20)
    time.sleep(0.22)

    # ── PHASE 8: Return to bedside (walk -X, any Y) ───────────
    status("Returning to patient for SpO2 check 🔴", '#80deea')
    walk_to(nurse, *WP_BEDSIDE)
    turn_to(nurse, face_yaw(nurse.x, nurse.y, P_WRIST_X, P_WRIST_Y), steps=24)
    time.sleep(0.22)

    # SpO2 check — probe to patient wrist
    status("Measuring SpO2 (oxygen level)... ⏳", '#b3e5fc')
    nurse.set_target(P_WRIST_X, P_WRIST_Y, P_WRIST_Z)
    move_arm(nurse, 1.0, steps=40)
    time.sleep(0.38)

    for _ in range(3):
        nurse.nod = 0.08; nurse.update(); time.sleep(0.28)
        nurse.nod = 0.0;  nurse.update(); time.sleep(0.28)

    show_vitals(o2_val, 0)
    status(f"SpO2 = {o2_val}% — {'NORMAL ✔' if o2_val >= 95 else 'LOW ⚠ Alerting!'}", '#00e676' if o2_val >= 95 else '#ff5252')

    move_larm(nurse, 0.80, steps=16)
    time.sleep(0.32)
    move_larm(nurse, 0.55, steps=14)
    move_arm(nurse,  0.0,  steps=28)
    time.sleep(0.28)

    # Glucose check — slightly different wrist position
    status("Measuring Glucose level... 🩸", '#ffe082')
    nurse.set_target(P_WRIST_X, P_WRIST_Y + 0.14, P_WRIST_Z)
    move_arm(nurse, 1.0, steps=38)
    time.sleep(0.38)

    for _ in range(3):
        nurse.nod = 0.08; nurse.update(); time.sleep(0.26)
        nurse.nod = 0.0;  nurse.update(); time.sleep(0.26)

    normal_gl = 70 <= gl_val <= 140
    show_vitals(o2_val, gl_val)
    status(f"Glucose = {gl_val} mg/dL — {'NORMAL ✔' if normal_gl else 'ABNORMAL ⚠'}", '#aed581' if normal_gl else '#ff9800')

    move_larm(nurse, 0.88, steps=16)
    time.sleep(0.32)
    move_larm(nurse, 0.60, steps=14)
    move_arm(nurse,  0.0,  steps=28)
    time.sleep(0.28)

    # ── PHASE 9: Curtain logic ────────────────────────────────
    if o2_val < 96:
        status("O2 low — closing curtains for rest 😴", '#b3e5fc')
        walk_to(nurse, *WP_WINDOW)    # -X, +Y direction
        turn_to(nurse, face_yaw(nurse.x, nurse.y, WIN_X, WIN_CY), steps=20)
        nurse.set_target(WIN_X-0.18, WIN_CY-0.70, WIN_Z+0.48)
        move_arm(nurse, 0.85, steps=26)
        time.sleep(0.22)
        window.animate_close(steps=48)
        move_arm(nurse, 0.0, steps=20)
        # Walk back to bedside (+X, -Y direction)
        walk_to(nurse, *WP_BEDSIDE)
        turn_to(nurse, face_yaw(nurse.x, nurse.y, P_HEAD_X, P_HEAD_Y), steps=22)
    else:
        status("Vitals normal — curtains stay open 🌤", '#a5d6a7')
        time.sleep(0.40)

    # ── PHASE 10: Write notes, wave goodbye ──────────────────
    status("Recording vitals on chart 📋", '#b2dfdb')
    move_larm(nurse, 1.0, steps=20)
    do_nod(nurse, amp=0.14, reps=3)
    time.sleep(0.38)
    move_larm(nurse, 0.65, steps=16)
    time.sleep(0.25)

    status("Goodbye! Rest well. I'll check back soon 👋", '#fff9c4')
    do_nod(nurse, amp=0.28, reps=1)
    do_wave(nurse, reps=3)
    time.sleep(0.22)
    do_nod(nurse, amp=0.50, reps=1)    # bow
    time.sleep(0.22)

    # ── PHASE 11: Spin ───────────────────────────────────────
    status("🎉 Health check complete!", '#80cbc4')
    do_spin(nurse, rounds=1, steps=54)
    move_larm(nurse, 0.0, steps=16)
    time.sleep(0.32)

    # ── PHASE 12: Walk back to nurse station ──────────────────
    # Direction: -X, -Y diagonal (back to origin corner)
    status("Returning to nurse station (-X, -Y) 🚶", '#b0bec5')
    walk_to(nurse, *WP_STATION)
    turn_to(nurse, 0.0, steps=18)
    status("Standby at nurse station ✔", '#80deea')
    time.sleep(1.0)


# ═══════════════════════════════════════════════════════════════
#   MAIN
# ═══════════════════════════════════════════════════════════════
print("=" * 58)
print("  BROWSERBOTICS — NURSE ROBOT v7")
print("  Open clinic (no ceiling, open front)")
print("  Multi-directional walk: +X -X +Y -Y diagonal")
print("  Tasks: Scan → Foot check → Monitor → Cart → Bedside")
print("         O2 check → Glucose → Curtain → Goodbye → Return")
print("=" * 58)

build_room()
build_tv()
build_bed()
build_patient()

window = Window()
scan   = ScanBeam()

nurse = NurseRobot()
nurse.set_target(NURSE_STATION_X + 1.0, NURSE_STATION_Y, 0.70)
nurse.spawn()

bb.createDebugText("✚  NURSE-BOT v7  |  Open Clinic Room 204",
    (0, 0, 3.78), bb.getQuaternionFromEuler([0,0,0]),
    color='#00e5ff', size=0.26)
bb.createDebugText(
    "Scan → Monitor → Cart → Bedside → O2 → Glucose → Curtain → Bye",
    (0, 0, 3.56), bb.getQuaternionFromEuler([0,0,0]),
    color='white', size=0.14)

status("World ready — starting in 2s...", '#80deea')
time.sleep(2.0)

run_no = 0
while True:
    run_no += 1
    print(f"\n{'='*50}\n  HEALTH CHECK RUN #{run_no}\n{'='*50}")
    run_sequence(nurse, window, scan, run_no)
    print(f"  Run #{run_no} done. Next run in 4s...")
    time.sleep(4.0)
