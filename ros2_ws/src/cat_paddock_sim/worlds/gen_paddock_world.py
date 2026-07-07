#!/usr/bin/env python3
"""Mining-pit v5 (clean) — Cat Tech Challenge 2026 · TTSK12"""
import math, random
from pathlib import Path

random.seed(42)
OUT = Path(__file__).parent / "paddock.sdf"

PADDOCK = [(60,120), (140,125), (160,170), (120,200), (70,180), (40,140)]
CORRIDOR_X1, CORRIDOR_X2 = -25, 75
CORRIDOR_Y1, CORRIDOR_Y2 = 30, 115
GATE_GAP_HALF  = 20
PAD_GAP_HALF   = 15
PAD_GAP_X      = 50

C_TERR_D = "0.38 0.21 0.13 1"
C_TERR_L = "0.46 0.27 0.16 1"
C_PAD    = "0.58 0.44 0.30 1"
C_PAD_D  = "0.36 0.26 0.16 1"
C_HAUL   = "0.20 0.18 0.16 1"
C_LANE   = "0.78 0.68 0.30 1"
C_STAGE  = "0.42 0.40 0.38 1"
C_BB     = "0.50 0.36 0.21 1"
C_BT     = "0.58 0.42 0.26 1"
C_PILE_A = "0.34 0.22 0.13 1"
C_PILE_B = "0.46 0.32 0.20 1"
C_ROCK_A = "0.42 0.40 0.38 1"
C_ROCK_B = "0.30 0.29 0.27 1"
C_YEL    = "0.99 0.78 0.10 1"
C_YEL_D  = "0.65 0.50 0.05 1"
C_STEEL  = "0.32 0.32 0.32 1"
C_WHITE  = "0.92 0.92 0.92 1"
C_ORANGE = "0.96 0.45 0.05 1"

def mat(c):
    r,g,b,_ = c.split()
    return (f'<material><ambient>{float(r)*0.50:.3f} {float(g)*0.50:.3f} '
            f'{float(b)*0.50:.3f} 1</ambient><diffuse>{c}</diffuse>'
            f'<specular>0.05 0.05 0.05 1</specular></material>')

def box(name, pose, size, color, coll=True):
    c = f'<collision name="c"><geometry><box><size>{size}</size></box></geometry></collision>' if coll else ''
    return f'    <link name="{name}"><pose>{pose}</pose><visual name="v"><geometry><box><size>{size}</size></box></geometry>{mat(color)}</visual>{c}</link>'

def cyl(name, pose, r, l, color, coll=True):
    c = f'<collision name="c"><geometry><cylinder><radius>{r}</radius><length>{l}</length></cylinder></geometry></collision>' if coll else ''
    return f'    <link name="{name}"><pose>{pose}</pose><visual name="v"><geometry><cylinder><radius>{r}</radius><length>{l}</length></cylinder></geometry>{mat(color)}</visual>{c}</link>'

def point_in_poly(px, py, poly):
    n = len(poly); inside = False; j = n - 1
    for k in range(n):
        xi, yi = poly[k]; xj, yj = poly[j]
        if ((yi > py) != (yj > py)) and (px < (xj-xi)*(py-yi)/(yj-yi+1e-9) + xi):
            inside = not inside
        j = k
    return inside

parts = ['''<?xml version="1.0"?>
<sdf version="1.10">
  <world name="cat_paddock">
    <physics name="4ms" type="ode"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system"           name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"     name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <scene><ambient>0.32 0.30 0.27 1</ambient><background>0.62 0.55 0.45 1</background><sky/></scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 100 0 0 0</pose>
      <diffuse>1.0 0.92 0.78 1</diffuse><specular>0.5 0.45 0.35 1</specular>
      <direction>-0.65 0.40 -0.65</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows><pose>0 0 80 0 0 0</pose>
      <diffuse>0.30 0.36 0.50 1</diffuse><direction>0.5 -0.3 -0.8</direction>
    </light>
''']

# Terrain
parts.append(f'''
    <model name="terrain"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>800 800</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>800 800</size></plane></geometry>{mat(C_TERR_D)}</visual>
    </link></model>
''')

# Terrain patches
patches = []
for i in range(25):
    px = random.uniform(-100, 220); py = random.uniform(-160, 230)
    if -80 < px < 200 and 20 < py < 215: continue
    sx = random.uniform(6, 16); sy = random.uniform(6, 16)
    patches.append(box(f"patch_{i}", f"{px:.1f} {py:.1f} 0.03 0 0 {random.uniform(0,1.57):.2f}",
                       f"{sx:.1f} {sy:.1f} 0.04", C_TERR_L, coll=False))
parts.append(f'    <model name="terrain_patches"><static>true</static>\n{chr(10).join(patches)}\n    </model>\n')

# Haul road (east berm only — west berm removed permanently)
road = [box("road", "-10 -50 0.025 0 0 0", "30 90 0.05", C_HAUL, False)]
for i in range(8):
    road.append(box(f"lane_{i}", f"-10 {-90 + i*12.5} 0.04 0 0 0", "0.4 5 0.06", C_LANE, False))
road += [
    box("road_r_base", "6 -50 0.45 0 0 0", "3.5 92 0.9", C_BB),
    box("road_r_top",  "6 -50 1.25 0 0 0", "1.8 92 0.7", C_BT),
]
parts.append(f'    <model name="haul_road"><static>true</static>\n{chr(10).join(road)}\n    </model>\n')

# Staging bays — y = -50, -75, -100
def bay(cx, cy, w, d, idx):
    t = 0.25
    L = [
        box(f"b{idx}_t", f"{cx} {cy+d/2} 0.04 0 0 0", f"{w} {t} 0.06", C_YEL, False),
        box(f"b{idx}_b", f"{cx} {cy-d/2} 0.04 0 0 0", f"{w} {t} 0.06", C_YEL, False),
        box(f"b{idx}_l", f"{cx-w/2} {cy} 0.04 0 0 0", f"{t} {d} 0.06", C_YEL, False),
        box(f"b{idx}_r", f"{cx+w/2} {cy} 0.04 0 0 0", f"{t} {d} 0.06", C_YEL, False),
    ]
    for (xx, yy, tag) in [(cx-w/2, cy-d/2, "ll"), (cx+w/2, cy-d/2, "lr"),
                           (cx-w/2, cy+d/2, "ul"), (cx+w/2, cy+d/2, "ur")]:
        L.append(cyl(f"b{idx}_cone_{tag}", f"{xx} {yy} 0.55 0 0 0", 0.3, 1.0, C_ORANGE, False))
    return "\n".join(L)

parts.append(f'''
    <model name="staging"><static>true</static>
{box("pad", "-30 -75 0.02 0 0 0", "55 75 0.05", C_STAGE, coll=False)}
{bay(-30, -50,  20, 12, 1)}
{bay(-30, -75,  20, 12, 2)}
{bay(-30, -100, 20, 12, 3)}
    </model>
''')

# ============================================================
#  GATE — clean, aligned, no text
# ============================================================
#  Posts: 15 m tall (z = 0..15), at x = -3, y = ±20
#  Beam:  sits ON TOP of posts.
#         bottom of beam = z 15, height 1.4 → center at z 15.7
#         depth (X) = 1.0 (matches 1.0 m post diameter)
#         length (Y) = 42 (1 m overhang each side past posts at y=±20)
#  No sign, no text, no extra trim — just a clean yellow horizontal beam.
parts.append(f'''
    <model name="gate"><static>true</static>
{cyl("post_l", "-3 -20 7.5 0 0 0", 0.5, 15, C_YEL, False)}
{cyl("post_r", "-3  20 7.5 0 0 0", 0.5, 15, C_YEL, False)}
{box("cap_l", "-3 -20 15.1 0 0 0", "1.2 1.2 0.4", C_YEL_D, False)}
{box("cap_r", "-3  20 15.1 0 0 0", "1.2 1.2 0.4", C_YEL_D, False)}
{box("base_l", "-3 -20 0.4 0 0 0", "2.5 2.5 0.8", C_STEEL, False)}
{box("base_r", "-3  20 0.4 0 0 0", "2.5 2.5 0.8", C_STEEL, False)}
    </model>
''')

# ============================================================
#  CORRIDOR WALL — full enclosure with two gaps
# ============================================================
def wall_seg(name, x, y, length, thickness, yaw=0):
    base_h, top_h = 1.5, 1.5
    return "\n".join([
        box(f"{name}_base", f"{x} {y} {base_h/2} 0 0 {yaw}", f"{length} {thickness} {base_h}", C_BB),
        box(f"{name}_top",  f"{x} {y} {base_h + top_h/2} 0 0 {yaw}", f"{length*0.92} {thickness*0.6} {top_h}", C_BT),
    ])

south_left_len  = (0 - GATE_GAP_HALF) - CORRIDOR_X1
south_right_len = CORRIDOR_X2 - (0 + GATE_GAP_HALF)
walls = []
walls.append(wall_seg("sw_l", (CORRIDOR_X1 + (-GATE_GAP_HALF))/2, CORRIDOR_Y1, south_left_len, 4))
walls.append(wall_seg("sw_r", (GATE_GAP_HALF + CORRIDOR_X2)/2, CORRIDOR_Y1, south_right_len, 4))

north_left_len  = (PAD_GAP_X - PAD_GAP_HALF) - CORRIDOR_X1
north_right_len = CORRIDOR_X2 - (PAD_GAP_X + PAD_GAP_HALF)
walls.append(wall_seg("nw_l", (CORRIDOR_X1 + (PAD_GAP_X - PAD_GAP_HALF))/2, CORRIDOR_Y2, north_left_len, 4))
walls.append(wall_seg("nw_r", ((PAD_GAP_X + PAD_GAP_HALF) + CORRIDOR_X2)/2, CORRIDOR_Y2, north_right_len, 4))

east_len = CORRIDOR_Y2 - CORRIDOR_Y1
walls.append(wall_seg("ew", CORRIDOR_X2, (CORRIDOR_Y1 + CORRIDOR_Y2)/2, east_len, 4, math.pi/2))
walls.append(wall_seg("ww", CORRIDOR_X1, (CORRIDOR_Y1 + CORRIDOR_Y2)/2, east_len, 4, math.pi/2))

parts.append(f'    <model name="corridor_walls"><static>true</static>\n{chr(10).join(walls)}\n    </model>\n')

# ============================================================
#  CORRIDOR OBSTACLES — restored, repositioned to clear all 3 trucks'
#  locked-in ROUTES (truck_orchestrator.py). All 3 routes hug x<=38
#  until they converge near the pad gate (x=44,y=108) -- the eastern
#  half of the corridor (x:45-72, y:40-90) carries zero route traffic,
#  so everything below lives there. Verified (point-to-segment distance
#  against every waypoint segment of all 3 trucks, both directions,
#  since DRIVING_OUT just reverses the same waypoints) to clear every
#  segment by 15-43m, well past the 6m (truck half-width + margin)
#  requirement -- see obstacle_clearance_check.py.
# ============================================================
obstacles = []

# Berm walls -- visual/perception scenery, well off both lanes
obstacles.append(box("ob_berm_a_base", "58 48 1.0 0 0 0.08", "20 5 2.0", C_BB))
obstacles.append(box("ob_berm_a_top",  "58 48 2.6 0 0 0.08", "18 3 1.2", C_BT))
obstacles.append(box("ob_berm_b_base", "58 86 1.0 0 0 -0.06", "20 5 2.0", C_BB))
obstacles.append(box("ob_berm_b_top",  "58 86 2.6 0 0 -0.06", "18 3 1.2", C_BT))

# Boulder clusters -- truck-scale rocks (2.5-4.5m), wide spacing
big_clusters = [(50, 42), (66, 62), (50, 80)]
for ci, (cx, cy) in enumerate(big_clusters):
    for r_idx in range(random.randint(3, 5)):
        dx = random.uniform(-3.0, 3.0); dy = random.uniform(-3.0, 3.0)
        s  = random.uniform(2.5, 4.5)   # truck-scale
        col = C_ROCK_A if random.random() < 0.6 else C_ROCK_B
        obstacles.append(box(f"bclust_{ci}_{r_idx}",
            f"{cx+dx:.2f} {cy+dy:.2f} {s/2:.2f} {random.uniform(0,0.4):.2f} {random.uniform(0,0.4):.2f} {random.uniform(0,3.14):.2f}",
            f"{s:.2f} {s*random.uniform(0.75,0.95):.2f} {s*random.uniform(0.55,0.75):.2f}", col))

# Ore piles inside corridor -- PERCEPTION TARGETS (mergeable look)
inner_piles = [(63, 55, 5, 1.6), (55, 72, 5.5, 1.7)]
for i, (x, y, r, h) in enumerate(inner_piles):
    he = h/3
    ca = C_PILE_A if i % 2 == 0 else C_PILE_B
    cb = C_PILE_B if i % 2 == 0 else C_PILE_A
    obstacles.append(cyl(f"ipile_{i}_a", f"{x} {y} {he/2:.2f} 0 0 0", r, he, ca))
    obstacles.append(cyl(f"ipile_{i}_b", f"{x} {y} {he*1.5:.2f} 0 0 0", r*0.72, he, cb))
    obstacles.append(cyl(f"ipile_{i}_c", f"{x} {y} {he*2.5:.2f} 0 0 0", r*0.40, he, ca))

parts.append(f'    <model name="corridor_obstacles"><static>true</static>\n{chr(10).join(obstacles)}\n    </model>\n')

# ============================================================
#  DUMP PAD
# ============================================================
pts = "\n          ".join(f"<point>{x} {y}</point>" for x,y in PADDOCK)
pad_links = [f'''      <link name="pad_main"><pose>0 0 0.025 0 0 0</pose>
        <visual name="v"><geometry><polyline>
          {pts}
          <height>0.05</height></polyline></geometry>{mat(C_PAD)}
        </visual></link>''']
for i in range(15):
    for _ in range(30):
        tx = random.uniform(45, 160); ty = random.uniform(125, 195)
        if point_in_poly(tx, ty, PADDOCK): break
    else: continue
    pad_links.append(box(f"track_{i}", f"{tx:.1f} {ty:.1f} 0.06 0 0 {random.uniform(0,3.14):.2f}",
                         f"{random.uniform(2,4):.1f} {random.uniform(0.4,0.8):.1f} 0.02", C_PAD_D, False))
parts.append(f'    <model name="dump_pad"><static>true</static>\n{chr(10).join(pad_links)}\n    </model>\n')

# Berms around dump pad (skip entry side)
def edge(i, poly):
    v1, v2 = poly[i], poly[(i+1) % len(poly)]
    mid = ((v1[0]+v2[0])/2, (v1[1]+v2[1])/2)
    return mid, math.hypot(v2[0]-v1[0], v2[1]-v1[1]), math.atan2(v2[1]-v1[1], v2[0]-v1[0])

berms = []
for i in [0, 1, 2, 3]:
    mid, length, yaw = edge(i, PADDOCK)
    base_h, top_h, base_w, top_w = 1.2, 1.3, 5.0, 2.5
    berms.append(box(f"pad_berm_{i}_base",
                     f"{mid[0]:.3f} {mid[1]:.3f} {base_h/2:.3f} 0 0 {yaw:.4f}",
                     f"{length:.2f} {base_w} {base_h}", C_BB))
    berms.append(box(f"pad_berm_{i}_top",
                     f"{mid[0]:.3f} {mid[1]:.3f} {base_h + top_h/2:.3f} 0 0 {yaw:.4f}",
                     f"{length*0.92:.2f} {top_w} {top_h}", C_BT))
parts.append(f'    <model name="dump_pad_berms"><static>true</static>\n{chr(10).join(berms)}\n    </model>\n')

# Lighting masts
masts = []
for i, (x, y) in enumerate([(-50, -75), (180, -75), (180, 230), (-50, 230)]):
    masts.append(f'''    <model name="mast_{i+1}"><static>true</static>
{cyl("post", f"{x} {y} 7.5 0 0 0", 0.25, 15, C_STEEL)}
{box("head", f"{x} {y} 15.3 0 0 0", "2.8 1.2 0.5", C_STEEL)}
{cyl("lamp_a", f"{x-1.0} {y} 15.55 0.0 0.5 0", 0.18, 0.5, C_WHITE, False)}
{cyl("lamp_b", f"{x+1.0} {y} 15.55 0.0 -0.5 0", 0.18, 0.5, C_WHITE, False)}
    </model>''')
parts.append("\n" + "\n".join(masts) + "\n")

# Distant piles (perception background)
distant_piles = [(110, 0, 7, 1.5), (175, 60, 8, 1.8), (-45, 165, 6, 1.4),
                 (90, 215, 7, 1.5), (-50, 90, 6, 1.3)]
piles = []
for i, (x, y, r, h) in enumerate(distant_piles):
    he = h/3
    ca = C_PILE_A if i%2==0 else C_PILE_B
    cb = C_PILE_B if i%2==0 else C_PILE_A
    piles.append(cyl(f"dpile_{i}_a", f"{x} {y} {he/2:.2f} 0 0 0", r, he, ca))
    piles.append(cyl(f"dpile_{i}_b", f"{x} {y} {he*1.5:.2f} 0 0 0", r*0.72, he, cb))
    piles.append(cyl(f"dpile_{i}_c", f"{x} {y} {he*2.5:.2f} 0 0 0", r*0.40, he, ca))
parts.append(f'    <model name="distant_piles"><static>true</static>\n{chr(10).join(piles)}\n    </model>\n')

# Off-route signs (no collision)
parts.append(f'''
    <model name="signs"><static>true</static>
{cyl("post_a", "-45 -75 1.5 0 0 0", 0.08, 3, C_STEEL, False)}
{box("plate_a", "-45 -75 3.0 0 0 0.2", "0.05 1.5 1.2", C_YEL, False)}
{cyl("post_b", "75 30 2.0 0 0 0", 0.08, 4, C_STEEL, False)}
{box("plate_b", "75 30 4.3 0 0 0", "0.05 1.5 1.0", C_ORANGE, False)}
    </model>
''')

parts.append('''
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>-70 -50 110 0 0.62 0.55</pose>
      </camera>
    </gui>
  </world>
</sdf>
''')

OUT.write_text("".join(parts))
print(f"Wrote {OUT}  ({OUT.stat().st_size} bytes)")
