import math

neu_file = r"D:\work4\cfd\test_j3_defeatured.neu"

with open(neu_file) as f:
    text = f.read()

lines = text.splitlines()
in_cells = False
cells = {}
in_nodes = False
nodes = {}

for line in lines:
    if "NODAL COORDINATES" in line:
        in_nodes = True
        continue
    elif "ENDOFSECTION" in line and in_nodes:
        in_nodes = False
        continue
    elif "ELEMENTS/CELLS" in line:
        in_cells = True
        continue
    elif "ENDOFSECTION" in line and in_cells:
        in_cells = False
        continue
        
    if in_nodes:
        parts = line.split()
        if len(parts) >= 4 and parts[0].isdigit():
            nodes[int(parts[0])] = (float(parts[1]), float(parts[2]), float(parts[3]))
            
    if in_cells:
        parts = line.split()
        if len(parts) >= 6 and parts[0].isdigit():
            cells[int(parts[0])] = [int(p) for p in parts[3:7]]

target = (-0.746856, 1.23832, -1.26550)
closest_cell = None
min_dist = 1e9

for cid, c_nodes in cells.items():
    pts = [nodes[n] for n in c_nodes]
    cx = sum(p[0] for p in pts) / 4.0
    cy = sum(p[1] for p in pts) / 4.0
    cz = sum(p[2] for p in pts) / 4.0
    d = math.sqrt((cx - target[0])**2 + (cy - target[1])**2 + (cz - target[2])**2)
    if d < min_dist:
        min_dist = d
        closest_cell = (cid, c_nodes, pts, (cx, cy, cz))

cid, c_nodes, pts, (cx, cy, cz) = closest_cell
print(f"Closest cell in NEU: ID={cid}, dist={min_dist*1e3:.4f} mm, centroid=({cx:.6f}, {cy:.6f}, {cz:.6f})")
print("Nodes:")
for n, p in zip(c_nodes, pts):
    print(f"  Node {n:5d}: ({p[0]:.8f}, {p[1]:.8f}, {p[2]:.8f})")

import itertools
print("\nEdges:")
for (n1, p1), (n2, p2) in itertools.combinations(zip(c_nodes, pts), 2):
    d = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)
    print(f"  Edge {n1}-{n2}: {d*1e3:.6f} mm ({d*1e6:.2f} um)")
