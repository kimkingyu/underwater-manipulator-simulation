"""Generate sc_build_fluid_domain_qb.py from 09_seal_j2j3/sc_build_fluid_domain_v5.py (pose qb)."""
from pathlib import Path

ROOT = Path(r"D:\work4\new_model_sim")
s = (ROOT / "09_seal_j2j3" / "sc_build_fluid_domain_v5.py").read_text(encoding="utf-8")


def rep(a, b):
    global s
    assert s.count(a) == 1, (a, s.count(a))
    s = s.replace(a, b)


rep("# Water domain v4 (static case)",
    "# POSE qb = [0.45, -0.50, -0.40] (13_pose_q1): v5 seals built at q = 0, then every extended part is moved\n"
    "# by its joint-chain rotation (uvms_geometry.m axes/centres). l2 cylinder-gap fill (c) dropped: at qb l2 is\n"
    "# >3 mm from base0/l1. gap_faces (0.2 mm J2/J3 plane gaps) found in the posed frame. Writes 13_pose_q1 only.\n"
    "# Water domain v4 (static case)")
rep(r"D = r'D:\work4\new_model_sim\09_seal_j2j3'", r"D = r'D:\work4\new_model_sim\13_pose_q1'")
rep("OUT_SCDOC = os.path.join(D, 'fluid_domain_v5.scdoc')\nOUT_JSON = os.path.join(D, 'fluid_domain_v5.json')",
    "OUT_SCDOC = os.path.join(D, 'fluid_domain_qb.scdoc')\nOUT_JSON = os.path.join(D, 'fluid_domain_qb.json')\n"
    "Q = [0.45, -0.50, -0.40]\n"
    "JC = [(-4e-06, -0.0366, -0.460702), (0.016686, -0.0666, -0.511202), (0.014796, -0.1226, -0.313202)]\n"
    "JA = [(0, 0, -1), (-1, 0, 0), (1, 0, 0)]\n"
    "GROUP = {'hull': 0, 'base0': 0, 'l1': 1, 'l2': 2, 'link3': 3, 'pin4': 3, 'jaw1': 3, 'jaw2': 3}\n"
    "GAP_PLANES_X = (0.0148, 0.0150)\n"
    "SOI_ZS = [-0.344, -0.351, -0.358, -0.365, -0.372, -0.379, -0.386, -0.392]")
rep("def u(value):",
    "def pose_matrix(group, q):\n"
    "    m = Matrix.Identity\n"
    "    for j in range(group, 0, -1):\n"
    "        m = Matrix.CreateRotation(Line.Create(Point.Create(*JC[j - 1]), Direction.Create(*JA[j - 1])), q[j - 1]) * m\n"
    "    return m\n\n\n"
    "def u(value):")
rep("        if m == 'l2':\n            e.Unite(lst(cyl_fill), TOL_SEAL)\n", "")
rep("    for m in DISTAL:\n        ext[m].Transform(move)\n",
    "    for m in DISTAL:\n        ext[m].Transform(move)\n"
    "    for m in ext:\n        ext[m].Transform(pose_matrix(GROUP[m], Q))\n"
    "    olap = {}\n"
    "    nm = sorted(ext)\n"
    "    for i in range(len(nm)):\n"
    "        for j in range(i + 1, len(nm)):\n"
    "            if GROUP[nm[i]] != GROUP[nm[j]]:\n"
    "                x = intersect(ext[nm[i]], ext[nm[j]])\n"
    "                if x is not None:\n"
    "                    olap[nm[i] + '/' + nm[j]] = float(x.Volume) * 1e9\n"
    "    report['posed_overlap_mm3'] = olap\n"
    "    m2 = pose_matrix(2, Q)\n"
    "    soi = [m2 * Point.Create(0.000483, -0.122603, z) for z in SOI_ZS]\n"
    "    report['soi_centres_m'] = [[p.X, p.Y, p.Z] for p in soi]\n"
    "    nrm = pose_matrix(1, Q) * Direction.Create(1, 0, 0)\n"
    "    report['gap_plane_normal'] = [nrm.X, nrm.Y, nrm.Z]\n"
    "    save()\n"
    "    if any(v > 1.0 for v in olap.values()):\n"
    "        raise Exception('Posed parts overlap: %s' % olap)\n")
rep("""    stream = codecs.open(V3_JSON, 'r', 'utf-8')
    report['named_selections_v3'] = json.load(stream)['named_selections']
    stream.close()
    stream = codecs.open(V4_JSON, 'r', 'utf-8')
    report['named_selections_v4'] = json.load(stream)['named_selections']
    stream.close()
""", """    gap, c1 = [], Point.Create(*JC[0])
    for dface in list(water.Faces):
        face = dface.Shape
        if face.Geometry.GetType().Name != 'Plane':
            continue
        offs = [(p.X - c1.X) * nrm.X + (p.Y - c1.Y) * nrm.Y + (p.Z - c1.Z) * nrm.Z for p in samples(face)]
        for x0 in GAP_PLANES_X:
            if all(abs(o - (x0 - JC[0][0])) <= 2e-6 for o in offs):
                gap.append(dface)
                break
    report['gap_faces'] = [{'area_mm2': float(f.Shape.Area) * 1e6} for f in gap]
    if gap:
        groups['gap_faces'] = gap
""")
(ROOT / "13_pose_q1" / "sc_build_fluid_domain_qb.py").write_text(s, encoding="utf-8")
print("ok")
