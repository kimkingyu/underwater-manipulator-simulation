"""wb_mesh_v5h_e3.wbjn: real-hull v5h geometry, E3 arm settings + soft 40 mm hull face sizing
(trial H1_E3_hull_soft40: 439,764 tets). Saves a copy auvm_water_v5h_e3.wbpj, generates, applies the v5g
export fix (gap sizing scoped by ids, helper NS deleted) if a gap_faces NS exists, creates dp0/FFF/MECH,
updates the mesh component (writes FFF.msh) and saves."""
from pathlib import Path

M = Path("D:/work4/new_model_sim/12_coarse_mesh")
s = (M / "wb_trials_v5h_e3.wbjn").read_text(encoding="utf-8")
for a, b in (("wb_trials_v5h_e3_trace", "wb_mesh_v5h_e3_trace"), ("v5h_e3_trials.wbpj", "auvm_water_v5h_e3.wbpj"),
             ("trials_v5h_e3.txt", "mesh_v5h_e3_messages.txt")):
    assert a in s, a
    s = s.replace(a, b)

a = s.index('    gen("H0_E3_hull_free", r)')
b = s.index('except Exception, e:\n    w("INSIDE ERROR')
new = '''    hull = mesh.AddSizing()
    hull.Location = ns["wall_auv"]
    hull.Name = "size_wall_auv"
    r.append(put(hull, "GrowthRate", 1.4))
    r.append(put(hull, "ElementSize", Quantity("40 [mm]")))
    try:
        hull.Behavior = SizingBehavior.Soft
    except Exception, e:
        r.append("FAILED behavior %%s" %% e)
    if "gap_faces" in ns:
        gsz = [c for c in mesh.Children if c.GetType().Name == "Sizing" and c.Name == "size_gap_faces"][0]
        gsel = ExtAPI.SelectionManager.CreateSelectionInfo(SelectionTypeEnum.GeometryEntities)
        gsel.Ids = list(ns["gap_faces"].Location.Ids)
        gsz.Location = gsel
        ns["gap_faces"].Delete()
        w("gap_faces NS replaced by ids %%s" %% list(gsel.Ids))
    else:
        w("no gap_faces NS (already scoped by ids)")
    w("named selections %%s" %% [x.Name for x in m.NamedSelections.Children])
    gen("V5H_E3_hull_soft40", r)
'''
s = s[:a] + new + s[b:]

old_run = '''    step("3 trials", lambda: (mesh.Edit(Interactive=False),
                              mesh.SendCommand(Language="Python", Command=INSIDE),
                              mesh.Exit()))'''
assert old_run in s
s = s.replace(old_run, '''    comp = s.GetComponent(Name="Mesh")
    ok = step("3 settings + generate", lambda: (mesh.Edit(Interactive=False),
                                               mesh.SendCommand(Language="Python", Command=INSIDE),
                                               mesh.Exit()))
    step("4 mkdir MECH", lambda: Directory.CreateDirectory(D + r"\\auvm_water_v5h_e3_files\\dp0\\FFF\\MECH"))
    ok = ok and step("5 update mesh component (writes FFF.msh)", lambda: comp.Update(AllDependencies=True))
    step("6 save", lambda: Save(Overwrite=True))''')
s = s.replace("from System.IO import File\n", "from System.IO import File, Directory\n", 1)
(M / "wb_mesh_v5h_e3.wbjn").write_text(s, encoding="utf-8")
print("ok")
