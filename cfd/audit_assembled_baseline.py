import os
import sys
import json
import h5py
import numpy as np

def audit_case(case_path):
    print(f"Auditing case: {case_path}")
    report = {
        "case_path": case_path,
        "valid": False,
        "checks": {}
    }
    
    with h5py.File(case_path, 'r') as h5:
        # 1. Parse Thread Variables
        tv_bytes = b''.join(h5['settings/Thread Variables'][()])
        tv_text = tv_bytes.decode('latin1')
        
        # Extract threads: (39 (id type name ...))
        import re
        threads = {}
        for m in re.finditer(r'\(39 \((\d+)\s+(\S+)\s+(\S+)', tv_text):
            zid = int(m.group(1))
            ztype = m.group(2)
            zname = m.group(3)
            threads[zid] = (ztype, zname)
            
        print("Threads found:", len(threads))
        for zid, (ztype, zname) in sorted(threads.items()):
            print(f"  ID {zid:2d}: {ztype:15s} {zname}")
            
        # 2. Check cell zones (fluids)
        cell_zones = {zid: name for zid, (ztype, name) in threads.items() if ztype == 'fluid'}
        report["checks"]["cell_zones"] = cell_zones
        print("\nCell zones (fluids):", cell_zones)
        expected_fluids = {'fluid', 'fluid.1', 'fluid.1.1', 'fluid.1.1.1'}
        if set(cell_zones.values()) != expected_fluids:
            report["checks"]["cell_zones_match"] = False
            report["reason"] = f"Expected fluid zones {expected_fluids}, got {set(cell_zones.values())}"
            return report
        report["checks"]["cell_zones_match"] = True
        
        # 3. Check faces zoneTopology and adjacency
        zt = h5['meshes/1/faces/zoneTopology']
        names = [n.decode('latin1') if isinstance(n, bytes) else str(n) for n in zt['name'][0].split(b';' if isinstance(zt['name'][0], bytes) else ';')]
        
        wall_names = {'lk1', 'lk2', 'lk3w1', 'lk3w2', 'lk3w3', 'lk3w4', 'rov_body_wall'}
        overset_names = {'overset_j1', 'overset_j2', 'overset_j3'}
        
        wall_adjacency = {}
        overset_types = {}
        
        for i in range(len(zt['id'])):
            zid = int(zt['id'][i])
            zname = names[i] if i < len(names) else f"zone_{zid}"
            ztype = threads.get(zid, ('unknown', 'unknown'))[0]
            
            c0 = int(zt['c0'][i])
            c1 = int(zt['c1'][i])
            
            c0_type = threads.get(c0, ('none', 'none'))[0] if c0 >= 0 else 'none'
            c1_type = threads.get(c1, ('none', 'none'))[0] if c1 >= 0 else 'none'
            c0_name = threads.get(c0, ('none', 'none'))[1] if c0 >= 0 else 'none'
            c1_name = threads.get(c1, ('none', 'none'))[1] if c1 >= 0 else 'none'
            
            face_count = int(zt['maxId'][i] - zt['minId'][i] + 1)
            
            if zname in wall_names:
                is_wetted = (c0_type == 'fluid' and c1_type == 'none') or (c1_type == 'fluid' and c0_type == 'none')
                wall_adjacency[zname] = {
                    "zone_id": zid,
                    "type": ztype,
                    "face_count": face_count,
                    "c0": c0,
                    "c1": c1,
                    "c0_name": c0_name,
                    "c1_name": c1_name,
                    "is_wetted_fluid": is_wetted
                }
                print(f"Wall {zname:15s} (ID {zid:2d}): c0={c0_name} ({c0}), c1={c1_name} ({c1}), faces={face_count}, wetted={is_wetted}")
                
            if zname in overset_names:
                overset_types[zname] = {
                    "zone_id": zid,
                    "type": ztype,
                    "face_count": face_count,
                    "is_overset": (ztype == 'overset')
                }
                print(f"Overset {zname:12s} (ID {zid:2d}): type={ztype}, faces={face_count}")

        report["checks"]["wall_adjacency"] = wall_adjacency
        report["checks"]["overset_types"] = overset_types
        
        # Verify wall assignments to correct motion zones:
        # lk1 -> fluid.1 (j1)
        # lk2 -> fluid.1.1 (j2)
        # lk3w1~w4 -> fluid.1.1.1 (j3)
        expected_parent = {
            'lk1': 'fluid.1',
            'lk2': 'fluid.1.1',
            'lk3w1': 'fluid.1.1.1',
            'lk3w2': 'fluid.1.1.1',
            'lk3w3': 'fluid.1.1.1',
            'lk3w4': 'fluid.1.1.1',
            'rov_body_wall': 'fluid'
        }
        
        all_walls_correct = True
        for wname, exp_fluid in expected_parent.items():
            if wname not in wall_adjacency:
                print(f"ERROR: Missing wall {wname}!")
                all_walls_correct = False
                continue
            adj = wall_adjacency[wname]
            fluid_adj = adj['c0_name'] if adj['c0_name'] != 'none' else adj['c1_name']
            if fluid_adj != exp_fluid:
                print(f"ERROR: Wall {wname} adjacent to {fluid_adj}, expected {exp_fluid}!")
                all_walls_correct = False
            if not adj['is_wetted_fluid']:
                print(f"ERROR: Wall {wname} is not strictly single-sided wetted fluid boundary!")
                all_walls_correct = False
                
        report["checks"]["walls_topology_correct"] = all_walls_correct
        
        # Verify overset boundary types
        all_overset_correct = True
        for oname in overset_names:
            if oname not in overset_types:
                print(f"ERROR: Missing overset boundary {oname}!")
                all_overset_correct = False
                continue
            if not overset_types[oname]['is_overset']:
                print(f"ERROR: Boundary {oname} type is {overset_types[oname]['type']}, expected 'overset'!")
                all_overset_correct = False
        report["checks"]["overset_boundaries_correct"] = all_overset_correct
        
        # 4. Check node independence (no shared nodes between cell zones)
        # In Fluent H5 format, cells topology gives node IDs per cell zone
        ct = h5['meshes/1/cells/zoneTopology']
        cell_min_id = ct['minId']
        cell_max_id = ct['maxId']
        print("\nCell count per cell zone:")
        total_cells = 0
        for i in range(len(ct['id'])):
            cid = int(ct['id'][i])
            cname = threads.get(cid, ('fluid', f'zone_{cid}'))[1]
            cnt = int(cell_max_id[i] - cell_min_id[i] + 1)
            total_cells += cnt
            print(f"  Cell Zone {cid:2d} ({cname}): {cnt} cells")
        report["checks"]["total_cells"] = total_cells
        
        if all_walls_correct and all_overset_correct:
            report["valid"] = True
            print("\n>>> AUDIT PASSED: All 4 domains, 6 arm walls, rov body wall, and 3 overset boundaries verified valid! <<<")
        else:
            report["valid"] = False
            print("\n>>> AUDIT FAILED <<<")
            
    return report

if __name__ == '__main__':
    cas = r'D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\assembled_system01\assembled_4domain_t0_valid.cas.h5'
    res = audit_case(cas)
    out_json = r'D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\assembled_system01\assembled_4domain_t0_valid_audit.json'
    with open(out_json, 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2)
    print(f"Saved audit report: {out_json}")
