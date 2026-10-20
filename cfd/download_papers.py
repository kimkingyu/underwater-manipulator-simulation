import urllib.request
import os
from pathlib import Path

papers_dir = Path("D:/work4/docs/papers")
papers_dir.mkdir(parents=True, exist_ok=True)

urls = {
    "06_McMillan1995_Dynamic_Simulation_Underwater_Robotic_Vehicles.pdf": 
        "https://calhoun.nps.edu/bitstreams/e0bfe044-0da8-4676-8829-081501c55a33/download",
    "07_JMSE2021_Maneuverability_Hydrodynamics_Tethered_Underwater_Robot_Overset_Grid.pdf": 
        "https://www.mdpi.com/2077-1312/9/6/561/pdf",
    "08_JMSE2023_Hydrodynamics_Analysis_Underwater_Foldable_Arm_CFD.pdf": 
        "https://www.mdpi.com/2077-1312/11/7/1395/pdf",
    "09_JMSE2021_Dynamics_Simulation_Grasping_Underwater_Vehicle_Manipulator.pdf": 
        "https://www.mdpi.com/2077-1312/9/10/1131/pdf",
    "10_Machines2024_Unified_Approach_Modeling_Simulation_UVMS.pdf": 
        "https://www.mdpi.com/2075-1702/12/2/94/pdf"
}

headers = {
    "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36"
}

for filename, url in urls.items():
    dest = papers_dir / filename
    print(f"Downloading {filename} from {url}...")
    try:
        req = urllib.request.Request(url, headers=headers)
        with urllib.request.urlopen(req, timeout=30) as resp, open(dest, "wb") as f:
            content = resp.read()
            f.write(content)
        size_kb = len(content) / 1024
        if content.startswith(b"%PDF"):
            print(f"  [OK] Valid PDF: {filename} ({size_kb:.1f} KB)")
        else:
            print(f"  [WARN] Not a PDF header (first 50 bytes: {content[:50]})")
    except Exception as e:
        print(f"  [FAILED] {filename}: {e}")
