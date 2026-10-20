import os
from pathlib import Path

awp = r'D:\Program Files\ANSYS Inc\v242'
os.environ.update({'AWP_ROOT242': awp, 'ANSYSWB_SYSDIR': 'winx64', 'PROCESSOR_ARCHITECTURE': 'AMD64'})
directories = [
    'aisol/bin/winx64', 'Framework/bin/Win64', 'tp/hdf5/1.12.2/winx64',
    'tp/IntelCompiler/2023.1.0/winx64', 'tp/IntelMKL/2023.1.0/winx64',
    'tp/qt/5.15.16/winx64/bin', 'scdm/Addins/ANSYS 24.2', 'scdm'
]
os.environ['PATH'] = ';'.join([f'{awp}/{p}' for p in directories] + [os.environ.get('PATH', '')])

# Let's inspect the step 10 checkpoint data from earlier:
cas_path = r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\solve_phaseA_production_run02\phaseA_step_0010.cas.h5"
dat_path = r"D:\work4\cfd\wetted_arm_rebuild\articulated_mesh_v2\cad_attempt04\solve_phaseA_production_run02\phaseA_step_0010.dat.h5"

import ansys.fluent.core as pyfluent

session = pyfluent.launch_fluent(precision="double", processor_count=1, mode="solver")
session.tui.file.read_case(cas_path)
session.tui.file.read_data(dat_path)

# Let's find which cell has max turb viscosity ratio
# We can mark cells with turb viscosity ratio > 1000
res = session.scheme_eval.scheme_eval('''
(let ((worst-cell #f)
      (max-mut 0.0)
      (worst-thread #f))
  (for-each
    (lambda (th)
      (let ((mut-val (thread-max-value th "cell-turb-viscosity-ratio")))
        (display (format "Thread ~a (ID ~a): max mut = ~a\\n" (symbol->string (thread-name th)) (thread-id th) mut-val))
        (if (> mut-val max-mut)
            (begin
              (set! max-mut mut-val)
              (set! worst-thread th)))))
    (get-cell-threads))
  (display (format "Overall worst thread: ~a, max mut = ~a\\n" (thread-id worst-thread) max-mut))
)
''')
print(res)

session.exit()
