import os
import sys

awp = r'D:\Program Files\ANSYS Inc\v242'
os.environ.update({'AWP_ROOT242': awp, 'ANSYSWB_SYSDIR': 'winx64', 'PROCESSOR_ARCHITECTURE': 'AMD64'})
import ansys.fluent.core as pyfluent
from ansys.fluent.core.launcher.pyfluent_enums import UIMode

session = pyfluent.launch_fluent(
    product_version='24.2',
    mode='solver',
    precision='double',
    processor_count=1,
    ui_mode=UIMode.NO_GUI_OR_GRAPHICS,
    cleanup_on_exit=True,
)
try:
    opt = session.tui.define.overset_interfaces.options
    print("Options subcommands:")
    for name in dir(opt):
        if not name.startswith('_'):
            print(" ", name)
finally:
    session.exit()
