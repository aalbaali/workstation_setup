set history save
set history filename ~/.gdb_history

python
import os
import sys
sys.path.insert(0, os.path.expanduser('~/Dev/workstation_setup/eigen'))
from printers import register_eigen_printers
register_eigen_printers (None)
end
