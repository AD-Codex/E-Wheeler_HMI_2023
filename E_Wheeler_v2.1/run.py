
#!/usr/bin/env python3

import webbrowser
import time
import subprocess


url = 'http://127.0.0.1:1880/ui/'

webbrowser.open(url)


time.sleep(10)

#subprocess.run(["python", "SOC.py"])
subprocess.call(["python", "SOC.py"])
