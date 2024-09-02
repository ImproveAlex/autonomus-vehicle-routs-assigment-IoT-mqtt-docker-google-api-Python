import re
import subprocess

host = 'jetbrains.com'
ping_output = subprocess.check_output(['ping', '-c', '5', host]).decode('utf-8')

for line in ping_output.split('\n'):
    print(line)
