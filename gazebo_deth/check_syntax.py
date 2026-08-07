import subprocess
import sys

try:
    result = subprocess.run(['node', '-c', 'web_ui/app.js'], capture_output=True, text=True)
    with open('syntax_result.txt', 'w') as f:
        f.write("RETURN CODE: " + str(result.returncode) + "\n")
        f.write("STDOUT:\n" + result.stdout + "\n")
        f.write("STDERR:\n" + result.stderr + "\n")
except Exception as e:
    with open('syntax_result.txt', 'w') as f:
        f.write("EXCEPTION: " + str(e))
