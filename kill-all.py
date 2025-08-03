import sys
import subprocess

if __name__ == '__main__':
    result = []

    for _line in sys.stdin:
        line = _line.strip()
        if line:
            k = line.find(' ')
            while k < len(line) and line[k] == ' ':
                k += 1

            end_index = line.find(' ', k)
            result.append(line[k:end_index])

    command = ["kill", "-9"]
    command.extend(result[:-1])
    subprocess.run(command)

# then run the following in the terminal:
# ps aux | grep -i ros | kill-all
# ps aux | grep -i gazebo | kill-all