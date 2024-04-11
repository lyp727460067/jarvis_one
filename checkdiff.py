import subprocess, sys

def get_content_after_last_space(filename):
    result = []
    with open(filename, 'r', encoding="utf-8") as file:
        for line in file:
            parts = line.rsplit('\t', 1)
            if len(parts) == 2:
                name = parts[1].strip()
                if name.endswith((".cpp", ".c")):
                    result.append(name)
    return result

branch = sys.argv[1]
output = sys.argv[2]

with open("test.diff", "w", encoding="utf-8") as file:
    result = subprocess.run(["git", "diff", branch, "--name-status"], capture_output=True, text=True)
    file.write(f"{result.stdout}")

names = get_content_after_last_space("test.diff")
error_flag = False
with open(output, "w", encoding="utf-8") as file:
    for name in names:
        print(name)
        result = subprocess.run(["cppcheck", "--enable=warning,performance", name], capture_output=True, text=True)
        if result.stderr:
            print(result.stderr)
            file.write(f"{result.stderr}\n")
            error_flag = True
if error_flag:
    exit(1)
