# import os

# with open(os.path.join("params.txt"), "r") as f:
#     for line in f:
#         command = "..\\fu_sai\\WindowsRelease\\\src\\\HuaweiCodeCraft\\x64\\Release\\HuaweiCodeCraft.exe " + line[0:-1]
#         os.system("echo " + command + " >> res.txt")
#         os.system(".\\Robot.exe \"" + command + "\" -m pre_official_maps\\3.txt -f >> res.txt")	
# f.close()


import matplotlib.pyplot as plt
with open('data.txt', 'r') as f:
    x = []
    y = []
    for line in f.readlines():
        if len(line) > 0:
            data = line.split(",")
            x.append(int(data[0]))
            y.append(int(data[1]))
    
plt.axis([0, 100, 100, 0])
plt.scatter(x, y)
plt.show()


