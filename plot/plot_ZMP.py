import matplotlib.pyplot as plt

FILE1 = "ZMPx.txt"
FILE2 = "ZMPy.txt"
FILE3 = "realZMPx.txt"
FILE4 = "realZMPy.txt"
xlist = []
with open(FILE1, "r") as f:
    for line in f:
        xlist.append(float(line))
print(xlist)
ylist = []
with open(FILE2, "r") as f:
    for line in f:
        ylist.append(float(line))
print(ylist)
realxlist = []
with open(FILE3, "r") as f:
    for line in f:
        realxlist.append(float(line))

realylist = []
with open(FILE4, "r") as f:
    for line in f:
        realylist.append(float(line))


plt.plot(xlist,ylist,linewidth=4.0,linestyle='--',color='r')
plt.plot(realxlist,realylist,linewidth=3.0,color='b')
print(xlist.__len__())
plt.ylabel("y",fontsize=30)
plt.xlabel("x",fontsize=30)
plt.legend(['Optimized ZMP trajectory','CoM trajectory'],fontsize=20)
plt.title("Optimized ZMP trajectory and CoM trajectory",fontsize=30)
plt.yticks(fontsize=30)
plt.xticks(fontsize=30)
plt.xlim(-0.2, 4)
plt.ylim(-0.2, 1)
plt.show()

FILE1 = "ZMPx1.txt"
FILE2 = "ZMPy1.txt"
FILE3 = "realZMPx1.txt"
FILE4 = "realZMPy1.txt"
xlist = []
with open(FILE1, "r") as f:
    for line in f:
        xlist.append(float(line))
print(xlist)
ylist = []
with open(FILE2, "r") as f:
    for line in f:
        ylist.append(float(line))
print(ylist)
realxlist = []
with open(FILE3, "r") as f:
    for line in f:
        realxlist.append(float(line))

realylist = []
with open(FILE4, "r") as f:
    for line in f:
        realylist.append(float(line))


plt.plot(xlist,ylist,linewidth=4.0,linestyle='--',color='r')
plt.plot(realxlist,realylist,linewidth=3.0,color='b')
print(xlist.__len__())
plt.ylabel("y",fontsize=30)
plt.xlabel("x",fontsize=30)
plt.legend(['Optimized ZMP trajectory','CoM trajectory'],fontsize=20)
plt.title("Optimized ZMP trajectory and CoM trajectory",fontsize=30)
plt.yticks(fontsize=30)
plt.xticks(fontsize=30)
plt.xlim(-0.2, 4)
plt.ylim(-0.2, 0.6)
plt.show()