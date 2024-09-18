import matplotlib.pyplot as plt

FILE1 = "CoMx.txt"
FILE2 = "CoMy.txt"
FILE3 = "CoMt.txt"
FILE4 = "CoMx1.txt"
FILE5 = "CoMy1.txt"
FILE6 = "CoMt1.txt"
xlist = []
with open(FILE1, "r") as f:
    for line in f:
        xlist.append(float(line))

ylist = []
with open(FILE2, "r") as f:
    for line in f:
        ylist.append(float(line))

tlist = []
with open(FILE3, "r") as f:
    for line in f:
        tlist.append(float(line))
tlist = [x/1000000 for x in tlist]

xlist1 = []
with open(FILE4, "r") as f:
    for line in f:
        xlist1.append(float(line))

ylist1 = []
with open(FILE5, "r") as f:
    for line in f:
        ylist1.append(float(line))

tlist1 = []
with open(FILE6, "r") as f:
    for line in f:
        tlist1.append(float(line))
tlist1 = [x/1000000 for x in tlist1]

print(tlist.__len__())
print(tlist1.__len__())


plt.plot(tlist,xlist,linewidth=3.0,color='b')
plt.plot(tlist1,xlist1,linewidth=3.0,color='r')
plt.axhline(y=0.4, color='g', linewidth=3.00)
plt.ylabel("CoM foeward velocity(m/s)",fontsize=30)
plt.xlabel("Time(s)",fontsize=30)
plt.legend(['CoM Forward velocity after training','CoM Forward velocity before training','Reference velocity'],fontsize=20)
plt.title("Forward velocity tracking performance with different weights before and after training",fontsize=30)
plt.yticks(fontsize=30)
plt.xticks(fontsize=30)
plt.xlim(0, 3)
plt.ylim(-0.2, 0.75)
plt.show()

plt.plot(tlist,ylist,linewidth=3.0,color='b')
plt.plot(tlist1,ylist1,linewidth=3.0,color='r')
plt.axhline(y=0.25, color='g', linewidth=3.00)
plt.ylabel("CoM lateral velocity(m/s)",fontsize=30)
plt.xlabel("Time(s)",fontsize=30)
plt.legend(['CoM lateral velocity after training','CoM lateral velocity before training','Reference velocity'],fontsize=20)
plt.title("Lateral velocity tracking performance with different weights before and after training",fontsize=30)
plt.yticks(fontsize=30)
plt.xticks(fontsize=30)
plt.xlim(0, 3)
plt.ylim(-0.5, 0.75)
plt.show()

