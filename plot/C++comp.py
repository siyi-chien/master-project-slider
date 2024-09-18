import matplotlib.pyplot as plt


FILE1= "CoMtlala.txt"
FILE2='CoMxlala.txt'
FILE3='CoMylala.txt'
FILE5='ZMPxlala.txt'
FILE6='ZMPylala.txt'


timelist = []
x_velocitylist = []
y_velocitylist = []
z_velocitylist = []
forward_velocitylist = []
lateral_velocitylist = []



with open(FILE1, "r") as f:
    for line in f:
        timelist.append(float(line))
 
with open(FILE2, "r") as f:
    for line in f:
        x_velocitylist.append(float(line))
with open(FILE3, "r") as f:
    for line in f:
        y_velocitylist.append(float(line))
with open(FILE5, "r") as f:
    for line in f:
        forward_velocitylist.append(float(line))
with open(FILE6, "r") as f:
    for line in f:
        lateral_velocitylist.append(float(line))
timelist = [x/1000000 for x in timelist]


print(timelist.__len__())
plt.plot(timelist,x_velocitylist,linewidth=3.0,color='b')

plt.plot(timelist,forward_velocitylist,color='red',linewidth=3.0)
plt.ylabel("CoM forward velocity(m/s)",size=30)
plt.yticks(fontsize=30)
plt.xticks(fontsize=30)
plt.xlabel("Time(s)",size=30)
plt.xlim(timelist[0], 14,5)
plt.ylim(-0.6, 0.4)
plt.legend(['CoM forward velocity','Random forward reference velocity'],fontsize=20)
plt.title("Forward velocity tracking performance",size=30)
plt.show()

plt.plot(timelist,y_velocitylist,linewidth=3.0,color='b')
plt.plot(timelist,lateral_velocitylist,color='red',linewidth=3.0)

plt.ylabel("CoM fateral velocity(m/s)",size=30)
plt.xlabel("Time(s)",size=30)
plt.ylim(-0.5, 0.8)
plt.yticks(fontsize=30)
plt.xticks(fontsize=30)
plt.xlim(timelist[0], 14.5)
plt.legend(['CoM lateral velocity','Random lateral reference velocity'],fontsize=20)
plt.ylim(-0.5, 0.8)
