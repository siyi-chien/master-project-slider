import matplotlib.pyplot as plt

FILE1= "time.txt"
FILE2='x_velocity.txt'
FILE3='y_velocity.txt'
FILE4='z_velocity.txt'
FILE5='forward_velocity.txt'
FILE6='lateral_velocity.txt'

FILE7 = "stable_time.txt"
FILE8='stable_x_velocity.txt'
FILE9='stable_y_velocity.txt'
FILE10='stable_z_velocity.txt'
FILE11='stable_forward_velocity.txt'
FILE12='stable_lateral_velocity.txt'

timelist = []
x_velocitylist = []
y_velocitylist = []
z_velocitylist = []
forward_velocitylist = []
lateral_velocitylist = []
stable_timelist = []
stable_x_velocitylist = []
stable_y_velocitylist = []
stable_z_velocitylist = []
stable_forward_velocitylist = []
stable_lateral_velocitylist = []


with open(FILE1, "r") as f:
    for line in f:
        timelist.append(float(line))
 
with open(FILE2, "r") as f:
    for line in f:
        x_velocitylist.append(float(line))
with open(FILE3, "r") as f:
    for line in f:
        y_velocitylist.append(float(line))
with open(FILE4, "r") as f:
    for line in f:
        z_velocitylist.append(float(line))
with open(FILE5, "r") as f:
    for line in f:
        forward_velocitylist.append(float(line))
with open(FILE6, "r") as f:
    for line in f:
        lateral_velocitylist.append(float(line))
with open(FILE7, "r") as f:
    for line in f:
        stable_timelist.append(float(line))
with open(FILE8, "r") as f:
    for line in f:
        stable_x_velocitylist.append(float(line))
with open(FILE9, "r") as f:
    for line in f:
        stable_y_velocitylist.append(float(line))
with open(FILE10, "r") as f:
    for line in f:
        stable_z_velocitylist.append(float(line))
with open(FILE11, "r") as f:
    for line in f:
        stable_forward_velocitylist.append(float(line))
with open(FILE12, "r") as f:
    for line in f:
        stable_lateral_velocitylist.append(float(line))

timelist = [x *(4.9)/30 for x in timelist]
stable_timelist = [x *(4.9)/30 for x in stable_timelist]

print(timelist.__len__())
plt.plot(timelist,x_velocitylist,linewidth=3.0,color='b')
plt.plot(stable_timelist,stable_x_velocitylist,color='r',linewidth=3.0)
plt.plot(timelist,forward_velocitylist,color='green',linewidth=3.0)
plt.ylabel("Forward velocity(m/s)",size=30)
plt.yticks(fontsize=30)
plt.xticks(fontsize=30)
plt.xlabel("Time(s)",size=30)
plt.xlim(timelist[0], 7.9/3)
plt.ylim(-0.1, 0.9)
plt.legend(['CoM Forward velocity without constraints','CoM Forward velocity with constraints','Reference velocity'],fontsize=20)
plt.title("Forward velocity tracking performance with/without constraints",size=30)
plt.show()

plt.plot(timelist,y_velocitylist,linewidth=3.0,color='b')
plt.plot(timelist,lateral_velocitylist,color='green',linewidth=3.0)
plt.plot(stable_timelist,stable_y_velocitylist,color='r',linewidth=3.0)
plt.ylabel("Lateral velocity(m/s)",size=30)
plt.xlabel("Time(s)",size=30)
plt.yticks(fontsize=30)
plt.xticks(fontsize=30)
plt.xlim(timelist[0], 7.9/3)
plt.legend(['CoM Lateral velocity without constraints','CoM Lateral velocity with constraints','Reference velocity'],fontsize=20)
plt.ylim(-0.5, 2)


plt.title("Lateral velocity tracking performance with/without constraints",size=30)
plt.show()

plt.plot(timelist,z_velocitylist,linewidth=3.0,color='b')
plt.plot(stable_timelist,stable_z_velocitylist,color='r',linewidth=3.0)
z_ref = [x * 0 for x in z_velocitylist]
plt.plot(timelist,z_ref,color='g',linewidth=3.0)
plt.ylabel("Vertical velocity(m/s)",size=30)
plt.xlabel("Time(s)",size=30)
plt.xlim(timelist[0], 7.9/3)
plt.ylim(-0.1, 0.2)
plt.title("Vertical velocity tracking performance with/without constraints",size=30)
plt.legend(['CoM vertical velocity without constraints','CoM vertical velocity with constraints','Reference velocity'],fontsize=20)
plt.yticks(fontsize=30)
plt.xticks(fontsize=30)
plt.show()
