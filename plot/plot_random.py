import matplotlib.pyplot as plt

FILE1= "ran2time.txt"
FILE2='ran2x_velocity.txt'
FILE3='ran2y_velocity.txt'
FILE4='ran2z_velocity.txt'
FILE5='ran2forward_velocity.txt'
FILE6='ran2lateral_velocity.txt'

FILE1= "random_time.txt"
FILE2='random_x_velocity.txt'
FILE3='random_y_velocity.txt'
FILE4='random_z_velocity.txt'
FILE5='random_forward_velocity.txt'
FILE6='random_lateral_velocity.txt'

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

offset = 0
i = 0
while i < timelist.__len__()-1:
    while  i < timelist.__len__()-1 and timelist[i]<timelist[i+1]:
        timelist[i] += offset
        i+=1
    timelist[i] += offset
    offset = timelist[i] 
    i+=1
        
timelist = [x *(4.9)/30 for x in timelist]
print(timelist.__len__())
plt.plot(timelist,x_velocitylist,linewidth=3.0,color='b')
#plt.plot(stable_timelist,stable_x_velocitylist,color='green',linewidth=3.0)
plt.plot(timelist,forward_velocitylist,color='red',linewidth=3.0)
plt.ylabel("CoM forward velocity(m/s)",size=30)
plt.yticks(fontsize=30)
plt.xticks(fontsize=30)
plt.xlabel("Time(s)",size=30)
plt.xlim(timelist[0], 44/3)
plt.legend(['CoM forward velocity','Random forward reference velocity'],fontsize=20)
plt.title("Forward velocity tracking performance",size=30)
plt.show()

plt.plot(timelist,y_velocitylist,linewidth=3.0,color='b')
plt.plot(timelist,lateral_velocitylist,color='red',linewidth=3.0)
plt.ylabel("CoM fateral velocity(m/s)",size=30)
plt.xlabel("Time(s)",size=30)
plt.yticks(fontsize=30)
plt.xticks(fontsize=30)
plt.xlim(timelist[0], 44/3)
plt.legend(['CoM lateral velocity','Random lateral reference velocity'],fontsize=20)
plt.ylim(-0.5, 0.8)
plt.show()

plt.plot(timelist,z_velocitylist,linewidth=3.0,color='b')
plt.plot(timelist,lateral_velocitylist,color='red',linewidth=3.0)
plt.ylabel("CoM fateral velocity(m/s)",size=30)
plt.xlabel("Time(s)",size=30)
plt.yticks(fontsize=30)
plt.xticks(fontsize=30)
plt.xlim(timelist[0], 44/3)
plt.legend(['CoM lateral velocity','Random lateral reference velocity'],fontsize=20)
plt.ylim(-0.5, 0.8)
plt.show()
