import matplotlib.pyplot as plt

FILE1= "robotime.txt"
FILE2='robox_velocity.txt'
FILE3='roboy_velocity.txt'
FILE4='roboz_velocity.txt'
FILE5='roboforward_velocity.txt'
FILE6='robolateral_velocity.txt'

# FILE1= "random_time.txt"
# FILE2='random_x_velocity.txt'
# FILE3='random_y_velocity.txt'
# FILE4='random_z_velocity.txt'
# FILE5='random_forward_velocity.txt'
# FILE6='random_lateral_velocity.txt'

FILE7 = "stable_time.txt"
FILE8='stable_x_velocity.txt'
FILE9='stable_y_velocity.txt'
FILE10='stable_z_velocity.txt'
FILE11='stable_forward_velocity.txt'
FILE12='stable_lateral_velocity.txt'

FILEx = "CoMx111.txt"
FIFEt = "CoMt111.txt"
t=[]
x=[]

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
with open(FILEx, "r") as f:
    for line in f:
        x.append(float(line))
with open(FIFEt, "r") as f:
    for line in f:
        t.append(float(line))   

t= [m/1000000 for m in t]
print(t)
offset = 0
i = 0
flag = 1
while i < t.__len__()-1:
    t[i] -= offset
    if t[i+1]>t[i]+0.2 and flag==1:
        offset= t[i+1]-t[i]
        flag = 0

    i+=1
plt.plot(t,x,linewidth=2.0,color='b')
plt.axhline(y=0.3, color='r', linewidth=3.00)
plt.xlim(1, 2.5)
plt.ylim(0.2, 0.5)
plt.ylabel("CoM forward velocity(m/s)",size=20)
plt.yticks(fontsize=20)
plt.xticks(fontsize=20)
plt.xlabel("Time(s)",size=20)
# plt.xlim(3.45, 9)
# plt.ylim(0.15, 0.35)
plt.title("Forward velocity tracking performance",size=20)
plt.legend(['Forward CoM velocity','Reference velocity'],fontsize=20)
plt.show()


offset = 0
i = 0
while i < timelist.__len__()-1:
    while  i < timelist.__len__()-1 and timelist[i]<timelist[i+1]:
        timelist[i] += offset
        i+=1
    timelist[i] += offset
    offset = timelist[i] 
    i+=1

timelist = [x *(4.9)/10 for x in timelist]
for i in range(timelist.__len__()):
    if timelist[i+1]>timelist[i]+10:
        offset= timelist[i]
        gap = timelist[i+1]-timelist[i]
        break
for i in range(timelist.__len__()):
    if timelist[i]>offset:
        timelist[i] = timelist[i]-gap

print(timelist.__len__())
plt.plot(timelist,x_velocitylist,linewidth=3.0,color='b')
#plt.plot(stable_timelist,stable_x_velocitylist,color='green',linewidth=2.0)
plt.plot(timelist,forward_velocitylist,color='red',linewidth=3.0)
plt.ylabel("CoM forward velocity(m/s)",size=20)
plt.yticks(fontsize=20)
plt.xticks(fontsize=20)
plt.xlabel("Time(s)",size=20)
# plt.xlim(3.45, 9)
# plt.ylim(0.15, 0.35)
plt.title("Forward velocity tracking performance",size=20)
plt.legend(['Forward velocity','Reference velocity'],fontsize=20)
plt.show()

timelist = [x/3 for x in timelist]
plt.plot(timelist,y_velocitylist,linewidth=3.0,color='b')
plt.plot(timelist,lateral_velocitylist,color='red',linewidth=2.0)
#plt.plot(stable_timelist,stable_y_velocitylist,color='green',linewidth=2.0)
plt.ylabel("CoM lateral velocity(m/s)",size=20)
plt.xlabel("Time(s)",size=20)
plt.legend(['Lateral CoM velocity','Reference velocity'],fontsize=20)
plt.yticks(fontsize=20)
plt.xticks(fontsize=20)
plt.xlim(3.45/3, 12/3)
# plt.ylim(-0.4, 0.6)


plt.title("Lateral velocity tracking performance",size=20)
plt.show()

plt.plot(timelist,z_velocitylist,linewidth=2.0)
#plt.plot(stable_timelist,stable_z_velocitylist,color='green',linewidth=2.0)
z_ref = [x * 0 for x in z_velocitylist]
plt.plot(timelist,z_ref,color='red',linewidth=2.0)
plt.ylabel("Vertical velocity(m/s)",size=20)
plt.xlabel("Time(s)",size=20)
plt.xlim(timelist[0], 10)
plt.ylim(-0.1, 0.06)
plt.title("Vertical velocity tracking performance",size=20)
plt.yticks(fontsize=20)
plt.xticks(fontsize=20)
plt.show()
