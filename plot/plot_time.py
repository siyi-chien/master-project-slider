import matplotlib.pyplot as plt

FILE = "time_consuption.txt"
FILE1 = "time-final.txt"
FILE2 = "time-hotstart.txt"
numlist = []
with open(FILE1, "r") as f:
    for line in f:
        numlist.append(float(line))
print(numlist)
numlist =[x*1000 for x in numlist]

numlist1 = []
with open(FILE2, "r") as f:
    for line in f:
        numlist1.append(float(line))
print(numlist1)
numlist1 =[x*1000 for x in numlist1]
#caculate the average of a list
def average(numlist):
    return sum(numlist)/len(numlist)
print(average(numlist))
print(average(numlist1))
figure1 = plt.plot(numlist[:100],linewidth=3.0,color='blue')
figure2 = plt.plot(numlist1[:100],linewidth=3.0,color='red')
plt.ylabel("Time consumption(ms)",fontsize=30)
plt.xlabel("Number",fontsize=30)
plt.title("Time consumption of MPC",fontsize=30)
plt.legend(['Current computation cost ','Original computation cost'],fontsize=20)
plt.yticks(fontsize=30)
plt.xticks(fontsize=30)

plt.show()

