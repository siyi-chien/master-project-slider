import matplotlib.pyplot as plt

FILE = "score.txt"
numlist = []
with open(FILE, "r") as f:
    for line in f:
        numlist.append(float(line))
print(numlist)
numlist =[2*x for x in numlist]
figure1 = plt.plot(numlist[:350],linewidth=3.0,color='b')
plt.ylabel("Score",fontsize=30)
plt.xlabel("Epoch",fontsize=30)
plt.title("Iteration of scores during training",fontsize=30)
plt.yticks(fontsize=30)
plt.legend(['Score'],fontsize=20)
plt.xticks(fontsize=30)
plt.show()