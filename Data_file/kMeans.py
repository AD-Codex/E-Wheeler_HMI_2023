import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

velocity_input = 'velocity_input.txt'
voltage_input = 'voltage_output.txt'
current_input = 'current_output.txt'
avg_vel = 'average_velocity.txt'
avg_pow = 'average_power.txt'
power_contents = []

with open(avg_vel) as f:
    Vvalues = f.readlines()
    print("vel read")

velocity_avg = [ eval(x) for x in Vvalues]


with open(avg_pow) as f:
    Pvalues = f.readlines()
    print("pow read")

power_avg = [ eval(x) for x in Pvalues]


# plt.scatter( velocity_avg, power_avg)
# plt.show()

data = list(zip(velocity_avg, power_avg))
# print(data)

kmeans = KMeans( n_clusters=3, random_state=0, n_init="auto").fit(data)
# print(kmeans)

print( kmeans.cluster_centers_)
print( kmeans.predict([[109,144]]))

plt.scatter(velocity_avg, power_avg, c=kmeans.labels_)
plt.show()



# Vel_means=[]
# count = 0
# vel_sum  = 0
# for i in range(len(velocity_contents)):
#     if (count < 60):
#         vel_sum = vel_sum + float(velocity_contents[i])
#     else:
#         vel_ave = vel_sum/60
#         Vel_means.append(vel_ave)
#         #print(vel_ave)
#         vel_sum = 0
#         count=0

#     count = count +1

# print("\n")

# Power_means=[]
# count = 0
# power_sum  = 0
# for i in range(len(voltage_contents)):
#     if (count < 60):
#         power_sum = power_sum + float(voltage_contents[i])*float(current_contents[i])
#     else:
#         power_ave = power_sum/60
#         Power_means.append(power_ave)
#         # print(power_ave)
#         power_sum = 0
#         count=0

#     count = count +1
