import time

energy_list = []
Rcount = 0   	# real value count +1
Setcount = 1

sample = 5		# number of sample
sTime = 1		# s time betwen 2 samples

def M_mean():
	M_sum = 0
	for i in range(sample-1):
		M = (energy_list[Rcount-sample+i+1] - energy_list[Rcount-sample+i]) / sTime
		M_sum = M_sum + M
	M_mean = M_sum / (sample-1)

	return M_mean

def predict():
	m_mean = M_mean()
	for i in range(sample):
		x_predict = m_mean*sTime + energy_list[Rcount-1+i]
		#energy_list.append(x_predict)
		if ( len(energy_list) < Rcount +i+1):
			print("1")
			energy_list.append(x_predict)
		else:
			print("2")
			energy_list[Rcount+i] = x_predict
		print(energy_list)

def PPower():
	power = (energy_list[Rcount+4] - energy_list[Rcount]) / ( sTime*(sample-1))
	print("energy_list[Rcount-1]", energy_list[Rcount])
	print("energy_list[Rcount+4]", energy_list[Rcount+4])
	return power


while ( True):

	if ( len(energy_list) < sample):
		time.sleep(1)
		value = float(input("input value : "))
		if ( Rcount ==0):
			energy_list.append(value)
		else:
			energy = value + energy_list[Rcount-1]	
			energy_list.append(energy)
		Rcount = Rcount + 1
		

	else:
		predict()
		print("Rcount", Rcount)

		print("Ppower", PPower())

		time.sleep(1)
		value = float(input("input value : "))
		energy = value + energy_list[Rcount-1]
		if (value == 0):
			break

		energy_list[Rcount] = energy
		Rcount = Rcount + 1
		

		


print(energy_list)



