import matplotlib.pyplot as plt
import numpy as np 
import pandas as pd 

time =[]

for i in range (0,300):
  time.append(i)

  
#SOH Y axis adjustment


SOH = [0.7999999998263889, 0.7999999997106482, 0.799999999537037, 0.7999999994212963, 0.7999999994212963, 0.7999999994212963, 0.7999999993055555, 0.7999999992476852, 0.7999999991319444, 0.799999999074074, 0.7999999989583333, 0.7999999989583333, 0.7999999989004629, 0.7999999987847222, 0.7999999986689814, 0.799999998611111, 0.7999999985532407, 0.7999999984374999, 0.7999999982638888, 0.799999998148148, 0.7999999979745369, 0.7999999979166665, 0.7999999978009258, 0.799999997685185, 0.7999999976273147, 0.7999999975115739, 0.7999999973379628, 0.799999997222222, 0.799999997222222, 0.7999999971643517, 0.7999999970486109, 0.7999999968749998, 0.7999999968171294, 0.799999996759259, 0.7999999929398146, 0.7999999891782406, 0.7999999853587961, 0.7999999816550925, 0.7999999778935184, 0.799999974074074, 0.7999999701388888, 0.7999999663194444, 0.7999999625578703, 0.7999999587962963, 0.7999999550347222, 0.7999999511574074, 0.7999999473379629, 0.7999999434606481, 0.7999999396990739, 0.7999999358796294, 0.7999999320601848, 0.7999999282407403, 0.7999999244212957, 0.7999999207754623, 0.7999999169560178, 0.7999999131944436, 0.7999999094328695, 0.7999999056712953, 0.7999999019675915, 0.799999898148147, 0.7999998943287024, 0.7999998905671283, 0.7999998868055541, 0.7999998831018503, 0.7999998793981465, 0.7999998756365724, 0.7999998719328686, 0.7999998682291648, 0.7999998644675906, 0.7999998606481461, 0.7999998569444423, 0.7999998530671274, 0.7999998493634236, 0.7999998456018494, 0.7999998418402753, 0.7999998380787011, 0.7999998343749973, 0.7999998305555528, 0.7999998267939786, 0.7999998230324045, 0.7999998192708303, 0.7999998155092561, 0.799999811747682, 0.7999998079861077, 0.7999998042824038, 0.7999998005208295, 0.7999997968171256, 0.7999997931134217, 0.7999997894675882, 0.7999997857060139, 0.79999978200231, 0.7999997783564765, 0.7999997745370319, 0.799999770833328, 0.7999997671296241, 0.7999997634259202, 0.7999997597222163, 0.7999997560185124, 0.7999997523148085, 0.7999997485532342, 0.7999997448495303, 0.7999997412036968, 0.7999997374421225, 0.7999997337384186, 0.7999997300347147, 0.7999997262731404, 0.7999997225694365, 0.7999997188657326, 0.7999997151620287, 0.7999997115161952, 0.7999997078124913, 0.7999997041087874, 0.7999997003472131, 0.7999996966435092, 0.799999692881935, 0.799999689178231, 0.7999996854166568, 0.7999996815972121, 0.7999996778935082, 0.7999996741898043, 0.7999996705439707, 0.7999996667823963, 0.7999996630786923, 0.7999996593171179, 0.7999996556134139, 0.7999996518518395, 0.7999996481481355, 0.7999996444444315, 0.7999996407407275, 0.7999996369791531, 0.7999996331597083, 0.7999996294560043, 0.7999996258101707, 0.7999996221064667, 0.799999618460633, 0.7999996148147994, 0.799999611053225, 0.799999607349521, 0.799999603645817, 0.799999599942113, 0.7999995961805386, 0.7999995924189642, 0.7999995887731306, 0.7999995850694266, 0.799999581423593, 0.7999995777198889, 0.7999995739583146, 0.7999995702546105, 0.7999995665509065, 0.7999995629050729, 0.7999995591434985, 0.7999995554976649, 0.7999995517939609, 0.7999995480902569, 0.7999995444444232, 0.7999995407407192, 0.7999995370370152, 0.7999995333333112, 0.7999995296296072, 0.7999995259259031, 0.7999995222221989, 0.7999995185184948, 0.7999995147569203, 0.7999995109953458, 0.7999995072916417, 0.799999503645808, 0.7999994999421038, 0.7999994961805293, 0.7999994925346956, 0.7999994888309915, 0.7999994851851577, 0.7999994814814536, 0.7999994777777495, 0.7999994741319157, 0.7999994704282116, 0.7999994666087668, 0.7999994628471923, 0.7999994592013585, 0.799999455439784, 0.7999994516203391, 0.7999994478587646, 0.7999994442129309, 0.7999994405092268, 0.7999994368055227, 0.7999994331018185, 0.7999994293981144, 0.7999994256944103, 0.7999994219328358, 0.7999994181712613, 0.7999994145254276, 0.7999994108795938, 0.7999994071758897, 0.7999994034721856, 0.7999993997684814, 0.7999993960647773, 0.7999993924189436, 0.7999993887152395, 0.7999993849536648, 0.7999993813078311, 0.7999993776041269, 0.7999993739004226, 0.799999370138848, 0.7999993664351438, 0.7999993627314396, 0.7999993590277353, 0.7999993553240311, 0.7999993516781972, 0.7999993480323634, 0.7999993443286592, 0.7999993406249549, 0.7999993368055099, 0.7999993330439353, 0.7999993292823607, 0.7999993255786565, 0.7999993218749523, 0.7999993181133777, 0.799999314351803, 0.7999993107059692, 0.799999307002265, 0.7999993032406904, 0.7999992995369861, 0.7999992958332819, 0.7999992920717073, 0.799999288368003, 0.7999992846642988, 0.7999992809605946, 0.7999992773147607, 0.7999992735531861, 0.7999992697916115, 0.7999992660879073, 0.799999262384203, 0.7999992587383692, 0.799999255034665, 0.7999992513309607, 0.7999992476851269, 0.7999992439814226, 0.7999992403355888, 0.7999992366318844, 0.7999992328703097, 0.7999992292244757, 0.7999992255207714, 0.7999992217591967, 0.7999992181133627, 0.7999992144675288, 0.7999992108216948, 0.7999992071758608, 0.7999992034721565, 0.7999991996527114, 0.7999991958911367, 0.7999991921295619, 0.7999991884258576, 0.7999991847221533, 0.7999991810763193, 0.799999177372615, 0.7999991736689106, 0.7999991700230766, 0.7999991663193723, 0.799999162615668, 0.7999991589119636, 0.7999991552082593, 0.7999991515045549, 0.799999147858721, 0.7999991442707574, 0.7999991406249234, 0.7999991369790894, 0.7999991332753851, 0.7999991296295511, 0.7999991259258468, 0.7999991222221424, 0.7999991184605677, 0.7999991148147337, 0.799999111053159, 0.7999991073494547, 0.7999991037036207, 0.7999991000577867, 0.7999990963540824, 0.7999990927082484, 0.7999990890624145, 0.7999990854165804, 0.7999990817707463, 0.7999990780091715, 0.799999074305467, 0.7999990706596329, 0.7999990668401877, 0.7999990631364833, 0.7999990593749084, 0.799999055671204, 0.7999990519674995, 0.7999990483216655, 0.799999044617961, 0.7999990409142566, 0.7999990372105521, 0.799999033564718, 0.7999990298610136, 0.7999990262151795, 0.7999990224536047, 0.7999990187499002, 0.7999990151619365, 0.799999011458232, 0.7999990076966572]
print(type(SOH))

SOHmod = [(i - 0.7999999990000000)*1000000000 for i in SOH]
print(SOHmod)

plt.figure(figsize = (30,5))
plt.plot(time,SOH,'mo')
plt.title('SOH',fontsize = 40)
plt.xticks(fontsize = 40)
plt.yticks(fontsize = 40)
plt.xlabel('Time Step',fontsize = 40)
plt.ylabel('SOH' , fontsize = 40)
plt.ylim([0,1])
