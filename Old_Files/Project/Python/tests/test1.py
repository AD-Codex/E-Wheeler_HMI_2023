def find(str,ch):
    for i, ltr in enumerate(str):
        if ltr == ch:
            yield i
            
str = ['$s' ,'12', '13','14']
ch = '12'

find(str,ch)
