import re
a = 'v0.10'
b =''.join(re.split(r'[^A-Za-z]', a))
print(b)
a = a.replace(b,'')
print(a)