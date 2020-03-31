import os

cwd = os.getcwd()
print(cwd)

file = open(cwd+"\DummyPacket","wb") 
 
file.write("Hello World")

file.close() 
