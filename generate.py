import pyrosim.pyrosim as pyrosim
pyrosim.Start_SDF("boxes.sdf")
l = 1
w = 1
h = 1
x = 0
y = 0
z = 0.5
# Create 10 blocks with a for loop
# for ii in range(10):
#     newl = length*(0.9**ii)
#     neww = width*(0.9**ii)
#     newh = height*(0.9**ii)

#     pyrosim.Send_Cube(name="Box", pos=[x, y, z], size=[
#                       newl,neww,newh])
#     z += newh

# Create 25 towers
for ii in range(5):
    for jj in range(5):
        for kk in range(10):
            pyrosim.Send_Cube(
                name="Box", pos=[x+ii, y+jj, z], size=[l*(0.9**kk), w*(0.9**kk), h*(0.9**kk)])
            z += 1

pyrosim.End()
