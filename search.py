import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER

# for i in range(5):
#     os.system("python3 generate.py")
#     os.system("python3 simulate.py")
os.system("rm fitness*.txt")
os.system("rm body*.urdf")
os.system("rm brain*.nndf")

phc = PARALLEL_HILL_CLIMBER()
phc.Evolve()
phc.Show_Best()
