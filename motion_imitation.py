import pybullet as p
import time
import pybullet_data

# Start pybullet simulation
p.connect(p.GUI)
# p.connect(p.DIRECT) # don't render

# load urdf file path
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# load urdf and set gravity
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
robot = p.loadURDF("W:/Mathematical Computations/Projects/Virtueorigin/package_robot/urdf/Main_URDF.urdf", cubeStartPos, cubeStartOrientation) // path to Main_URDF.urdf file
jointIds = []
joint_para = []
joint_state = []

position, orientation = p.getBasePositionAndOrientation(robot)
joint_num = p.getNumJoints(robot)


for i in range(joint_num):
    jointIds.append(i)
    info = p.getJointInfo(robot, i)
    joint_para.append(list(info))
    joint_state.append(list(p.getJointState(robot,i)))
# step through the simluation
for i in range (1000000):
    p.stepSimulation()
    time.sleep(1./240.)


print(joint_state)

p.disconnect()