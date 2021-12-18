import rtde_receive
import rtde_control
import time

rtde_c = rtde_control.RTDEControlInterface("172.22.22.2")
rtde_r = rtde_receive.RTDEReceiveInterface("172.22.22.2")
actual_q = rtde_r.getActualQ()
actual_tcp_force = rtde_r.getActualTCPForce()

print("Connected")
print(actual_q)
print(actual_tcp_force)

joint_q = [-2.246578041707174, -1.3520143044045945, -1.5585254430770874, -0.2304435533336182, -4.035606447850363, -1.5]

joint_q2 = [-2.246578041707174, -1.3520143044045945, -1.5585254430770874, -0.2304435533336182, -4.035606447850363, -1.6]

# Move to initial joint position with a regular moveJ
rtde_c.moveJ(joint_q)
rtde_c.moveJ(joint_q2)

start_time = time.time()
while time.time()-start_time<3:
	# print(time.time()-start_time)
	
	rtde_c.speedL([-0.01, 0, 0, 0, 0, 0] ,0.5, 0.5)
rtde_c.speedL([0, 0, 0, 0, 0, 0] ,0.5, 0.5)
rtde_c.stopScript()
print("Finished")