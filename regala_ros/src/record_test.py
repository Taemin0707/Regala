import os
import time
import sys
from subprocess import Popen, PIPE, STDOUT

# os.system("rosrun turtlesim turtlesim_node")
# time.sleep(2)
# os.system("Ctrl-C")
# sys.exit()

# call('rosrun turtlesim turtlesim_node')

# cmd = ['rosrun', 'turtlesim', 'turtlesim_node']
cmd = ['rosrun', 'image_view', 'video_recorder', image:='right_camera/image_raw']
proc = Popen(cmd, preexec_fn=os.setsid)
# proc = Popen(cmd, shell=True, preexec_fn=os.setsid)

time.sleep(10)
print("kill")
cmd = ['rosnode', 'kill', '/turtlesim']
# cmd = 'rosnode kill /turtlesim'
# Popen(cmd, shell=True, stdout=PIPE, stderr=STDOUT, preexec_fn=os.setsid)
# Popen(cmd, shell=True, preexec_fn=os.setsid)
proc = Popen(cmd, preexec_fn=os.setsid)
time.sleep(1)
proc.terminate()


print("Done")