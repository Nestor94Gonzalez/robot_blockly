#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Erle Robotics LLC
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import rospy
import time
from std_msgs.msg import String
from autobahn.asyncio.websocket import WebSocketServerProtocol, \
    WebSocketServerFactory
import roslaunch
import os
import sys
import inspect
import pprint
from multiprocessing import Process
import threading

try:
    import asyncio
except ImportError:
    # Trollius >= 0.3 was renamed
    import trollius as asyncio


class BlocklyServerProtocol(WebSocketServerProtocol):
    def onConnect(self, request):
        print("Client connecting: {0}".format(request.peer))

    def onOpen(self):
        print("WebSocket connection open.")

    def onMessage(self, payload, isBinary):
        #Debug

        '''
        if isBinary:
            print("Binary message received: {0} bytes".format(len(payload)))
        else:
            print("Text message received: {0}".format(payload.decode('utf8')))
        '''
        ## Do stuff
        # pub = rospy.Publisher('blockly', String, queue_size=10)
        # time.sleep(1)
        # pub.publish("blockly says: "+payload.decode('utf8'))

        self.build_ros_node(payload.decode('utf8'))
#        print('The file generated contains...')        
#        os.system('cat test.py')

#        os.system("python3 test.py")

        # echo back message verbatim
        # self.sendMessage(payload, isBinary)

    def onClose(self, wasClean, code, reason):
        print("WebSocket connection closed: {0}".format(reason))    


    def build_ros_node(self,blockly_code):    

        print("\nFull blockly_code:\n"+blockly_code)
        print("building the ros node...")

        
        #filename = "test.py"
        #target = open(filename, 'w+')
        #target.truncate() # empties the file

        __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
        target = open(os.path.join(__location__, 'test.py'), 'w+')
        
       
        #print(os.path.dirname(os.path.realpath(__file__))+"/test.py")


        ###########################
        # Start building the ROS node:

        target.write("#!/usr/bin/env python3\n")
        target.write("import rospy\n")
        target.write("import time\n")
        target.write("from std_msgs.msg import String\n")
        

        #### PARSER ###
        print("Parsing code ...")
        newCode = []
        importList = []
        for line in blockly_code.splitlines():
            print("LINE "+line)

            #if blockly_code.startswith('EB2__'):
            indentation=0
            if "EB2__" in line:
                indentation = len(line) - len(line.lstrip(' '))
                print("indentation:"+str(indentation))
                line = line.lstrip(' ')
            if line.startswith('EB2__') or line.startswith('EB2__') :
                ex_requests = line.split("EB2__")
                ex_requests = filter(None,ex_requests)

                for s in ex_requests:
                    code = "EB2__"+s
                    print("code: "+code)
                    function_name = code.split("?",1)[0]
                    param = code.split("?",1)[1]
                    parameters = param.split("&") 
                    print(function_name)
                    print(parameters)
                    print("\n==============================")                  
                    exec("pprint.pprint(inspect.getsource(sys.modules[__name__]."+function_name+"))")           
                    print(type(eval("inspect.getsource(sys.modules[__name__]."+function_name+")")))
                    function_code = str(eval("inspect.getsource(sys.modules[__name__]."+function_name+")"))
                    print(type(function_code))
                    #newCode.insert(0, "\n")
                    
                    #ADD MAIN
                    function_code += "if __name__ == '__main__':\n"

                    param_str = ""
                    first = True
                    for param in parameters:
                        if first:
                            first = False
                            param_str += param
                        else:
                            param_str += ", "+param
                        
                    print(param_str)

                    function_code += "    "+function_name+"("+param_str+")\n"

                    #ADD required indentation
                    if indentation>0:
                        id_whitespaces = ""
                        for i in range(indentation):
                            id_whitespaces+=" "
                        indented_function_code = ""
                        print("CODE ERROR ---------------------")
                        for line in function_code.splitlines():
                            print("IND LINE"+line)
                            print(id_whitespaces)
                            print(line)
                            indented_function_code += id_whitespaces+line+"\n"
                        print("IND2LINE"+indented_function_code)
                        newCode.append("\n"+indented_function_code+"\n")
                    else:
                        print("IND 2")
                        ###newCode.insert(0, "\n"+function_code)
                        newCode.append("\n"+function_code+"\n")

            elif line.startswith('#!'):
                print("SKIPPED LINE "+line)
                newCode.append("\n") 
                pass
            elif line.startswith('import') or line.startswith('from'):
                importList.append(line)
            else:
                newCode.append(line+"\n")

        #add imports 
        importList.insert(0,"rospy.init_node('blockly_node', anonymous=True)")
        print(importList)
        for line in importList:
            newCode.insert(0,line+"\n")


        print("NEW CODE :")
        for line in newCode:
            print(line)
            target.write(line)

        #target.write(newCode)
        
        # close the file
        print("closing file")
        target.close()
        #os.system('cat test.py')

        
        os.system('cat '+os.path.join(__location__, 'test.py'))
        os.system('python3 '+os.path.join(__location__, 'test.py'))
        print("\nEND")

def EB2__erle_brain__turn_on_orange_led(parameter1):

    print("EB2__erle_brain__turn_on_orange_led w/ parameter:"+parameter1)
    #rospy.init_node('blockly_node', anonymous=True)
    pub = rospy.Publisher('/statusleds', String, queue_size=10)
    rate = rospy.Rate(10)
    start = time.time()
    flag = True #time flag
    orange_led = parameter1
    led = orange_led
    if (led == 'TRUE'):
        msg = 'orange'
    else:
        msg = 'orange_off'
    while not rospy.is_shutdown() and flag:
        sample_time=time.time()
        if ((sample_time - start) > 1):
          flag=False
        pub.publish(msg)
        rate.sleep()

def EB2__erle_brain__get_orientation(ow_,ox_,oy_,oz_):
    import rospy
    import subprocess
    import rosnode
    from sensor_msgs.msg import Imu

    global ow
    global ox
    global oy
    global oz

    print("EB2__erle_brain__get_orientation w/ params:"+ow_+ox_+oy_+oz_)
    ros_nodes = rosnode.get_node_names()
    if not '/imu_talker' in ros_nodes:
        command='/home/erle/catkin_ws_imu/src/ros_erle_imu/src/imu_talker.cpp'
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
    msg_imu = rospy.wait_for_message('/imu9250', Imu, timeout=1)
  
    ow_ = msg_imu.orientation.w
    ox_ = msg_imu.orientation.x
    oy_ = msg_imu.orientation.y
    oz_ = msg_imu.orientation.z
 
    ow = ow_
    ox = ox_
    oy = oy_
    oz = oz_

    print(type(ow))

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def wait_until_ros_node_shutdown(loop):
    while not rospy.is_shutdown():
        time.sleep(.1)
        yield

    loop.stop()



def talker():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.

    rospy.init_node('blockly_server', anonymous=True)
    rospy.Subscriber("blockly", String, callback)

    factory = WebSocketServerFactory(u"ws://0.0.0.0:9000", debug=False)
    factory.protocol = BlocklyServerProtocol

    loop = asyncio.get_event_loop()
    coro = loop.create_server(factory, '0.0.0.0', 9000)
    server = loop.run_until_complete(coro)
    asyncio.async(wait_until_ros_node_shutdown(loop))

    loop.run_forever()

    print("Closing...")
    server.close()
    loop.run_until_complete(server.wait_closed())
    loop.close()

if __name__ == '__main__':
    talker()
