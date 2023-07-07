import socket
import threading
import numpy as np
import math
from robot import Robot, ArUcoDetector
import time

test_sequence = [[200, 200],[200, 800], [800, 800], [800, 200]]
curr_test_point = 0

# Get data from user
def handle_client(client_socket):
    global curr_test_point
    robot = Robot("COM5")
    aruco = ArUcoDetector()
    aruco.id_updater[1] = robot.update
    aruco.robots[1] = robot
    aruco.marker_sizes[3] = 0.027
    received = ['400', '400']
    received = np.array(list(map(float, received)))
    #try:
    while True:
        # try:
        #     data = client_socket.recv(1024)
    # except:
        #     pass
        #
        # if not data:
        #     break
        # print(data.decode())
        # response = "Response from server"
        #received = data.decode().split(' ')
        # for point in test_sequence:
        #     print(point)


        ## square
        if not robot.nowTravelling:
            robot.goTo(test_sequence[curr_test_point])
            curr_test_point+=1
            print(curr_test_point)
            if curr_test_point >= len(test_sequence):
                curr_test_point = 0


        ## go to marker
        if 3 in aruco.found_markers.keys():
            aruco3Pos = aruco.found_markers[3][:2, 3]
            delta = aruco3Pos-robot.pos

            invNormTarg = delta / np.linalg.norm(delta)
            robot.goTo(aruco3Pos-invNormTarg*50)


        # target = received - robot.pos
        # robot.setSpeed(min(70, int(np.linalg.norm(target)*0.4)))
        # robot.holdAng(math.atan2(*target[::-1]))
        # while 500 > np.linalg.norm(received - robot.pos) > 10:
        #print(robot.pos, robot.targetAng, robot.currAng)
        #     time.sleep(0.1)
        time.sleep(0.1)
        #print("reached")
        #client_socket.send(response.encode())

    #client_socket.close()
    # except Exception as e:
    #     print(e)
    #     robot.move(0, 0)
handle_client(1)

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('192.168.88.94', 8882))
server_socket.listen(10)
server_socket.setblocking(0)
server_socket.settimeout(0.1)


pending_connections = []


# Accept connection and adding into the array
def accept_connections():
    while True:
        try:
            client_socket, address = server_socket.accept()
            pending_connections.append(client_socket)
            print("Pending connections:")
            for i, conn in enumerate(pending_connections):
                print(f"{i + 1}. {conn.getpeername()}")
        except:
            pass


# Managing connections
def process_connections():
    while True:
        choice = input("Enter the index of the connection to process (or 'q' to quit): ")
        if choice.lower() == 'q':
            break

        try:
            index = int(choice) - 1
            if index < 0 or index >= len(pending_connections):
                print("Invalid choice. Try again.")
            else:
                client_socket = pending_connections.pop(index)
                client_thread = threading.Thread(target=handle_client, args=(client_socket,))
                client_thread.start()
        except ValueError:
            print("Invalid choice. Try again.")
    server_socket.close()


accept_thread = threading.Thread(target=accept_connections)
accept_thread.start()

process_thread = threading.Thread(target=process_connections)
process_thread.start()

accept_thread.join()
process_thread.join()