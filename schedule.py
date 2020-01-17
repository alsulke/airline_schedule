import threading
import socket
import sys

#Class containing the timetable's variables and functions
class TimeTable:
    timetable = []
    dataLock = threading.Lock()

    def addFlight(flight):
        TimeTable.timetable.append(flight)

    def ReturnDetailsStr(index):
        tmpTuple = ("ROK",TimeTable.timetable[index].code,TimeTable.timetable[index].state,TimeTable.timetable[index].time)
        return " ".join(tmpTuple)

    def SearchFlight(searchItem):
        for x in range(len(TimeTable.timetable)):
            if TimeTable.timetable[x].code == searchItem:
                return x
        return "RERR"

    def AddFlight(protocolData):
        TimeTable.timetable.append(Flight(protocolData[1],protocolData[2],protocolData[3]))

    def DeleteFlight(protocolData):
        position = TimeTable.SearchFlight(protocolData[1])
        TimeTable.timetable.pop(position)

    def ChangeFlight(protocolData):
        position = TimeTable.SearchFlight(protocolData[1])
        TimeTable.timetable[position].state = protocolData[2]
        TimeTable.timetable[position].time = protocolData[3]
        return TimeTable.timetable[position]

#Class containing the flight's details
class Flight:

    def __init__(self,code,state,time):
        self.code = code
        self.state = state
        self.time = time


def Worker(conn, add):
    while True:
        data = conn.recv(1024)
        protocolData = data.decode().split(" ")
        #Reply to the READ action
        if protocolData[0] == "READ":
            TimeTable.dataLock.acquire()
            position = TimeTable.SearchFlight(protocolData[1])
            if position != "RERR":
                conn.sendall(TimeTable.ReturnDetailsStr(position).encode())
            else:
                conn.sendall(position.encode())
            TimeTable.dataLock.release()
        #Reply to the WRITE action
        elif protocolData[0] == "WRITE":
            TimeTable.dataLock.acquire()
            n1W = len(TimeTable.timetable)
            TimeTable.AddFlight(protocolData)
            n2W = len(TimeTable.timetable)
            if n2W > n1W:
                conn.sendall("WOK".encode())
            else:
                conn.sendall("WERR".encode())
            TimeTable.dataLock.release()
        #Reply to the DEL action
        elif protocolData[0] == "DEL":
            TimeTable.dataLock.acquire()
            n1D = len(TimeTable.timetable)
            TimeTable.DeleteFlight(protocolData)
            n2D = len(TimeTable.timetable)
            if n2D < n1D:
                conn.sendall("DOK".encode())
            else:
                conn.sendall("DERR".encode())
            TimeTable.dataLock.release()
        #Reply to the CHANGE action
        elif protocolData[0] == "CHANGE":
            TimeTable.dataLock.acquire()
            changedObj = TimeTable.ChangeFlight(protocolData)
            if changedObj.state == protocolData[2] and changedObj.time == protocolData[3]:
                conn.sendall("CHOK".encode())
            else:
                conn.sendall("CHERR".encode())
            TimeTable.dataLock.release()


def main():
    f1 = Flight("ah123","boarding","17.30")

    TimeTable.addFlight(f1)

    serverAdd = ("localhost",1236)

    #Creating, binding and listening on localhost:1234
    doorSock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    doorSock.bind(serverAdd)
    doorSock.listen(10)

    while True:
        print("Waiting for a connection...")

        conn, add = doorSock.accept()

        t = threading.Thread(target=Worker, args = (conn, add))
        t.start()


if __name__ == '__main__':
    main()
