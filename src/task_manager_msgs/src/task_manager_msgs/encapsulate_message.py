#!/usr/bin/python

from io import BytesIO,StringIO
from task_manager_msgs.msg import EncapsulatedMessage


def encapsulate(msg):
    E = EncapsulatedMessage()
    E.type = msg._type
    E.md5sum = msg._md5sum
    strout=BytesIO()
    msg.serialize(strout)
    E.data = strout.getvalue()
    # print("Encapsulated")
    # print(E)
    return E


def decapsulate(msg, encapsulated):
    if msg._type != encapsulated.type:
        raise TypeError("Decapsulate: invalid msg type")
    if msg._md5sum != encapsulated.md5sum:
        raise TypeError("Decapsulate: invalid md5 sum")
    msg.deserialize(encapsulated.data)



if __name__ == '__main__':
    E1 = EncapsulatedMessage()
    E1.type = "test/test"
    E1.md5sum = "0123456"
    E1.data = range(10)
    print("Original data")
    print(E1)
    
    E2 = encapsulate(E1)
    print("Encapsulated")
    print(E2)

    
    E3 = EncapsulatedMessage()
    decapsulate(E3,E2)
    print("Recovered data")
    print(E3)

