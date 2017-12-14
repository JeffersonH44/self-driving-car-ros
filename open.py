import capnp
import zmq
import time

capnp.remove_import_hook()
car_capnp = capnp.load('./src/celsius/msg/msg_capnp/car.capnp')

#import car_capnp
context = zmq.Context()
subs = context.socket(zmq.SUB)
subs.connect('tcp://localhost:8021')
subs.setsockopt(zmq.SUBSCRIBE, '')

while True:
    m = subs.recv()
    print car_capnp.CarState.from_bytes(m)
