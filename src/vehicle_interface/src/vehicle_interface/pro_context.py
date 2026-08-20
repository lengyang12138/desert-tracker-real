import zmq
import json

class SerialTopicSocket(zmq.Socket):

    def sendPro(self, topic, obj, flags = 0, protocol = -1):
        #tobj = json.dumps(obj)
        tobj = json.dumps(obj).encode('utf-8')
        return self.send_multipart([topic, tobj])

    def recvPro(self, flags=0):
        topic, tobj = self.recv_multipart(flags)
        return json.loads(tobj)

class proContext(zmq.Context):
    _socket_class = SerialTopicSocket
