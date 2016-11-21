__author__ = 'Ikechukwu Ofodile -- ikechukwu.ofodile@estcube.eu'


from time import sleep

class Coilgun:

    def __init__(self, port):
        self.port = port
        self.port.write("<cc>\n")
        sleep(0.5)
        print "coilgun initialized"

    def kick(self):
        self.port.write("<ck>\n")
        sleep(1)

    def shut_down(self):
        self.port.write("<ce>\n")
        sleep(1)
        print("coilgun shut down")
