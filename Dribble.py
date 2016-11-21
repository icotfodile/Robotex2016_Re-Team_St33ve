__author__ = 'Ikechukwu Ofodile -- ikechukwu.ofodile@estcube.eu'

from time import sleep


class Dribbler:

    def __init__(self, port):
        self.port = port
        print "dribbler initialized"

    def activate(self, on):
        if on:
            self.port.write("<dn>\n")
            sleep(0.03)
        else:
            self.port.write("<df>\n")

    def ball_in(self):
        self.port.write("<bd>\n")
        sleep(0.03)
        try:
            n = self.port.readline()
            if n[0] == '1':
                return True
            else:
                return False
        except Exception, e:
            print "Serial port error: " + str(e)

    def shut_down(self):
        self.port.write("<df>\n")
        sleep(0.1)
        print("dribbler shut down")