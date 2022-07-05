#!/usr/bin/env python 
# coding: utf-8

import time
from mySocket import MySocket

class wifibot:
    def __init__(self):
        self.titre = "wifibot"
        self.port = 15020
        self.ip = "192.168.1.22"
        self.cmd = [0] * 9
        self.cmd[0] = 255
        self.cmd[1] = 7
        self.speedGauche = 35
        self.speedDroite = 35
        self.client = MySocket()
        self.client.connect()

    def deconnection(self):
        self.client.close()
        print("deconnection")

    def crc16(self, cmd):
        crc = 0xFFFF
        polynome = 0xa001
        cptbit = 0
        parity = 0

        for cptoctet in range(1, len(cmd) - 2):
            crc ^= cmd[cptoctet]
            for cptbit in range(0, 8):
                parity = crc
                crc >>= 1
                if parity % 2 == 1:
                    crc ^= polynome
        cmd[7] = crc & 0x00FF
        cmd[8] = crc >> 8

    def changeSpeed(self, leftspeed, rightspeed, cmd, sock):
        lspeed = abs(leftspeed)
        rspeed = abs(rightspeed)
        cmd[2] = lspeed & 0x00FF
        cmd[3] = lspeed >> 8
        cmd[4] = rspeed & 0x00FF
        cmd[5] = rspeed >> 8
        cmd[6] = (((0x00 if leftspeed < 0 else 0x04) + (0x00 if rightspeed < 0 else 0x01)) << 4) + 0x08
        self.crc16(cmd)
        buff = bytearray(cmd)
        sock.sende(buff)
        msg = sock.receive(1024)
        time.sleep(0.1)

    def arret(self):
        self.changeSpeed(0, 0, self.cmd, self.client)
        print("arret")

    def avancer(self):
        self.changeSpeed(self.speedGauche+20, self.speedDroite+20, self.cmd, self.client)

    def reculer(self):
        self.changeSpeed(-self.speedGauche, -self.speedDroite, self.cmd, self.client)

    def droite(self):
        self.changeSpeed(self.speedGauche, -self.speedDroite, self.cmd, self.client)
        print("droite")

    def gauche(self):
        self.changeSpeed(-self.speedGauche, self.speedDroite, self.cmd, self.client)
