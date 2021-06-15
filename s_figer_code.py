#!/usr/bin/env python3.5
# -*- coding:utf-8 -*-
import serial
import time
import threading
import RPi.GPIO as GPIO

#*****************************************************************************************
ACK_SUCCESS = 0x00
ACK_FAIL = 0x01
CMD_getimage  = 0x01  #从传感器读入指纹并存于图像缓存区
CMD_genchar   = 0x02   #根据原始图像生成指纹特征存于 CharBuffer1 或 CharBuffer2
CMD_match     = 0x03   #精确比对 CharBuffer1 与 CharBuffer2 中的特征文件
CMD_search    = 0x04  #以 CharBuffer1 或 CharBuffer2 中的特征文件搜索整个或部分指纹库
CMD_regmodel  = 0x05  #将 CharBuffer1 与 CharBuffer2 中的特征文件合并生成模板存于 CharBuffer2
CMD_storechar = 0x06  #将特征缓冲区中的文件储存到 flash 指纹库中
CMD_loadchar  = 0x07   #从 flash 指纹库中读取一个模板到特征缓冲区
CMD_upchar    = 0x08   #将特征缓冲区中的文件上传给上位机
CMD_dowchar   = 0x09   #从上位机下载一个特征文件到特征缓冲区
#********************************************************
# definition of Connebction Packages
CMD_head      = [0xEF,0x01]
CMD_addr      = [0xFF,0xFF,0xFF,0xFF]
Command_mark  = 0x01
cmd_zero = 0x00
# Command_mark = 0x01
# Data_mark    = 0x02
# End_packages = 0x08
# Reaction_packages = 07
#*******************************************************
pin_21 = 21  #MOS管高电平输入电压控制脚针
pin_22 = 22  #蜂鸣器电平输出脚针
pin_24 = 24  #继电器高电平输出脚针
pin_18 = 18  #指纹唤醒输入脚针
#*************************************************
#指纹缓存区号
#CharBuffer1   = 0x01
#CharBuffer2   = 0x02
#*************************************************
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_18, GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(pin_21,GPIO.OUT)
pc_cmd_rxcn =[]
ser = serial.Serial("/dev/ttyAMA0", 57600)


def TxAndRxCmd(cmd_cn):
    global g_rx_cn
    g_rx_cn     =[]
    Checksum = 0
    tx_cn = []
    tx=""
    for i in CMD_head:
        tx_cn.append(i)
    for i in CMD_addr:
        tx_cn.append(i)
    tx_cn.append(Command_mark)
    tx_cn.append(cmd_zero)
    Checksum += Command_mark
    for byte in cmd_cn:
        tx_cn.append(byte)
        Checksum += byte
    tx_cn.append(cmd_zero)
    tx_cn.append(Checksum)
    for i in tx_cn:
        tx += chr(i)

    ser.flushInput()
    ser.write(tx)
    time.sleep(0.4)
    bytes_recv = ser.inWaiting()
    time.sleep(0)
    if bytes_recv != 0:
        g_rx_cn += ser.read(bytes_recv)
        if len(g_rx_cn) > 9:
            for i in range(len(g_rx_cn)):
                g_rx_cn[i] = ord(g_rx_cn[i])
            if g_rx_cn[9]  == 0x00:
                return ACK_SUCCESS
            if g_rx_cn[0] != CMD_head:
                return ACK_FAIL
            if g_rx_cn[9] == 0x01:
                return ACK_FAIL
            if g_rx_cn[9] == 0x03:
                return ACK_FAIL
            if g_rx_cn[9] == 0x06:
                return ACK_FAIL
            if g_rx_cn[9] == 0x09:
                return ACK_FAIL
            if g_rx_cn[9] == 0x02:
                return 0x02
def TxandRx_get():
    global g_rx_cn
    cmd_cn = [0x03,CMD_getimage]
    r = TxAndRxCmd(cmd_cn)
    if r == ACK_SUCCESS:
        return ACK_SUCCESS
    if r == ACK_FAIL:
        return ACK_FAIL
    if r == 0x02:
        return 0x02

def TxandRx_gen():
    global g_rx_cn
    cmd_cn = [0x04,CMD_genchar,0x02]
    r = TxAndRxCmd(cmd_cn)
    if r == ACK_SUCCESS:
        return ACK_SUCCESS
    if r == ACK_FAIL:
        return ACK_FAIL

def TxandRx_search():
    global g_rx_cn
    cmd_cn = [0x08,0x04,0x02,0x00,0x00,0x00,0x07]
    r = TxAndRxCmd(cmd_cn)
    if r == ACK_SUCCESS:
        return ACK_SUCCESS
    if r == ACK_FAIL:
        return ACK_FAIL

def open_door():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin_24,GPIO.OUT)
    GPIO.output(pin_24,GPIO.HIGH)
    time.sleep(2)
    GPIO.output(pin_24,GPIO.LOW)
    GPIO.cleanup(24)

def voice_fail():
        i=0
        while i < 3:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(pin_22, GPIO.OUT)
            GPIO.output(pin_22, GPIO.HIGH)
            time.sleep(0.1)
            i+=1
            GPIO.cleanup(22)
            time.sleep(0.1)
def voice_success():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin_22,GPIO.OUT)
    GPIO.output(pin_22,GPIO.HIGH)
    time.sleep(1)
    GPIO.cleanup(22)

def main():
    while 1:
        r = TxandRx_get()
        if r == ACK_SUCCESS:
            r = TxandRx_gen()
            if r == ACK_SUCCESS:
                r = TxandRx_search()
                if r == ACK_SUCCESS:
                    voice_success()
                    open_door()
                elif r == ACK_FAIL:
                    voice_fail()
            elif r == ACK_FAIL:
                 voice_fail()

if __name__ == '__main__':
        try:
            main()
        except KeyboardInterrupt:
            if ser != None:
                ser.close()
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(pin_22,GPIO.OUT)
            GPIO.setup(pin_24,GPIO.OUT)
            GPIO.cleanup(pin_22)
            GPIO.cleanup(pin_24)
            print("\n\n Test finished ! \n")
