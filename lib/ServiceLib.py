#-------------------------------------------------------------------------
# ServiceLib
# This file cntains user defined functions and constants to be used in
# main programm
#-------------------------------------------------------------------------
import pycom
import _thread
import machine
import math
import network
import os
import time
import utime
import gc
import array
import struct
import binascii
from network import LoRa
from machine import SPI
from machine import Timer
import json
from L76GNSV4 import L76GNSS
from pytrack import Pytrack


py = Pytrack()
l76 = L76GNSS(py, timeout=5)
#----------------------------------------------------------------
# Colori LED
#----------------------------------------------------------------
LED_Blu_steady = 0x0000af,0
LED_Blu_blinking = 0x0000af,1
LED_Red_steady = 0xaf0000,0
LED_Red_blinking = 0xaf0000,1
LED_Green_steady = 0x00af00,0
LED_Green_blinking = 0x00af00,1
LED_Yellow_steady = 0x0f0f00,0
LED_Yellow_blinking = 0xafaf00,1

# Global Variables
OPC_parameters_Counts_K = array.array('i',[1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000])
default_parameters_Counts_K = array.array('i',[1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000])
SogliaUp_gruppo_1_3 = 1000
SogliaUp_gruppo_3_8 = 1000
SogliaUp_gruppo_8_17 = 1000
SogliaDown_gruppo_1_3 = 1000
SogliaDown_gruppo_3_8 = 1000
SogliaDown_gruppo_8_17 = 1000


JSON_Default = "{instrument_SN: 10,loraTx_Delay: 0,channels_K:[{channel_k; 1,},{channel_k; 1,},{channel_k; 1,},{channel_k; 1,},{channel_k; 1,},{channel_k; 1,},{channel_k; 1,},{channel_k; 1,},{channel_k; 1,},{channel_k; 1,},{channel_k; 1,},{channel_k; 1,},{channel_k; 1,},{channel_k; 1,},{channel_k; 1,},{channel_k; 1,}],SogliaUp_gruppo_1_3: 5000,SogliaUp_gruppo_3_8: 1000,SogliaUp_gruppo_8_17: 200,ON_Alarm_time: 2,OFF_Alarm_time: 2}"

def OPCLoraEvent():
    print('Lora Event')
    return

def OPCLoraRx():
    print('Packet Received')
    return


def OPCLoraTx():
    print('Packet Sent')
    return
def OPCLoraTxFail():
    print('Packet Not Sent')
    return

#-------------------------------------------------------------------------
# LORAConnection_OOTA
# This function connect to LoRa Wan in OOTA mode waiting for LoraConnectionTimeout
# (in ms) the ack for the connection
#-------------------------------------------------------------------------
def LORAConnection_OOTA(LoraConnectionTimeout):
    LoraConnected = False
    lora = LoRa(mode=LoRa.LORAWAN, region=LoRa.EU868)
    print("LoRa MAC Address:  ", binascii.hexlify(
        lora.mac()).upper().decode('utf-8'))
    print("LoRa EUI: ", binascii.hexlify(machine.unique_id()))
    # create an OTAA authentication parameters
    app_eui = binascii.unhexlify('8493983621D5A1BC')
    app_key = binascii.unhexlify('9F68AE5FD1CE35D1F83CD48446CF7BBF')
    # L2
    # app_key = binascii.unhexlify('3F4A90ABF82EBD4067CCA30FD1D7ACDE')
    # L10
    # app_key = binascii.unhexlify('F8165748326F03346935F84BDE710AE4')
    # L12
    # app_key = binascii.unhexlify('38F812962FDF2BD64A78223F398381F3')
    # L13
    # app_key = binascii.unhexlify('8C0DF8D758F1B9340C9545B05DE3820B')
    # L14
    # app_key = binascii.unhexlify('82FBAED74A43BA32225751F9BC2BB693')


    #lora.callback(trigger = LoRa.RX_PACKET_EVENT | LoRa.TX_PACKET_EVENT | LoRa.TX_FAILED_EVENT, handler = OPCLoraEvent)
    #lora.callback(trigger = LoRa.TX_PACKET_EVENT, handler = OPCLoraTx)
    #lora.callback(trigger = LoRa.TX_FAILED_EVENT, handler = OPCLoraTxFail)

    # join a network using OTAA (Over the Air Activation)
    #lora.join(activation=LoRa.OTAA, auth=(app_eui, app_key), timeout=0)
    try:
        lora.join(activation=LoRa.OTAA, auth=(app_eui, app_key), timeout=LoraConnectionTimeout)
        time.sleep_ms(8000)
        if(lora.has_joined()):
            LoraConnected = True
    except TimeoutError:
        print("timeout expired")
        LoraConnected = False
    # wait until the module has joined the network
    #while not lora.has_joined():
    #    time.sleep_ms(2500)
    #    print('Not yet joined...')
    #pass
    #print('Joined!')
    print('Lora connected = ',LoraConnected)
    return LoraConnected, lora

#--------------------------------------------------------------
# Initialize ConfigFile
# return true if Filewritten correctly
# return false if some problem happened
#--------------------------------------------------------------
def ConfigJsonDefaultFile():

    try:
        f = open('/sd/OPC_ConfigJson.txt', 'w')


        f.write(JSON_Default)    # Device ID  valore numerico di 2 cifre (da 10 a 99)

    except Exception as error:
        print('Errore in scrittura file n.', error)
        return False


    return True


def OPC_Config_file_Init():
    default_parameters_instrument_SN = 10
    default_parameters_loraTx_Delay = (default_parameters_instrument_SN - 10) * 2 # Per ora non mettiamo conrolli particolari: il Tx delay è sfasato di 2 minuti tra tutti gli strumenti per evutare probemi di conflitto in tx
    #default_parameters_threshold_K = array.array('f',[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1])
    default_parameters_Counts_K = array.array('i',[1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000])

    print('Scrittura file OPC_Config.txt con parametri di default')
    try:
        f = open('/sd/OPC_Config.txt', 'w')
        #f.write('10'+'\n')    # Device ID  valore numerico di 2 cifre (da 10 a 99)
        #f.write('2' +'\n')   # minutoinvio
        #f.write(str(default_parameters_instrument_SN) +'\n')    # Device ID  valore numerico di 2 cifre (da 10 a 99)
        f.write('{:0>20}'.format(str(default_parameters_instrument_SN)) +'\n')
        #f.write(default_parameters_instrument_SN +'\n')
        #f.write(str(default_parameters_loraTx_Delay) +'\n')
        f.write('{:0>20}'.format(str(default_parameters_loraTx_Delay)) +'\n')

        for i in range(16):
        #for element in default_parameters_threshold_K:
            #f.write(str(default_parameters_threshold_K[i]) +'\n')
            f.write('{:0>20}'.format(str(default_parameters_Counts_K[i]) +'\n'))
            #print(str(default_parameters_Counts_K[i]) +'\n')
            #f.write(string(element) +'\n')
        # viene generato un allarme se almeno uno dei tre gruppi di particelle supera il livello SogliaUp per un tempo >= ON_Alarm_time
        # l'allarme rientra se tutti e tre i gruppi di particelle scendono sotto SogliaDown per un tempo >= OFF_Alarm_time
        # il livello delle particelle è in conteggi al minuto
        # i tempi per generare allarme o per rientrarvi sono espressi in numeri di cicli (ogni ciclo è un minuto)
        # il metodo di calcolo del numero di cicli è spiegato nel pezzo di programma relativo
        #f.write('5000' +'\n')   # SogliaUp_gruppo_1_3  #
        f.write('{:0>20}'.format(str('5000')) +'\n')
        #f.write('3000' +'\n')   # SogliaDown_gruppo_1_3
        #f.write('1000'+ '\n')   # SogliaUp_gruppo_3_8
        f.write('{:0>20}'.format(str('1000')) +'\n')
        #f.write('500' + '\n')   # SogliaDown_gruppo_3_8
        #f.write('200'+ '\n')   # SogliaUp_gruppo_8_17
        f.write('{:0>20}'.format(str('200')) +'\n')
        #f.write('100' + '\n')   # SogliaDown_gruppo_8_17
        #f.write('2' + '\n')   # ON_Alarm_time
        f.write('{:0>20}'.format(str('2')) +'\n')
        #f.write('2' + '\n')   # OFF_Alarm_time
        f.write('{:0>20}'.format(str('2')) +'\n')
        f.close()

    except Exception as error:
        print('Errore in scrittura file n.', error)
        return False

    return True








#--------------------------------------------------------------
# Initialize GPS
# return true if found position, coord nad date
#return false if position not found
#--------------------------------------------------------------

def GPSInit(max_time_wait=20):
    py = Pytrack()
    l76 = L76GNSS(py, timeout=5)
    ciclo1 = 0
    getsat = True

    while getsat and (ciclo1 < max_time_wait):
        ciclo1 = ciclo1 + 1
        print(ciclo1)
        print('TASK 1: Looking for GPS SAT')
        coord = l76.coordinates()
        data = l76.getUTCDateTimeTuple()
        print(data)
        time.sleep(1)
        if data == None:
            time.sleep_ms(50)
        else:
            getsat = False
    if(getsat==False):
        print('TASK 1: got GPS SAT',coord,data)
        return coord,data,True
    else:
        print('TASK 1: not got GPS SAT',coord,data)
        return coord,data,False

#--------------------------------------------------------------
# Read Time from GPS
#--------------------------------------------------------------

def GPSGetTime():
    return l76.getUTCDateTimeTuple()
#--------------------------------------------------------------
# Read Coord from GPS
#--------------------------------------------------------------
#
def GPSGetCoord():
    return l76.coordinates()



#--------------------------------------------------------------
# Read Coord from GPS
#--------------------------------------------------------------
#
class Clock:

    def __init__(self):
        self.seconds = 0
        self.StatusOn = False
        self.__alarm = Timer.Alarm(self._seconds_handler,0.2 , periodic=True)

    def _seconds_handler(self, alarm):
        self.seconds += 1
        self.seconds %= 2
        #print("%02d seconds have passed" % self.seconds)
        if self.seconds ==1:
            if(LastColor[1] == 1): # Blinking Led
                if(self.StatusOn == True):
                    pycom.rgbled(0x000000) #nmero
                    #print('LED Spento')
                    self.StatusOn = False
                else:
                    pycom.rgbled(LastColor[0])
                    #print('LED Acceso ',LastColor[0])
                    self.StatusOn = True

            #alarm.cancel() # stop counting after 10 seconds


#----------------------------------------------------------------
# Set led
#----------------------------------------------------------------
def SetLed(Color):
    global LastColor
    LastColor = Color
    #print(LastColor)
    pycom.rgbled(Color[0])

#----------------------------------------------------------------
# OPC_Cmd_ThresholdsSet
#----------------------------------------------------------------
#def OPC_Cmd_ThresholdsSet(data,s,NumDevice):
def OPC_Cmd_ThresholdsSet(ReceicedThreshold_1,ReceicedThreshold_2,ReceicedThreshold_3,s,Device_Info):

    global SogliaUp_gruppo_1_3
    global SogliaUp_gruppo_3_8
    global SogliaUp_gruppo_8_17
    global SogliaDown_gruppo_1_3
    global SogliaDown_gruppo_3_8
    global SogliaDown_gruppo_8_17

    print('Threshold set before: ', SogliaUp_gruppo_1_3)
    print('Command received: Threasolds Set')
    if(ReceicedThreshold_1 == 65535):
        ReceicedThreshold_1 = int(SogliaUp_gruppo_1_3)
    else:
        SogliaUp_gruppo_1_3 = int(ReceicedThreshold_1)

    if(ReceicedThreshold_2 == 65535):
        ReceicedThreshold_2 = int(SogliaUp_gruppo_3_8)
    else:
        SogliaUp_gruppo_3_8 = int(ReceicedThreshold_2)

    if(ReceicedThreshold_3 == 65535):
        ReceicedThreshold_3 = int(SogliaUp_gruppo_8_17)
    else:
        SogliaUp_gruppo_8_17 = int(ReceicedThreshold_3)

    #imposto soglie in memoria
    SogliaUp_gruppo_1_3 = ReceicedThreshold_1
    SogliaUp_gruppo_3_8 = ReceicedThreshold_2
    SogliaUp_gruppo_8_17 = ReceicedThreshold_3
    SogliaDown_gruppo_1_3 = int(SogliaUp_gruppo_1_3*0.7)
    SogliaDown_gruppo_3_8 = int(SogliaUp_gruppo_3_8*0.7)
    SogliaDown_gruppo_8_17 = int(SogliaUp_gruppo_8_17*0.7)

    #salva le soglie nel file
    OPC_Config_file_Threshold_update(ReceicedThreshold_1,ReceicedThreshold_2,ReceicedThreshold_3)
    #invio soglie modificate per conferma
    OPC_Send_Thresholds(s,Device_Info,SogliaUp_gruppo_1_3,SogliaUp_gruppo_3_8,SogliaUp_gruppo_8_17)
    print('Threshold set after: ', SogliaUp_gruppo_1_3)
    #return SogliaUp_gruppo_1_3,SogliaUp_gruppo_3_8,SogliaUp_gruppo_8_17
    return

#----------------------------------------------------------------
# OPC_Send_Thresholds
#----------------------------------------------------------------

def OPC_Send_Thresholds(s,Device_Info,LocSogliaUp_gruppo_1_3,LocSogliaUp_gruppo_3_8,LocSogliaUp_gruppo_8_17):


    print(' Invio Soglie:')
    print('S1: ',LocSogliaUp_gruppo_1_3)
    print('S2: ',LocSogliaUp_gruppo_3_8)
    print('S3: ',LocSogliaUp_gruppo_8_17)

    StartMg = bytearray('N,'+ str(Device_Info) + ',S,')
    SetThresholdAnswer = bytearray (13)
    SetThresholdAnswer[0] = StartMg[0]
    SetThresholdAnswer[1] = StartMg[1]
    SetThresholdAnswer[2] = StartMg[2]
    SetThresholdAnswer[3] = StartMg[3]
    SetThresholdAnswer[4] = StartMg[4]
    SetThresholdAnswer[5] = StartMg[5]
    SetThresholdAnswer[6] = StartMg[6]
    SetThresholdAnswer[7] = int(LocSogliaUp_gruppo_1_3//256)
    SetThresholdAnswer[8] = int(LocSogliaUp_gruppo_1_3%256)
    SetThresholdAnswer[9] = int(LocSogliaUp_gruppo_3_8//256)
    SetThresholdAnswer[10] = int(LocSogliaUp_gruppo_3_8%256)
    SetThresholdAnswer[11] = int(LocSogliaUp_gruppo_8_17//256)
    SetThresholdAnswer[12] = int(LocSogliaUp_gruppo_8_17%256)
    #SetThresholdAnswer = 'N,'+ str(NumDevice) + ',S,' + str(lista)
    print('Soglie 1',SetThresholdAnswer[7],hex(SetThresholdAnswer[7]))
    print('Soglie 1',SetThresholdAnswer[8],hex(SetThresholdAnswer[8]))
    print('Soglie 2',SetThresholdAnswer[9],hex(SetThresholdAnswer[9]))
    print('Soglie 2',SetThresholdAnswer[10],hex(SetThresholdAnswer[10]))
    print('Soglie 3',SetThresholdAnswer[11],hex(SetThresholdAnswer[11]))
    print('Soglie 3',SetThresholdAnswer[12],hex(SetThresholdAnswer[12]))
    print('stringa Thresholds inviata: ', SetThresholdAnswer)
    s.send(SetThresholdAnswer)
    return True

#--------------------------------------------------------------
# Update Alarm Thresholds
# return True if SD write is ok
# return FALSE IF sd WRITE IS NOT OK
#--------------------------------------------------------------
def OPC_Config_file_Threshold_update(Alarm_Thresholds_1,Alarm_Thresholds_2,Alarm_Thresholds_3):


    LocOPC_parameters_Counts_K = array.array('i',[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1])


    try:
        f = open('/sd/OPC_Config.txt', 'a+')
        print('Apertura file per cambio soglie')
        f.seek(0,0)
        LocOPC_parameters_instrument_SN = int(f.readline())
        print('SN ', LocOPC_parameters_instrument_SN)
        #minutoinvio = f.readline()
        LocOPC_parameters_loraTx_Delay = int(f.readline())
        for i in range(0,16):
            LocOPC_parameters_Counts_K[i] = int(f.readline())
        LocSogliaUp_gruppo_1_3 = int(f.readline())
        LocSogliaDown_gruppo_1_3 = int(SogliaUp_gruppo_1_3 * 0.7)
        LocSogliaUp_gruppo_3_8 = int(f.readline())
        LocSogliaDown_gruppo_3_8 = int(SogliaUp_gruppo_3_8 * 0.7)
        LocSogliaUp_gruppo_8_17 = int(f.readline())
        LocSogliaDown_gruppo_8_17 = int(SogliaUp_gruppo_8_17 * 0.7)
        LocON_Alarm_time = int(f.readline())
        LocOFF_Alarm_time = int(f.readline())

        f.seek(0,0)
        f.write('{:0>20}'.format(str(LocOPC_parameters_instrument_SN)) +'\n')
        f.write('{:0>20}'.format(str(LocOPC_parameters_loraTx_Delay)) +'\n')

        for i in range(16):
            f.write('{:0>20}'.format(str(LocOPC_parameters_Counts_K[i])) +'\n')

        f.write('{:0>20}'.format(str(Alarm_Thresholds_1)) +'\n')
        print('Scrivo nuova soglia 1:',str(Alarm_Thresholds_1))
        f.write('{:0>20}'.format(str(Alarm_Thresholds_2)) +'\n')
        print('Scrivo nuova soglia 2:',str(Alarm_Thresholds_2))
        f.write('{:0>20}'.format(str(Alarm_Thresholds_3)) +'\n')
        print('Scrivo nuova soglia 3:',str(Alarm_Thresholds_3))

        f.write('{:0>20}'.format(str(LocON_Alarm_time)) +'\n')
        f.write('{:0>20}'.format(str(LocOFF_Alarm_time)) +'\n')
    except Exception as error:
        print('Errore in scrittura file n.', error)
        return False


    f.close()

    return True

#----------------------------------------------------------------
# OPC_Recv_Msg
#----------------------------------------------------------------
#def OPC_Recv_Msg(s,NumDevice,LocSogliaUp_gruppo_1_3,SogliaUp_gruppo_3_8,SogliaUp_gruppo_8_17,SogliaDown_gruppo_1_3,SogliaDown_gruppo_3_8,SogliaDown_gruppo_8_17):
def OPC_Recv_Msg(s,NumDevice):

    global SogliaUp_gruppo_1_3
    global SogliaUp_gruppo_3_8
    global SogliaUp_gruppo_8_17
    global SogliaDown_gruppo_1_3
    global SogliaDown_gruppo_3_8
    global SogliaDown_gruppo_8_17
    global OPC_parameters_Counts_K

    data = bytearray(128)
    #data[0] = 1
    #print('data bytes n. ', len(data))
    for i in range(0, 3):


        time.sleep_ms(3000)
        data = s.recv(64)
        time.sleep_ms(1000)
        print('Received data from server', data)
        if(len(data) > 0):
            break

    if(len(data) > 0):
        print('Received bytes n. ', len(data))
        print('ID: ', data[0])
        for i in range(0,len(data)):
            print('byte ', i , ': ', hex(data[i]))


        if(data[0] == 202):#0xCA):    #Threshold set
            #recived command write new thresholds

            ReceivedThreshold_1 = int(data[1]*256 + data[2])
            ReceivedThreshold_2 = int(data[3]*256 + data[4])
            ReceivedThreshold_3 = int(data[5]*256 + data[6])
            #OPC_Cmd_ThresholdsSet(data,s,NumDevice)
            #SogliaUp_gruppo_1_3,SogliaUp_gruppo_3_8,SogliaUp_gruppo_8_17 = OPC_Cmd_ThresholdsSet(ReceivedThreshold_1,ReceivedThreshold_2,ReceivedThreshold_3,s,NumDevice)
            OPC_Cmd_ThresholdsSet(ReceivedThreshold_1,ReceivedThreshold_2,ReceivedThreshold_3,s,NumDevice)

        if(data[0] == 0x51):    # Threshold read
            #recived command read thresholds
            print('Command received: Threasolds Read')
            OPC_Send_Thresholds(s,Device_Info,ReceivedThreshold_1,ReceivedThreshold_2,ReceivedThreshold_3)
            #data1 = 0x52 + default_parameters_threshold_K
            #data1 = '52' + ',' + SogliaUp_gruppo_1_3[:-1] +  ',' + SogliaUp_gruppo_3_8[:-1] + ',' +  SogliaUp_gruppo_8_17[:-1]
            #print('Data sent: ',data1)
            #OPC_Send_Thresholds(s)
            #s.send(data1)
        if(data[0] == 0xCC):    # Channels K factor set
            print('Command received: Channels K factor Set')
            kFactor_read = array.array('i',[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
            for i in range(0,16):
                kFactor_read[i] = data[(2*(i+1)-1)] * 256 + data[2*(i+1)]
                #pippo =int(data[(2*(i+1)-1)]*256 + data[(2*(i+1)])
                print('k: ', kFactor_read[i])
            OPC_Cmd_Counts_KFactor_Set(s,NumDevice,kFactor_read)

        if(data[0] == 0x52): #Channels K factor read
            print('Command received: Channels K factor read')
            OPC_Send_KFactor(s,Device_Info,OPC_parameters_Counts_K)

    else:
        print('No Msg received')

    #return SogliaUp_gruppo_1_3,SogliaUp_gruppo_3_8,SogliaUp_gruppo_8_17
    return
#--------------------------------------------------------------------
#
#   OPC_Cmd_Counts_KFactor_Set:
#   Set KFactor for each channel received
#--------------------------------------------------------------------
def OPC_Cmd_Counts_KFactor_Set(s,Device_Info,skFactor_read):
    global OPC_parameters_Counts_K

    print('Threshold set before: ', OPC_parameters_Counts_K)
    print('Command received: Threasolds Set')
    for i in range (0,16):
        print('Threshold set before: ', OPC_parameters_Counts_K[i])
        if (skFactor_read[i] != 65535):
            OPC_parameters_Counts_K[i] = skFactor_read[i]
    print('Threshold set after: ', OPC_parameters_Counts_K)

    OPC_Send_KFactor(s,Device_Info,OPC_parameters_Counts_K)

    return True

#----------------------------------------------------------------
# OPC_Send_KFactor
#----------------------------------------------------------------

def OPC_Send_KFactor(s,Device_Info,skFactor_read):


    print(' Invio K Factor:')
    for i in range(0,16):
        print('{}'.format('K '+str(i)+ ' : '+ str(skFactor_read[i])))


    StartMg = bytearray('N,'+ str(Device_Info) + ',K,')
    SetKFactorAnswer = bytearray (39)
    SetKFactorAnswer[0] = StartMg[0]
    SetKFactorAnswer[1] = StartMg[1]
    SetKFactorAnswer[2] = StartMg[2]
    SetKFactorAnswer[3] = StartMg[3]
    SetKFactorAnswer[4] = StartMg[4]
    SetKFactorAnswer[5] = StartMg[5]
    SetKFactorAnswer[6] = StartMg[6]
    for i in range(0,16):
        SetKFactorAnswer[7+i*2] = int(skFactor_read[i]//256)
        SetKFactorAnswer[8+i*2] = int(skFactor_read[i]%256)

    for i in range(0,38):
        print('Kfator {}'.format(str(i)+' ' + str(hex(SetKFactorAnswer[i*2+1]))+' '+str(hex(SetKFactorAnswer[i*2]))))


    print('stringa K Factor inviata: ', SetKFactorAnswer)
    s.send(SetKFactorAnswer)
    return True

#--------------------------------------------------------------
# Update Alarm Thresholds
# return True if SD write is ok
# return FALSE IF sd WRITE IS NOT OK
#--------------------------------------------------------------
def OPC_Config_file_KFactor_update(OPC_parameters_Counts_K_To_Set):


    LocOPC_parameters_Counts_K = array.array('i',[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1])


    try:
        f = open('/sd/OPC_Config.txt', 'a+')
        print('Apertura file per cambio soglie')
        f.seek(0,0)
        LocOPC_parameters_instrument_SN = int(f.readline())
        print('SN ', OPC_parameters_instrument_SN)
        #minutoinvio = f.readline()
        LocOPC_parameters_loraTx_Delay = int(f.readline())
        for i in range(0,16):
            LocOPC_parameters_Counts_K[i] = int(f.readline())
        LocSogliaUp_gruppo_1_3 = int(f.readline())
        LocSogliaDown_gruppo_1_3 = int(SogliaUp_gruppo_1_3 * 0.7)
        LocSogliaUp_gruppo_3_8 = int(f.readline())
        LocSogliaDown_gruppo_3_8 = int(SogliaUp_gruppo_3_8 * 0.7)
        LocSogliaUp_gruppo_8_17 = int(f.readline())
        LocSogliaDown_gruppo_8_17 = int(SogliaUp_gruppo_8_17 * 0.7)
        LocON_Alarm_time = int(f.readline())
        LocOFF_Alarm_time = int(f.readline())

        # Write new parameters and old untouched ones
        f.seek(0,0)
        f.write('{:0>20}'.format(str(LocOPC_parameters_instrument_SN)) +'\n')
        f.write('{:0>20}'.format(str(LocOPC_parameters_loraTx_Delay)) +'\n')

        for i in range(16):
            f.write('{:0>20}'.format(str(OPC_parameters_Counts_K_To_Set[i])) +'\n')

        f.write('{:0>20}'.format(str(LocSogliaUp_gruppo_1_3)) +'\n')
        f.write('{:0>20}'.format(str(LocSogliaUp_gruppo_3_8)) +'\n')
        f.write('{:0>20}'.format(str(LocSogliaUp_gruppo_8_17)) +'\n')

        f.write('{:0>20}'.format(str(LocON_Alarm_time)) +'\n')
        f.write('{:0>20}'.format(str(LocOFF_Alarm_time)) +'\n')
    except Exception as error:
        print('Errore in scrittura file n.', error)
        return False


    f.close()

    return True


def stampaSoglia():
    global SogliaUp_gruppo_1_3
    print('Soglia da funzione',SogliaUp_gruppo_1_3)
    SogliaUp_gruppo_1_3 = 2000
