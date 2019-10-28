'''
***********************************
     OPCN2LW04OTA
***********************************
******************************************************************************************************************
SINTESI DELLE OPERAZIONI


Leggo da OPC istogramma (conteggi dei 16 canali) e
- Ogni minuto scrivo su SD i conteggi dei 16 canali

- Ogmi minuto controllo i livelli di 3 raggruppamenti di particelle e se almeno uno supera soglia allarme per un certo tempo
   prefissato genero allarme, durante l'allarme invio su LoRa messaggio  N,XX(che è il nome dello strumento), A, livelli dei 3 raggruppamenti;
   se invece ero già in allarme ma tutti e tre i gruppi sono rientrati sotto le soglie di rientro per un certo tempo prefissato chiudo l'allarme
   e invio messaggio  N,XX(che è il nome dello strumento), R, livelli dei 3 raggruppamenti
- Ogni ora invio su Lorawan i livelli medi orari dei tre gruppi (con ridondanza ovvero con invio anche dei livelli dell'ora precedente)
   con messaggio N,XX(che è il nome dello strumento), M, livelli dei 3 raggruppamenti dell'ora appena finita, livelli dei 3 raggruppamenti
   dell'ora precedente

impostazioni:
1 soglie di allarme per i tre gruppi
3 soglie di rientro da allarme per i tre gruppi
3 durata del tempo di superamento soglie necessaria per far scattare l'allarme
4 durata del tempo sotto soglie di rientro necessaria per far cessare l'allarme
5 data/ora corrente
6 chiavi per collegamento a rete LoRa
7 nome del dispositivo
8 minuto di ogni ora in cui vengono inviati i dati su LoRa
9 versione SW
le impostazioni 1,2,3,4,7 sono fornite tramite sd (file: OPC_Config.txt)
l'impostazione 5 è fornita da gps e se non è possibile collegarsi al gps da valori di default
impostazioni 6,8,9 sono scritte nel codice




********************************************************************************************************************

'''

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
import sys

from machine import I2C
from machine import RTC
from machine import SD
from machine import Timer
from machine import WDT #Ferrera

#from LIS2HH12 import LIS2HH12
from L76GNSV4 import L76GNSS
from pytrack import Pytrack

from machine import SPI
from machine import Pin
from machine import RTC

from network import LoRa, WLAN
import socket
import binascii
import time
from ServiceLib import *
import ServiceLib
#from ServiceLib import LORAConnection_OOTA
#from ServiceLib import OPC_Config_file_Init
#from ServiceLib import GPSInit
#from ServiceLib import GPSGetCoord
#from ServiceLib import GPSGetTime
#from ServiceLib import SetLed
#from ServiceLib import Clock

#versionesw ='OPCN2LW05OTA'
versionesw ='OPCN2LW-OTA-05.08'

rtc_pending = False
lora_ota = True #modalità LoRa OTAA
lora_control = False    #Modalità LoRa ABP

wan_control = False
spi_control    = True


sd_presence   = True
sd_write_conf = True
wdt_enabled = True
wdt_timeout = 300000 #WTD timeoyut 5 minuti

intervallo_campionamento = 4 # ogni 4 secondi sarà interrogato OPCN2 il quale risponderà con i conteggi di particelle accumulati nei
                             # 4 secondi per ogni canale e poi azzererà i conteggi
warning_flow_low_limit = int(100)
ultimo_giorno = 'vuoto'
latitudine = 0
longitudine = 0
stringaHMprecedente = 'vuoto'
a_Minimum_Integration_Time = 23 # durata ciclo aquisizioni all'interno di un minuto: all'interno di un minuto faccio acquisizioni ogni 4 secondi
                                # fino al 24-esimo secondo poi faccio altro ma di fatto acquisisco tutti i conteggi dell'OPC perchè
                                # OPC azzera conteggi solo dopo ogni acquisizione quindi la prima acquisizione di ogni ciclo tiene conto
                                # dei conteggi dal 24-esimo secondo del minuto precedente
ServerUpdateParmetersMsgCounter = 0
#--------------------------------------
#Variabili di configurazione attuale
#--------------------------------------
LoraConnected = False
GPSFound = False
OPCConnected = False
SDCardOk = False

#---------------------------------------
# Costanti di sistema
#---------------------------------------
#da aggiungere

# SogliaUp_gruppo_1_3 = 7000 # soglia massima per il numero di particelle del gruppo_1_3, oltre si va in allarme
# SogliaUp_gruppo_3_8 = 2000 # soglia massima per il numero di particelle del gruppo_3_8, oltre si va in allarme
# SogliaUp_gruppo_8_17 = 100 # soglia massima per il numero di particelle del gruppo_1_3, oltre si va in allarme

# SogliaDown_gruppo_1_3 = 6000 # soglia di rientro da allarme per il nmero di particelle del gruppo_1_3, sotto questa soglia si rientra da allarme
# SogliaDown_gruppo_3_8 = 1500 # soglia di rientro da allarme per il nmero di particelle del gruppo_3_8, sotto questa soglia si rientra da allarme
# SogliaDown_gruppo_8_17 = 50 # soglia di rientro da allarme per il nmero di particelle del gruppo_8_17, sotto questa soglia si rientra da allarme
#default_parameters_instrument_SN = 10
OPC_parameters_instrument_SN = 18
Device_Info = ''
#default_parameters_threshold_K = array.array('f',[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1])
#default_parameters_threshold_K = array.array('i',[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1])
# OPC_parameters_Counts_K = array.array('i',[1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000])
# default_parameters_Counts_K = array.array('i',[1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000])
# SogliaUp_gruppo_1_3 = 0
# SogliaUp_gruppo_3_8 = 0
# SogliaUp_gruppo_8_17 = 0
# SogliaDown_gruppo_1_3 = 0
# SogliaDown_gruppo_3_8 = 0
# SogliaDown_gruppo_8_17 = 0
global SogliaUp_gruppo_1_3
global SogliaUp_gruppo_3_8
global SogliaUp_gruppo_8_17
global SogliaDown_gruppo_1_3
global SogliaDown_gruppo_3_8
global SogliaDown_gruppo_8_17
global OPC_parameters_Counts_K
#default_parameters_threshold_K =[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
#default_parameters_threshold_K =['1','1','1','1','1','1','1','1','1','1','1','1','1','1','1','1']
superamenti_1_3 = 0 # numero dei superamenti soglia Up per il gruppo 1_3
superamenti_3_8 = 0
superamenti_8_17 = 0

rientri_1_3 = 0 # numero dei rientri sotto soglia Down per il gruppo 1_3
rientri_3_8 = 0
rientri_8_17 = 0

alarm_1_3 = False  # allarme generato per troppi superamenti consecutivi (o annullato a seguito di altrettanti rientri consecutivi sotto soglia)
alarm_3_8 = False
alarm_8_17 = False

stringaHMprecedente = 'NN,NN,NN'
stringawarning = '00'
byteWarning = bytearray(1)
warning_flow_low_limit_bit = int(1)
warning_SD_Not_Mounted_bit = int(2)
tot_minuti = 0
tot_oraincorso_gruppo_1_3 = 0 # in questa variabile accumulo ogni minuto i conteggi dell'ora in corso per questa fascia di particelle
tot_oraincorso_gruppo_3_8 = 0
tot_oraincorso_gruppo_8_17 = 0


ThresoldAlarmOffAttempts = 0 # numebr of attempts to be done to send alarm switch off message
#py = Pytrack()
#l76 = L76GNSS(py, timeout=5)

pycom.heartbeat(False)

#set timer for led managing
clock = Clock()
#pycom.rgbled(0x00007f) #blu
#pycom.rgbled(0x0000ff) #blu
#pycom.rgbled(0x00000f) #Ferrera meno forte blu
SetLed(LED_Blu_steady)


#pycom.rgbled(0x007f00) #verde
#pycom.rgbled(0x000000) #off

if wdt_enabled == True:
    wdt = WDT(timeout=wdt_timeout)  # enable WDT with a timeout

print('')
print('*************************')
print('VERSIONE SW ', versionesw )
print('*************************')
print('')
print('Soglia 1',ServiceLib.SogliaUp_gruppo_1_3)
print('K',ServiceLib.OPC_parameters_Counts_K)
#--------------------------------------------------------------
# Initialize SPI and first command to opc (to obtain no stand alone mode)
#--------------------------------------------------------------T
if spi_control == True:
    try:
        #pycom.rgbled(0x000f00)
        SetLed(LED_Blu_blinking)
        #  SPI non-default pins for CLK, MOSI and MISO (``P9``, ``P10`` and ``P11``) SPI_SS = Pin('G23')
        spi = SPI(0, mode=SPI.MASTER, baudrate=500000, polarity=0, phase=1, pins=('P9', 'P10', 'P11'),  firstbit=SPI.MSB)
        time.sleep_ms(1000)
        # Read & Print Firmware Version
        #pycom.rgbled(0x007f00) #verde
        #pycom.rgbled(0x0000ff) #blu
        #SPI_SS = Pin('G23', mode=Pin.OUT, value=1)
        #Ferrera Modifica pin per evitare conflitto tra SPI e LED RGB
        #SPI_SS = Pin('G6', mode=Pin.OUT, value=1)
        SPI_SS = Pin('P19', mode=Pin.OUT, value=1)
        SPI_SS(1)
        print("Firmware Version from device #")




        tmp = bytearray(1)
        rbuf = bytearray(60) # dobbiamo inserire i 60 campi della descrizione sw OPCN2

        SPI_SS(0)
        spi.write_readinto(bytes([0x3F]), tmp)
        SPI_SS(1)
        time.sleep_ms(10)

        for i in range(0, 60): # la funzione range non utilizza il numero superiore (60 in questo caso) noi dobbiamo leggere 60 campi (da 0 a 59)
            SPI_SS(0)
            spi.write_readinto(bytes([0x3F]), tmp)
            SPI_SS(1)
            time.sleep_ms(1)
            rbuf[i] = tmp[0]


        print(rbuf)
        time.sleep_ms(1000)
    except (ValueError, TypeError):
        print (" Eccezione su prima comunicazione OPC Errore valore: versione sw non disponibile")
        OPCConnected = False
        #pycom.rgbled(0xFF0000)
        SetLed(LED_Red_blinking)
        time.sleep_ms(3000)
        sys.exit('OPC non riconosciuto')
    else:
        # immettere qui un controllo di correttezza semantica della risposta in modo da essere sicuri che il dialogo sia ok
        if('OPC'in rbuf ):
            OPCConnected = True
            print('riconosciuta scritta OPC in risposta')
        else:
            OPCConnected = False
            print('non riconosciuta scritta OPC in risposta')
            #pycom.rgbled(0xFF0000)
            SetLed(LED_Red_blinking)
            time.sleep_ms(5000)
            sys.exit('OPC non riconosciuto')

#--------------------------------------------------------------
# Initialize GPS
#--------------------------------------------------------------
#pycom.rgbled(0x0f0f00)
SetLed(LED_Yellow_blinking)
coord,data,GPSFound=GPSInit(20)   ##20
if(GPSFound):
    print('GPS Found: ',coord,data)
    SetLed(LED_Yellow_steady)
    time.sleep_ms(2000)

else:
    print('GPS Not Found:')
    data = [2019,09,12,12,00,00]
    print('GPS Not Found:default data: ', data)
#--------------------------------------------------------------
# Initialize RTC WITH ACTUAL RUNNING TIME
#--------------------------------------------------------------

rtc = RTC(id=0)
dataLocal = data[0],data[1],data[2],data[3]+2,data[4],data[5]
print(dataLocal)
rtc.init(datetime=dataLocal)
start_time=(2018,1,18,12,53,None,None,None)
print(rtc.now())

#led = pycom.heartbeat(False)


#--------------------------------------------------------------
# Initialize SD Operations
#--------------------------------------------------------------
if sd_presence == True:
    #pycom.rgbled(0x0f0f0f)
    print('SD card write operations')
    try:
        sd = SD()
        # 2 attempt to mount SD, if fails, set warning 2 and work without write on SD Card

        os.mount(sd, '/sd')
        SD_Mounted = True
    except Exception as error:
        print('Errore SD Mount n.', error)
        SD_Mounted = False
    if(SD_Mounted == False):
        try:
            os.mount(sd, '/sd')
            SD_Mounted = True
        except Exception as error:
            print('Errore SD Mount n.', error)
            SD_Mounted = False

    if(SD_Mounted == False):
        sd_presence = False
        byteWarning[0] = warning_SD_Not_Mounted_bit | int(byteWarning[0])
    else:
        sd_presence = True
        byteWarning[0] = ~warning_SD_Not_Mounted_bit & int(byteWarning[0])

    # check the content
    # print('DISK Directory Content: ', os.listdir('/sd'))
if(sd_presence == True):
    if sd_write_conf:
        # se il file esiste bisogna solo leggere la configurazione scritta, altrimenti imposta il default
        Device_Info = ''
        try:
            f = open('/sd/OPC_Config.txt', 'r')
            Device_Info = f.readline()
            byteWarning[0] = ~warning_SD_Not_Mounted_bit & int(byteWarning[0])
            f.close()
        except Exception as error:
            print('Errore apertura file n.', error)
            byteWarning[0] = warning_SD_Not_Mounted_bit | int(byteWarning[0])

        if (Device_Info == ''):
            #file non presente: scrivo un file con i parametri di default
            OPC_Config_file_Init()
    try:
        f = open('/sd/OPC_Config.txt', 'r+')
        print('apertura file config.txt')
        Device_Info = int(f.readline())
        #print('Dev Info',Device_Info)
        #minutoinvio = f.readline()
        OPC_parameters_loraTx_Delay = int(f.readline())
        #print('Tx Delay:',OPC_parameters_loraTx_Delay)
        for i in range(0,16):
        #for element in OPC_parameters_Counts_K:
            ServiceLib.OPC_parameters_Counts_K[i] = int(f.readline())
            #element = int(f.readline())
            #print(element)
            #print(OPC_parameters_Counts_K[i])
        # SogliaUp_gruppo_1_3 = int(f.readline())
        # SogliaDown_gruppo_1_3 = int(SogliaUp_gruppo_1_3 * 0.7)
        # SogliaUp_gruppo_3_8 = int(f.readline())
        # SogliaDown_gruppo_3_8 = int(SogliaUp_gruppo_3_8 * 0.7)
        # SogliaUp_gruppo_8_17 = int(f.readline())
        # SogliaDown_gruppo_8_17 = int(SogliaUp_gruppo_8_17 * 0.7)
        ServiceLib.SogliaUp_gruppo_1_3 = int(f.readline())
        ServiceLib.SogliaDown_gruppo_1_3 = int(ServiceLib.SogliaUp_gruppo_1_3 * 0.7)
        ServiceLib.SogliaUp_gruppo_3_8 = int(f.readline())
        ServiceLib.SogliaDown_gruppo_3_8 = int(ServiceLib.SogliaUp_gruppo_3_8 * 0.7)
        ServiceLib.SogliaUp_gruppo_8_17 = int(f.readline())
        ServiceLib.SogliaDown_gruppo_8_17 = int(ServiceLib.SogliaUp_gruppo_8_17 * 0.7)
        ON_Alarm_time = int(f.readline())
        OFF_Alarm_time = int(f.readline())
        byteWarning[0] = ~warning_SD_Not_Mounted_bit & int(byteWarning[0])
    except Exception as error:
        print('Errore apertura file n.', error)
        byteWarning[0] = warning_SD_Not_Mounted_bit | int(byteWarning[0])

else :
    # TODO:     FARE PIU' ROBUSTO ED ELEGANTE
    #Imposto i default se non posso eggere l'SD
    Device_Info = OPC_parameters_instrument_SN
    #minutoinvio = '2' + '\n'
    OPC_parameters_loraTx_Delay = int((Device_Info -10) * 2)
    #for element in OPC_parameters_Counts_K:
    for i in range (0,16):
        ServiceLib.OPC_parameters_Counts_K[i] = 1000
    ServiceLib.SogliaUp_gruppo_1_3 = '5000' + '\n'
    ServiceLib.SogliaDown_gruppo_1_3 = '3000' + '\n'
    ServiceLib.SogliaUp_gruppo_3_8 = '1000' + '\n'
    ServiceLib.SogliaDown_gruppo_3_8 = '500' + '\n'
    ServiceLib.SogliaUp_gruppo_8_17 = '200' + '\n'
    ServiceLib.SogliaDown_gruppo_8_17 = '100' + '\n'
    ON_Alarm_time = '2' + '\n'
    OFF_Alarm_time = '2' + '\n'


NumDevice = int(Device_Info)
print('Device Configuration:')
print('Device_Info = ', NumDevice)
#print('minutoinvio = ', minutoinvio)
print('TxDelay = ', OPC_parameters_loraTx_Delay)
for i in range(16):
    print('threshold_K ', i, '= ',ServiceLib.OPC_parameters_Counts_K[i])
print('SogliaUp_gruppo_1_3 = ', ServiceLib.SogliaUp_gruppo_1_3)
print('SogliaDown_gruppo_1_3 = ', ServiceLib.SogliaDown_gruppo_1_3)
print('SogliaUp_gruppo_3_8 = ', ServiceLib.SogliaUp_gruppo_3_8)
print('SogliaDown_gruppo_3_8 = ', ServiceLib.SogliaDown_gruppo_3_8)
print('SogliaUp_gruppo_8_17 = ', ServiceLib.SogliaUp_gruppo_8_17)
print('SogliaDown_gruppo_8_17 = ', ServiceLib.SogliaDown_gruppo_8_17)
print('ON_Alarm_time = ',  ON_Alarm_time)
print('OFF_Alarm_time = ',  OFF_Alarm_time)


#--------------------------------------------------------------
# Initialize WiFi.
#--------------------------------------------------------------
if wan_control == True:
    pycom.rgbled(0xffffff)
    wlan = network.WLAN(mode=network.WLAN.STA)
    wlan.connect('AndroidAP7092', auth=(network.WLAN.WPA2, 'fwzc8494'))
    i=0
    while i < 20:
        i = i + 1
        if wlan.isconnected() == False :
            print("Finding Default WIFI: Not connected yet")
            time.sleep_ms(500)
        else: break

    if i < 20:
        print("Connected!")
    else :
        print("No default WIFI to get connected")
else:
    # WLAN OFF
    wlan = WLAN() # change********************************************************
    wlan.deinit() # change***********spengo wifi e risparmio 70mA********************************************



if wan_control == True and wlan.isconnected() == True:
    # setup RTC
    rtc = machine.RTC()
    rtc.ntp_sync("pool.ntp.org")
    utime.sleep_ms(750)
    print('\nRTC Set from NTP to UTC:', rtc.now())
    utime.timezone(7200)
    print('Adjusted from UTC to EST timezone', utime.localtime(), '\n')


#--------------------------------------------------------------
# Initialize LoRa
#--------------------------------------------------------------

# ***************  IN QUESTA APPLICAZIONE NON USIAMO LA SEGUENTE MODALITA' ABP ******************
## LORAWAN mode.
if lora_control == True:
    lora = LoRa(mode=LoRa.LORAWAN, frequency=868000000,
                public=True, sf=7, tx_power=14)
    lora.BW_125KHZ
    lora.CODING_4_5
    print('potenza 14')
    print("LoRa MAC Address:  ", binascii.hexlify(
        lora.mac()).upper().decode('utf-8'))
    print("LoRa EUI: ", binascii.hexlify(machine.unique_id()))

    # create an ABP authentication params
    dev_addr = struct.unpack(">l", binascii.unhexlify(
        'C0 7C 73 07'.replace(' ', '')))[0]
    nwk_swkey = binascii.unhexlify(
        'AA 12 12 C1 D4 BF 60 8E 36 60 8E CF 9D FD 57 6E'.replace(' ', ''))
    app_swkey = binascii.unhexlify(
        '0F DF 6A 5F B3 1A A6 0F 7A 3B 98 C0 65 EF 4D EA'.replace(' ', ''))

    # join a network using ABP (Activation By Personalization)
    if(lora.has_joined() == False):
        print("LoRa not Joined")
    else:
        print("LoRa Joined")
    i =0
    while(lora.has_joined() == False):
        lora.join(activation=LoRa.ABP, auth=(dev_addr, nwk_swkey, app_swkey))
        print("LoRa not Joined")
        i=i+1
        utime.sleep_ms(1000)
        if (i>5):
            print("Timeout")
            break
    if(lora.has_joined() == False):
        print("Definitely:LoRa not Joined")
    else:
        print("Definitely:LoRa Joined")
    # create a LoRa socket
    s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
    # set the LoRaWAN data rate
    s.setsockopt(socket.SOL_LORA, socket.SO_DR, 5)
    # make the socket non-blocking
    s.setblocking(False)
    print("LoRa MAC Address:  ", binascii.hexlify(
        lora.mac()).upper().decode('utf-8'))
    print("LoRa EUI: ", binascii.hexlify(machine.unique_id()))
# *******************************************************************************************

#************************************************************************************************
#***************  IN QUESTA APPLICAZIONE  USIAMO LA SEGUENTE MODALITA' OTA **********************
#************************************************************************************************
# LoRa LOTA mode
if lora_ota == True:
    #LoraConnected = LORAConnection_OOTA(10000)
    SetLed(LED_Green_blinking)
    LoraConnected, lora = LORAConnection_OOTA(0)
    print('Lora connected = ',LoraConnected)
    if(LoraConnected == False):
        for i in range(0, 60):
            if(lora.has_joined() == False):
                time.sleep_ms(5000)
                print('Not yet joined...',i)
            else:
                LoraConnected = True
                print('Joined!')
                break
    if(LoraConnected == True):
        SetLed(LED_Green_steady)
        # create a LoRa socket
        s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
        # set the LoRaWAN data rate
        s.setsockopt(socket.SOL_LORA, socket.SO_DR, 5)

#**********************************************************************************************


if rtc_pending == True:
    matching = 0
    while matching == 0:
        time.sleep_ms(1000)
        cur_time = rtc.now()
        curt = list(cur_time)
        curt[5] = None
        curt[6] = None
        curt[7] = None
        cur_time = list(curt)
        starter = list(start_time)
        print('CURRENT TIME: ', cur_time, '      PLANNED START TIME: ', starter)
        if cur_time == starter:
            print('START TIME MATCHED !.....Starting now')
            matching = 1
            break

if spi_control == True:
    #pycom.rgbled(0x007f00) #verde
    time.sleep_ms(1000)
    if True:
        print("Firmware Version from device #")
        tmp = bytearray(1)
        rbuf = bytearray(60) # dobbiamo inserire i 60 campi della descrizione sw OPCN2

        SPI_SS(0)
        spi.write_readinto(bytes([0x3F]), tmp) # comunico a OPCN2 che voglio leggere i campi descrizione sw
        SPI_SS(1)
        time.sleep_ms(10) # ritardo consigliato da alphasense
        for i in range(0, 60): # la funzione range non utilizza il numero superiore (60 in questo caso) noi dobbiamo leggere 60 campi (da 0 a 59) ]]]]]]]]]]]]
            SPI_SS(0)
            spi.write_readinto(bytes([0x3F]), tmp)
            SPI_SS(1)
            time.sleep_ms(1)
            rbuf[i] = tmp[0]
        print(rbuf)
        time.sleep_ms(1000)


    # Make Sure Laser & Fan are ON
    print("Powering Fan & Laser #")
    rbuf = bytearray(2)
    tmp = bytearray(1)
    SPI_SS(0)
    spi.write_readinto(bytes([0x03]), tmp)
    SPI_SS(1)
    time.sleep_ms(10)
    rbuf[0] = tmp[0]
    SPI_SS(0)
    spi.write_readinto(bytes([0x00]), tmp)
    SPI_SS(1)
    time.sleep_ms(50)
    rbuf[1] = tmp[0]
    print(rbuf)  # se rbuf[1]=x03 OPCN2 è partito (accensione laser e pompa)
    #time.sleep_ms(5000)
    time.sleep_ms(2000)




#****************************************************************************
#****************************************************************************
#
#                 M A I N          L O O P
#
#****************************************************************************
#****************************************************************************


    #pycom.rgbled(0x000f00)
    while True:
        if wdt_enabled == True:
            wdt.feed()
        print('riparte il main loop',  GPSGetCoord())
        print('OPC Threshold 1',ServiceLib.SogliaUp_gruppo_1_3)
        print('OPC K1',ServiceLib.OPC_parameters_Counts_K)
        Histogram_Data = array.array('I', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0]) # SEMBRA INUTILE AZZERARE AD OGNI CICLO
        AVG_Histogram_Data = array.array('I', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        MTOF_Data = array.array('I', [0, 0, 0, 0]) # SEMBRA INUTILE AZZERARE AD OGNI CICLO
        SFR_Data = 0. # SEMBRA INUTILE AZZERARE AD OGNI CICLO
        AVG_SFR_Data = 0.
        # TC_Data = 0. # # i sensori di temperatura e pressione sono presenti in OPCN2 solo a richiesta
        #AVG_TC_Data =0.
        #PA_Data = 0. # SEMBRA INUTILE AZZERARE AD OGNI CICLO
        #AVG_PA_Data = 0.
        SMP_Data = 0. # SEMBRA INUTILE AZZERARE AD OGNI CICLO
        Sampling_Time = 0.

        PM1_Data = 0.
        PM2_5_Data = 0.
        PM10_Data = 0.
        integr=int(a_Minimum_Integration_Time)
        print('**** Live Integration time ****', integr)
        cur_integr = 0
        iterazioni= 0

        while (cur_integr < integr): # ripeto questo ciclo per un periodo di tempo almeno uguale a quello prefissato (24 sec.)
            iterazioni=iterazioni + 1 # contatore dei cicli del while
            cur_integr=cur_integr + intervallo_campionamento # aggiorno il valore del periodo corrente
            time.sleep(intervallo_campionamento) # l'intervallo tra campionamenti è di 4 sec. quindi interrogo OPCN2 dopo aver atteso 4sec
                                 # trascorso l'intervallo tra campionamenti inizio a chiedere le misure a OPCN2
            rbuf = bytearray(80) # rbuf() è il buffer dovre verranno immagazzinati tutti i bytes letti da OPC ad ogni ciclo di campionamento
                                # gli elemnti del buffer conterranno
                                # da indice 1 a indice 32 l'istogramma ovvero il numero di particelle (2 bytes) per ognuna delle 16 classi
                                # da 33 a 36 MToF ovvero il tempo di volo medio delle particelle
                                # da 37 a 40 SFR il valor medio del flusso di aspirazione
                                # da 41 a 44 i valori di T e P (sensori normalmente non presenti)
                                # da 44 a 48 la durata del campionamento a cui si riferiscono le particelle contate
                                # da 49 a 50 il cheksum
                                # da 51 a 54 il valore di PM1
                                # da 55 a 58 il valore di PM2.5
                                # da 59 a 62 il valore di PM10
            temp = bytearray(1)
            flt_rbuf = bytearray(4)
            # i_rbuf = bytearray(4) # i sensori di temperatura e pressione sono presenti in OPCN2 solo a richiesta
            SPI_SS(0)
            spi.write(bytes([0x30]))
            SPI_SS(1)
            time.sleep_ms(10)
            SPI_SS(0)
            rbuf = spi.read(62,write=0x00) # leggo i 62 campi su descritti
            SPI_SS(1)
            print('### REAL TIME DATA ### Progress time [s]:', cur_integr)

            #pycom.rgbled(0x007f00) #verde
            time.sleep_ms(4)
            for i in range(0, 16):
                Histogram_Data[i] = rbuf[(i * 2) + 1] * 256 + rbuf[i*2]
                #print('Ch raw counts',Histogram_Data[i])
                #Histogram_Data[i] = math.trunc(Histogram_Data[i]*(ServiceLib.OPC_parameters_Counts_K[i]/1000))
            print('Histogram Data', Histogram_Data)

            for i in range(0, 4):
                MTOF_Data[i] = 0xFF & rbuf[32 + i ]
            print('MTOF of Bin1, Bin3, Bin5, Bin7 uS/3 ', MTOF_Data) # TEMPO DI VOLO DELLE PARTICELLE

            flt_rbuf[0]=rbuf[36]
            flt_rbuf[1]=rbuf[37]
            flt_rbuf[2]=rbuf[38]
            flt_rbuf[3]=rbuf[39]
            SFR_Data = struct.unpack('f', bytearray(flt_rbuf)) # ottengo una stringa con numero float tra parentesi
            SFR_Data_tronc = '%6.2f' % SFR_Data # ottengo una stringa senza parentesi e con numero con due soli decimali
            fSFR_Data= float(SFR_Data_tronc) # ora posso ottenere un numero float convertito dalla stringa
            print('Flow ml/sec', fSFR_Data)
            AVG_SFR_Data = AVG_SFR_Data + SFR_Data[0]


            #i_rbuf[0] = rbuf[40] # i sensori di temperatura e pressione sono presenti in OPCN2 solo a richiesta
            #i_rbuf[1] = rbuf[41]
            #i_rbuf[2] = rbuf[42]
            #i_rbuf[3] = rbuf[43]
            # TC_Data = struct.unpack('I', i_rbuf)
            # print('TC :', TC_Data)
            #AVG_TC_Data = AVG_TC_Data + TC_Data[0]

            flt_rbuf[0] = rbuf[44]
            flt_rbuf[1] = rbuf[45]
            flt_rbuf[2] = rbuf[46]
            flt_rbuf[3] = rbuf[47]
            SMP_Data = struct.unpack('f', flt_rbuf)
            SMP_Data_tronc = '%6.1f' % SMP_Data
            fSMP_Data= float(SMP_Data_tronc)
            print('Sampling Period sec.', fSMP_Data)
            Sampling_Time = Sampling_Time + SMP_Data[0]

            # non è stato usato il checksum

            #PM DATA
            flt_rbuf[0] = rbuf[50]
            flt_rbuf[1] = rbuf[51]
            flt_rbuf[2] = rbuf[52]
            flt_rbuf[3] = rbuf[53]
            PM1_Data = struct.unpack('f', flt_rbuf)
            #PM1_Data_tronc = '%8.0f' % PM1_Data
            PM1_Data_tronc = '%8d' % PM1_Data
            #sPM1= str(PM1_Data)
            iPM1= int(PM1_Data_tronc)
            print('PM1', iPM1)

            #print('PM1', PM1_Data_tronc)

            flt_rbuf[0] = rbuf[54]
            flt_rbuf[1] = rbuf[55]
            flt_rbuf[2] = rbuf[56]
            flt_rbuf[3] = rbuf[57]
            PM2_5_Data = struct.unpack('f', flt_rbuf)
            PM2_5_Data_tronc = '%8d' % PM2_5_Data
            iPM2_5= int(PM2_5_Data_tronc)
            print('PM2_5', iPM2_5)

            flt_rbuf[0] = rbuf[58]
            flt_rbuf[1] = rbuf[59]
            flt_rbuf[2] = rbuf[60]
            flt_rbuf[3] = rbuf[61]
            PM10_Data = struct.unpack('f', flt_rbuf)
            PM10_Data_tronc = '%8d' % PM10_Data
            iPM10= int(PM10_Data_tronc)
            print('PM10',iPM10)
            print('*')


            for i in range(0, 16):
                AVG_Histogram_Data[i] = AVG_Histogram_Data[i]+ Histogram_Data[i] # accumulo via via i valori dei conteggi prima di fare la media al minuto


        print('************** COMPLETATO IL CICLO   N°', tot_minuti + 1,'  ************************')
        #pycom.rgbled(0xFF00)
        curt = rtc.now()
        print('RTC Time:', curt)

        # Check if counts are relaiable
        # if not system will be RESET
        sum = 0
        for i in range(0, 16):
            sum = sum + AVG_Histogram_Data[i]
        if sum < 10:
            print('SYSTEM RESET for low counts: ', sum)
            SetLed(LED_Red_blinking)
            time.sleep_ms(5000)
            machine.reset()


        # Ferrera protection on div0 error
        if Sampling_Time !=0:
            print('Counts befor correction ',AVG_Histogram_Data)
            for i in range(0, 16):
                AVG_Histogram_Data[i] = int(AVG_Histogram_Data[i] * (60. /Sampling_Time)*(ServiceLib.OPC_parameters_Counts_K[i]/1000.0)) # conteggi riportati in medie al minuto
            print('Counts after correction ',AVG_Histogram_Data)
            print(' Sampling Time ',Sampling_Time)
        else:
            for i in range(0, 16):
                AVG_Histogram_Data[i] = int(0)
        print('AVERAGED Histogram :', AVG_Histogram_Data)
        AVG_SFR_Data = (AVG_SFR_Data / iterazioni)*60 # riporto il flusso medio da valori al secondo a valori al minuto
        print('AVERAGED Flow ml/min:', int(AVG_SFR_Data))
        #check flow for # WARNING:
        if(AVG_SFR_Data < warning_flow_low_limit):
            print('Warning flow low limit')
            byteWarning[0] = warning_flow_low_limit_bit | int(byteWarning[0])
            print(byteWarning)
        else:
            print('no Warning flow low limit')
            byteWarning[0] = ~warning_flow_low_limit_bit & int(byteWarning[0])
            print(byteWarning)
            #byteWarning[0] &= ~(warning_flow_low_limit_bit)
        # AVG_TC_Data = AVG_TC_Data * (4. / float(a_Minimum_Integration_Time))
        # print('AVERAGED Temperature:', AVG_TC_Data)
        # Sampling_Time = Sampling_Time * (4. / float(a_Minimum_Integration_Time))
        print('Cycle Sampling time sec:', int(Sampling_Time))

        # *******************************************************************************************************
        # *******************************************************************************************************

        ''' ***********************************
        analisi conteggi particelle per rilevare superamenti_1_3 dei limiti
        nella analisi non si considerano polveri di piccolo diametro
        si considerano 3 gruppi di particelle: il primo da 1u a 3u, il secondo da 3u a 8u, il terzo da 8u a 17u
        gruppo_1_3		4,5,6,7
        gruppo_3_8		8,9,10,11
        gruppo_8_17		12,13,14,15,16

            ***********************************
        '''
        # calcolo il numero di particelle al minuto per ogni gruppo
        gruppo_1_3 = AVG_Histogram_Data[3]+AVG_Histogram_Data[4]+AVG_Histogram_Data[5]+AVG_Histogram_Data[6]
        print('gruppo_1_3:', gruppo_1_3)
        gruppo_3_8 = AVG_Histogram_Data[7]+AVG_Histogram_Data[8]+AVG_Histogram_Data[9]+AVG_Histogram_Data[10]
        print('gruppo_3_8:', gruppo_3_8)
        gruppo_8_17 = AVG_Histogram_Data[11]+AVG_Histogram_Data[12]+AVG_Histogram_Data[13]+AVG_Histogram_Data[14]+AVG_Histogram_Data[15]
        print('gruppo_8_17:', gruppo_8_17)

        # confronto i livelli di particelle di ogni gruppo con i limiti MAX
        # confronto per il gruppo_1_3
        if  (gruppo_1_3) > int(ServiceLib.SogliaUp_gruppo_1_3) :
            superamenti_1_3 = superamenti_1_3 + 1
        else:
            superamenti_1_3 = superamenti_1_3 - 1
            if superamenti_1_3 < 0:
                superamenti_1_3 = 0
        if superamenti_1_3 > int(ON_Alarm_time):
            superamenti_1_3 = int(ON_Alarm_time)
            alarm_1_3 = True #attivo l'allarme
            print('***   A L L A R M E   gruppo_1_3  ***')
        print('superamenti_1_3 ',superamenti_1_3 )
        # confronto per il gruppo_3_8
        if  (gruppo_3_8) > int(ServiceLib.SogliaUp_gruppo_3_8) :
            superamenti_3_8 = superamenti_3_8 + 1
        else:
            superamenti_3_8 = superamenti_3_8 - 1
            if superamenti_3_8 < 0:
                superamenti_3_8 = 0
        if superamenti_3_8 > int(ON_Alarm_time):
            superamenti_3_8 = int(ON_Alarm_time)
            alarm_3_8 = True #attivo l'allarme
            print('***   A L L A R M E   gruppo_3_8  ***')
        print('superamenti_3_8 ',superamenti_3_8 )
        # confronto per il gruppo_8_17
        if  (gruppo_8_17) > int(ServiceLib.SogliaUp_gruppo_8_17) :
            superamenti_8_17 = superamenti_8_17 + 1
        else:
            superamenti_8_17 = superamenti_8_17 - 1
            if superamenti_8_17 < 0:
                superamenti_8_17 = 0
        if superamenti_8_17 > int(ON_Alarm_time):
            superamenti_8_17 = int(ON_Alarm_time)
            alarm_8_17 = True #attivo l'allarme
            print('***   A L L A R M E   gruppo_8_17  ***')
        print('superamenti_8_17 ',superamenti_8_17 )



        # gestisco gli allarmi
        if ((alarm_1_3 or alarm_3_8) or alarm_8_17) == True:  # se un allarme è attivo
            print('*************** ALLARME ****************************')

            # **************************************************************
            # invio su LoRa messaggio di allarme con le misure dei livelli di particelle dei tre gruppi
            # **************************************************************

            print('invio allarme e misure su rete LoRa')
            # Misure Allarme AM
            stringaAM=str(gruppo_1_3) + ","+ str(gruppo_3_8) + "," + str(gruppo_8_17)

            if lora_ota == True:
                if LoraConnected == False:
                    # provo a riconnettere
                    #LoraConnected = LORAConnection_OOTA(10000)
                    LoraConnected = LORAConnection_OOTA(0)
                    if(LoraConnected == False):
                        for i in range(0, 3):
                            if(lora.has_joined() == False):
                                time.sleep_ms(2000)
                                print('Not yet joined...')
                            else:
                                LoraConnected = True
                                print('Joined!')
                                break
                    if(LoraConnected == True):
                        # create a LoRa socket
                        s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
                        # set the LoRaWAN data rate
                        s.setsockopt(socket.SOL_LORA, socket.SO_DR, 5)

                #controllo se la connessione è riuscita, in caso positivo invio, altrimenti non faccio nulla (per ora)
                if LoraConnected == True:
                    s.setblocking(True)
                    print('Sending stringa AM...')
                    stringaAMtotale = 'N,'+ str(NumDevice) + ',A,' + stringaAM
                    print('stringa inviata: ', stringaAMtotale)
                    s.send(stringaAMtotale)
                    # make the socket non-blocking
                    # (because if there's no data received it will block forever...)
                    s.setblocking(False)
                    #receive msg, parse and react
                    #OPC_Recv_Msg(s,NumDevice,SogliaUp_gruppo_1_3,SogliaUp_gruppo_3_8,SogliaUp_gruppo_8_17,SogliaDown_gruppo_1_3,SogliaDown_gruppo_3_8,SogliaDown_gruppo_8_17)
                    OPC_Recv_Msg(s,NumDevice)
                    # set numeber of attempts to send the future alarm off message
                    ThresoldAlarmOffAttempts = 3

            # ------------------------------------------------------------------
            # poi attivo tutti gli allarmi indipendentemente dal gruppo che l'ha generato
            alarm_1_3 = True
            alarm_3_8 = True
            alarm_8_17 = True
            # così per rientrare da allarme devono scendere sotto le soglie minime i livelli di tutti i gruppi
            # verifico se ci sono le condizioni per rientrare da allarme
            # verifica per il gruppo_1_3
            if  (gruppo_1_3) < int(ServiceLib.SogliaDown_gruppo_1_3) :
                rientri_1_3 = rientri_1_3 + 1
            else:
                rientri_1_3 = rientri_1_3 - 1
                if rientri_1_3 < 0:
                    rientri_1_3 = 0
            if rientri_1_3 > int(ON_Alarm_time):
                rientri_1_3 = int(OFF_Alarm_time)
                alarm_1_3 = False #annullo la richiesta d'allarme per questo gruppo
                print('***  ANNULLATO  A L L A R M E   gruppo_1_3  ***')
            print('rientri_1_3 ',rientri_1_3 )
            # verifica per il gruppo_3_8
            if  (gruppo_3_8) < int(ServiceLib.SogliaDown_gruppo_3_8) :
                rientri_3_8 = rientri_3_8 + 1
            else:
                rientri_3_8 = rientri_3_8 - 1
                if rientri_3_8 < 0:
                    rientri_3_8 = 0
            if rientri_3_8 > int(ON_Alarm_time):
                rientri_3_8 = int(OFF_Alarm_time)
                alarm_3_8 = False #annullo la richiesta d'allarme per questo gruppo
                print('***  ANNULLATO  A L L A R M E   gruppo_3_8  ***')
            print('rientri_3_8 ',rientri_3_8 )
            # verifica per il gruppo_8_17
            if  (gruppo_8_17) < int(ServiceLib.SogliaDown_gruppo_8_17) :
                rientri_8_17 = rientri_8_17 + 1
            else:
                rientri_8_17 = rientri_8_17 - 1
                if rientri_8_17 < 0:
                    rientri_8_17 = 0
            if rientri_8_17 > int(ON_Alarm_time):
                rientri_8_17 = int(OFF_Alarm_time)
                alarm_8_17 = False #annullo la richiesta d'allarme per questo gruppo
                print('***  ANNULLATO  A L L A R M E   gruppo_8_17  ***')
            print('rientri_8_17 ',rientri_8_17 )
            if ((alarm_1_3 or alarm_3_8) or alarm_8_17) == False:  # se tutti gli allarmi sono rientrati
                print('***  RIENTRO DA TUTTI GLI  ALLARMI  ***')
                # non appena rientrano tutti gli allarmi invio su rete LoRa messaggio di Rientro Allarme
                # **************************************************************
                # invio su LoRa messaggio di RIENTRO da allarme con le misure dei livelli di particelle dei tre gruppi
                # **************************************************************
                print('invio RIENTRO da allarme e misure su rete LoRa')
                # Misure Rientro RM
                stringaRM=str(gruppo_1_3) + ',' + str(gruppo_3_8) + ',' + str(gruppo_8_17)

                if lora_ota == True:
                    if LoraConnected == False:
                        # provo a riconnettere
                        #LoraConnected = LORAConnection_OOTA(10000)
                        LoraConnected, lora = LORAConnection_OOTA(0)
                        if(LoraConnected == False):
                            for i in range(0, 5):
                                if(lora.has_joined() == False):
                                    time.sleep_ms(5000)
                                    print('Not yet joined...')
                                else:
                                    LoraConnected = True
                                    print('Joined!')
                                    break

                    #controllo se la connessione è riuscita, in caso positivo invio, altrimenti non faccio nulla (per ora)
                    if LoraConnected == True:
                        s.setblocking(True)
                        print('Sending stringa RM...')
                        stringaRMtotale = 'N,'+ str(NumDevice) + ',R,' + stringaRM
                        print('stringa inviata: ', stringaRMtotale)
                        s.send(stringaRMtotale)
                        # make the socket non-blocking
                        # (because if there's no data received it will block forever...)
                        s.setblocking(False)
                        #receive msg, parse and react
                        #OPC_Recv_Msg(s,NumDevice,ServiceLib.SogliaUp_gruppo_1_3,ServiceLib.SogliaUp_gruppo_3_8,ServiceLib.SogliaUp_gruppo_8_17,ServiceLib.SogliaDown_gruppo_1_3,ServiceLib.SogliaDown_gruppo_3_8,ServiceLib.SogliaDown_gruppo_8_17)
                        OPC_Recv_Msg(s,NumDevice)
        # Send again the message off Alarm Off for other 2 times
        if((((alarm_1_3 or alarm_3_8) or alarm_8_17) == False) and (ThresoldAlarmOffAttempts > 0)):
            print('Tentativi successivi invio sg rientro da allarme')
            stringaRM=str(gruppo_1_3) + ',' + str(gruppo_3_8) + ',' + str(gruppo_8_17)
            if lora_ota == True:
                if LoraConnected == False:
                    # provo a riconnettere
                    #LoraConnected = LORAConnection_OOTA(10000)
                    LoraConnected, lora = LORAConnection_OOTA(0)
                    if(LoraConnected == False):
                        for i in range(0, 5):
                            if(lora.has_joined() == False):
                                time.sleep_ms(5000)
                                print('Not yet joined...')
                            else:
                                LoraConnected = True
                                print('Joined!')
                                break

                #controllo se la connessione è riuscita, in caso positivo invio, altrimenti non faccio nulla (per ora)
                if LoraConnected == True:
                    s.setblocking(True)
                    print('Sending stringa RM...',ThresoldAlarmOffAttempts)
                    stringaRMtotale = 'N,'+ str(NumDevice) + ',R,' + stringaRM
                    print('stringa inviata: ', stringaRMtotale)
                    s.send(stringaRMtotale)
                    # make the socket non-blocking
                    # (because if there's no data received it will block forever...)
                    s.setblocking(False)
                    #receive msg, parse and react
                    #OPC_Recv_Msg(s,NumDevice,ServiceLib.SogliaUp_gruppo_1_3,ServiceLib.SogliaUp_gruppo_3_8,ServiceLib.SogliaUp_gruppo_8_17,ServiceLib.SogliaDown_gruppo_1_3,ServiceLib.SogliaDown_gruppo_3_8,ServiceLib.SogliaDown_gruppo_8_17)
                    OPC_Recv_Msg(s,NumDevice)
                    ThresoldAlarmOffAttempts -= 1



        # -------------------------------------------------------------
        # Se sta finendo il minuto in corso (tra il 45esimo e il 55esimo secondo) vado avanti per salvare su SD altrimenti aspetto
        # salvo su sd i conteggi di tutti i canali e
        # calcolo i totali di ognuna delle tre fasce di particelle per la porzione di ora fin qui campionata
        # potrei anche mettere un controllo che per questo specifico minuto ho già salvato i dati
        adesso = rtc.now()
        print('aspetto scoccare minuto per scrittura su SD')
        secondi = int(adesso[5])
        print('secondi',secondi)
        while (secondi < 45) or (secondi>55):
            adesso = rtc.now()
            secondi = int(adesso[5])
        print('secondi',secondi)
        dataalminuto=str(curt[0]) + ' ' +str(curt[1]) + ' ' + str(curt[2]) + ' ' + str(curt[3]) + ' ' + str(curt[4])

        # totali del parziale di questa ora

        tot_minuti = tot_minuti + 1
        tot_oraincorso_gruppo_1_3 = tot_oraincorso_gruppo_1_3 + gruppo_1_3
        tot_oraincorso_gruppo_3_8 = tot_oraincorso_gruppo_3_8 + gruppo_3_8
        tot_oraincorso_gruppo_8_17 = tot_oraincorso_gruppo_8_17 + gruppo_8_17


        # scrivo su sd i conteggi di questo minuto per tutti i canali del contaparticelle

        if sd_presence == True:


            try:
                #print('DISK Directory Content: ', os.listdir('/sd'))
                f = open('/sd/' + str(curt[0]) + '_' + str(curt[1]) + '_' + str(curt[2]), 'a')
                oggi = str(curt[0]) + str(curt[1]) + str(curt[2])
                if oggi != ultimo_giorno: # NEL CASO DI RESET DELLA SCHEDA E RIAVVIO QUESTO CONFRONTO NON FUNZIONA PERCHE' ANCHE SE E' LO
                                            # STESSO GIORNO 'oggi' NON E' STATA AGGIORNATA, PER RISOLVERE IL PROBLEMA BISOGNEREBBE CREARE UN FILE OGGI DOVE
                                            # ALL'AVVIO PRENDO IL VALORE DA ASSEGNARE ALLA VARIABILE oggi. ALTRIMENTI NEL FILE TROVO RIPETUTE LE INTESTAZIONI
                                            # COSA NON GRAVE
                    info_dispositivo = 'Dispositivo: N' + str(NumDevice) + ', Coordinate: ' + str(coord) + ',' + '\n'
                    f.write(info_dispositivo)
                    f.write('anno , mese , giorno , ora , minuti , 0.38 , 0.54 , 0.78 , 1 , 1.3 , 1.6 , 2.1 , 3.0 , 4.0 , 5.0 , 6.5 , 8.0 , 10.0 , 12.0 , 14.0 , 16.0 , Flusso(ml/min) , Tempo Campionamento (sec), ' + '\n')
                ultimo_giorno = oggi
                f.write( str(curt[0]) + ',' + str(curt[1]) + ',' + str(curt[2]) + ',' + str(curt[3]) + ',' + str(curt[4]) + ',')
                for i in range(0, 16):
                    f.write(str(AVG_Histogram_Data[i])+',')
                #AVG_SFR_tronc = '%5.2f' % AVG_SFR_Data
                #f.write(AVG_SFR_tronc +',' )
                f.write(str(int(AVG_SFR_Data)) +',' )
                #Sampling_Time_tronc = '%6.2f' % Sampling_Time
                f.write(str(int(Sampling_Time)) + ',' + '\n')
                f.close()

                byteWarning[0] = ~warning_SD_Not_Mounted_bit & int(byteWarning[0])
            except Exception as error:
                print('Errore in scrittura file n.', error)
                byteWarning[0] = warning_SD_Not_Mounted_bit | int(byteWarning[0])

        # *****************************************************************
        # *************** TRASMISSIONE SU RETE LORA ***********************
        # *****************************************************************
        # se trasmetto ogni ora imposto uno specifico minuto all'interno dell'ora
        # questo minuto sarà diverso per ogni dispositivo in modo da evitare collisioni nei dati trasmessi
        # per limitare il rischio di dati persi per mancato collegamento LoRa
        # effettuo un invio ridondante inviando ogni volta oltre ai dati dell'ora in corso
        # anche quelli dell'ora prima già trasmessi


        #minutoinvio=2  # -----change------- DOVRA' DIVENTARE UN PARAMETRO IMPOSTABILE ---------------------------------------

        adesso = rtc.now()
        print('è scoccata ora per invio dati su rete LoRa?')
        minuti = int(adesso[4])
        print('minuti',minuti)
        print('txDelay', OPC_parameters_loraTx_Delay)
        #if minuti==minutoinvio:
        #if (True):
        if minuti == int(OPC_parameters_loraTx_Delay):
        #if (minuti%5) == int(OPC_parameters_loraTx_Delay):
            # calcolo le medie orarie (in count/min) dei conteggi per le tre fasce e le metto nelle variabili fin qui usate per i conteggi di ogni minuto
            gruppo_1_3 = int(tot_oraincorso_gruppo_1_3 / tot_minuti) # metto INt per trascurare i decimali
            gruppo_3_8 = int(tot_oraincorso_gruppo_3_8 / tot_minuti)
            gruppo_8_17 = int(tot_oraincorso_gruppo_8_17 / tot_minuti)


            print('sì invio su rete LoRa')

            stringaHM=str(gruppo_1_3) + ','+ str(gruppo_3_8) + ','+ str(gruppo_8_17)

            if lora_ota == True:
                #controllo se la connessione è ok
                print('LoRa OTA')
                if LoraConnected == False:
                    # provo a riconnettere
                    print('Non Connessa, Provo a riconnettere')
                    #LoraConnected = LORAConnection_OOTA(10000)
                    LoraConnected, lora = LORAConnection_OOTA(0)
                    if(LoraConnected == False):
                        for i in range(0, 5):
                            if(lora.has_joined() == False):
                                time.sleep_ms(2000)
                                print('Not yet joined...')
                            else:
                                LoraConnected = True
                                print('Joined!')
                                break

                #controllo se la connessione è riuscita, in caso positivo invio, altrimenti non faccio nulla (per ora)
                if LoraConnected == True:
                    s.setblocking(True)
                    print('Sending strnga HM...')
                    #stringawarning = '00'
                    stringawarning = '{}'.format(byteWarning[0])
                    stringaHMtotale = 'N,'+ str(NumDevice) + ',M,' + stringaHM + ',' + stringaHMprecedente + ',W,' + stringawarning
                    print('stringa inviata: ', stringaHMtotale)
                    s.send(stringaHMtotale)
                    # make the socket non-blocking
                    # (because if there's no data received it will block forever...)
                    s.setblocking(False)
                    #receive msg, parse and react
                    #OPC_Recv_Msg(s,NumDevice,SogliaUp_gruppo_1_3,SogliaUp_gruppo_3_8,SogliaUp_gruppo_8_17,SogliaDown_gruppo_1_3,SogliaDown_gruppo_3_8,SogliaDown_gruppo_8_17)
                    OPC_Recv_Msg(s,NumDevice)
                    stringaHMprecedente = stringaHM

            # riazzero le variabili per l'ora successiva
            tot_minuti = 0
            tot_oraincorso_gruppo_1_3 = 0
            tot_oraincorso_gruppo_3_8 = 0
            tot_oraincorso_gruppo_8_17 = 0

            # Send parameters to server once every 24 cycles (24 hours)
            if (ServerUpdateParmetersMsgCounter == 0):

                ServerUpdateParmetersMsgCounter = 24
                if LoraConnected == True:
                    print('invio msg aggiornamento parametri al server')

                    print('S1 Ext: ',ServiceLib.SogliaUp_gruppo_1_3)
                    print('S2 Ext: ',ServiceLib.SogliaUp_gruppo_3_8)
                    print('S3 Ext: ',ServiceLib.SogliaUp_gruppo_8_17)
                    OPC_Send_Thresholds(s,Device_Info,ServiceLib.SogliaUp_gruppo_1_3,ServiceLib.SogliaUp_gruppo_3_8,ServiceLib.SogliaUp_gruppo_8_17)
                    print(' K Ext', ServiceLib.OPC_parameters_Counts_K)
                    OPC_Recv_Msg(s,NumDevice)
                    time.sleep_ms(2000)
                    OPC_Send_KFactor(s,Device_Info,ServiceLib.OPC_parameters_Counts_K)
                    OPC_Recv_Msg(s,NumDevice)
            ServerUpdateParmetersMsgCounter =- 1







#  * * * * * * * *           D A     F A R E          * * * * * * * * * * * * * * * *


#  GESTIONE DELLE ECCEZIONI
#  GESTIONE WARNING CON PAROLA DI STATO COSTITUITA DA DUE CARATTERI ESADECIMALI
#  VERIFICARE I DATI SCRITTI SU sd
#  CALCOLARE AUTONOMIA sd
#  PROVARE NUOVA VERSIONE HW CON PIN SPI MODIFICATI PER NON INTERFERIRE CON IL LED
#  CON NUOVA VERIONE HW GESTIRE LED CON VARI COLORI PER SEGNALAZIONI
#  RIVEDERE GESTIONE DEI PARAMETRI IMPOSTABILI
# c'erano evidenti errori nella lettura dei campi dell'OPCN2 e mancavano ritardi consigliati da alphasense, bisognerebbe provare a non mettere
#       le mdifiche hw proposte da di lellis che forse erano necessarie come toppa a causa di questi errori
# aggiornare orologio con gps ? ogni quanto?
# al primo collegamento con la rete invio un messaggio di presentazione? ad es. identificativo (N10), tipo sensore (OPCN2 o altro), coordinate gps
