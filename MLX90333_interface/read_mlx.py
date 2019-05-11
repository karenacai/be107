import spidev
class mlxSensor:
    def __init__(self,csnum=1,rate=16000,delay=10000,bits=8):
        self.spicom = spidev.SpiDev()
        self.spicom.open(0, csnum)
        self.rate = rate
        self.delay=delay
        self.bits=bits

    def getData(self):
        rdata = self.spicom.xfer([255,255,255,255,255,255,255,255],self.rate,self.delay,self.bits)
        alpha = int((rdata[2]<<8)+rdata[1])
        beta = int((rdata[4]<<8)+rdata[3])
        third = int((rdata[6]<<8)+rdata[5])
        return alpha,beta,third
    def getRawData(self):
        rdata = self.spicom.xfer([255,255,255,255,255,255,255,255],self.rate,self.delay,self.bits)
        return rdata
    def __del__(self):
        self.spicom.close()
if(__name__=="__main__"):
    mymlx = mlxSensor()
    while(True):
        alpha,beta,third = mymlx.getData()
        print("alpha {} beta {} third {}".format(alpha,beta,third))
        #rawdata = mymlx.getRawData()
        #alpha = int((rawdata[2]<<8)+rawdata[1])
        #beta = int((rawdata[4]<<8)+rawdata[3])
        #print("alpha {} beta {}".format(alpha,beta))
        #print("alpha {},{} beta {},{}".format(rawdata[1],rawdata[2],rawdata[3],rawdata[4]))
