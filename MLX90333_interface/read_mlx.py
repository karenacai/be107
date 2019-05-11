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
        alpha = int((rdata[1]<<8)+rdata[2])
        beta = int((rdata[3]<<8)+rdata[4])
        return alpha,beta
    def __del__(self):
        self.spicom.close()
if(__name__=="__main__"):
    mymlx = mlxSensor()
    while(True):
        alpha,beta = mymlx.getData()
        print("alpha {} beta {}".format(alpha,beta))
