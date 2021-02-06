
import time
class ProgressBar:

    def __init__(self, total, prefix = '', suffix = '', length=50, decimals=1, showEstimatedTime=True):
        self._total = total
        self._prefix = prefix
        self._suffix = suffix
        self._length = length
        self._decimals = decimals
        self._fill = '█'
        self._printEnd = "\r"
        self._showEstimatedTime = showEstimatedTime

        self._listetimeRef = []
        self._listepercentRef = []
        self._delaiMax = 15
        self._delaiMin = 0.5
        self.stopped = False

    def stop(self, finish=True):
        if finish: self.printP(self._total - 0.00005)
        if not(self.stopped):
            self.stopped = True
            print("")


    def printTitle(self, title):
        l = len(self._prefix) + self._length + len(self._suffix) + 25*self._showEstimatedTime + 11
        l1 = max(1,int((l-2-len(title))/2))
        string = "#"*l1 + " " + title + " " + "#"*(l-2-len(title)-l1)
        print("")
        print(string)


    def printP(self,iteration):
        percent = ("{0:." + str(self._decimals) + "f}").format(min(100,100 * (iteration / float(self._total))))
        percent = " "*(4+self._decimals-len(percent)) + percent

        t_estime = ''
        if self._showEstimatedTime:
            per = 100 * (iteration / float(self._total))
            self._listetimeRef.append(time.time())
            self._listepercentRef.append(per)

            dt = self._listetimeRef[-1] - self._listetimeRef[0]
            while (len(self._listetimeRef) > 2) and (dt > self._delaiMax):
                self._listetimeRef = self._listetimeRef[1:]
                self._listepercentRef = self._listepercentRef[1:]
                dt = self._listetimeRef[-1] - self._listetimeRef[0]

            if dt >= self._delaiMin:
                dp = self._listepercentRef[-1] - self._listepercentRef[0]
                if dp > 0:
                    percent_restants = 100 - per
                    t_estime = str(round(percent_restants * dt / dp, self._decimals))
                else:
                    t_estime = 'inf.'
            else:
                t_estime = 'N.def'

        suf = self._suffix
        if self._showEstimatedTime:
            suf += " / Temps estime : " + t_estime + " s" + " "*20

        filledLength = int(self._length * iteration // self._total)
        bar = self._fill * filledLength + '-' * (self._length - filledLength)
        print('\r%s |%s| %s%% %s' % (self._prefix, bar, percent, suf), end = self._printEnd)
        # Print New Line on Complete
        if iteration == self._total:
            self.stop()
