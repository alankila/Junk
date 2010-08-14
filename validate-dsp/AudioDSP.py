#!/usr/bin/python

import math, random

samplingFrequency = 44100;

class Biquad:
    __slots__ = ['mA1', 'mA2', 'mB0', 'mB1', 'mB2',
                'mY1', 'mY2', 'mX1', 'mX2'
    ]

    def __init__(self):
        self.mX1 = self.mX2 = self.mY1 = self.mY2 = 0;
        self.setCoefficients(1, 0, 0, 0, 0, 0);

    def setCoefficients(self, a0,  a1,  a2,  b0,  b1,  b2):
        self.mA1 = a1/a0
        self.mA2 = a2/a0
        self.mB0 = b0/a0
        self.mB1 = b1/a0
        self.mB2 = b2/a0

    def setLowShelf1(self, center_frequency, sampling_frequency, db_gain):
        w0 = 2 * math.pi * center_frequency / sampling_frequency;
        A = math.pow(10, db_gain/40.);
        
        b0 = math.sin(w0 / 2) * A + math.cos(w0 / 2)
        b1 = math.sin(w0 / 2) * A - math.cos(w0 / 2)
        a0 = math.sin(w0 / 2) / A + math.cos(w0 / 2)
        a1 = math.sin(w0 / 2) / A - math.cos(w0 / 2)

        self.setCoefficients(a0, a1, 0, b0, b1, 0)
    
    def setHighShelf1(self, center_frequency, sampling_frequency, db_gain):
        w0 = 2 * math.pi * center_frequency / sampling_frequency;
        A = math.pow(10, db_gain/40.);
        
        b0 = math.sin(w0 / 2) + math.cos(w0 / 2) * A
        b1 = math.sin(w0 / 2) - math.cos(w0 / 2) * A
        a0 = math.sin(w0 / 2) + math.cos(w0 / 2) / A
        a1 = math.sin(w0 / 2) - math.cos(w0 / 2) / A

        self.setCoefficients(a0, a1, 0, b0, b1, 0)


    def setPeakingEqualizer(self, center_frequency,  sampling_frequency,  db_gain,  bandwidth):
        w0 = 2 * math.pi * center_frequency / sampling_frequency;
        A = math.pow(10, db_gain/40.);
        alpha = math.sin(w0)/2 * math.sinh( math.log(2)/2 * bandwidth * w0/math.sin(w0) );

        b0 =   1 + alpha*A;
        b1 =  -2*math.cos(w0);
        b2 =   1 - alpha*A;
        a0 =   1 + alpha/A;
        a1 =  -2*math.cos(w0);
        a2 =   1 - alpha/A;
   
        self.setCoefficients(a0, a1, a2, b0, b1, b2);

    def setLowPass(self, center_frequency, sampling_frequency, bandwidth):
        w0 = 2 * math.pi * center_frequency / sampling_frequency;
        alpha = math.sin(w0)/2 * math.sinh( math.log(2)/2 * bandwidth * w0/math.sin(w0) );

        b0 =  (1 - math.cos(w0))/2
        b1 =   1 - math.cos(w0)
        b2 =  (1 - math.cos(w0))/2
        a0 =   1 + alpha
        a1 =  -2*math.cos(w0)
        a2 =   1 - alpha
        
        self.setCoefficients(a0, a1, a2, b0, b1, b2);

    def setHighPass(self, center_frequency, sampling_frequency, bandwidth):
        w0 = 2 * math.pi * center_frequency / sampling_frequency;
        alpha = math.sin(w0)/2 * math.sinh( math.log(2)/2 * bandwidth * w0/math.sin(w0) );

        b0 =  (1 + math.cos(w0))/2
        b1 = -(1 + math.cos(w0))
        b2 =  (1 + math.cos(w0))/2
        a0 =   1 + alpha
        a1 =  -2*math.cos(w0)
        a2 =   1 - alpha
        
        self.setCoefficients(a0, a1, a2, b0, b1, b2);

    def setBandPass(self, center_frequency, sampling_frequency, resonance):
        w0 = 2 * math.pi * center_frequency / sampling_frequency;
        alpha = math.sin(w0)/(2*resonance)

        b0 =   math.sin(w0)/2
        b1 =   0
        b2 =  -math.sin(w0)/2
        a0 =   1 + alpha
        a1 =  -2*math.cos(w0)
        a2 =   1 - alpha

        self.setCoefficients(a0, a1, a2, b0, b1, b2);

    def setHighShelf(self, center_frequency,  sampling_frequency,  db_gain,  slope):
        w0 = 2 * math.pi * center_frequency / sampling_frequency;
        A = math.pow(10, db_gain/40.);
        alpha = math.sin(w0)/2 * math.sqrt( (A + 1/A)*(1/slope - 1) + 2 );

        b0 =    A*( (A+1) + (A-1)*math.cos(w0) + 2*math.sqrt(A)*alpha );
        b1 = -2*A*( (A-1) + (A+1)*math.cos(w0)                   );
        b2 =    A*( (A+1) + (A-1)*math.cos(w0) - 2*math.sqrt(A)*alpha );
        a0 =        (A+1) - (A-1)*math.cos(w0) + 2*math.sqrt(A)*alpha  ;
        a1 =    2*( (A-1) - (A+1)*math.cos(w0)                   );
        a2 =        (A+1) - (A-1)*math.cos(w0) - 2*math.sqrt(A)*alpha  ;

        self.setCoefficients(a0, a1, a2, b0, b1, b2);

    def setLowShelf(self, center_frequency,  sampling_frequency,  db_gain,  slope):
        w0 = 2 * math.pi * center_frequency / sampling_frequency;
        A = math.pow(10, db_gain/40);
        alpha = math.sin(w0)/2 * math.sqrt( (A + 1/A)*(1/slope - 1) + 2 );

        b0 =    A*( (A+1) - (A-1)*math.cos(w0) + 2*math.sqrt(A)*alpha );
        b1 =  2*A*( (A-1) - (A+1)*math.cos(w0)                   );
        b2 =    A*( (A+1) - (A-1)*math.cos(w0) - 2*math.sqrt(A)*alpha );
        a0 =        (A+1) + (A-1)*math.cos(w0) + 2*math.sqrt(A)*alpha  ;
        a1 =   -2*( (A-1) + (A+1)*math.cos(w0)                   );
        a2 =        (A+1) + (A-1)*math.cos(w0) - 2*math.sqrt(A)*alpha  ;

        self.setCoefficients(a0, a1, a2, b0, b1, b2);

    def setRC(self, cf, sf):
        DT_div_RC = 2 * math.pi * cf/sf
        b0 = DT_div_RC / (1 + DT_div_RC)
        a1 = -1 + b0

        self.setCoefficients(1, a1, 0, b0, 0, 0);


    def process(self, x0):
        y0 = self.mB0 * x0 + self.mB1 * self.mX1 + self.mB2 * self.mX2 - self.mY1 * self.mA1 - self.mY2 * self.mA2;
        #y0 = y0 + int(random.random() * (1 << fixedPointDecimals)) >> fixedPointDecimals;

        self.mY2 = self.mY1;
        self.mY1 = y0;

        self.mX2 = self.mX1;
        self.mX1 = x0;

        return y0;

    def transfer(self, omega):
        nom = self.mB0 + self.mB1 / omega + self.mB2 / (omega*omega)
        den =        1 + self.mA1 / omega + self.mA2 / (omega*omega)
        return nom / den;

class EffectTone:
    __slots__ = ['mFilterL']

    def __init__(self):
        self.mFilterL = [Biquad(), Biquad(), Biquad(), Biquad(), Biquad()];
        for i in range(5):
            self.setBand(i, 0)

    def setBand(self, band, dB):
        centerFrequency = 62.5 * math.pow(4, band);

        if band == 0:
            self.mFilterL[band].setLowShelf(centerFrequency * 2, samplingFrequency, dB, 1.0);
        elif band == 4:
            self.mFilterL[band].setHighShelf(centerFrequency * 0.5, samplingFrequency, dB, 1.0);
        else:
            self.mFilterL[band].setPeakingEqualizer(centerFrequency, samplingFrequency, dB, 3.0);

    def process(self, x0):
        for j in range(5):
            x0 = self.mFilterL[j].process(x0);
        return x0

def lin2db(x):
    if x == 0:
        return -99
    return math.log(x) / math.log(10) * 20

def main():
    et = Biquad()
    #et.setRC(700, 44100);
    et.setHighShelf(800, 44100, -12.0, 0.72);
   
    startFreq = 20.0
    endFreq = 22050.0

    freq = startFreq;
    while freq < endFreq:
        arg = freq / 44100.0 * math.pi
        z = math.sin(arg) * 1j + math.cos(arg)
        vector = et.transfer(z)

        mag = lin2db(abs(vector))
        pha = math.atan2(vector.imag, vector.real)
        delay = 0
        if freq != 0:
            delay = (pha / math.pi) / freq * 1000000 # ms
        print "%d %.2f %.2f %f" % (freq, mag, pha, delay)
        
        freq *= 1.1

if __name__ == '__main__':
    main()
