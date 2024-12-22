class KalmanFilter:
    def __init__(self, q, r):
        self.x_k1_k1 = None
        self.q = q
        self.r = r
        
        self.P_k_k1 = 1
        self.Kg = 0
        self.P_k1_k1 = 1
        self.x_k_k1 = 0
        self.ADC_OLD_Value = 0
        self.Z_k = 0
        self.kalman_adc_old=0
        
    def kalman(self, adc_value):
       
        self.Z_k = adc_value
        
        if abs(self.kalman_adc_old - adc_value) >= 60:
            self.x_k1_k1 = adc_value*0.382 + self.kalman_adc_old * 0.618
        else:
            self.x_k1_k1 = self.kalman_adc_old
    
        self.x_k_k1 = self.x_k1_k1
        self.P_k_k1 = self.P_k1_k1 + self.q
    
        self.Kg = self.P_k_k1/(self.P_k_k1 + self.r)
    
        kalman_adc = self.x_k_k1 + self.Kg * (self.Z_k - self.kalman_adc_old)
        self.P_k1_k1 = (1 - self.Kg)*self.P_k_k1
        self.P_k_k1 = self.P_k1_k1
    
        self.kalman_adc_old = kalman_adc
        
        return kalman_adc
