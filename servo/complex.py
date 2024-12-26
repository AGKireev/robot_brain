"""
# TODO: Complex commands
    # fuc.radarScan()
    # Steady .. fuc.steady(300)
    # ? Keep Distance

scGear = base.ServoCtrl()

# Initialize PWM values and directions
pwm_config = config.read("pwm")

pwm0_direction = 1
pwm0_init = pwm_config["init_pwm0"]
pwm0_max = 520
pwm0_min = 100
pwm0_pos = pwm0_init

pwm1_direction = 1
pwm1_init = pwm_config["init_pwm1"]
pwm1_max = 520
pwm1_min = 100
pwm1_pos = pwm1_init

pwm2_direction = 1
pwm2_init = pwm_config["init_pwm2"]
pwm2_max = 520
pwm2_min = 100
pwm2_pos = pwm2_init

def radarScan(self):
    '''
    Perform a radar scan using the servo.
    '''
    logger.info('Functions: radarScan')

    global pwm0_pos
    scan_speed = 3
    result = []

    if pwm0_direction:
        pwm0_pos = pwm0_max
        scGear.set_servo_pwm(12, pwm0_pos)
        time.sleep(0.8)

        while pwm0_pos > pwm0_min:
            pwm0_pos -= scan_speed
            scGear.set_servo_pwm(12, pwm0_pos)
    else:
        pwm0_pos = pwm0_min
        scGear.set_servo_pwm(12, pwm0_pos)
        time.sleep(0.8)

        while pwm0_pos < pwm0_max:
            pwm0_pos += scan_speed
            scGear.set_servo_pwm(12, pwm0_pos)
    scGear.set_servo_pwm(12, pwm0_init)
    return result
"""