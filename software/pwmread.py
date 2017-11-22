import time
import pigpio

prev_tick = 0
pulsewidth_ms = 1.5

def pin_changed(gpio, level, tick):
    global prev_tick
    global pulsewidth_ms
    if level == 1:
        prev_tick = tick
    else:
        dt_ms = (tick - prev_tick) / 1000
        if (dt_ms > 1.0 and dt_ms < 2.0):
            pulsewidth_ms = pulsewidth_ms * 0.75 + dt_ms * 0.25
        

def main():
    pi = pigpio.pi()
    pi.set_mode(4, pigpio.INPUT)
    pi.set_pull_up_down(4, pigpio.PUD_OFF)
    pi.callback(4, pigpio.EITHER_EDGE, pin_changed)

    while True:
        global pulsewidth_ms
        time.sleep(0.5)
        duty = (pulsewidth_ms - 1.51) * 200
        print("pw = {:.3f}   duty = {:.3f}".format(pulsewidth_ms, duty))

if __name__ == "__main__":
    main()
        
