import pigpio
import threading
import time

class StepperMotor():
    def __init__(self,step_pin,dir_pin):
        self.step_pin = step_pin
        self.dir_pin = dir_pin

        self.pi = pigpio.pi()
        self.pi.set_mode(self.step_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.dir_pin, pigpio.OUTPUT)

        self.pi.write(self.dir_pin, 0)

        self.lock = threading.Lock()
    
        self.velocity = 0
        self.running = True 
        
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
    
    def set_velocity(self, velocity):
        with self.lock:
            self.velocity = velocity
            print(velocity)
    
    def run(self):
        self.pi.wave_tx_stop()
        last_velocity = None
        while self.running:
            
            with self.lock:
                velocity = self.velocity

            if velocity <= 0:
                time.sleep(0.1)
                continue

            if velocity == last_velocity:
                time.sleep(1)
                print("Same velocity, skipping")
                continue
            
            period = int(1e6 / velocity) # mu_seconds per step
            self.pi.wave_clear()
            self.pi.wave_add_generic([
                pigpio.pulse(1 << self.step_pin, 0, period//2),
                pigpio.pulse(0, 1 << self.step_pin, period//2)])

            wave_id = self.pi.wave_create()
            self.pi.wave_send_repeat(wave_id)

            time.sleep(0.05)
            last_velocity = velocity
  
    def stop(self):
        with self.lock:
            self.running = False
        self.pi.wave_tx_stop()
        self.pi.stop()


if __name__ == "__main__":
    motor = StepperMotor(step_pin=17, dir_pin=27)

    # try something obvious
    motor.set_velocity(500)   # 200 steps/sec
    time.sleep(2)
    motor.set_velocity(200)   # 100 steps/sec
    time.sleep(2)

    motor.set_velocity(0)
    motor.stop()
