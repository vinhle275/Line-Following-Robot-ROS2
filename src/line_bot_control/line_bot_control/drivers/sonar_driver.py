import RPi.GPIO as GPIO
import time
import statistics

class SonarSensor:
    def __init__(self, trigger_pin, echo_pin, sensor_id="sonar"):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.sensor_id = sensor_id
        
        # Cấu hình bộ lọc (giữ nguyên từ code cũ của bạn)
        self.window_size = 5
        self.ema_alpha = 0.3
        self.min_cm = 2
        self.max_cm = 400
        
        self.dist_buffer = []
        self.ema_value = None

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        GPIO.output(self.trigger_pin, False)

    def _read_raw_distance(self, timeout=0.02):
        """Đọc khoảng cách thô (Raw) từ cảm biến"""
        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trigger_pin, False)

        start_time = time.time()
        pulse_start = start_time
        pulse_end = start_time

        # Chờ Echo bắt đầu
        while GPIO.input(self.echo_pin) == 0:
            pulse_start = time.time()
            if pulse_start - start_time > timeout:
                return None

        # Chờ Echo kết thúc
        while GPIO.input(self.echo_pin) == 1:
            pulse_end = time.time()
            if pulse_end - start_time > timeout:
                return None

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        return round(distance, 2)

    def get_filtered_distance(self):
        """Đọc và áp dụng bộ lọc Median + EMA"""
        dist = self._read_raw_distance()

        # Nếu đọc lỗi hoặc ngoài phạm vi, trả về giá trị cũ
        if dist is None or dist < self.min_cm or dist > self.max_cm:
            return self.ema_value

        # Median Filter
        self.dist_buffer.append(dist)
        if len(self.dist_buffer) > self.window_size:
            self.dist_buffer.pop(0)
        
        med = statistics.median(self.dist_buffer)

        # EMA Filter
        if self.ema_value is None:
            self.ema_value = med
        else:
            self.ema_value = (self.ema_alpha * med) + ((1 - self.ema_alpha) * self.ema_value)

        return round(self.ema_value, 2)

    def cleanup(self):
        GPIO.cleanup([self.trigger_pin, self.echo_pin])