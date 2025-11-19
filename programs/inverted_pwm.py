# inverted_pwm.py
"""
Módulo para invertir la señal PWM enviada por el PCA9685.
Útil cuando se usan optoacopladores que invierten la señal eléctrica.
"""

class InvertedPWMChannel:
    """
    Clase que invierte la señal PWM de un canal del PCA9685.
    En lugar de invertir el ángulo, invierte el ciclo de trabajo (duty cycle).
    """
    def __init__(self, pwm_channel):
        self._pwm = pwm_channel

    def set_pwm(self, on, off):
        """
        Recibe los valores 'on' y 'off' (0–4095) y los invierte.
        """
        inv_on = 0 if off == 0 else 4096 - off
        inv_off = 0 if on == 0 else 4096 - on
        self._pwm.set_pwm(inv_on, inv_off)

    def duty_cycle(self, value):
        """
        Método equivalente a asignar un duty_cycle invertido.
        """
        if not 0 <= value <= 0xFFFF:
            raise ValueError("duty_cycle fuera de rango (0–65535).")

        inv_value = 0xFFFF - value
        self._pwm.duty_cycle = inv_value


class InvertedServo:
    """
    Servo con inversión de PWM a nivel físico.
    Toma el objeto servo del ServoKit y el canal PWM correspondiente.
    """
    def __init__(self, servo, pwm_channel):
        self._servo = servo
        self._pwm_inv = InvertedPWMChannel(pwm_channel)
        self._angle = None

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, value):
        if value is None:
            self._servo.angle = None
            return
        if not 0 <= value <= 180:
            raise ValueError("Ángulo fuera de rango (0–180).")

        # El ServoKit se encarga de traducir el ángulo a PWM normal
        self._servo.angle = value

        # Luego tomamos el duty ya generado en el canal y lo invertimos
        duty_raw = self._pwm_inv._pwm.duty_cycle
        self._pwm_inv.duty_cycle(duty_raw)

        self._angle = value
