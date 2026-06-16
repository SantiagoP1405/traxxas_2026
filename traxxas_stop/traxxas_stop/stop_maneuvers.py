#!/usr/bin/env python3
"""
=======================================================================
 StopManeuvers  -  traxxas_stop
 Clases de maniobra de parada CON retroalimentacion del ENCODER.

 
 ---------
   BrakeRamp    - rampa de frenado + espera |v| < stopped_threshold
   StopWait     - quieto wait_s segundos (solo temporal)
   ResumeRamp   - rampa de arranque + espera v > stable_threshold
   StopManeuver - agrupa las tres con FSM BRAKING->STOPPED->RESUMING->DONE

 Retroalimentacion del encoder
 -----------------------------
   BrakeRamp : no termina hasta que |v| < stopped_threshold.
               Si la rampa de tiempo acabo pero el vehiculo sigue
               moviendose, extiende publicando brake_pwm.
               encoder_timeout_s evita bloqueo infinito.
   StopWait  : solo temporal, no usa encoder.
   ResumeRamp: no termina hasta que v > stable_threshold Y se cumplio
               el tiempo de rampa minimo.
               encoder_timeout_s evita bloqueo infinito.
=======================================================================
"""
import time


class BrakeRamp:
    """
    Parametros
    ----------
    start_pwm         : PWM al iniciar la rampa
    brake_pwm         : PWM objetivo (freno completo, 2457)
    duration_s        : segundos de rampa
    stopped_threshold : velocidad [m/s] bajo la cual = detenido (0.03)
    encoder_timeout_s : maximo extra de espera si encoder no confirma (3.0)

    Uso
    ---
        ramp = BrakeRamp(2570, 2457, 2.0)
        ramp.start()
        while not ramp.done:
            pwm = ramp.tick(v_mps)   # v_mps del encoder
            publish(pwm)
    """

    def __init__(
        self,
        start_pwm        : int,
        brake_pwm        : int,
        duration_s       : float,
        stopped_threshold: float = 0.03,
        encoder_timeout_s: float = 3.0,
    ):
        self.start_pwm         = int(start_pwm)
        self.brake_pwm         = int(brake_pwm)
        self.duration_s        = max(float(duration_s), 0.01)
        self.stopped_threshold = float(stopped_threshold)
        self.encoder_timeout_s = float(encoder_timeout_s)
        self._t0              = 0.0
        self._ramp_done       = False
        self._encoder_done    = False
        self._enc_wait_start  = 0.0

    def start(self):
        self._t0             = time.time()
        self._ramp_done      = False
        self._encoder_done   = False

    @property
    def done(self) -> bool:
        return self._ramp_done and self._encoder_done

    @property
    def elapsed(self) -> float:
        return time.time() - self._t0

    def tick(self, v_mps: float) -> int:
        t = self.elapsed / self.duration_s
        if not self._ramp_done:
            if t >= 1.0:
                self._ramp_done     = True
                self._enc_wait_start = time.time()
            pwm = self.start_pwm + t * (self.brake_pwm - self.start_pwm)
            return max(self.brake_pwm, int(round(pwm)))

        if not self._encoder_done:
            if abs(v_mps) < self.stopped_threshold:
                self._encoder_done = True
            elif (time.time() - self._enc_wait_start) > self.encoder_timeout_s:
                self._encoder_done = True   # timeout de seguridad
        return self.brake_pwm


class StopWait:
    """
    Mantiene brake_pwm durante wait_s segundos (solo temporal).

    Uso
    ---
        wait = StopWait(2457, 5.0)
        wait.start()
        while not wait.done:
            pwm = wait.tick()
            publish(pwm)
    """

    def __init__(self, brake_pwm: int, wait_s: float):
        self.brake_pwm = int(brake_pwm)
        self.wait_s    = max(float(wait_s), 0.0)
        self._t0       = 0.0
        self._done     = False

    def start(self):
        self._t0   = time.time()
        self._done = False

    @property
    def done(self) -> bool:
        return self._done

    @property
    def elapsed(self) -> float:
        return time.time() - self._t0

    def tick(self, v_mps: float = 0.0) -> int:
        """v_mps se acepta para firma uniforme pero no se usa."""
        if time.time() - self._t0 >= self.wait_s:
            self._done = True
        return self.brake_pwm


class ResumeRamp:
    """
    Sube PWM linealmente brake_pwm -> cruise_pwm en duration_s segundos,
    luego espera que el encoder confirme v > stable_threshold.

    Parametros
    ----------
    brake_pwm         : PWM de partida (2457)
    cruise_pwm        : PWM objetivo de crucero
    duration_s        : segundos de rampa de arranque
    stable_threshold  : velocidad [m/s] que = "en marcha" (0.10)
    encoder_timeout_s : maximo de espera si encoder no confirma (4.0)

    Uso
    ---
        ramp = ResumeRamp(2457, 2570, 1.5)
        ramp.start()
        while not ramp.done:
            pwm = ramp.tick(v_mps)
            publish(pwm)
    """

    def __init__(
        self,
        brake_pwm        : int,
        cruise_pwm       : int,
        duration_s       : float,
        stable_threshold : float = 0.10,
        encoder_timeout_s: float = 4.0,
    ):
        self.brake_pwm        = int(brake_pwm)
        self.cruise_pwm       = int(cruise_pwm)
        self.duration_s       = max(float(duration_s), 0.01)
        self.stable_threshold = float(stable_threshold)
        self.encoder_timeout_s = float(encoder_timeout_s)
        self._t0              = 0.0
        self._ramp_done       = False
        self._encoder_done    = False
        self._enc_wait_start  = 0.0

    def start(self):
        self._t0           = time.time()
        self._ramp_done    = False
        self._encoder_done = False

    @property
    def done(self) -> bool:
        return self._ramp_done and self._encoder_done

    @property
    def elapsed(self) -> float:
        return time.time() - self._t0

    def tick(self, v_mps: float) -> int:
        """v_mps: velocidad del encoder [m/s]. Retorna PWM a publicar."""
        t = self.elapsed / self.duration_s
        if not self._ramp_done:
            if t >= 1.0:
                self._ramp_done      = True
                self._enc_wait_start = time.time()
            pwm = self.brake_pwm + t * (self.cruise_pwm - self.brake_pwm)
            return min(self.cruise_pwm, int(round(pwm)))

        # Rampa terminada: esperar encoder
        if not self._encoder_done:
            if abs(v_mps) >= self.stable_threshold:
                self._encoder_done = True
            elif (time.time() - self._enc_wait_start) > self.encoder_timeout_s:
                self._encoder_done = True
        return self.cruise_pwm


class StopManeuver:
    """
    Agrupa BrakeRamp + StopWait + ResumeRamp en una maniobra completa.
    FSM interna: BRAKING -> STOPPED -> RESUMING -> DONE

    Usada directamente por el action server.

    Parametros
    ----------
    start_pwm         : PWM al iniciar (cruise_pwm aprox.)
    brake_pwm         : PWM freno completo (2457)
    cruise_pwm        : PWM velocidad crucero
    brake_dur         : segundos de rampa de frenado
    wait_s            : segundos quieto
    resume_dur        : segundos de rampa de arranque
    stopped_threshold : |v| [m/s] para confirmar detencion (encoder)
    stable_threshold  : v [m/s] para confirmar rearranque (encoder)

    Uso
    ---
        m = StopManeuver(2570, 2457, 2570, 2.0, 5.0, 1.5)
        m.start()
        while not m.done:
            pwm, phase = m.tick(v_mps)
            publish(pwm)
            send_feedback(phase, elapsed=m.wait_elapsed)
    """

    BRAKING  = 'BRAKING'
    STOPPED  = 'STOPPED'
    RESUMING = 'RESUMING'
    DONE     = 'DONE'

    def __init__(
        self,
        start_pwm        : int,
        brake_pwm        : int,
        cruise_pwm       : int,
        brake_dur        : float,
        wait_s           : float,
        resume_dur       : float,
        stopped_threshold: float = 0.03,
        stable_threshold : float = 0.10,
    ):
        self._brake  = BrakeRamp(
            start_pwm, brake_pwm, brake_dur,
            stopped_threshold=stopped_threshold,
        )
        self._wait   = StopWait(brake_pwm, wait_s)
        self._resume = ResumeRamp(
            brake_pwm, cruise_pwm, resume_dur,
            stable_threshold=stable_threshold,
        )
        self._phase = self.BRAKING
        self._done  = False

    def start(self):
        """Inicia la maniobra completa. Llamar una sola vez."""
        self._phase = self.BRAKING
        self._done  = False
        self._brake.start()

    @property
    def done(self) -> bool:
        return self._done

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def wait_elapsed(self) -> float:
        """Segundos en fase STOPPED. 0 si aun no empezo."""
        if self._phase in (self.STOPPED, self.RESUMING, self.DONE):
            return self._wait.elapsed
        return 0.0

    def tick(self, v_mps: float) -> tuple:
        """
        Avanza un tick con retroalimentacion del encoder.
        v_mps : velocidad lineal del encoder [m/s]
        Retorna (pwm: int, phase: str).
        Las transiciones son automaticas cuando cada sub-fase termina.
        """
        if self._phase == self.BRAKING:
            pwm = self._brake.tick(v_mps)
            if self._brake.done:
                self._phase = self.STOPPED
                self._wait.start()
            return pwm, self._phase

        if self._phase == self.STOPPED:
            pwm = self._wait.tick(v_mps)
            if self._wait.done:
                self._phase = self.RESUMING
                self._resume.start()
            return pwm, self._phase

        if self._phase == self.RESUMING:
            pwm = self._resume.tick(v_mps)
            if self._resume.done:
                self._phase = self.DONE
                self._done  = True
            return pwm, self._phase

        return self._resume.cruise_pwm, self.DONE