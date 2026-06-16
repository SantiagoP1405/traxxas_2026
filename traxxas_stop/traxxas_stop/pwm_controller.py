#!/usr/bin/env python3
"""
=======================================================================
 PWMController  -  traxxas_stop
-----------------------------------------------------------------------
 Encapsula toda la logica de conversion PWM para throttle y direccion.

 Valores de hardware del Traxxas
 - Throttle: 2457 (freno/neutro) | 2570 (crucero) | 2600 (maximo)
 - Direccion: 1669 (izq) | 2642 (centro) | 3276 (der)
=======================================================================
"""
import numpy as np


class PWMController:

    def __init__(
        self,
        brake_pwm    : int   = 2457,
        cruise_pwm   : int   = 2570,
        max_pwm      : int   = 2600,
        dir_center   : int   = 2642,
        dir_max      : int   = 3276,
        dir_min      : int   = 1669,
        max_steer_rad: float = 0.436, # rad maximo de giro (25° aprox)
    ):
        self.brake_pwm     = int(brake_pwm)
        self.cruise_pwm    = int(cruise_pwm)
        self.max_pwm       = int(max_pwm)
        self.dir_center    = int(dir_center)
        self.dir_max       = int(dir_max)
        self.dir_min       = int(dir_min)
        self.max_steer_rad = float(max_steer_rad)
        self._prev_dir_pwm = int(dir_center)
        self.validate()

    # THROTTLE 
    def throttle_brake(self) -> int:
        return self.brake_pwm

    def throttle_cruise(self) -> int:
        return self.cruise_pwm

    def throttle_max_val(self) -> int:
        return self.max_pwm

    def throttle_lerp(self, t: float) -> int:
        return int(round(self._lerp(self.brake_pwm, self.cruise_pwm, t)))

    def throttle_from_distance(
        self,
        dist              : float,
        stop_dist         : float,
        prepare_dist      : float,
        approach_pwm_offset: int = 40,
    ) -> int:
        """
        Calcula PWM en funcion de la distancia al signo (ZED depth).

        Mapeo:
          dist >= prepare_dist            -> cruise_pwm
          stop_dist < dist < prepare_dist -> interpolado
          dist <= stop_dist               -> brake_pwm
          dist <= 0 (invalido)            -> cruise_pwm (sin info = seguir)

        approach_pwm_offset: counts sobre brake como vel. minima de
          acercamiento (evita que el vehiculo pare antes de llegar).
        """
        if dist <= 0.0:
            return self.cruise_pwm
        if dist <= stop_dist:
            return self.brake_pwm
        if dist >= prepare_dist:
            return self.cruise_pwm
        approach_pwm = self.brake_pwm + int(approach_pwm_offset)
        t = (dist - stop_dist) / max(prepare_dist - stop_dist, 1e-6)
        pwm = int(round(self._lerp(approach_pwm, self.cruise_pwm, t)))
        return max(self.brake_pwm, min(self.cruise_pwm, pwm))

    # DIRECCION 
    def steer_to_pwm(self, delta_rad: float) -> list:
        """
        Convierte angulo Pure Pursuit a PWM de direccion.

        El Traxxas tiene rango ASIMETRICO:
          delta > 0 (derecha):   center -> dir_max
          delta < 0 (izquierda): center -> dir_min

        Si hay cruce de lado antepone un pulso al centro para
        evitar golpe mecanico.
        """
        delta_rad = float(np.clip(delta_rad, -self.max_steer_rad, self.max_steer_rad))
        if delta_rad >= 0.0:
            t = delta_rad / self.max_steer_rad
            target = int(round(self.dir_center + t * (self.dir_max - self.dir_center)))
        else:
            t = (-delta_rad) / self.max_steer_rad
            target = int(round(self.dir_center - t * (self.dir_center - self.dir_min)))

        cmds = []
        prev_side = self._prev_dir_pwm - self.dir_center
        curr_side = target - self.dir_center
        if (prev_side > 0 and curr_side < 0) or (prev_side < 0 and curr_side > 0):
            cmds.append(self.dir_center)
        cmds.append(target)
        self._prev_dir_pwm = target
        return cmds

    def steer_center(self) -> int:
        """PWM de direccion centrada; resetea estado interno."""
        self._prev_dir_pwm = self.dir_center
        return self.dir_center

    # VALIDACION 
    def validate(self) -> bool:
        """Lanza ValueError si los valores configurados son incoherentes."""
        if not (self.brake_pwm < self.cruise_pwm <= self.max_pwm):
            raise ValueError(
                f'Throttle PWM incoherente: '
                f'brake={self.brake_pwm} cruise={self.cruise_pwm} max={self.max_pwm}'
            )
        if not (self.dir_min < self.dir_center < self.dir_max):
            raise ValueError(
                f'Direccion PWM incoherente: '
                f'min={self.dir_min} center={self.dir_center} max={self.dir_max}'
            )
        if self.max_steer_rad <= 0.0:
            raise ValueError(f'max_steer_rad debe ser positivo: {self.max_steer_rad}')
        return True

    @staticmethod
    def _lerp(a: float, b: float, t: float) -> float:
        return a + (b - a) * max(0.0, min(1.0, t))

    def __repr__(self) -> str:
        return (
            f'PWMController(throttle=[{self.brake_pwm},{self.cruise_pwm},{self.max_pwm}],'
            f' dir=[{self.dir_min},{self.dir_center},{self.dir_max}])'
        )