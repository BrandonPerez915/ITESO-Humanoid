# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

"""
Gamepad Controller Module for Berkeley Humanoid Lite

This module implements controllers for the Berkeley Humanoid Lite robot,
supporting gamepad devices (PS5 DualSense, Xbox, and generic controllers)
via Pygame (cross-platform, recommended for macOS) and inputs (Linux fallback).
"""

import os
import sys
import time
import threading
from typing import Dict

from common.logging import get_logger, status_line

log = get_logger("inputs.gamepad")

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False

try:
    from inputs import get_gamepad
    INPUTS_AVAILABLE = True
except ImportError:
    INPUTS_AVAILABLE = False

# Enable background events for SDL/Pygame so gamepad works even when window is not focused
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

class XInputEntry:
    """
    Constants for gamepad button and axis mappings (XInput / inputs library).
    """
    AXIS_X_L = "ABS_X"
    AXIS_Y_L = "ABS_Y"
    AXIS_TRIGGER_L = "ABS_Z"
    AXIS_X_R = "ABS_RX"
    AXIS_Y_R = "ABS_RY"
    AXIS_TRIGGER_R = "ABS_RZ"

    BTN_HAT_X = "ABS_HAT0X"
    BTN_HAT_Y = "ABS_HAT0Y"

    BTN_A = "BTN_SOUTH"
    BTN_B = "BTN_EAST"
    BTN_X = "BTN_NORTH"
    BTN_Y = "BTN_WEST"
    BTN_BUMPER_L = "BTN_TL"
    BTN_BUMPER_R = "BTN_TR"
    BTN_THUMB_L = "BTN_THUMBL"
    BTN_THUMB_R = "BTN_THUMBR"
    BTN_BACK = "BTN_SELECT"
    BTN_START = "BTN_START"


class Se2Gamepad:
    def __init__(self,
                 stick_sensitivity: float = 1.0,
                 dead_zone: float = 0.05,
                 ) -> None:
        self.stick_sensitivity = stick_sensitivity
        self.dead_zone = dead_zone

        self._stopped = threading.Event()
        self._run_forever_thread = None

        self.backend = None  # "pygame" or "inputs"
        self.reset()

        self.commands = {
            "velocity_x": 0.0,
            "velocity_y": 0.0,
            "velocity_yaw": 0.0,
            "mode_switch": 0,
        }


    def reset(self) -> None:
        self._states = {key: 0 for key in XInputEntry.__dict__.values()}


    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self.dead_zone:
            return 0.0
        return value


    def stop(self) -> None:
        log.info("Gamepad stopping...")
        self._stopped.set()
        if self._run_forever_thread and self._run_forever_thread.is_alive():
            self._run_forever_thread.join(timeout=1.0)


    def run(self) -> None:
        self._run_forever_thread = threading.Thread(target=self.run_forever, daemon=True)
        self._run_forever_thread.start()


    def run_forever(self) -> None:
        # On macOS or when pygame is available, prefer pygame
        if sys.platform == "darwin" or PYGAME_AVAILABLE:
            if PYGAME_AVAILABLE:
                self.backend = "pygame"
                self._run_pygame()
                return
            else:
                log.warning("Running on macOS but 'pygame' is not installed.")
                log.warning("Please install pygame (`pip install pygame` or `uv pip install pygame`) to use your gamepad.")

        # Fallback to inputs on Linux / Windows
        if INPUTS_AVAILABLE:
            self.backend = "inputs"
            self._run_inputs()
        else:
            log.error("Neither 'pygame' nor 'inputs' is available for gamepad control.")


    def _run_pygame(self) -> None:
        """Pygame-based gamepad handler (supports PS5 DualSense on macOS & cross-platform)."""
        pygame.init()
        pygame.joystick.init()

        joystick = None
        while not self._stopped.is_set():
            if joystick is None:
                pygame.joystick.quit()
                pygame.joystick.init()
                num_joysticks = pygame.joystick.get_count()
                if num_joysticks > 0:
                    joystick = pygame.joystick.Joystick(0)
                    joystick.init()
                    log.info("Gamepad detected via Pygame: %s", joystick.get_name())
                else:
                    time.sleep(1.0)
                    continue

            pygame.event.pump()

            try:
                num_axes = joystick.get_numaxes()
                num_buttons = joystick.get_numbuttons()

                # Axes for PS5 DualSense / standard SDL controllers:
                # Axis 0: Left Stick X  (-1 left, +1 right)
                # Axis 1: Left Stick Y  (-1 up, +1 down)
                # Axis 2: Right Stick X (-1 left, +1 right) [or Axis 3 on some mappings]
                raw_yaw = joystick.get_axis(0) if num_axes > 0 else 0.0
                raw_x = joystick.get_axis(1) if num_axes > 1 else 0.0

                raw_y = 0.0
                if num_axes > 2:
                    raw_y = joystick.get_axis(2)

                # Invert axes appropriately for robotics frame convention:
                # Pushing Left Stick UP -> positive forward velocity (velocity_x > 0)
                # Pushing Left Stick LEFT -> positive yaw velocity (velocity_yaw > 0)
                # Pushing Right Stick LEFT -> positive lateral velocity (velocity_y > 0)
                vx = -self._apply_deadzone(raw_x)
                vyaw = -self._apply_deadzone(raw_yaw)
                vy = -self._apply_deadzone(raw_y)

                self.commands["velocity_x"] = float(vx * self.stick_sensitivity)
                self.commands["velocity_y"] = float(vy * self.stick_sensitivity)
                self.commands["velocity_yaw"] = float(vyaw * self.stick_sensitivity)

                # Buttons (DualSense SDL standard on macOS):
                # 0: Cross / A
                # 1: Circle / B
                # 2: Square / X
                # 3: Triangle / Y
                # 4: Share / Create
                # 6: Options
                # 7: L3 (Left Stick Click)
                # 8: R3 (Right Stick Click)
                # 9: L1 (Left Bumper)
                # 10: R1 (Right Bumper)
                btn_cross = joystick.get_button(0) if num_buttons > 0 else 0
                btn_circle = joystick.get_button(1) if num_buttons > 1 else 0
                btn_square = joystick.get_button(2) if num_buttons > 2 else 0

                btn_lb = 0
                btn_rb = 0
                if num_buttons > 10:
                    btn_lb = joystick.get_button(9)
                    btn_rb = joystick.get_button(10)
                elif num_buttons > 5:
                    btn_lb = joystick.get_button(4)
                    btn_rb = joystick.get_button(5)

                btn_l3 = joystick.get_button(7) if num_buttons > 7 else 0
                btn_r3 = joystick.get_button(8) if num_buttons > 8 else 0

                mode_switch = 0
                # RL Running mode (Cross + R1)
                if btn_cross and btn_rb:
                    mode_switch = 3
                # RL Init mode (Cross + L1)
                elif btn_cross and btn_lb:
                    mode_switch = 2
                # Idle mode (Square or Circle or L3 or R3)
                elif btn_square or btn_circle or btn_l3 or btn_r3:
                    mode_switch = 1

                self.commands["mode_switch"] = mode_switch

            except Exception as e:
                # Handle disconnection
                log.warning("Gamepad reading error: %s", e)
                joystick = None

            time.sleep(0.01)  # 100 Hz poll rate


    def _run_inputs(self) -> None:
        """Inputs library handler (Linux XInput)."""
        while not self._stopped.is_set():
            try:
                events = get_gamepad()
                for event in events:
                    self._states[event.code] = event.state
                self._update_command_buffer_inputs()
            except Exception as e:
                time.sleep(0.5)


    def _update_command_buffer_inputs(self) -> Dict[str, float]:
        velocity_x = self._states.get(XInputEntry.AXIS_Y_L)
        velocity_y = self._states.get(XInputEntry.AXIS_X_R)
        velocity_yaw = self._states.get(XInputEntry.AXIS_X_L)

        if velocity_x is not None:
            self.commands["velocity_x"] = self._apply_deadzone(velocity_x / -32768.0) * self.stick_sensitivity
        if velocity_y is not None:
            self.commands["velocity_y"] = self._apply_deadzone(velocity_y / -32768.0) * self.stick_sensitivity
        if velocity_yaw is not None:
            self.commands["velocity_yaw"] = self._apply_deadzone(velocity_yaw / -32768.0) * self.stick_sensitivity

        mode_switch = 0

        # Enter RL control mode (A + Right Bumper)
        if self._states.get(XInputEntry.BTN_A) and self._states.get(XInputEntry.BTN_BUMPER_R):
            mode_switch = 3

        # Enter init mode (A + Left Bumper)
        if self._states.get(XInputEntry.BTN_A) and self._states.get(XInputEntry.BTN_BUMPER_L):
            mode_switch = 2

        # Enter idle mode (X or Left/Right Thumbstick)
        if self._states.get(XInputEntry.BTN_X) or self._states.get(XInputEntry.BTN_THUMB_L) or self._states.get(XInputEntry.BTN_THUMB_R):
            mode_switch = 1

        self.commands["mode_switch"] = mode_switch
        return self.commands


class Se2Keyboard:
    """
    Keyboard controller for Berkeley Humanoid Lite.
    Supports input from both the MuJoCo viewer window and the terminal stdin.
    """
    def __init__(self,
                 step_vx: float = 0.1,
                 step_vy: float = 0.05,
                 step_vyaw: float = 0.15,
                 max_vx: float = 0.8,
                 max_vy: float = 0.4,
                 max_vyaw: float = 0.8) -> None:
        self.step_vx = step_vx
        self.step_vy = step_vy
        self.step_vyaw = step_vyaw
        self.max_vx = max_vx
        self.max_vy = max_vy
        self.max_vyaw = max_vyaw

        self.commands = {
            "velocity_x": 0.0,
            "velocity_y": 0.0,
            "velocity_yaw": 0.0,
            "mode_switch": 3,  # RL running mode by default
        }

        self._stopped = threading.Event()
        self._thread = None


    def stop(self) -> None:
        self._stopped.set()


    def run(self) -> None:
        self.print_instructions()
        self._thread = threading.Thread(target=self._terminal_listener, daemon=True)
        self._thread.start()


    def print_instructions(self) -> None:
        log.info(
            "\n" + "=" * 62 + "\n"
            "          BERKELEY HUMANOID LITE - KEYBOARD CONTROL\n"
            + "=" * 62 + "\n"
            "  W / Up Arrow        : Forward (increase forward velocity)\n"
            "  S / Down Arrow      : Backward / Brake (backward velocity)\n"
            "  A / Left Arrow      : Turn left (Yaw +)\n"
            "  D / Right Arrow     : Turn right (Yaw -)\n"
            "  Q                   : Strafe left\n"
            "  E                   : Strafe right\n"
            "  SPACE               : Brake / Stop (Vx=0, Vy=0, Vyaw=0)\n"
            "  3                   : RL Running Mode (Walk)\n"
            "  2                   : RL Init Mode (Initial posture)\n"
            "  1                   : Idle Mode\n"
            + "=" * 62 + "\n"
            "  * You can press the keys in the MuJoCo 3D window\n"
            "    or directly in this terminal."
        )


    def handle_char(self, char: str) -> None:
        char = char.lower()
        vx = self.commands["velocity_x"]
        vy = self.commands["velocity_y"]
        vyaw = self.commands["velocity_yaw"]

        if char == 'w':
            vx = min(vx + self.step_vx, self.max_vx)
        elif char == 's':
            vx = max(vx - self.step_vx, -self.max_vx)
        elif char == 'a':
            vyaw = min(vyaw + self.step_vyaw, self.max_vyaw)
        elif char == 'd':
            vyaw = max(vyaw - self.step_vyaw, -self.max_vyaw)
        elif char == 'q':
            vy = min(vy + self.step_vy, self.max_vy)
        elif char == 'e':
            vy = max(vy - self.step_vy, -self.max_vy)
        elif char == ' ':
            vx = 0.0
            vy = 0.0
            vyaw = 0.0
        elif char == '1':
            self.commands["mode_switch"] = 1
            log.info("[KEYBOARD] Mode changed to: 1 (IDLE)")
            return
        elif char == '2':
            self.commands["mode_switch"] = 2
            log.info("[KEYBOARD] Mode changed to: 2 (INIT)")
            return
        elif char == '3':
            self.commands["mode_switch"] = 3
            log.info("[KEYBOARD] Mode changed to: 3 (RL RUNNING)")
            return
        else:
            return

        self.commands["velocity_x"] = round(vx, 2)
        self.commands["velocity_y"] = round(vy, 2)
        self.commands["velocity_yaw"] = round(vyaw, 2)
        status_line(
            f"[KEYBOARD] Vx: {self.commands['velocity_x']:+.2f} m/s | Vy: {self.commands['velocity_y']:+.2f} m/s | Vyaw: {self.commands['velocity_yaw']:+.2f} rad/s",
        )


    def handle_key(self, keycode: int) -> None:
        """Handle key event from MuJoCo viewer callback."""
        # GLFW / ASCII key codes:
        if keycode in (ord('W'), ord('w'), 265):  # 265 = GLFW_KEY_UP
            self.handle_char('w')
        elif keycode in (ord('S'), ord('s'), 264):  # 264 = GLFW_KEY_DOWN
            self.handle_char('s')
        elif keycode in (ord('A'), ord('a'), 263):  # 263 = GLFW_KEY_LEFT
            self.handle_char('a')
        elif keycode in (ord('D'), ord('d'), 262):  # 262 = GLFW_KEY_RIGHT
            self.handle_char('d')
        elif keycode in (ord('Q'), ord('q')):
            self.handle_char('q')
        elif keycode in (ord('E'), ord('e')):
            self.handle_char('e')
        elif keycode == 32:  # Space
            self.handle_char(' ')
        elif keycode == ord('1'):
            self.handle_char('1')
        elif keycode == ord('2'):
            self.handle_char('2')
        elif keycode == ord('3'):
            self.handle_char('3')


    def _terminal_listener(self) -> None:
        """Reads characters from stdin in the terminal (non-blocking)."""
        if not sys.stdin.isatty():
            return

        import select
        import tty
        import termios

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while not self._stopped.is_set():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    ch = sys.stdin.read(1)
                    if ch == '\x1b':
                        # Check for ANSI escape sequences (Arrow keys)
                        r2, _, _ = select.select([sys.stdin], [], [], 0.05)
                        if r2:
                            seq = sys.stdin.read(2)
                            if seq == '[A': ch = 'w'  # Up
                            elif seq == '[B': ch = 's'  # Down
                            elif seq == '[C': ch = 'd'  # Right
                            elif seq == '[D': ch = 'a'  # Left
                    self.handle_char(ch)
        except Exception:
            pass
        finally:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

