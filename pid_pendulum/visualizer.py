"""Pygame renderer for the inverted pendulum scene."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

import numpy as np

try:
    import pygame
except Exception:  # pragma: no cover - optional during headless tests
    pygame = None


@dataclass
class VisualConfig:
    width: int = 900
    height: int = 500
    cart_width: int = 120
    cart_height: int = 40
    pixels_per_meter: float = 180.0
    rod_pixels: int = 180
    target_fps: int = 60


class PendulumVisualizer:
    def __init__(self, cfg: VisualConfig, enabled: bool = True) -> None:
        self.cfg = cfg
        self.enabled = enabled and pygame is not None
        self.screen = None
        self.clock = None
        self.font = None

        if self.enabled:
            pygame.init()
            self.screen = pygame.display.set_mode((cfg.width, cfg.height))
            pygame.display.set_caption("PID Inverted Pendulum")
            self.clock = pygame.time.Clock()
            self.font = pygame.font.SysFont("Arial", 18)
        else:
            self.enabled = False

    def process_events(self) -> bool:
        if not self.enabled:
            return True
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
        return True

    def draw(
        self,
        state: np.ndarray,
        control_force: float,
        sim_time: float,
        gains: Dict[str, float],
    ) -> None:
        if not self.enabled:
            return

        x, _, theta, _ = state
        assert self.screen is not None
        assert self.clock is not None
        assert self.font is not None

        self.screen.fill((248, 248, 248))

        ground_y = int(self.cfg.height * 0.72)
        center_x = self.cfg.width // 2
        cart_x_px = int(center_x + x * self.cfg.pixels_per_meter)

        pygame.draw.line(self.screen, (60, 60, 60), (0, ground_y), (self.cfg.width, ground_y), 3)

        cart_rect = pygame.Rect(
            cart_x_px - self.cfg.cart_width // 2,
            ground_y - self.cfg.cart_height,
            self.cfg.cart_width,
            self.cfg.cart_height,
        )
        pygame.draw.rect(self.screen, (60, 110, 180), cart_rect, border_radius=6)

        pivot = (cart_x_px, ground_y - self.cfg.cart_height)
        bob_x = int(pivot[0] + self.cfg.rod_pixels * np.sin(theta))
        bob_y = int(pivot[1] - self.cfg.rod_pixels * np.cos(theta))

        pygame.draw.line(self.screen, (30, 30, 30), pivot, (bob_x, bob_y), 5)
        pygame.draw.circle(self.screen, (210, 70, 70), (bob_x, bob_y), 16)
        pygame.draw.circle(self.screen, (20, 20, 20), pivot, 5)

        text_lines = [
            f"time: {sim_time:6.2f} s",
            f"theta: {np.degrees(theta):7.2f} deg",
            f"force: {control_force:7.2f} N",
            f"Kp={gains['kp']:.2f}  Ki={gains['ki']:.2f}  Kd={gains['kd']:.2f}",
        ]
        for idx, line in enumerate(text_lines):
            txt = self.font.render(line, True, (20, 20, 20))
            self.screen.blit(txt, (12, 12 + idx * 24))

        pygame.display.flip()
        self.clock.tick(self.cfg.target_fps)

    def close(self) -> None:
        if self.enabled and pygame is not None:
            pygame.quit()
