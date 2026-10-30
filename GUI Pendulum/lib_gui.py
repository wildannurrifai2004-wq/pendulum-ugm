import math
import pygame

# posisi & ukuran tombol record
REC_BUTTON_RECT = pygame.Rect(20, 100, 120, 30)

# ====== Tombol kendali manual (pengganti stick) ======
# Gerak kiri/kanan (tekan-tahan, mengisi nilai ax)
LEFT_BTN_RECT  = pygame.Rect(20, 60, 75, 42)
RIGHT_BTN_RECT = pygame.Rect(102, 60, 75, 42)

# Tombol command (mengisi nilai 'buttons'), meniru kotak/segitiga/bulat/x
# value harus PERSIS sama dengan yang dicek STM32 (gJoy.buttons == value)
CMD_BUTTONS = {
    "homing_sin": {"rect": pygame.Rect(190, 60, 140, 36), "value": 8, "label": "Homing / Sin (8)"},
    "swingup":    {"rect": pygame.Rect(190, 102, 140, 36), "value": 4, "label": "Swing-up (4)"},
    "balance":    {"rect": pygame.Rect(190, 144, 140, 36), "value": 1, "label": "Balance (1)"},
    "stop":       {"rect": pygame.Rect(190, 186, 140, 36), "value": 2, "label": "Stop (2)"},
}

MANUAL_AX_SPEED = 32767  # nilai ax saat tombol kiri/kanan ditekan penuh


def get_manual_hit(pos):
    """Return 'left' / 'right' jika pos kena tombol gerak, else None."""
    if LEFT_BTN_RECT.collidepoint(pos):
        return "left"
    if RIGHT_BTN_RECT.collidepoint(pos):
        return "right"
    return None


def get_cmd_hit(pos):
    """Return key CMD_BUTTONS jika pos kena salah satu tombol command, else None."""
    for key, btn in CMD_BUTTONS.items():
        if btn["rect"].collidepoint(pos):
            return key
    return None


def draw_manual_controls(screen, font, held_direction, held_cmd_key):
    """
    held_direction: None / "left" / "right" -> untuk highlight tombol gerak
    held_cmd_key: None / key di CMD_BUTTONS -> untuk highlight tombol command
    """
    # tombol kiri
    color = (250, 180, 60) if held_direction == "left" else (80, 80, 90)
    pygame.draw.rect(screen, color, LEFT_BTN_RECT, border_radius=6)
    pygame.draw.rect(screen, (255, 255, 255), LEFT_BTN_RECT, 2, border_radius=6)
    t = font.render("KIRI", True, (255, 255, 255))
    screen.blit(t, t.get_rect(center=LEFT_BTN_RECT.center))

    # tombol kanan
    color = (250, 180, 60) if held_direction == "right" else (80, 80, 90)
    pygame.draw.rect(screen, color, RIGHT_BTN_RECT, border_radius=6)
    pygame.draw.rect(screen, (255, 255, 255), RIGHT_BTN_RECT, 2, border_radius=6)
    t = font.render("KANAN", True, (255, 255, 255))
    screen.blit(t, t.get_rect(center=RIGHT_BTN_RECT.center))

    # tombol command
    for key, btn in CMD_BUTTONS.items():
        rect = btn["rect"]
        color = (60, 200, 120) if held_cmd_key == key else (60, 120, 180)
        pygame.draw.rect(screen, color, rect, border_radius=6)
        pygame.draw.rect(screen, (255, 255, 255), rect, 2, border_radius=6)
        t = font.render(btn["label"], True, (255, 255, 255))
        screen.blit(t, t.get_rect(center=rect.center))


def init_display(width: int, height: int, caption: str):
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption(caption)
    font = pygame.font.SysFont("Arial", 18)
    clock = pygame.time.Clock()
    return screen, font, clock


def is_record_button(pos) -> bool:
    return REC_BUTTON_RECT.collidepoint(pos)

def draw_ui(screen, font, ax, ay, rx, ry, buttons, seq, recording: bool):
    screen.fill((30, 30, 30))

    # teks joystick
    line1 = f"AX: {ax:6d}  AY: {ay:6d}  RX: {rx:6d}  RY: {ry:6d}"
    line2 = f"Buttons: 0x{buttons:04X}"
    line3 = f"Seq: {seq}"

    screen.blit(font.render(line1, True, (255, 255, 255)), (20, 20))
    screen.blit(font.render(line2, True, (200, 200, 255)), (20, 45))
    screen.blit(font.render(line3, True, (200, 255, 200)), (20, 70))

    # tombol RECORD
    if recording:
        btn_color = (200, 50, 50)
        text = "REC ON"
    else:
        btn_color = (80, 80, 80)
        text = "REC OFF"

    pygame.draw.rect(screen, btn_color, REC_BUTTON_RECT)
    pygame.draw.rect(screen, (255, 255, 255), REC_BUTTON_RECT, 2)

    text_surf = font.render(text, True, (255, 255, 255))
    text_rect = text_surf.get_rect(center=REC_BUTTON_RECT.center)
    screen.blit(text_surf, text_rect)

def draw_pendulum(screen, cmX, theta):
    W, H = screen.get_size()

    ground_y = H - 80
    LEFT_MARGIN = 80
    #center_x = W // 2

    # scaling
    CM_TO_PX = 5        # 1 cm = 5 px
    L = 150             # panjang batang (px)

    # posisi cart
    cart_x = int(LEFT_MARGIN + cmX * CM_TO_PX)
    cart_y = ground_y

    cart_w = 80
    cart_h = 30

    # gambar ground
    pygame.draw.line(screen, (200, 200, 200),
                     (0, ground_y + cart_h // 2),
                     (W, ground_y + cart_h // 2), 2)

    # gambar cart
    cart_rect = pygame.Rect(
        cart_x - cart_w // 2,
        cart_y - cart_h // 2,
        cart_w,
        cart_h
    )
    pygame.draw.rect(screen, (100, 180, 255), cart_rect, border_radius=6)

    # posisi ujung pendulum
    pend_x = int(cart_x + L * math.sin(theta))
    pend_y = int(cart_y - L * math.cos(theta))

    # batang
    pygame.draw.line(screen, (255, 255, 255),
                     (cart_x, cart_y),
                     (pend_x, pend_y), 4)

    # massa
    pygame.draw.circle(screen, (255, 80, 80),
                       (pend_x, pend_y), 10)