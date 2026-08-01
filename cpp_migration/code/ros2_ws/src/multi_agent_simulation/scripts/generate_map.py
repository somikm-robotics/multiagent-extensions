from PIL import Image, ImageDraw

# ============================================================
# Map configuration
# ============================================================

MAP_SIZE_METERS = 40.0
RESOLUTION = 0.05

PIXELS = int(MAP_SIZE_METERS / RESOLUTION)

#WALL_THICKNESS_METERS = 0.10
#WALL_PIXELS = int(WALL_THICKNESS_METERS / RESOLUTION) * 2

# ============================================================
# Create map
# ============================================================

img = Image.new("L", (PIXELS, PIXELS), 255)
draw = ImageDraw.Draw(img)

# ============================================================
# Draw outer walls
# ============================================================

# Top
draw.line((0, 2, PIXELS-1, 2), fill=0, width=4)

# Bottom
draw.line((0, PIXELS-3, PIXELS-1, PIXELS-3), fill=0, width=4)

# Left
draw.line((2, 0, 2, PIXELS-1), fill=0, width=4)

# Right
draw.line((PIXELS-3, 0, PIXELS-3, PIXELS-1), fill=0, width=4)

# ============================================================
# Coordinate conversion
#
# Gazebo:
#   X : -20 -> +20
#   Y : -20 -> +20
#
# Image:
#   (0,0) top left
# ============================================================

def world_to_pixel(x, y):
    px = round((x + MAP_SIZE_METERS / 2.0) / RESOLUTION)
    py = round((MAP_SIZE_METERS / 2.0 - y) / RESOLUTION)

    px = max(0, min(PIXELS - 1, px))
    py = max(0, min(PIXELS - 1, py))

    return px, py


# ============================================================
# Helpers
# ============================================================

def draw_circle(x, y, radius):

    px, py = world_to_pixel(x, y)

    r = int(radius / RESOLUTION)

    draw.ellipse(
        (px-r, py-r, px+r, py+r),
        fill=0
    )


def draw_rectangle(x, y, width, length):

    px, py = world_to_pixel(x, y)

    w = int(width / RESOLUTION)
    l = int(length / RESOLUTION)

    draw.rectangle(
        (
            px-w//2,
            py-l//2,
            px+w//2,
            py+l//2
        ),
        fill=0
    )

# ============================================================
# Permanent obstacles
# ============================================================

# Office building
draw_rectangle(
    x=-19.0,
    y=17.0,
    width=2.5,
    length=2.5
)

# Wood case
draw_rectangle(
    x=4.5,
    y=9.0,
    width=1.0,
    length=1.0
)

# Mossy bark (log)
draw_rectangle(
    x=8.0,
    y=-7.5,
    width=0.6,
    length=2.5
)

# Medium rock fall
draw_circle(
    x=1.5,
    y=8.3,
    radius=1.0
)

# Small rock fall
draw_circle(
    x=6.5,
    y=7.2,
    radius=0.6
)

# ============================================================
# Save
# ============================================================

img.save("mining_map.pgm")

print("Created mining_map.pgm")