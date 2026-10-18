import math


def _index(x, y, width):
    return y * width + x


def _chamfer(free, blocked, width, height):
    huge = 10 ** 9
    dist = [0] * (width * height)
    for i, is_free in enumerate(free):
        if is_free:
            dist[i] = huge

    for y in range(height):
        for x in range(width):
            i = _index(x, y, width)
            if not free[i]:
                continue
            best = dist[i]
            if y > 0:
                best = min(best, dist[_index(x, y - 1, width)] + 3)
                if x > 0:
                    best = min(best, dist[_index(x - 1, y - 1, width)] + 4)
                if x + 1 < width:
                    best = min(best, dist[_index(x + 1, y - 1, width)] + 4)
            if x > 0:
                best = min(best, dist[_index(x - 1, y, width)] + 3)
            dist[i] = best

    for y in range(height - 1, -1, -1):
        for x in range(width - 1, -1, -1):
            i = _index(x, y, width)
            if not free[i]:
                continue
            best = dist[i]
            if y + 1 < height:
                best = min(best, dist[_index(x, y + 1, width)] + 3)
                if x > 0:
                    best = min(best, dist[_index(x - 1, y + 1, width)] + 4)
                if x + 1 < width:
                    best = min(best, dist[_index(x + 1, y + 1, width)] + 4)
            if x + 1 < width:
                best = min(best, dist[_index(x + 1, y, width)] + 3)
            dist[i] = best
    return dist


def _cell_to_world(col, row, resolution, origin_x, origin_y, yaw):
    local_x = (col + 0.5) * resolution
    local_y = (row + 0.5) * resolution
    world_x = origin_x + math.cos(yaw) * local_x - math.sin(yaw) * local_y
    world_y = origin_y + math.sin(yaw) * local_x + math.cos(yaw) * local_y
    return world_x, world_y


def _world_to_cell(x, y, resolution, origin_x, origin_y, yaw, width, height):
    dx = x - origin_x
    dy = y - origin_y
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    col = int(math.floor(local_x / resolution))
    row = int(math.floor(local_y / resolution))
    if col < 0 or row < 0 or col >= width or row >= height:
        return None
    return col, row


def _touches_unknown(x, y, unknown, width, height):
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        nx = x + dx
        ny = y + dy
        if nx < 0 or ny < 0 or nx >= width or ny >= height:
            continue
        if unknown[_index(nx, ny, width)]:
            return True
    return False


def analyse_map(data, width, height, resolution, origin_x, origin_y, yaw=0.0, robot_xy=None):
    empty = {
        'free_count': 0,
        'interior_count': 0,
        'mean_width': None,
        'narrow': None,
        'wide': None,
        'robot_width': None,
        'robot_clearance': None,
        'image': [[0] * width for _ in range(height)] if width and height else [],
    }
    if width <= 0 or height <= 0 or resolution <= 0 or len(data) < width * height:
        return empty

    free = [False] * (width * height)
    unknown = [False] * (width * height)
    for i, value in enumerate(data[:width * height]):
        value = int(value)
        if value < 0:
            unknown[i] = True
        elif value < 50:
            free[i] = True

    dist = _chamfer(free, unknown, width, height)
    image = [[0] * width for _ in range(height)]
    widths = []
    interior = []
    widest = None
    narrowest = None

    for y in range(height):
        for x in range(width):
            i = _index(x, y, width)
            if unknown[i]:
                image[y][x] = 30
                continue
            if not free[i] or dist[i] >= 10 ** 9:
                image[y][x] = 0
                continue
            clearance = dist[i] * resolution / 3.0
            passage = clearance * 2.0
            shade = 80 + int(min(175, passage * 20))
            image[y][x] = shade
            spot = (passage, clearance, x, y)
            widths.append(spot)
            if not _touches_unknown(x, y, unknown, width, height):
                interior.append(spot)

    used = interior if interior else widths
    for passage, clearance, x, y in used:
        world = _cell_to_world(x, y, resolution, origin_x, origin_y, yaw)
        if narrowest is None or passage < narrowest[0]:
            narrowest = (passage, world[0], world[1])
        if widest is None or passage > widest[0]:
            widest = (passage, world[0], world[1])

    robot_width = None
    robot_clearance = None
    if robot_xy is not None:
        cell = _world_to_cell(
            robot_xy[0], robot_xy[1], resolution, origin_x, origin_y, yaw, width, height)
        if cell is not None:
            i = _index(cell[0], cell[1], width)
            if free[i] and dist[i] < 10 ** 9:
                robot_clearance = dist[i] * resolution / 3.0
                robot_width = robot_clearance * 2.0

    mean_width = None
    if used:
        mean_width = sum(item[0] for item in used) / float(len(used))

    return {
        'free_count': len(widths),
        'interior_count': len(interior),
        'mean_width': mean_width,
        'narrow': narrowest,
        'wide': widest,
        'robot_width': robot_width,
        'robot_clearance': robot_clearance,
        'image': image,
    }


def geometry_text(result):
    def metres(value):
        if value is None:
            return 'none'
        return '{:.2f}'.format(value)

    def spot(value):
        if value is None:
            return 'none'
        return '{:.2f} m at {:.2f} {:.2f}'.format(value[0], value[1], value[2])

    lines = [
        'Advanced 2 cave geometry',
        'free cells: ' + str(result['free_count']),
        'interior cells: ' + str(result['interior_count']),
        'mean passage width m: ' + metres(result['mean_width']),
        'narrowest passage: ' + spot(result['narrow']),
        'widest passage: ' + spot(result['wide']),
        'robot passage width m: ' + metres(result['robot_width']),
        'robot clearance m: ' + metres(result['robot_clearance']),
        'width is twice the gap from a free cell to the nearest wall or unknown cell',
    ]
    return '\n'.join(lines) + '\n'
