"""Independent occupancy/path checks shared by the end-to-end importer tests."""
import heapq
import math


def expand_rectangles(document):
    assert isinstance(document["width"], int) and document["width"] > 0
    assert isinstance(document["height"], int) and document["height"] > 0
    assert document["origin"] == "bottom-left"
    assert math.isfinite(document["cell_size"]) and document["cell_size"] > 0
    assert len(document["maps"]) == 1
    map_id, map_data = next(iter(document["maps"].items()))
    assert isinstance(map_data["desc"], str)
    blocked = set()
    for rect in map_data["obstacles_rect_x1y1x2y2"]:
        assert len(rect) == 4 and all(type(value) is int for value in rect)
        x1, y1, x2, y2 = rect
        assert 0 <= x1 <= x2 < document["width"]
        assert 0 <= y1 <= y2 < document["height"]
        for y in range(y1, y2 + 1):
            for x in range(x1, x2 + 1):
                blocked.add((x, y))
    return map_id, map_data, blocked


def dijkstra(width, height, blocked, start, goal, connectivity):
    start, goal = tuple(start), tuple(goal)
    queue = [(0.0, start)]
    distances = {start: 0.0}
    offsets = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    if connectivity == 8:
        offsets += [(1, 1), (-1, 1), (1, -1), (-1, -1)]
    while queue:
        distance, (x, y) = heapq.heappop(queue)
        if distance > distances[(x, y)]:
            continue
        if (x, y) == goal:
            return distance
        for dx, dy in offsets:
            point = (x + dx, y + dy)
            if not (0 <= point[0] < width and 0 <= point[1] < height) or point in blocked:
                continue
            if dx and dy and ((x + dx, y) in blocked or (x, y + dy) in blocked):
                continue
            candidate = distance + (math.sqrt(2.0) if dx and dy else 1.0)
            if candidate < distances.get(point, math.inf):
                distances[point] = candidate
                heapq.heappush(queue, (candidate, point))
    return None


def verify_path(result, width, height, cell_size, blocked, start, goal, connectivity):
    assert result["status"] == "STATUS_OK", result
    assert result["schema_version"] == 3
    assert result["start"] == start and result["goal"] == goal
    assert result["input_source"] == "external JSON"
    path = result["path_cells"]
    assert path == result["path"] and path[0] == start and path[-1] == goal
    cost = 0.0
    for x, y in path:
        assert 0 <= x < width and 0 <= y < height and (x, y) not in blocked
    for (x, y), (nx, ny) in zip(path, path[1:]):
        dx, dy = abs(nx - x), abs(ny - y)
        assert max(dx, dy) == 1 and dx + dy > 0
        if dx and dy:
            assert connectivity == 8
            assert (nx, y) not in blocked and (x, ny) not in blocked
            cost += math.sqrt(2.0)
        else:
            cost += 1.0
    expected = dijkstra(width, height, blocked, start, goal, connectivity)
    assert expected is not None
    assert math.isclose(cost, expected, rel_tol=1e-12, abs_tol=1e-10)
    assert math.isclose(result["core_cost_grid_units"], expected, rel_tol=1e-12, abs_tol=1e-10)
    assert math.isclose(result["path_length"], expected * cell_size, rel_tol=1e-12, abs_tol=1e-10)
    assert result["path_centers"] == [[(x + 0.5)*cell_size, (y + 0.5)*cell_size] for x, y in path]
    return {"steps": len(path) - 1, "cost_grid_units": expected, "path_length": expected * cell_size}
