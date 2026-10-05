"""Independent acceptance checks for Code2 benchmark JSON/log outputs.

No application modules or generated fixtures are imported. Grid rectangles,
query endpoints, rounded targets and campus roads are transcribed from the
application specification; optimal costs are recomputed with Dijkstra.
"""
from __future__ import annotations

import argparse
from functools import lru_cache
import heapq
import json
import math
from pathlib import Path
import re
import sys


RECTANGLES = {
    "M1_U_shape": ((30,30,32,70),(68,30,70,70),(30,68,70,70)),
    "M2_dead_end": ((10,45,80,47),(10,53,80,55),(78,45,80,55)),
    "M3_complex": ((20,20,80,22),(20,20,22,80),(20,78,80,80),
                   (78,20,80,45),(78,55,80,80),(35,22,37,65),
                   (50,35,52,78),(65,22,67,65),(80,45,92,47),
                   (80,53,92,55),(88,85,98,86),(88,95,98,96),
                   (88,85,89,96),(97,85,98,96)),
    "free": (),
}
ROBOT_CASES = {
    "M1-C1": ("M1_U_shape",(50,60),(50,85),110.84,129.00),
    "M1-C2": ("M1_U_shape",(50,60),(85,85),103.84,122.00),
    "M1-C3": ("M1_U_shape",(40,65),(15,90),109.70,122.00),
    "M2-C1": ("M2_dead_end",(75,50),(90,50),154.31,159.00),
    "M2-C2": ("M2_dead_end",(75,50),(75,80),146.77,162.00),
    "M2-C3": ("M2_dead_end",(40,50),(90,50),119.31,124.00),
    "M3-C1": ("M3_complex",(28,70),(95,50),128.15,151.00),
    "M3-C2": ("M3_complex",(58,30),(10,90),183.74,206.00),
    "M3-C3": ("M3_complex",(44,70),(95,20),145.67,165.00),
    "M3-C4": ("M3_complex",(72,30),(5,5),150.67,170.00),
    "M3-C5": ("M3_complex",(28,70),(93,90),None,None),
    "EC-1": ("free",(50,50),(50,50),0.0,0.0),
    "EC-2": ("M1_U_shape",(31,50),(50,50),None,None),
    "EC-3": ("free",(150,50),(50,50),None,None),
}
BLOCKED = {name: {(x,y) for x1,y1,x2,y2 in rectangles
                  for x in range(x1,x2+1) for y in range(y1,y2+1)}
           for name,rectangles in RECTANGLES.items()}
COORDINATES = {"A":(0,0),"B":(3,1),"C":(1,4),"D":(5,3),
               "E":(4,6),"F":(7,4),"G":(8,1)}
ROADS = (("A","B",320),("A","C",410),("B","C",360),("B","D",280),
         ("C","E",320),("D","E",300),("D","F",220),("E","F",280),
         ("F","G",350),("D","G",420))
CAMPUS_QUERIES = {
    "baseline": ("A","F",820,False), "A-G":("A","G",1020,False),
    "C-G":("C","G",950,False), "E-B":("E","B",580,False),
    "A-A":("A","A",0,False), "F-A":("F","A",820,False),
    "unknown":(None,None,None,False), "disconnected":("A","F",None,True),
}


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def field(record, *names):
    present = [(key, record[key]) for key in names if key in record]
    require(bool(present), "Missing field: " + "/".join(names))
    require(all(value == present[0][1] for _,value in present),
            "Conflicting aliases: " + "/".join(names))
    return present[0][1]


def json_read(path):
    def invalid_constant(value):
        raise ValueError("Nonstandard JSON number: " + value)
    return json.loads(path.read_text(encoding="utf-8-sig"),
                      parse_constant=invalid_constant)


def free(cell, blocked):
    x,y = cell
    return 0 <= x < 100 and 0 <= y < 100 and cell not in blocked


def robot_neighbors(cell, blocked, connectivity):
    x,y = cell
    for dx in (-1,0,1):
        for dy in (-1,0,1):
            if not (dx or dy) or (connectivity == 4 and dx and dy):
                continue
            neighbor = (x+dx,y+dy)
            if not free(neighbor, blocked):
                continue
            if dx and dy and (not free((x+dx,y),blocked)
                              or not free((x,y+dy),blocked)):
                continue
            yield neighbor, math.sqrt(2.0) if dx and dy else 1.0


def dijkstra(start, goal, neighbors):
    queue = [(0.0,start)]
    distances = {start:0.0}
    while queue:
        cost,current = heapq.heappop(queue)
        if cost != distances[current]:
            continue
        if current == goal:
            return cost
        for neighbor,weight in neighbors(current):
            candidate = cost+weight
            if candidate < distances.get(neighbor,math.inf):
                distances[neighbor] = candidate
                heapq.heappush(queue,(candidate,neighbor))
    return None


@lru_cache(None)
def robot_optimum(map_id,start,goal,connectivity):
    blocked = BLOCKED[map_id]
    require(free(start,blocked) and free(goal,blocked), "Oracle endpoints invalid")
    return dijkstra(start,goal,lambda cell: robot_neighbors(cell,blocked,connectivity))


def campus_graph(disconnected):
    graph = {name:{} for name in COORDINATES}
    for a,b,weight in ROADS:
        if disconnected and "F" in (a,b):
            continue
        graph[a][b] = weight
        graph[b][a] = weight
    return graph


def timing_check(record):
    value = field(record,"execution_time_ms","computational_time_ms","time_ms")
    run_count = field(record,"run_count","runs","repetitions","timing_runs","benchmark_runs")
    expected_runs = 0 if record["status"] == "STATUS_INVALID" else 10
    require(run_count == expected_runs,
            "Ten timed searches required for valid input; invalid input must not call core")
    if expected_runs == 0:
        require(value is None,"Invalid input must have absent timing, not measured validation time")
    else:
        require(isinstance(value,(int,float)) and math.isfinite(value) and value >= 0,
                "Time must be finite nonnegative milliseconds")
    scope = field(record,"timing_scope")
    require(scope in ("core_search","core-call-only",
                     "Astar::findPath only; fresh internal search state on every call",
                     "astar::Astar::findPath only; fresh state per call",
                     "astar::Astar::findPath only; fresh state per call; validation and export excluded"),
            "Timing must identify core call only")
    statistic = field(record,"timing_statistic","time_statistic","timing_aggregation")
    require(statistic in ("mean","arithmetic mean"), "Timing statistic must be mean")
    samples = record.get("search_times_ms",record.get("timing_samples_ms",record.get("run_times_ms")))
    if samples is not None:
        require(len(samples) == expected_runs,"Timing sample count differs from core calls")
        require(all(isinstance(t,(int,float)) and math.isfinite(t) and t >= 0 for t in samples),
                "Timing samples must be finite and nonnegative")
        if expected_runs:
            require(abs(sum(samples)/expected_runs-value) <= 1e-6,"Reported mean differs from samples")
    return {"mean_ms":value,"run_count":run_count,"scope":scope,
            "raw_samples_checked":samples is not None}


def check_log(log_path,record,robot):
    text = log_path.read_text(encoding="utf-8-sig")
    require(bool(text.strip()),"Empty log")
    def metric(*labels):
        for label in labels:
            match = re.search(r'^[^\n]*?'+re.escape(label)+r'\s*:\s*([^\n]*)$',text,re.MULTILINE)
            if match:
                return match.group(1).strip()
        raise AssertionError("Log missing metric: " + "/".join(labels))
    def number(*labels):
        value = metric(*labels)
        match = re.match(r'([-+]?\d+(?:\.\d+)?)',value)
        require(match is not None,"Log value is not numeric: " + value)
        return float(match.group(1))
    status = record["status"]
    require(metric("status","Status","Trạng thái") == status,"Log/JSON status mismatch")
    distance = field(record,"path_length","path_length_meters","cost")
    distance_labels = ("path_length","path_length_meters","Path length","Độ dài đường đi")
    printed = metric(*distance_labels)
    if distance is None:
        require(printed.startswith("N/A"),"Failed log distance must be N/A")
    else:
        require(abs(number(*distance_labels)-distance) <= 1e-6,
                "Log/JSON distance mismatch")
    require(number("total_steps","Total steps (edges)","Số chặng di chuyển") == record["total_steps"],"Move counts disagree")
    require(number("path_cell_count","Path cell count","Số ô trong đường đi","path_node_count","Số nút trong đường đi")
            == field(record,"path_cell_count","path_node_count"),"Path counts disagree")
    require(number("nodes_expanded","Nodes expanded (neighbor processing; goal excluded)","Số nút đã mở rộng")
            == field(record,"evaluated_nodes","number_of_nodes_expanded","nodes_expanded"),
            "Expansion counts disagree")
    reported_time = field(record,"execution_time_ms","computational_time_ms","time_ms")
    time_labels = ("execution_time_ms","computational_time_ms","time_ms (10x)","time_ms","Computational time","Thời gian tính toán")
    if reported_time is None:
        require(metric(*time_labels).startswith("N/A"),"Invalid log timing must be N/A")
    else:
        require(abs(number(*time_labels)-reported_time) <= 1e-6,"Log/JSON timing mismatch")
    if robot:
        require(number("n_turns") == field(record,"turns","n_turns"),"Turn counts disagree")
    # The human-readable timing metadata must identify all three components.
    require((record["timing_scope"] in text or "Astar::findPath only" in text) and "mean" in text,
            "Log must identify arithmetic mean and core timing scope")
    if robot:
        log_runs = number("timing_runs","benchmark_runs","run_count")
    else:
        match = re.search(r'mean of (\d+) fresh core searches',text)
        require(match is not None,"Log must identify timed search count")
        log_runs = int(match.group(1))
    require(log_runs == field(record,"timing_runs","benchmark_runs","run_count"),"Timing run counts disagree")
    if robot:
        require(number("unique_obstacle_cells") == record["unique_obstacle_cells"],"Obstacle counts disagree")
        path_line = metric("path")
        printed_path = [[int(x),int(y)] for x,y in re.findall(r'\((-?\d+)\s*,\s*(-?\d+)\)',path_line)]
        require(printed_path == record["path"],"Log/JSON coordinate paths disagree")
    else:
        path_line = metric("Path","Lộ trình tìm được")
        printed_path = [] if path_line in ("N/A","[]") else [item.strip() for item in path_line.split("->")]
        require(printed_path == record["path"],"Log/JSON campus paths disagree")
    return {"log":str(log_path),"shared_metrics_agree":True}


def robot_check(path,experiment):
    record = json_read(path)
    case_id = field(record,"case","case_id")
    require(case_id in ROBOT_CASES,"Unknown robot case: " + str(case_id))
    map_id,start,goal,eight_target,four_target = ROBOT_CASES[case_id]
    connectivity = 4 if experiment == "E3" else 8
    require(field(record,"experiment","configuration") == experiment,"Experiment metadata mismatch")
    require(field(record,"connectivity") == connectivity,"Connectivity metadata mismatch")
    expected_heuristic = {"E1":"octile","E2":"zero","E3":"manhattan"}[experiment]
    actual_heuristic = field(record,"heuristic").lower()
    require(actual_heuristic == expected_heuristic or
            (expected_heuristic == "zero" and actual_heuristic == "zero (dijkstra)"),"Heuristic mismatch")
    blocked = BLOCKED[map_id]
    require(field(record,"unique_obstacle_cells") == len(blocked),"Unique obstacle count differs from rectangle union")
    expected_status = ("STATUS_INVALID" if not free(start,blocked) or not free(goal,blocked)
                       else "STATUS_NO_PATH" if case_id == "M3-C5" else "STATUS_OK")
    require(record["status"] == expected_status,"Robot status mismatch")
    require(field(record,"start","start_cell") == list(start),"Start metadata mismatch")
    require(field(record,"goal","goal_cell") == list(goal),"Goal metadata mismatch")
    if not case_id.startswith("EC"):
        require(field(record,"map","map_id") == map_id,"Map identity mismatch")
    cells = field(record,"path","path_cells")
    require(isinstance(cells,list),"Path must be an array")
    require(all(isinstance(p,list) and len(p)==2 and
                all(isinstance(v,int) and not isinstance(v,bool) for v in p) for p in cells),
            "Cells must be pairs of integer indices")
    require(field(record,"path_cell_count") == len(cells),"Cell count mismatch")
    require(record["total_steps"] == max(0,len(cells)-1),"Moves must equal cell count minus one")
    expansions = field(record,"evaluated_nodes","nodes_expanded")
    require(isinstance(expansions,int) and expansions>=0,"Invalid expansion count")
    distance = field(record,"path_length","cost")
    turns = field(record,"turns","n_turns")
    timing = timing_check(record)
    measured = {"json":str(path),"application":"APP1","case":case_id,
                "experiment":experiment,"status":record["status"],"timing":timing}
    if expected_status != "STATUS_OK":
        require(cells == [] and distance is None and turns == 0,"Failure must have empty path/null cost/zero turns")
        if expected_status == "STATUS_INVALID":
            require(expansions == 0,"Invalid inputs must not expand nodes")
            require(timing["mean_ms"] is None,"Invalid validation must be excluded from core timing")
        else:
            require(robot_optimum(map_id,start,goal,connectivity) is None,"Expected sealed goal is reachable")
            measured["unreachable_verified"] = True
    else:
        require(cells and tuple(cells[0]) == start and tuple(cells[-1]) == goal,"Path endpoints disagree")
        require(all(free(tuple(p),blocked) for p in cells),"Path enters obstacle/outside grid")
        vectors = []
        recomputed = 0.0
        for a,b in zip(cells,cells[1:]):
            edges = dict(robot_neighbors(tuple(a),blocked,connectivity))
            require(tuple(b) in edges,"Illegal step or diagonal corner cutting")
            recomputed += edges[tuple(b)]
            vectors.append((b[0]-a[0],b[1]-a[1]))
        require(abs(recomputed-distance) <= 1e-6,"Reported length differs from path")
        require(turns == sum(a!=b for a,b in zip(vectors,vectors[1:])),"Turn count differs from path")
        optimum = robot_optimum(map_id,start,goal,connectivity)
        require(abs(recomputed-optimum) <= 1e-9,"Returned path is not optimal")
        target = four_target if connectivity == 4 else eight_target
        require(abs(distance-target) <= 0.01,"Rounded specification target mismatch")
        if start == goal:
            require(expansions == turns == distance == record["total_steps"] == 0,"Trivial metrics must be zero")
        measured.update({"optimal_cost":optimum,"path_cost":recomputed,"turns":turns,
                         "moves":len(cells)-1,"cells":len(cells),"legal_and_optimal":True})
    log_path = path.parent.parent/"log"/path.name.replace("benchmark_result_","benchmark_log_").replace(".json",".txt")
    measured.update(check_log(log_path,record,True))
    return record,measured


def campus_check(path,query_name,algorithm):
    record = json_read(path)
    start = field(record,"start_node","start")
    goal = field(record,"goal_node","goal")
    expected_start,expected_goal,target,disconnected = CAMPUS_QUERIES[query_name]
    if expected_start is not None:
        require(start == expected_start and goal == expected_goal,"Campus endpoint metadata mismatch")
    else:
        require(start not in COORDINATES or goal not in COORDINATES,"Unknown-ID test uses valid IDs")
    graph = campus_graph(disconnected)
    expected_status = "STATUS_INVALID" if query_name == "unknown" else "STATUS_NO_PATH" if disconnected else "STATUS_OK"
    require(record["status"] == expected_status,"Campus status mismatch")
    path_nodes = field(record,"path","path_node_ids")
    require(field(record,"path_node_count","path_cell_count") == len(path_nodes),"Node count mismatch")
    require(record["total_steps"] == max(0,len(path_nodes)-1),"Campus move count mismatch")
    distance = field(record,"path_length_meters","cost_m","cost")
    expansions = field(record,"number_of_nodes_expanded","nodes_expanded")
    require(isinstance(expansions,int) and expansions>=0,"Invalid expansion count")
    require(field(record,"heuristic") == ("euclidean" if algorithm == "a_star" else "zero"),"Campus heuristic mismatch")
    if algorithm == "a_star":
        require(field(record,"heuristic_scale","alpha","heuristic_alpha") == 1,"Baseline Euclidean scale must equal one")
    timing = timing_check(record)
    measured = {"json":str(path),"application":"APP2","query":query_name,
                "algorithm":algorithm,"status":record["status"],"timing":timing}
    if expected_status != "STATUS_OK":
        require(path_nodes == [] and distance is None,"Campus failures require empty path and null cost")
        if expected_status == "STATUS_INVALID":
            require(expansions == 0 and timing["mean_ms"] is None,"Invalid query must not search")
        else:
            require(dijkstra(start,goal,lambda node:graph[node].items()) is None,"Disconnected fixture is reachable")
            measured["unreachable_verified"] = True
    else:
        require(path_nodes and path_nodes[0] == start and path_nodes[-1] == goal,"Campus path endpoints disagree")
        require(all(node in graph for node in path_nodes),"Path contains unknown campus node")
        cost = 0
        for a,b in zip(path_nodes,path_nodes[1:]):
            require(b in graph[a],"Path uses a nonexistent road")
            cost += graph[a][b]
        optimum = dijkstra(start,goal,lambda node:graph[node].items())
        require(cost == distance == optimum == target,"Campus path cost is not optimal/specification target")
        if query_name == "baseline":
            require(path_nodes == ["A","B","D","F"] and expansions == 5,"A-F must have specified path and five expansions")
        if start == goal:
            require(expansions == distance == record["total_steps"] == 0,"Trivial campus metrics must be zero")
        measured.update({"optimal_cost_m":optimum,"path_cost_m":cost,"legal_and_optimal":True})
    log_path = path.with_name(path.name.replace("result","log").replace(".json",".txt"))
    measured.update(check_log(log_path,record,False))
    return record,measured


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root",type=Path,default=Path(__file__).resolve().parents[1]/"verified")
    args = parser.parse_args()
    root = args.root.resolve()
    summary = {"validator":"independent Python standard-library specification oracle",
               "root":str(root),"required_robot_pairs":42,"required_campus_pairs":16,
               "records":[],"errors":[],"comparison_files_verified":0,
               "paired_robot_cost_agreements":0}
    outputs = {}
    def run(path,operation):
        try:
            record,details = operation()
            outputs[str(path)] = record
            summary["records"].append(details)
        except Exception as exc:
            summary["errors"].append({"file":str(path),"error":str(exc)})
    for experiment in ("E1","E2","E3"):
        folder = root/"APP1"/experiment/"result"
        actual = {p.name for p in folder.glob("*.json")}
        expected = {f"benchmark_result_{case_id}.json" for case_id in ROBOT_CASES}
        if actual != expected:
            summary["errors"].append({"file":str(folder),"error":"Robot file set differs from fourteen required cases",
                                      "missing":sorted(expected-actual),"unexpected":sorted(actual-expected)})
        for case_id in ROBOT_CASES:
            path = folder/f"benchmark_result_{case_id}.json"
            run(path,lambda path=path,experiment=experiment:robot_check(path,experiment))
    for case_id in ROBOT_CASES:
        first_path = root/"APP1"/"E1"/"result"/f"benchmark_result_{case_id}.json"
        second_path = root/"APP1"/"E2"/"result"/f"benchmark_result_{case_id}.json"
        if str(first_path) not in outputs or str(second_path) not in outputs:
            continue
        first = outputs[str(first_path)]
        second = outputs[str(second_path)]
        same_cost = (first["path_length"] is None and second["path_length"] is None)
        if first["path_length"] is not None and second["path_length"] is not None:
            same_cost = abs(first["path_length"]-second["path_length"]) <= 1e-9
        if same_cost and first["status"] == second["status"]:
            summary["paired_robot_cost_agreements"] += 1
        else:
            summary["errors"].append({"file":str(first_path),"error":"E1/E2 status or optimal costs disagree"})
    for query_name in CAMPUS_QUERIES:
        folder = root/"APP2" if query_name == "baseline" else root/"APP2_cases"/query_name
        individual = {}
        for algorithm,filename in (("a_star","a_star_result_App2.json"),("dijkstra","dijkstra_result_App2.json")):
            path = folder/filename
            run(path,lambda path=path,query_name=query_name,algorithm=algorithm:campus_check(path,query_name,algorithm))
            if str(path) in outputs:
                individual[algorithm] = outputs[str(path)]
        combined_path = folder/"benchmark_result_App2.json"
        try:
            require(len(individual)==2,"Individual campus pair failed validation")
            combined = json_read(combined_path)
            algorithms = field(combined,"algorithms")
            for algorithm,record in individual.items():
                combined_record = algorithms[algorithm]
                for key,value in combined_record.items():
                    if key in record:
                        require(value == record[key],"Combined/individual mismatch: "+algorithm+"/"+key)
                require(field(combined_record,"path") == record["path"],"Combined path differs")
                require(field(combined_record,"path_length_meters") == record["path_length_meters"],"Combined cost differs")
            require(individual["a_star"]["path_length_meters"] == individual["dijkstra"]["path_length_meters"],
                    "A* and Dijkstra costs disagree")
            summary["comparison_files_verified"] += 1
        except Exception as exc:
            summary["errors"].append({"file":str(combined_path),"error":str(exc)})
    summary["robot_pairs_verified"] = sum(record["application"]=="APP1" for record in summary["records"])
    summary["campus_pairs_verified"] = sum(record["application"]=="APP2" for record in summary["records"])
    summary["passed"] = not summary["errors"]
    summary["timing_limit"] = "Metadata and optional sample means checked; source review is needed to prove timer placement and state reset."
    root.mkdir(parents=True,exist_ok=True)
    target = root/"validation_summary.json"
    target.write_text(json.dumps(summary,indent=2,ensure_ascii=False,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps({key:summary[key] for key in ("passed","robot_pairs_verified","campus_pairs_verified","comparison_files_verified")},indent=2))
    if summary["errors"]:
        print(json.dumps(summary["errors"],indent=2,ensure_ascii=False))
    print("Summary: "+str(target))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
