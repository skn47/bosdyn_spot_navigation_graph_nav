import heapq
from collections import deque
from copy import deepcopy

def id_to_short_code(id):
    tokens = id.split('-')
    if len(tokens) > 2:
        return f'{tokens[0][0]}{tokens[1][0]}'
    return None

def pretty_print_waypoints(waypoint_id, waypoint_name, short_code_to_count, localization_id):
    short_code = id_to_short_code(waypoint_id)
    if short_code is None or short_code_to_count[short_code] != 1:
        short_code = '  '

    waypoint_symbol = '->' if localization_id == waypoint_id else '  '
    print(
        f'{waypoint_symbol} Waypoint name: {waypoint_name} id: {waypoint_id} short code: {short_code}'
    )

def find_unique_waypoint_id(short_code, graph, name_to_id):
    if graph is None:
        print(
            'Please list the waypoints in the map before trying to navigate to a specific one (Option #4).'
        )
        return

    if len(short_code) != 2:
        if short_code in name_to_id:
            if name_to_id[short_code] is not None:
                return name_to_id[short_code]
            else:
                print(
                    f'The waypoint name {short_code} is used for multiple different unique waypoints. Please use '
                    f'the waypoint id.')
                return None
        return short_code

    ret = short_code
    for waypoint in graph.waypoints:
        if short_code == id_to_short_code(waypoint.id):
            if ret != short_code:
                return short_code
            ret = waypoint.id
    return ret

def update_waypoints_and_edges(graph, localization_id, do_print=True):
    name_to_id = dict()
    adj = dict()
    waypoints = dict()
    edges = dict()

    short_code_to_count = {}
    waypoint_to_timestamp = []
    for waypoint in graph.waypoints:
        waypoints[waypoint.id] = waypoint
        adj.setdefault(waypoint.id, [])
        timestamp = -1.0
        try:
            timestamp = waypoint.annotations.creation_time.seconds + waypoint.annotations.creation_time.nanos / 1e9
        except:
            pass
        waypoint_to_timestamp.append((waypoint.id, timestamp, waypoint.annotations.name))

        short_code = id_to_short_code(waypoint.id)
        if short_code not in short_code_to_count:
            short_code_to_count[short_code] = 0
        short_code_to_count[short_code] += 1

        waypoint_name = waypoint.annotations.name
        if waypoint_name:
            if waypoint_name in name_to_id:
                name_to_id[waypoint_name] = None
            else:
                name_to_id[waypoint_name] = waypoint.id

    waypoint_to_timestamp = sorted(waypoint_to_timestamp, key=lambda x: (x[1], x[2]))

    if do_print:
        print(f'{len(graph.waypoints):d} waypoints:')
        for waypoint in waypoint_to_timestamp:
            pretty_print_waypoints(waypoint[0], waypoint[2], short_code_to_count, localization_id)

    for edge in graph.edges:
        adj[edge.id.from_waypoint].append((edge.id.to_waypoint, edge.annotations.cost.value))
        if edge.id.to_waypoint in edges:
            if edge.id.from_waypoint not in edges[edge.id.to_waypoint]:
                edges[edge.id.to_waypoint].append(edge.id.from_waypoint)
        else:
            edges[edge.id.to_waypoint] = [edge.id.from_waypoint]
        if do_print:
            print(f'(Edge) from waypoint {edge.id.from_waypoint} to waypoint {edge.id.to_waypoint} '
                  f'(cost {edge.annotations.cost.value})')

    return name_to_id, adj, waypoints, edges

def sort_waypoints_chrono(graph):
    waypoint_to_timestamp = []
    for waypoint in graph.waypoints:
        timestamp = -1.0
        try:
            timestamp = waypoint.annotations.creation_time.seconds + waypoint.annotations.creation_time.nanos / 1e9
        except:
            pass
        waypoint_to_timestamp.append((waypoint.id, timestamp, waypoint.annotations.name))

    waypoint_to_timestamp = sorted(waypoint_to_timestamp, key=lambda x: (x[1], x[2]))
    return waypoint_to_timestamp

def sort(graph, adj):
    if (has_cycle(adj)):
        sorted_keys = sort_waypoints_chrono(graph)
        return [sorted[i][0] for i in range(len(sorted_keys))]
    sorted = []
    visited = {wp : False for wp in adj.keys()}
    def dfs(u):
        if visited[u]: return
        visited[u] = True
        for e in adj[u]:
            v = e[0]
            dfs(v)
        sorted.append(u)
    for u in adj.keys():
        dfs(u)
    return sorted[::-1]

def has_cycle(adj):
    on_stack = visited = {wp : False for wp in adj.keys()}
    def dfs(u):
        on_stack[u] = visited[u] = True
        for e in adj[u]:
            v = e[0]
            if on_stack[v]: return True
            elif not visited[v]:
                if dfs(v): return True
        on_stack[u] = False
        return False
    for u in adj.keys():
        if not visited[u]:
            if dfs(u): return True
    return False

def find_satisfying_waypoints(adj, src, dist_constr):
    q, dist = deque([src]), {wp : -1 for wp in adj.keys()}
    dist[src] = 0
    selected = [src]
    while q:
        u = q.popleft()
        for v, c in adj[u]:
            if dist[v] == -1:
                if c + dist[u] >= dist_constr:
                    dist[v] = 0
                    selected.append(v)
                else:
                    dist[v] = c + dist[u]
                q.append(v)
    return selected

def euler_circuit(adj, src):
    adj = deepcopy(adj)
    circuit, stack = [], [src]
    while stack:
        u = stack[-1]
        if adj[u]:
            v = adj[u].pop()[0]
            stack.append(v)
        else:
            circuit.append(stack.pop())
    return circuit[::-1]

def dijkstra(adj, src, dest):
    pq, dist = [(src, 0)], {u : float('inf') for u in adj.keys()}
    dist[src] = 0
    path = []
    while pq:
        u,cur_dist = heapq.heappop(pq)
        if cur_dist > dist[u]: continue
        path.append(u)
        if u == dest: return (path, dist[dest])
        for v,c in adj[u]:
            if c + dist[u] < dist[v]:
                dist[v] = c + dist[u]
                heapq.heappush(pq, (v, dist[v]))
    return None