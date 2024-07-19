from flask import Flask, render_template, request, jsonify
import heapq

app = Flask(__name__)

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost from start node to current node
        self.h = 0  # Heuristic cost from current node to target node
        self.f = 0  # Total cost: f = g + h

    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)
    
    def __lt__(self, other):
        return self.f < other.f

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/find_path', methods=['POST'])
def find_path():
    # Retrieve grid size from form data
    rows = int(request.form['rows'])
    cols = int(request.form['cols'])

    # Retrieve grid values from form data
    grid = []
    for i in range(rows):
        row_values = list(map(int, request.form[f'row_{i}'].split()))
        grid.append(row_values)

    # Retrieve start and goal positions from form data
    start_position = tuple(map(int, request.form['start_position'].split(',')))
    goal_position = tuple(map(int, request.form['goal_position'].split(',')))

    # Perform A* algorithm
    path = astar(grid, start_position, goal_position)

    if path:
        return render_template('result.html', path=path)
    else:
        return jsonify({'error': 'No path found'})

def astar(grid, start, end):
    open_list = []
    closed_list = set()
    heapq.heappush(open_list, Node(start))

    movements = [(0, -1), (0, 1), (-1, 0), (1, 0)]

    while open_list:
        current_node = heapq.heappop(open_list)
        closed_list.add(current_node)

        if current_node.position == end:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        for movement in movements:
            new_position = (current_node.position[0] + movement[0], current_node.position[1] + movement[1])

            if 0 <= new_position[0] < len(grid) and 0 <= new_position[1] < len(grid[0]):
                if grid[new_position[0]][new_position[1]] != 1:
                    neighbor_node = Node(new_position, current_node)
                    if neighbor_node in closed_list:
                        continue
                    tentative_g = current_node.g + 1
                    if neighbor_node not in open_list or tentative_g < neighbor_node.g:
                        neighbor_node.g = tentative_g
                        neighbor_node.h = ((new_position[0] - end[0]) ** 2 + (new_position[1] - end[1]) ** 2) ** 0.5
                        neighbor_node.f = neighbor_node.g + neighbor_node.h
                        heapq.heappush(open_list, neighbor_node)

    return None

if __name__ == '__main__':
    app.run(debug=True)
