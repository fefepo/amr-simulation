import pygame
import sys
import heapq

pygame.init()

# =========================================================
# 1. 화면 설정
# =========================================================
WIDTH, HEIGHT = 950, 600
GRID_WIDTH = 600
ROWS, COLS = 10, 10
CELL_SIZE = GRID_WIDTH // COLS

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Multi-AMR Warehouse Simulator")

clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 28)
big_font = pygame.font.SysFont(None, 42)

# =========================================================
# 2. 색상 정의
# =========================================================
WHITE = (255, 255, 255)
GRAY = (200, 200, 200)
BLACK = (40, 40, 40)

BLUE = (50, 100, 255)
GREEN = (80, 200, 120)
PURPLE_ROBOT = (170, 90, 255)

RED = (255, 80, 80)
ORANGE = (255, 170, 70)
PINK = (255, 120, 200)

YELLOW = (255, 230, 120)
LIGHT_GREEN = (180, 240, 180)
LIGHT_PURPLE = (220, 180, 255)

PURPLE = (150, 100, 255)

# =========================================================
# 3. 맵
#    0 = 이동 가능
#    1 = 장애물
#
# 우회가 가능하도록 중앙과 하단에 통로를 열어 둠
# =========================================================
warehouse = [
    [0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
    [0, 1, 0, 1, 0, 0, 1, 0, 1, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 1, 1, 1, 0, 0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 상단 우회
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 메인 복도
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # 하단 우회
    [0, 1, 1, 1, 0, 0, 1, 1, 1, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 1, 0, 0, 1, 0, 0, 0]
]

# =========================================================
# 4. A* 알고리즘
# =========================================================
def heuristic(a, b):
    """맨해튼 거리"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(node, blocked_cells=None):
    """임시 장애물 포함 이웃 노드 반환"""
    if blocked_cells is None:
        blocked_cells = set()

    x, y = node
    neighbors = []

    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < COLS and 0 <= ny < ROWS:
            if warehouse[ny][nx] == 0 and (nx, ny) not in blocked_cells:
                neighbors.append((nx, ny))

    return neighbors


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def a_star(start, goal, blocked_cells=None):
    """A* 경로 탐색"""
    if blocked_cells is None:
        blocked_cells = set()

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_neighbors(current, blocked_cells):
            tentative_g = g_score[current] + 1

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []

# =========================================================
# 5. 로봇 클래스
# =========================================================
class Robot:
    def __init__(self, name, color, path_color, start_pos, goal_pos, goal_color, priority):
        self.name = name
        self.color = color
        self.path_color = path_color
        self.goal_color = goal_color

        self.start_pos = list(start_pos)
        self.pos = list(start_pos)
        self.goal = list(goal_pos)

        self.path = []
        self.path_index = 0
        self.success = False
        self.priority = priority

        self.replan_count = 0
        self.wait_count = 0

    def reset(self):
        self.pos = self.start_pos[:]
        self.path = []
        self.path_index = 0
        self.success = False
        self.replan_count = 0
        self.wait_count = 0

    def calculate_path(self, blocked_cells=None):
        if blocked_cells is None:
            blocked_cells = set()

        self.path = a_star(tuple(self.pos), tuple(self.goal), blocked_cells)
        self.path_index = 1

    def recalculate_path(self, blocked_cells=None):
        if blocked_cells is None:
            blocked_cells = set()

        self.path = a_star(tuple(self.pos), tuple(self.goal), blocked_cells)
        self.path_index = 1
        self.replan_count += 1

    def next_position(self):
        if self.success:
            return tuple(self.pos)

        if not self.path or self.path_index >= len(self.path):
            return tuple(self.pos)

        return self.path[self.path_index]

    def move_one_step(self):
        if self.success:
            return

        if not self.path or self.path_index >= len(self.path):
            return

        next_pos = self.path[self.path_index]
        self.pos[0], self.pos[1] = next_pos
        self.path_index += 1

        if self.pos == self.goal:
            self.success = True

    def wait(self):
        self.wait_count += 1

    def draw(self):
        x = self.pos[0] * CELL_SIZE
        y = self.pos[1] * CELL_SIZE
        pygame.draw.rect(screen, self.color, (x + 8, y + 8, CELL_SIZE - 16, CELL_SIZE - 16))

    def draw_goal(self):
        x = self.goal[0] * CELL_SIZE
        y = self.goal[1] * CELL_SIZE
        pygame.draw.rect(screen, self.goal_color, (x + 12, y + 12, CELL_SIZE - 24, CELL_SIZE - 24))

    def draw_path(self):
        for pos in self.path:
            if list(pos) != self.pos and list(pos) != self.goal:
                x = pos[0] * CELL_SIZE
                y = pos[1] * CELL_SIZE
                pygame.draw.rect(
                    screen,
                    self.path_color,
                    (x + 18, y + 18, CELL_SIZE - 36, CELL_SIZE - 36)
                )

# =========================================================
# 6. 로봇 생성
#    목표는 모서리 쪽 + 전부 빈칸
# =========================================================
robot1 = Robot(
    name="Robot 1",
    color=BLUE,
    path_color=YELLOW,
    start_pos=(1, 5),
    goal_pos=(9, 5),   # 같은 줄 오른쪽
    goal_color=RED,
    priority=1
)

robot2 = Robot(
    name="Robot 2",
    color=GREEN,
    path_color=LIGHT_GREEN,
    start_pos=(8, 5),
    goal_pos=(0, 5),   # 같은 줄 왼쪽
    goal_color=ORANGE,
    priority=2
)

robot3 = Robot(
    name="Robot 3",
    color=PURPLE_ROBOT,
    path_color=LIGHT_PURPLE,
    start_pos=(5, 10),
    goal_pos=(0, 5),
    goal_color=PINK,
    priority=3
)

robots = [robot1, robot2, robot3]

auto_move = False
last_move_time = 0
move_delay = 450

# =========================================================
# 7. 유효성 검사
# =========================================================
def validate_goals():
    for robot in robots:
        gx, gy = robot.goal
        if warehouse[gy][gx] == 1:
            print(f"[ERROR] {robot.name} goal {robot.goal} is on obstacle.")

validate_goals()

# =========================================================
# 8. 화면 출력
# =========================================================
def draw_grid():
    for row in range(ROWS):
        for col in range(COLS):
            x = col * CELL_SIZE
            y = row * CELL_SIZE

            if warehouse[row][col] == 1:
                pygame.draw.rect(screen, BLACK, (x, y, CELL_SIZE, CELL_SIZE))
            else:
                pygame.draw.rect(screen, WHITE, (x, y, CELL_SIZE, CELL_SIZE))

            pygame.draw.rect(screen, GRAY, (x, y, CELL_SIZE, CELL_SIZE), 1)


def draw_status():
    panel_x = 620

    title = font.render("STATUS", True, PURPLE)
    screen.blit(title, (panel_x, 20))

    t1 = font.render(f"Blue: {'DONE' if robot1.success else 'MOVING'}", True, BLUE)
    t2 = font.render(f"Green: {'DONE' if robot2.success else 'MOVING'}", True, GREEN)
    t3 = font.render(f"Purple: {'DONE' if robot3.success else 'MOVING'}", True, PURPLE_ROBOT)

    screen.blit(t1, (panel_x, 70))
    screen.blit(t2, (panel_x, 105))
    screen.blit(t3, (panel_x, 140))

    r1 = font.render(f"Blue replans: {robot1.replan_count}", True, BLUE)
    r2 = font.render(f"Green replans: {robot2.replan_count}", True, GREEN)
    r3 = font.render(f"Purple replans: {robot3.replan_count}", True, PURPLE_ROBOT)

    screen.blit(r1, (panel_x, 200))
    screen.blit(r2, (panel_x, 235))
    screen.blit(r3, (panel_x, 270))

    w1 = font.render(f"Blue waits: {robot1.wait_count}", True, BLUE)
    w2 = font.render(f"Green waits: {robot2.wait_count}", True, GREEN)
    w3 = font.render(f"Purple waits: {robot3.wait_count}", True, PURPLE_ROBOT)

    screen.blit(w1, (panel_x, 330))
    screen.blit(w2, (panel_x, 365))
    screen.blit(w3, (panel_x, 400))

    info1 = font.render("SPACE : Start", True, BLACK)
    info2 = font.render("R : Reset", True, BLACK)

    screen.blit(info1, (panel_x, 470))
    screen.blit(info2, (panel_x, 505))

    if all(robot.success for robot in robots):
        overlay = pygame.Surface((GRID_WIDTH, HEIGHT))
        overlay.set_alpha(150)
        overlay.fill((255, 255, 255))
        screen.blit(overlay, (0, 0))

        text = big_font.render("ALL ROBOTS SUCCESS!", True, RED)
        text_rect = text.get_rect(center=(GRID_WIDTH // 2, HEIGHT // 2))
        screen.blit(text, text_rect)

# =========================================================
# 9. 시작 / 초기화
# =========================================================
def reset_simulation():
    global auto_move, last_move_time

    for robot in robots:
        robot.reset()

    auto_move = False
    last_move_time = 0


def prepare_paths():
    for robot in robots:
        robot.calculate_path()

# =========================================================
# 10. 핵심 이동 로직
# =========================================================
def simulate_one_step():
    """
    원하는 동작:
    - 처음에는 모두 기본 최단 경로로 직진
    - 충돌이 실제로 발생할 상황이 되면 그때만 재탐색
    - 우선순위 낮은 로봇만 우회
    """

    movable = [r for r in robots if not r.success]
    movable.sort(key=lambda r: r.priority)

    # 이번 턴에 이미 앞선 로봇이 이동한 위치
    reserved_positions = set()

    for robot in movable:
        current_pos = tuple(robot.pos)
        next_pos = robot.next_position()

        # 경로가 없으면 기본 경로 다시 계산
        if next_pos == current_pos:
            robot.recalculate_path()
            next_pos = robot.next_position()

            if next_pos == current_pos:
                robot.wait()
                reserved_positions.add(current_pos)
                continue

        # -------------------------------------------------
        # 1) 이미 앞선 로봇이 이번 턴에 이동한 칸이면 재탐색
        # -------------------------------------------------
        if next_pos in reserved_positions:
            robot.recalculate_path(blocked_cells=reserved_positions)
            next_pos = robot.next_position()

            if next_pos == current_pos or next_pos in reserved_positions:
                robot.wait()
                reserved_positions.add(current_pos)
                continue

        # -------------------------------------------------
        # 2) 아직 안 움직인 다른 로봇과 "실제 충돌 직전"인지 검사
        #    - 같은 칸으로 들어가려는 경우
        #    - 서로 자리 바꾸는 경우
        # -------------------------------------------------
        conflict_detected = False
        blocked_for_replan = set(reserved_positions)

        for other in movable:
            if other.name == robot.name:
                continue

            other_current = tuple(other.pos)
            other_next = other.next_position()

            # case A: 같은 칸으로 동시에 들어가려는 상황
            same_target = (
                next_pos == other_next and
                next_pos != current_pos and
                other_next != other_current
            )

            # case B: 서로 자리 바꾸기
            swap_conflict = (
                next_pos == other_current and
                other_next == current_pos and
                next_pos != current_pos and
                other_next != other_current
            )

            if same_target or swap_conflict:
                conflict_detected = True
                blocked_for_replan.add(other_current)

        # -------------------------------------------------
        # 3) 충돌 직전일 때만 재탐색
        # -------------------------------------------------
        if conflict_detected:
            robot.recalculate_path(blocked_cells=blocked_for_replan)
            next_pos = robot.next_position()

            # 재탐색 후에도 못 움직이면 대기
            if next_pos == current_pos or next_pos in reserved_positions:
                robot.wait()
                reserved_positions.add(current_pos)
                continue

        # -------------------------------------------------
        # 4) 최종 이동
        # -------------------------------------------------
        robot.move_one_step()
        reserved_positions.add(tuple(robot.pos))

# =========================================================
# 11. 메인 루프
# =========================================================
running = True

while running:
    current_time = pygame.time.get_ticks()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                prepare_paths()
                auto_move = True

            if event.key == pygame.K_r:
                reset_simulation()

    if auto_move and current_time - last_move_time > move_delay:
        simulate_one_step()
        last_move_time = current_time

    screen.fill((245, 245, 245))
    pygame.draw.rect(screen, WHITE, (0, 0, GRID_WIDTH, HEIGHT))

    draw_grid()

    for robot in robots:
        robot.draw_goal()

    for robot in robots:
        robot.draw_path()

    for robot in robots:
        robot.draw()

    draw_status()

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()