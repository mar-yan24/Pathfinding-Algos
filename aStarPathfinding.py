"""

pseudocode:

def SetUpNodes():
	OPEN (set of nodes to be evaluated)
	CLOSED(set of nodes already evaluated)
	BAR (is a barrier node, cannot traverse)

	add the starting node to OPEN

while True:
	current = the node in OPEN with the lowest f cost
	put current in CLOSED

	if current is the ending node:
		return or QUIT

	for each neighbor of the current node:
		if neighbor is not BAR or neighbor is in CLOSED:
			skip to next neighbor node

		if new path to neighbor is shorter or neighbor is not in open:
			set new f cost to neighbor node
			set parent of neighbor to current
			if neighbor is not in OPEN: add neighbor to OPEN

"""

"""

g_cost(): This is the distance from the current node we are looking at to the starting node. This is the main thing that Dijkstra's
algorithm looks at as they dont have the heuristic function

h_cost(): This is just the heuristic function in A* that makes it different from Dijkstra's algorithm as it now looks 
for the most efficient path. Dijkstra's algorithm looks for all paths that exist. This path gives different values depending
on how far the current node is from the ending node

f_cost(): The f cost is the total score (by adding the g and h cost) that the node has. The bigger the score it has means the less 
likely it is to be a node along the most efficient path.

"""


import pygame
import math
from queue import PriorityQueue

WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm")

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)

class Node:
	def __init__(self, row, col, width, total_rows):
		self.row = row
		self.col = col
		self.x = row * width
		self.y = col * width
		self.color = WHITE
		self.neighbors = []
		self.width = width
		self.total_rows = total_rows

	def get_pos(self):
		return self.row, self.col

	#if the node is closed then it must be red
	def is_closed(self):
		return self.color == RED

	#if node is open then it is green
	def is_open(self):
		return self.color == GREEN

	#if the node is black then it is a barrier
	def is_barrier(self):
		return self.color == BLACK

	#if the node is orange then it is the starting node
	def is_start(self):
		return self.color == ORANGE

	#if the node is purple, then it is the ending node
	def is_end(self):
		return self.color == TURQUOISE

	#reset the color to white
	def reset(self):
		self.color = WHITE

	#just setting all the colors
	def make_closed(self):
		self.color = RED

	def make_open(self):
		self.color = GREEN

	def make_barrier(self):
		self.color = BLACK

	def make_start(self):
		self.color = ORANGE

	def make_end(self):
		self.color = TURQUOISE

	def make_path(self):
		self.color = PURPLE

	def draw(self, win):
		pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

	def update_neighbors(self, grid):
		self.neighbors = []
		if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): #DOWN
			self.neighbors.append(grid[self.row + 1][self.col])
		if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): #UP
			self.neighbors.append(grid[self.row - 1][self.col])
		if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): #RIGHT
			self.neighbors.append(grid[self.row][self.col + 1])
		if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): #LEFT
			self.neighbors.append(grid[self.row][self.col - 1])

	def __lt__(self, other):
		return False

#finding the heuristic function (distance is found using manhattan distance)
def h(p1, p2):
	x1, y1 = p1
	x2, y2 = p2
	return abs(x1 - x2) + abs(y1 - y2)

def reconstruct_path(came_from, current, draw): #the current node starts at the end node traverse from the end node to the start node and draw it
	while current in came_from:
		current = came_from[current]
		current.make_path()
		draw()

def algorithm(draw, grid, start, end):
	count = 0 
	open_set = PriorityQueue()
	open_set.put((0, count, start)) #put start node in open set
	came_from = {} #keeps track of where we came from
	g_score = {spot: float("inf") for row in grid for spot in row} #keeps track of current shortest distance to get from start node to this node
	g_score[start] = 0
	f_score = {spot: float("inf") for row in grid for spot in row} #keeps track of predicted distance from this node to the end node
	f_score[start] = h(start.get_pos(), end.get_pos())

	open_set_hash = {start} #something in open set?

	#while open set is not empty, do not quit game
	while not open_set.empty():
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()

		current = open_set.get()[2] #priority queue
		open_set_hash.remove(current)

		if current == end:
			reconstruct_path(came_from, end, draw)
			end.make_end()
			return True #finished path

		for neighbor in current.neighbors: #consider all neighbors of current node
			temp_g_score = g_score[current] + 1 #calculte the tentative g score

			if temp_g_score < g_score[neighbor]: #if that less than the g score in the table then update because there is a better path
				came_from[neighbor] = current
				g_score[neighbor] = temp_g_score
				f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
				if neighbor not in open_set_hash: #add to the open set hash because better path is found
					count += 1
					open_set.put((f_score[neighbor], count, neighbor))
					open_set_hash.add(neighbor)
					neighbor.make_open()
		draw()

		if current != start:
			current.make_closed()

	return False

#making the grid, list inside list
def make_grid(rows, width):
	grid = []
	gap = width // rows
	for i in range(rows):
		grid.append([])
		for j in range (rows):
			spot = Node(i, j, gap, rows)
			grid[i].append(spot)

	return grid

#just drawing the grid
def draw_grid(win, rows, width):
	gap = width // rows
	for i in range(rows + 1):
		pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
		for j in range(rows + 1):
			pygame.draw.line(win, GREY, (j * gap, 0), (j* gap, width))

#drawing grdd
def draw(win, grid, rows, width):
	win.fill(WHITE)

	for row in grid:
		for spot in row:
			spot.draw(win)

	draw_grid(win, rows, width)
	pygame.display.update()

#where was the mouse clicked
def get_clicked_pos(pos, rows, width):
	gap = width // rows
	y, x = pos

	row = y // gap
	col = x // gap
	return row, col

#the main loop
def main(win, width):
	ROWS = 30
	grid = make_grid(ROWS, width)

	start = None
	end = None
	space_count = 0

	run = True
	while run:
		draw(win, grid, ROWS, width)
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

			if pygame.mouse.get_pressed()[0] and space_count == 0: #LEFT MOUSE BUTTON (create the barrier nodes)
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, ROWS, width)
				spot = grid[row][col]
				if not start and spot != end:
					start = spot
					start.make_start()

				elif not end and spot != start:
					end = spot
					end.make_end()

				elif spot != end and spot != start:
					spot.make_barrier()

			elif pygame.mouse.get_pressed()[2]: #RIGHT MOUSE BUTTON (want to delete the nodes)
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, ROWS, width)
				spot = grid[row][col]
				spot.reset()
				if spot == start:
					start = None

				elif spot == end:
					end = None

			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_SPACE and start and end and space_count == 0:
					space_count += 1;
					for row in grid:
						for spot in row:
							spot.update_neighbors(grid)
					algorithm(lambda: draw(win, grid, ROWS, width), grid, start, end)

				if event.key == pygame.K_c:
					start = None
					end = None
					grid = make_grid(ROWS, width)
					space_count = 0


	pygame.quit()

main(WIN, WIDTH)