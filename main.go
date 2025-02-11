package main

import (
	"container/heap"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/hajimehoshi/ebiten/v2/vector"
	"image/color"
	"math"
	"time"
)

const (
	screenWidth  = 400
	screenHeight = 400
	gridSize     = 10
	cellSize     = screenWidth / gridSize
)

type Cell struct {
	x, y  int
	wall  bool
	start bool
	end   bool
}

type Game struct {
	grid        [gridSize][gridSize]*Cell
	startPoint  *Cell
	endPoint    *Cell
	path        []*Cell
	currentStep int // Tracks animation progress
	lastUpdate  time.Time
}

func NewGame() *Game {
	game := &Game{}
	for i := 0; i < gridSize; i++ {
		for j := 0; j < gridSize; j++ {
			game.grid[i][j] = &Cell{x: i, y: j}
		}
	}
	game.lastUpdate = time.Now()
	return game
}

func (g *Game) Update() error {
	if ebiten.IsMouseButtonPressed(ebiten.MouseButtonLeft) {
		x, y := ebiten.CursorPosition()
		gridX, gridY := x/cellSize, y/cellSize

		if gridX < gridSize && gridY < gridSize {
			cell := g.grid[gridX][gridY]
			if g.startPoint == nil {
				cell.start = true
				g.startPoint = cell
			} else if g.endPoint == nil {
				cell.end = true
				g.endPoint = cell
			} else {
				cell.wall = true
			}
		}
	}

	// Start animation when right-clicking
	if ebiten.IsMouseButtonPressed(ebiten.MouseButtonRight) && g.startPoint != nil && g.endPoint != nil {
		g.path = g.aStar()
		g.currentStep = 0 // Reset animation
		g.lastUpdate = time.Now()
	}

	// Animate path movement
	if g.currentStep < len(g.path) {
		if time.Since(g.lastUpdate) > 100*time.Millisecond { // Delay between steps
			g.currentStep++
			g.lastUpdate = time.Now()
		}
	}

	return nil
}

func (g *Game) Draw(screen *ebiten.Image) {
	// Draw grid cells
	for i := 0; i < gridSize; i++ {
		for j := 0; j < gridSize; j++ {
			cell := g.grid[i][j]
			rectColor := color.RGBA{200, 200, 200, 255} // Light gray for empty cells

			if cell.wall {
				rectColor = color.RGBA{0, 0, 0, 255} // Black for obstacles
			} else if cell.start {
				rectColor = color.RGBA{0, 255, 0, 255} // Green for start point
			} else if cell.end {
				rectColor = color.RGBA{255, 0, 0, 255} // Red for end point
			}

			// Draw the filled cell
			ebitenutil.DrawRect(screen, float64(cell.x*cellSize), float64(cell.y*cellSize), float64(cellSize), float64(cellSize), rectColor)
		}
	}

	// Draw grid lines using vector.StrokeLine
	gridColor := color.RGBA{50, 50, 50, 255} // Dark gray for grid lines
	for i := 0; i <= gridSize; i++ {
		// Vertical lines
		vector.StrokeLine(screen, float32(i*cellSize), 0, float32(i*cellSize), screenHeight, 1, gridColor, false)
		// Horizontal lines
		vector.StrokeLine(screen, 0, float32(i*cellSize), screenWidth, float32(i*cellSize), 1, gridColor, false)
	}

	// Draw the animated path up to the current step
	for i := 0; i < g.currentStep && i < len(g.path); i++ {
		cell := g.path[i]
		ebitenutil.DrawRect(screen, float64(cell.x*cellSize), float64(cell.y*cellSize), float64(cellSize), float64(cellSize), color.RGBA{0, 0, 255, 128})
	}
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (int, int) {
	return screenWidth, screenHeight
}

// --- A* Pathfinding Implementation ---

type Node struct {
	cell   *Cell
	gCost  float64
	hCost  float64
	parent *Node
	index  int
}

func (n *Node) fCost() float64 {
	return n.gCost + n.hCost
}

type PriorityQueue []*Node

func (pq PriorityQueue) Len() int           { return len(pq) }
func (pq PriorityQueue) Less(i, j int) bool { return pq[i].fCost() < pq[j].fCost() }
func (pq PriorityQueue) Swap(i, j int)      { pq[i], pq[j] = pq[j], pq[i]; pq[i].index = i; pq[j].index = j }

func (pq *PriorityQueue) Push(x interface{}) {
	n := x.(*Node)
	n.index = len(*pq)
	*pq = append(*pq, n)
}

func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	old[n-1] = nil
	*pq = old[0 : n-1]
	return item
}

func heuristic(a, b *Cell) float64 {
	dx := math.Abs(float64(a.x - b.x))
	dy := math.Abs(float64(a.y - b.y))
	return dx + dy
}

func (g *Game) getNeighbors(cell *Cell) []*Cell {
	var neighbors []*Cell
	dirs := [][2]int{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}

	for _, d := range dirs {
		nx, ny := cell.x+d[0], cell.y+d[1]
		if nx >= 0 && nx < gridSize && ny >= 0 && ny < gridSize {
			if !g.grid[nx][ny].wall {
				neighbors = append(neighbors, g.grid[nx][ny])
			}
		}
	}
	return neighbors
}

func (g *Game) aStar() []*Cell {
	startNode := &Node{cell: g.startPoint, gCost: 0, hCost: heuristic(g.startPoint, g.endPoint)}
	endCell := g.endPoint

	openSet := PriorityQueue{}
	heap.Init(&openSet)
	heap.Push(&openSet, startNode)

	cameFrom := make(map[*Cell]*Node)
	cameFrom[g.startPoint] = startNode

	gScore := make(map[*Cell]float64)
	gScore[g.startPoint] = 0

	for openSet.Len() > 0 {
		current := heap.Pop(&openSet).(*Node)

		if current.cell == endCell {
			var path []*Cell
			for n := current; n != nil; n = n.parent {
				path = append([]*Cell{n.cell}, path...)
			}
			return path
		}

		for _, neighbor := range g.getNeighbors(current.cell) {
			tentativeG := current.gCost + 1

			if _, seen := gScore[neighbor]; !seen || tentativeG < gScore[neighbor] {
				neighborNode := &Node{cell: neighbor, gCost: tentativeG, hCost: heuristic(neighbor, endCell), parent: current}
				heap.Push(&openSet, neighborNode)
				gScore[neighbor] = tentativeG
				cameFrom[neighbor] = neighborNode
			}
		}
	}

	return nil
}

func main() {
	game := NewGame()
	ebiten.SetWindowSize(screenWidth, screenHeight)
	ebiten.SetWindowTitle("A* Pathfinding Animation")
	if err := ebiten.RunGame(game); err != nil {
		panic(err)
	}
}
