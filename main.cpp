#include <SDL2/SDL.h>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_render.h>
#include <SDL2/SDL_video.h>
#include <array>
#include <random>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>

#define NORTH 0
#define SOUTH 1
#define WEST 2
#define EAST 3

const int WIDTH = 1280;
const int HEIGHT = 720;
const int CELL_SIZE = 5;
constexpr int COLS = WIDTH / CELL_SIZE;
constexpr int ROWS = HEIGHT / CELL_SIZE;

struct Segment {
    int sx, sy, ex, ey;
    bool exist = true;
};

// I am aware these structs below can be implemented more securely, like making
// member variables private and accessing them through getters and setters.
// Member variables and methods in structs are public by default
// I am too lazy to do that
struct Cell {
    int x, y;
    bool visited = false;
    std::array<Segment, 4> segments;

    Cell() = default;
    Cell(int x, int y) : x(x), y(y) {
        int rel_x = x * CELL_SIZE;
        int rel_y = y * CELL_SIZE;

        segments[NORTH] = { rel_x, rel_y, rel_x + CELL_SIZE, rel_y, true };
        segments[SOUTH] = { rel_x, rel_y + CELL_SIZE, rel_x + CELL_SIZE, rel_y + CELL_SIZE, true };
        segments[WEST]  = { rel_x, rel_y, rel_x, rel_y + CELL_SIZE, true };
        segments[EAST]  = { rel_x + CELL_SIZE, rel_y, rel_x + CELL_SIZE, rel_y + CELL_SIZE, true };
    }

    void remove_wall(int direction) { segments[direction].exist = false; }
};

struct Maze {
    std::vector<Cell> maze;
    std::random_device rd;
    std::mt19937 rng;

    Maze() : rng(rd()) {
        maze.reserve(COLS * ROWS);
        for (int y = 0; y < ROWS; y++)
            for (int x = 0; x < COLS; x++)
                maze.emplace_back(x, y);
    }

    int opposite(int dir) {
        return (dir == NORTH) ? SOUTH : (dir == SOUTH) ? NORTH : (dir == WEST) ? EAST : WEST;
    }

    void generate_maze(int x, int y) {
        Cell& current = maze[y * COLS + x];
        current.visited = true;

        std::vector<int> directions = {NORTH, SOUTH, WEST, EAST};
        std::shuffle(directions.begin(), directions.end(), rng);

        for (int dir : directions) {
            int nx = x + (dir == EAST) - (dir == WEST);
            int ny = y + (dir == SOUTH) - (dir == NORTH);

            if (nx >= 0 && nx < COLS && ny >= 0 && ny < ROWS && !maze[ny * COLS + nx].visited) {
                current.remove_wall(dir);
                maze[ny * COLS + nx].remove_wall(opposite(dir));
                generate_maze(nx, ny);
            }
        }
    }

    void draw(SDL_Renderer* renderer) {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        for (auto& cell : maze)
            for (auto& segment : cell.segments)
                if (segment.exist)
                    SDL_RenderDrawLine(renderer, segment.sx, segment.sy, segment.ex, segment.ey);
    }

    bool is_walkable(int x, int y, int direction) {
        if (x < 0 || y < 0 || x >= COLS || y >= ROWS) return false;
        return !maze[y * COLS + x].segments[direction].exist;
    }
};

// *********** Pathfinding *********** 
struct Node {
    int x, y, g_cost, h_cost;
    int f_cost() const { return g_cost + h_cost; }
    bool operator>(const Node& other) const { return f_cost() > other.f_cost(); }
};

std::vector<std::pair<int, int>> a_star(Maze& maze, SDL_Renderer* renderer) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    std::unordered_map<int, std::pair<int, int>> came_from;
    std::vector<int> g_cost(COLS * ROWS, INT_MAX);
    std::vector<bool> visited(COLS * ROWS, false);

    auto index = [](int x, int y) { return y * COLS + x; };
    auto heuristic = [](int x, int y) { return std::abs(COLS - 1 - x) + std::abs(ROWS - 1 - y); };

    open_set.push({0, 0, 0, heuristic(0, 0)});
    g_cost[index(0, 0)] = 0;

    while (!open_set.empty()) {
        Node current = open_set.top();
        open_set.pop();

        if (visited[index(current.x, current.y)]) continue;
        visited[index(current.x, current.y)] = true;

        // Draw visited cells in green
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
        SDL_Rect cell_rect = { current.x * CELL_SIZE, current.y * CELL_SIZE, CELL_SIZE, CELL_SIZE };
        SDL_RenderFillRect(renderer, &cell_rect);
        SDL_RenderPresent(renderer);

        if (current.x == COLS - 1 && current.y == ROWS - 1) {
            std::vector<std::pair<int, int>> path;
            for (auto at = std::make_pair(current.x, current.y); came_from.count(index(at.first, at.second)); at = came_from[index(at.first, at.second)])
                path.push_back(at);
            path.push_back({0, 0});
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int dir = 0; dir < 4; dir++) {
            int nx = current.x + (dir == EAST) - (dir == WEST);
            int ny = current.y + (dir == SOUTH) - (dir == NORTH);

            if (!maze.is_walkable(current.x, current.y, dir)) continue;
            int new_g_cost = current.g_cost + 1;

            if (new_g_cost < g_cost[index(nx, ny)]) {
                g_cost[index(nx, ny)] = new_g_cost;
                came_from[index(nx, ny)] = {current.x, current.y};
                open_set.push({nx, ny, new_g_cost, heuristic(nx, ny)});

                // Draw expanding nodes in blue (one might call them explorers, they explore unexplored nodes)
                SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
                SDL_Rect expand_rect = { nx * CELL_SIZE, ny * CELL_SIZE, CELL_SIZE, CELL_SIZE };
                SDL_RenderFillRect(renderer, &expand_rect);
                SDL_RenderPresent(renderer);
            }
        }
    }
    return {};
}

// Absolute start and end points. Changing them here will be visually not okay for the lack of better term
void draw_start_end(SDL_Renderer* renderer) { 
    SDL_Rect srect = { 0, 0, CELL_SIZE, CELL_SIZE };
    SDL_Rect erect = { WIDTH - CELL_SIZE, HEIGHT - CELL_SIZE, CELL_SIZE, CELL_SIZE };

    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    SDL_RenderFillRect(renderer, &srect);
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_RenderFillRect(renderer, &erect);

    SDL_RenderPresent(renderer);
}

// Draw the shortest path after finding the path from start to goal
void draw_path(SDL_Renderer* renderer, const std::vector<std::pair<int, int>>& path, Maze& maze) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
    SDL_RenderClear(renderer);
    maze.draw(renderer);
    draw_start_end(renderer);

    SDL_SetRenderDrawColor(renderer, 255, 0, 255, 255);
    for (size_t i = 1; i < path.size(); i++) {
        int x1 = path[i - 1].first * CELL_SIZE + CELL_SIZE / 2;
        int y1 = path[i - 1].second * CELL_SIZE + CELL_SIZE / 2;
        int x2 = path[i].first * CELL_SIZE + CELL_SIZE / 2;
        int y2 = path[i].second * CELL_SIZE + CELL_SIZE / 2;
        SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
        SDL_RenderPresent(renderer);
    }
}

int main() {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = SDL_CreateWindow("A* Visualization", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    Maze maze;
    maze.generate_maze(0, 0);
    maze.draw(renderer);
    draw_start_end(renderer);

    std::vector<std::pair<int, int>> path = a_star(maze, renderer);
    draw_path(renderer, path, maze);

    SDL_Delay(5000);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}
