#include <vector>
#include <assert.h>
#include <math.h>
#include <queue>
#include <iostream>
#include <cstring>
#include <unordered_set>
#include <unordered_map>
#include <fstream>

using namespace std;

constexpr float TOTWIDTH = 20000, TOTHEIGHT = 20000, RADIUS = 9999;
constexpr int WIDTH = 1000, HEIGHT = 1000;
constexpr int CHECKRADIUS = 6;
constexpr float SPEED = 140;

// File to place all the coordinates in CSV format.
std::ofstream out_file("../output.txt", ios_base::trunc);

// A* works best using Manhatten distance
// I'm not sure why, but I tried with both manhatten and Euclidean, and Euclidean gives very bad results
// and is also very slow.
static float manhatten(float x1, float y1, float x2, float y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

struct Coord {
    float x, y;

    Coord operator+(const Coord &other) {
        return Coord{x + other.x, y + other.y};
    }

    Coord operator-(const Coord &other) {
        return Coord{x - other.x, y - other.y};
    }

    float magn() {
        return manhatten(x, y, 0, 0);
    }

    Coord operator/(float a) {
        return Coord{x / a, y / a};
    }

    Coord operator*(float a) {
        return Coord{x * a, y * a};
    }

    Coord norm() {
        return *this / magn();
    }

    Coord round_int() const {
        return Coord{floor(x), floor(y)};
    }

    float manhatten_distance_to(const Coord &b) const {
        return abs(b.x - x) + abs(b.y - y);
    }

    bool operator==(const Coord &other) const {
        return abs(x - other.x) + abs(y - other.y) < 0.01;
    }

    bool operator<(const Coord &other) const {
        return (x + y) < (other.x + other.y);
    }
};

// Hashing method for Coord
struct CoordHash {
    size_t operator()(const Coord &coord) const {
        return hash<float>()(coord.x) ^ hash<float>()(coord.y);
    }
};

std::ostream &operator<<(std::ostream &os, const Coord &c) {
    os << c.x << " " << c.y << " ";
    return os;
}

// Represents a time slice in the 3d search space. Handles position bookings.
// Planes can book a coordinate, which means no other plane can book the same coordinate at this time slice.
struct TimeSlice {
    vector<vector<short>> matrix;
    float timestart, timeend;

    TimeSlice(float start, float end) : timestart(start), timeend(end), matrix(HEIGHT) {
        for (auto &a : matrix) {
            a.resize(WIDTH);
        }
    };

    // Debug only: block a rectangle to see how A* would react
    void block_rect(Coord tl, int width, int height) {
        for (int y = tl.y; y < tl.y + height; y++) {
            for (int x = tl.x; x < tl.x + width; x++) {
                matrix[y][x] = 1;
            }
        }
    }

    // Can a plane book this position at coord?
    bool can_book(Coord coord, int id) const {
        int x = (int) coord.x;
        int y = (int) coord.y;
        for (int yi = y - CHECKRADIUS; yi <= y + CHECKRADIUS; yi++) {
            for (int xi = x - CHECKRADIUS; xi <= x + CHECKRADIUS; xi++) {
                if (xi > 0 && yi > 0 && xi < WIDTH && yi < HEIGHT) {
                    if (matrix[yi][xi] != 0 && matrix[yi][xi] != id) {
                        return false;
                    }
                }
            }
        }
        if (x >= 0 && y >= 0 && x < WIDTH && y < HEIGHT) {
            return true;
        } else {
            return false;
        }
    }

    // Book this coord so no other planes can go here
    void book(Coord coord, int identifier) {
        assert (can_book(coord, identifier));
        matrix[(int) coord.y][(int) coord.x] = identifier;
    }

    // Convert real-life coordinates to/from A* grid coordinates.
    static Coord pos_to_square(Coord coord) {
        return Coord{coord.x * (float) WIDTH / TOTWIDTH, coord.y * (float) HEIGHT / TOTHEIGHT};
    }

    static Coord square_to_pos(Coord coord) {
        return Coord{coord.x * TOTWIDTH / WIDTH, coord.y * TOTHEIGHT / HEIGHT};
    }

};


// Find the index in the 3d search space that intersects time
int find_time_index(const vector<TimeSlice> &map, float time) {
    for (int i = 0; i < map.size(); i++) {
        if (map[i].timestart <= time && map[i].timeend >= time) {
            return i;
        }
    }
    throw runtime_error("Unexpected");
}


// A point along a plane's path
struct PathPoint {
    Coord pos;
    Coord prev;
    float time;
    float h_score;
    float g_score;
    int time_index = 0;

    static int operations;

    float score() const {
        return h_score + g_score;
    }

    // Construct a default point
    static PathPoint default1() {
        PathPoint p;
        memset(&p, 0, sizeof p);
        return p;
    }

    bool operator>(const PathPoint &other) const {
        if (score() == other.score()) {
            // If tie, then return furthest away
            return g_score < other.g_score;
        }
        return score() > other.score();
    }

    // Try to explore a new position, and make a PathPoint struct representing that exploration.
    PathPoint check_position(Coord endsquare, int timepoint_index, Coord check_pos) {
        const auto end = TimeSlice::square_to_pos(endsquare);
        auto pos_latlng = TimeSlice::square_to_pos(check_pos);
        auto this_latlng = TimeSlice::square_to_pos(pos);
        auto dist = manhatten(pos_latlng.x, pos_latlng.y, end.x, end.y);
        auto dist_prev1 = manhatten(pos_latlng.x, pos_latlng.y, this_latlng.x, this_latlng.y);
        float time_new =
                time + dist_prev1 / SPEED;
        float g_score_new = g_score + dist_prev1;
        float h_score_new = dist;
        Coord new_prev;

        if (pos.round_int() == check_pos.round_int()) {
            new_prev = prev;
        } else {
            new_prev = pos;
        }
        return PathPoint{check_pos, new_prev, time_new, h_score_new, g_score_new, timepoint_index};
        // todo: check for doubling back
    }


    // Main A* implementation
    template<typename CostMap, typename Queue>
    void expand_node(CostMap &cost_map, Queue &queue, const vector<TimeSlice> &map, Coord endsquare) {
        operations++;
        int timepoint_index = time_index;
        while (map[timepoint_index].timeend < time) {
            timepoint_index++;
        }
        for (int x = -1; x <= 1; x++) {
            for (int y = -1; y <= 1; y++) {
                auto check_pos = pos + Coord{(float) x, (float) y};
                if (x == 0 && y == 0) {
                    // We not checking here, so override it to check the optimal
                    auto optimal = (endsquare - pos);
                    if (optimal.magn() > 1) optimal = optimal.norm();
                    check_pos = pos + optimal;
                }
                if (map.at(timepoint_index).can_book(check_pos, 0)) {
                    const auto p = check_position(endsquare, timepoint_index, check_pos);
                    PathPoint &old = cost_map.at((int) p.pos.y).at((int) p.pos.x);
                    if (old.score() > p.score() || old.score() == 0) {
                        old = p;
                        queue.push(p);
                    }
                }
            }
        }


    };
};

int PathPoint::operations = 0;


void book_path(vector<TimeSlice> &map, const vector<PathPoint> &path, int identifier) {
    for (auto &p : path) {
        map[p.time_index].book(p.pos, identifier);
    }
}

struct PathPlanner {
    priority_queue<PathPoint, std::vector<PathPoint>, std::greater<PathPoint>> open;
    vector<vector<PathPoint>> cost_map;
    const Coord startsquare, endsquare;
    const Coord end;
    float time;

    PathPlanner(Coord start, Coord end, float time) :
            startsquare(start),
            endsquare(end),
            end(end),
            time(time) {
        cost_map.resize(HEIGHT);
        for (int i = 0; i < HEIGHT; i++) {
            cost_map[i].resize(WIDTH);
            for (int j = 0; j < WIDTH; j++) {
                cost_map[i][j] = PathPoint::default1();
            }
        }

    }

    // Try to find a path, return false if no path can be found.
    bool try_find_path(const vector<TimeSlice> &map) {
        auto pathpoint = PathPoint{startsquare, startsquare, time, 0, 1, find_time_index(map, time)};
        open.push(pathpoint);
        cost_map[startsquare.y][startsquare.x] = open.top();
        while (true) {
            if (open.empty()) {
                cerr << "Empty set, no path found!\n";
                return false;
            }
            PathPoint next = open.top();
            if (!map[next.time_index].can_book(next.pos, 0)) {
                assert(next.pos == startsquare);
                map[next.time_index].can_book(next.pos, 0);
                cerr << "Already occupied: square " << next.pos << "\n";
                return false;
            }
            open.pop();
            if (next.pos.round_int() == endsquare.round_int()) {
                return true;
            }
            next.expand_node(cost_map, open, map, end);
        }
    }

    // Extract the path list from the cost_map populated by A* algorithm.
    // Simply starts from the end, walks the path using the `prev` pointer on the struct.
    vector<PathPoint> get_path() const {
        vector<PathPoint> path;
        PathPoint curmin = cost_map[(int) endsquare.y][(int) endsquare.x];
        while (true) {
            auto prev_pos = curmin.prev;
//            bool changed = false;
//            for (int i = -1; i <= 1; i++) {
//                for (int j = -1; j <= 1; j++) {
//                    if (i == 0 && j == 0) continue;
//                    if ((unsigned) (curmin.pos.y + i) >= HEIGHT || (unsigned) (curmin.pos.x + j) >= WIDTH) continue;
//                    auto &checking = cost_map[curmin.pos.y + i][curmin.pos.x + j];
//                    if (checking.time == 0) continue;
//                    if (curmin.score() == 0 || curmin.g_score > checking.g_score) {
//                        curmin = checking;
//                        changed = true;
//                    }
//                }
//            }

            path.push_back(curmin);
            if (curmin.pos.round_int() == startsquare.round_int()) break;

            curmin = cost_map[prev_pos.y][prev_pos.x];
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

};

struct Plane {
    static int global_id;
    int id;
    vector<PathPoint> planned_path;
    float completion_time;

    static Plane plan_path(vector<TimeSlice> &map, Coord start, Coord end, float time, float earliest_time) {
        auto id = global_id++;
        auto path = PathPlanner(start, end, time);
        auto found_path = path.try_find_path(map);
        if (!found_path) {
            // We have a problem...the path cannot be found?
            // Assume that the circling around will not have consequences, like crashing into another plane.
            // Ran out of time to implement proper crash checking :(

            // Since one circling is equivalent to increasing the time by 2 * pi * (1000 m) / 140 (m/s) = 44.88 s
            return plan_path(map, start, end, time + 44.88, earliest_time);
        }
        auto planned_path = path.get_path();
        auto completion_time = planned_path.back().time;

        if (completion_time < earliest_time) {
            // We have a problem...plane order is not the same. This plane has to arrive after earliest_time
            // Again, just circle according to the method above
            return plan_path(map, start, end, time + 44.88, earliest_time);
        }

        for (auto &p : planned_path) {
            out_file << p.pos.x << "," << p.pos.y << "," << p.time << "," << id << "\n";
        }
        book_path(map, planned_path, id);

        return Plane{id, std::move(planned_path), completion_time};
    }
};

struct RunwayPoint {
    Coord squarepos;
};

struct PositionTransmission {
    Coord coord;
    int plane_id;
};


// Find closest runway to a point
const RunwayPoint &find_best_runway(const vector<RunwayPoint> &runways, Coord plane_square_pt) {
    auto min_runway = std::min_element(runways.begin(), runways.end(), [&](auto &r1, auto &r2) {
        return r1.squarepos.manhatten_distance_to(plane_square_pt) <
               r2.squarepos.manhatten_distance_to(plane_square_pt);
    });
    return *min_runway;
}

int Plane::global_id = 5;


class ATC {
    // The list of available runways for planes to land on.
    // Planes will land on the runway closest to them
    // Deviation from original problem statement: I modeled the runway as a point because

    vector<RunwayPoint> runways;
    unordered_map<int, Plane> paths;
    vector<TimeSlice> map;
    float last_plane_time;
public:
    float timer;

    bool is_free_square(Coord coord) const {
        int idx = find_time_index(map, timer);
        return map[idx].can_book(coord, 0);
    }

    ATC() : timer(1.f), last_plane_time(1.f) {
        constexpr float timediff = 2;
        for (float time = 1; time < 5000; time += timediff) {
            map.emplace_back(time, time + timediff);
        }

        const auto runway_left = Coord{TOTWIDTH / 2 - 250, TOTHEIGHT / 2};
        const auto runway_right = Coord{TOTWIDTH / 2 + 250, TOTHEIGHT / 2};

        // Left runway- bottom entrance
        runways.push_back(RunwayPoint{TimeSlice::pos_to_square(runway_left)});
        runways.push_back(RunwayPoint{TimeSlice::pos_to_square(runway_right)});
    }

    void tick(float dt) {
        timer += dt;
    }

    // Handle a data transmission.
    // If we've seen the plane before, do nothing. The plane already knows its path and knows where to go.
    // If we haven't, then calculate the path, and add it to the known paths.
    void transmit(PositionTransmission transmission) {
        if (paths.contains(transmission.plane_id)) {
            // Do nothing. We ignore this plane as we've already solved its position.
        } else {
            // Solve the transmission and route the plane to its destination
            auto &best_runway = find_best_runway(runways, transmission.coord);
            auto plane = Plane::plan_path(map, transmission.coord, best_runway.squarepos, timer, last_plane_time);
            last_plane_time = plane.completion_time;
            paths.emplace(plane.id, std::move(plane));
            cerr << "Last time: " << last_plane_time << "\n";
        }
    }
};

constexpr int iterations = 1000;

int main() {
    srand(100);
    ATC atc;
    for (int i = 0; i < iterations; i++) {
        atc.tick(5.f / 60);

        float random_angle = (rand() % 10000) / 10000.0 * 2 * M_PI;
        // Convert angle to a point on the circumference of the circle
        auto spawn_loc = Coord{cos(random_angle), sin(random_angle)} * RADIUS + Coord{RADIUS, RADIUS};
        auto spawn_square = TimeSlice::pos_to_square(spawn_loc);

        if (atc.is_free_square(spawn_square) && i % 10 == 0) {
            atc.transmit(PositionTransmission{spawn_square, i});
        }
        out_file.flush();
    }

    cerr << "Done\n";
}