//
// Modified by jungwon on 19. 1. 21.
// Add side constraint on 19. 5. 14.
// grid free on 19. 8. 29.
//

#ifndef SWARM_PLANNER_ENVIRONMENT_H
#define SWARM_PLANNER_ENVIRONMENT_H

#include <ecbs.hpp>
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>
//#include <../../../include/sp_const.hpp>
#include <sp_const.hpp>

using namespace libMultiRobotPlanning;

namespace libMultiRobotPlanning {
    struct State {
        State(int time, int x, int y, int z) : time(time), x(x), y(y), z(z) {}

        bool operator==(const State &s) const {
            return time == s.time && x == s.x && y == s.y && z == s.z;
        }

        State operator-(const State &s) const {
            return {time, x - s.x, y - s.y, z - s.z};
        }

        bool equalExceptTime(const State &s) const { return x == s.x && y == s.y && z == s.z; }

        friend std::ostream &operator<<(std::ostream &os, const State &s) {
            return os << s.time << ": (" << s.x << "," << s.y << "," << s.z << ")";
            // return os << "(" << s.x << "," << s.y << "," << s.z <<")";
        }

        int time;
        int x;
        int y;
        int z;
    };

    struct Vector {
        Vector(double x, double y, double z) : x(x), y(y), z(z) {}
        Vector (const State& s): x((double)s.x), y((double)s.y), z((double)s.z) {}

        bool operator==(const Vector &v) const {
            return x == v.x && y == v.y && z == v.z;
        }

        Vector operator-(const Vector &v) const {
            return {x - v.x, y - v.y, z - v.z};
        }

        double norm(){
            return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
        }

        void normalize(){
            double vector_norm = norm();
            x /= vector_norm;
            y /= vector_norm;
            z /= vector_norm;
        }

        double dot(const Vector& v){
            return x*v.x+y*v.y+z*v.z;
        }

        double min_dist_to_origin(Vector b){
            Vector a = *this;
            double min_dist = norm();

            if(!(a == b)) {
                double dist = b.norm();
                if (min_dist > dist) {
                    min_dist = dist;
                }

                Vector n = b - a;
                n.normalize();
                double a_dot_n = a.dot(n);
                n.x *= a_dot_n;
                n.y *= a_dot_n;
                n.z *= a_dot_n;
                Vector c = a - n;
                dist = c.norm();

                if ((c - a).dot(c - b) < 0 && min_dist > dist) {
                    min_dist = dist;
                }
            }
            return min_dist;
        }

        double x;
        double y;
        double z;
    };
}

namespace std {
    template <>
    struct hash<State> {
        size_t operator()(const State& s) const {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);
            boost::hash_combine(seed, s.z);
            return seed;
        }
    };
}  // namespace std

namespace libMultiRobotPlanning {
    enum class Action {
        Up,
        Down,
        Left,
        Right,
        Top,
        Bottom,
        Wait,
    };

    std::ostream &operator<<(std::ostream &os, const Action &a) {
        switch (a) {
            case Action::Up:
                os << "Up";
                break;
            case Action::Down:
                os << "Down";
                break;
            case Action::Left:
                os << "Left";
                break;
            case Action::Right:
                os << "Right";
                break;
            case Action::Top:
                os << "Top";
                break;
            case Action::Bottom:
                os << "Bottom";
                break;
            case Action::Wait:
                os << "Wait";
                break;
        }
        return os;
    }

    struct Conflict {
        enum Type {
            Vertex,
            Edge,
        };

        int time;
        size_t agent1;
        size_t agent2;
        Type type;

        int x1;
        int y1;
        int z1;
        int x1_2;
        int y1_2;
        int z1_2;
        int x2;
        int y2;
        int z2;
        int x2_2;
        int y2_2;
        int z2_2;

        friend std::ostream &operator<<(std::ostream &os, const Conflict &c) {
            switch (c.type) {
                case Vertex:
                    return os << c.time << ": (" << c.agent1 << "," << c.agent2
                              << "): Vertex(" << c.x1 << "," << c.y1 << "," << c.z1 << ")";
                case Edge:
                    return os << c.time << ": (" << c.agent1 << "," << c.agent2
                              << "): Edge(" << c.x1 << "," << c.y1 << "," << c.z1 << ","
                              << c.x1_2 << "," << c.y1_2 << "," << c.z1_2 << ","
                              << c.x2 << "," << c.y2 << "," << c.z2 << ","
                              << c.x2_2 << "," << c.y2_2 << "," << c.z2_2 << ")";
            }
            return os;
        }
    };

    struct VertexConstraint {
        VertexConstraint(int time, int x, int y, int z) : time(time), x(x), y(y), z(z) {}

        int time;
        int x;
        int y;
        int z;

        bool operator<(const VertexConstraint &other) const {
            return std::tie(time, x, y, z) < std::tie(other.time, other.x, other.y, other.z);
        }

        bool operator==(const VertexConstraint &other) const {
            return std::tie(time, x, y, z) == std::tie(other.time, other.x, other.y, other.z);
        }

        friend std::ostream &operator<<(std::ostream &os, const VertexConstraint &c) {
            return os << "VC(" << c.time << "," << c.x << "," << c.y << "," << c.z << ")";
        }
    };
}

namespace std {
    template <>
    struct hash<VertexConstraint> {
        size_t operator()(const VertexConstraint& s) const {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);
            boost::hash_combine(seed, s.z);
            return seed;
        }
    };
}  // namespace std

namespace libMultiRobotPlanning {
    struct EdgeConstraint {
        EdgeConstraint(int time, int x1, int y1, int z1, int x2, int y2, int z2)
                : time(time), x1(x1), y1(y1), z1(z1), x2(x2), y2(y2), z2(z2) {}

        int time;
        int x1;
        int y1;
        int z1;
        int x2;
        int y2;
        int z2;

        bool operator<(const EdgeConstraint &other) const {
            return std::tie(time, x1, y1, z1, x2, y2, z2) <
                   std::tie(other.time, other.x1, other.y1, other.z1, other.x2, other.y2, other.z2);
        }

        bool operator==(const EdgeConstraint &other) const {
            return std::tie(time, x1, y1, z1, x2, y2, z2) ==
                   std::tie(other.time, other.x1, other.y1, other.z1, other.x2, other.y2, other.z2);
        }

        friend std::ostream &operator<<(std::ostream &os, const EdgeConstraint &c) {
            return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.z1 << ","
                      << c.x2 << "," << c.y2 << "," << c.z2 << ")";
        }
    };
}

namespace std {
    template <>
    struct hash<EdgeConstraint> {
        size_t operator()(const EdgeConstraint& s) const {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x1);
            boost::hash_combine(seed, s.y1);
            boost::hash_combine(seed, s.z1);
            boost::hash_combine(seed, s.x2);
            boost::hash_combine(seed, s.y2);
            boost::hash_combine(seed, s.z2);
            return seed;
        }
    };
}  // namespace std

namespace libMultiRobotPlanning {
    struct Constraints {
        std::unordered_set<VertexConstraint> vertexConstraints;
        std::unordered_set<EdgeConstraint> edgeConstraints;

        void add(const Constraints &other) {
            vertexConstraints.insert(other.vertexConstraints.begin(),
                                     other.vertexConstraints.end());
            edgeConstraints.insert(other.edgeConstraints.begin(),
                                   other.edgeConstraints.end());
        }

        bool overlap(const Constraints &other) {
            std::vector<VertexConstraint> vertexIntersection;
            std::vector<EdgeConstraint> edgeIntersection;
            std::set_intersection(vertexConstraints.begin(), vertexConstraints.end(),
                                  other.vertexConstraints.begin(),
                                  other.vertexConstraints.end(),
                                  std::back_inserter(vertexIntersection));
            std::set_intersection(edgeConstraints.begin(), edgeConstraints.end(),
                                  other.edgeConstraints.begin(),
                                  other.edgeConstraints.end(),
                                  std::back_inserter(edgeIntersection));
            return !vertexIntersection.empty() || !edgeIntersection.empty();
        }

        friend std::ostream &operator<<(std::ostream &os, const Constraints &c) {
            for (const auto &vc : c.vertexConstraints) {
                os << vc << std::endl;
            }
            for (const auto &ec : c.edgeConstraints) {
                os << ec << std::endl;
            }
            return os;
        }
    };

    struct Location {
        Location(int x, int y, int z) : x(x), y(y), z(z) {}

        int x;
        int y;
        int z;

        bool operator<(const Location &other) const {
            return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
        }

        bool operator==(const Location &other) const {
            return std::tie(x, y, z) == std::tie(other.x, other.y, other.z);
        }

        friend std::ostream &operator<<(std::ostream &os, const Location &c) {
            return os << "(" << c.x << "," << c.y << "," << c.z << ")";
        }
    };
}
namespace std {
    template <>
    struct hash<Location> {
        size_t operator()(const Location& s) const {
            size_t seed = 0;
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);
            boost::hash_combine(seed, s.z);
            return seed;
        }
    };
}  // namespace std

///
namespace libMultiRobotPlanning {
    class Environment {
    public:
        Environment(std::vector<int> dimx, std::vector<int> dimy, std::vector<int> dimz,
                    std::vector<double> grid_x_min, std::vector<double> grid_y_min, std::vector<double> grid_z_min,
                    std::vector<double> grid_res,
                    std::vector<std::unordered_set<Location>> obstacles,
                    std::vector<std::unordered_set<EdgeConstraint>> obstacle_sections,
                    std::vector<Location> goals,
                    std::vector<std::vector<SwarmPlanning::CollisionModel_internal>> quad_collision_model)
                : m_dimx(std::move(dimx)),
                  m_dimy(std::move(dimy)),
                  m_dimz(std::move(dimz)),
                  m_grid_x_min(std::move(grid_x_min)),
                  m_grid_y_min(std::move(grid_y_min)),
                  m_grid_z_min(std::move(grid_z_min)),
                  m_grid_res(std::move(grid_res)),
                  m_obstacles(std::move(obstacles)),
                  m_obstacle_sections(std::move(obstacle_sections)),
                  m_goals(std::move(goals)),
                  m_agentIdx(0),
                  m_constraints(nullptr),
                  m_lastGoalConstraint(-1),
                  m_highLevelExpanded(0),
                  m_lowLevelExpanded(0),
                  m_quad_collision_model(std::move(quad_collision_model)) {}

        Environment(const Environment &) = delete;

        Environment &operator=(const Environment &) = delete;

        //find last goal constraint!
        void setLowLevelContext(size_t agentIdx, const Constraints *constraints) {
            assert(constraints);
            m_agentIdx = agentIdx;
            m_constraints = constraints;
            m_lastGoalConstraint = -1;
            for (const auto &vc : constraints->vertexConstraints) {
                if (vc.x == m_goals[m_agentIdx].x && vc.y == m_goals[m_agentIdx].y && vc.z == m_goals[m_agentIdx].z) {
                    m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
                }
            }
        }

        int admissibleHeuristic(const State &s) {
            return std::abs(s.x - m_goals[m_agentIdx].x) +
                   std::abs(s.y - m_goals[m_agentIdx].y) +
                   std::abs(s.z - m_goals[m_agentIdx].z);
        }

        // low-level, get numConflict(equal state) from given solution
        int focalStateHeuristic(
                const State &s, int /*gScore*/,
                const std::vector<PlanResult<State, Action, int> > &solution) {
            int numConflicts = 0;
            for (size_t i = 0; i < solution.size(); ++i) {
                if (i != m_agentIdx && !solution[i].states.empty()) {
                    State state2 = getState(i, solution, s.time);
                    if (isVertexConflict(m_agentIdx, i, s, state2)) {
                        ++numConflicts;
                    }
                }
            }
            return numConflicts;
        }

        // low-level, get numConflict(s1a <-> s1b) from given solution
        int focalTransitionHeuristic(
                const State &s1a, const State &s1b, int /*gScoreS1a*/, int /*gScoreS1b*/,
                const std::vector<PlanResult<State, Action, int> > &solution) {
            int numConflicts = 0;
            for (size_t i = 0; i < solution.size(); ++i) {
                if (i != m_agentIdx && !solution[i].states.empty()) {
                    State s2a = getState(i, solution, s1a.time);
                    State s2b = getState(i, solution, s1b.time);
                    if (isEdgeConflict(m_agentIdx, i, s1a, s1b, s2a, s2b)) {
                        ++numConflicts;
                    }
                }
            }
            return numConflicts;
        }

        // Count all conflicts
        int focalHeuristic(
                const std::vector<PlanResult<State, Action, int> > &solution) {
            int numConflicts = 0;

            int max_t = 0;
            for (const auto &sol : solution) {
                max_t = std::max<int>(max_t, sol.states.size() - 1);
            }

            for (int t = 0; t < max_t; ++t) {
                // check drive-drive vertex collisions
                for (size_t i = 0; i < solution.size(); ++i) {
                    State state1 = getState(i, solution, t);
                    for (size_t j = i + 1; j < solution.size(); ++j) {
                        State state2 = getState(j, solution, t);
                        if (isVertexConflict(i, j, state1, state2)) {
                            ++numConflicts;
                        }
                    }
                }
                // drive-drive edge (swap)
                for (size_t i = 0; i < solution.size(); ++i) {
                    State state1a = getState(i, solution, t);
                    State state1b = getState(i, solution, t + 1);
                    for (size_t j = i + 1; j < solution.size(); ++j) {
                        State state2a = getState(j, solution, t);
                        State state2b = getState(j, solution, t + 1);
                        if (isEdgeConflict(i, j, state1a, state1b, state2a, state2b)) {
                            ++numConflicts;
                        }
                    }
                }
            }
            return numConflicts;
        }

        bool isSolution(const State &s) {
            return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y && s.z == m_goals[m_agentIdx].z &&
                   s.time > m_lastGoalConstraint;
        }

        void getNeighbors(const State &s,
                          std::vector<Neighbor<State, Action, int> > &neighbors) {
            // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
            // for(const auto& vc : constraints.vertexConstraints) {
            //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y << "," << vc.z <<
            //   std::endl;
            // }
            neighbors.clear();
            {
                State n(s.time + 1, s.x, s.y, s.z);
                if (stateValid(n) && transitionValid(s, n)) {
                    neighbors.emplace_back(
                            Neighbor<State, Action, int>(n, Action::Wait, 1));
                }
            }
            {
                State n(s.time + 1, s.x - 1, s.y, s.z);
                if (stateValid(n) && transitionValid(s, n)) {
                    neighbors.emplace_back(
                            Neighbor<State, Action, int>(n, Action::Left, 1));
                }
            }
            {
                State n(s.time + 1, s.x + 1, s.y, s.z);
                if (stateValid(n) && transitionValid(s, n)) {
                    neighbors.emplace_back(
                            Neighbor<State, Action, int>(n, Action::Right, 1));
                }
            }
            {
                State n(s.time + 1, s.x, s.y + 1, s.z);
                if (stateValid(n) && transitionValid(s, n)) {
                    neighbors.emplace_back(
                            Neighbor<State, Action, int>(n, Action::Up, 1));
                }
            }
            {
                State n(s.time + 1, s.x, s.y - 1, s.z);
                if (stateValid(n) && transitionValid(s, n)) {
                    neighbors.emplace_back(
                            Neighbor<State, Action, int>(n, Action::Down, 1));
                }
            }
            {
                State n(s.time + 1, s.x, s.y, s.z + 1);
                if (stateValid(n) && transitionValid(s, n)) {
                    neighbors.emplace_back(
                            Neighbor<State, Action, int>(n, Action::Top, 1));
                }
            }
            {
                State n(s.time + 1, s.x, s.y, s.z - 1);
                if (stateValid(n) && transitionValid(s, n)) {
                    neighbors.emplace_back(
                            Neighbor<State, Action, int>(n, Action::Bottom, 1));
                }
            }
        }

        bool getFirstConflict(
                const std::vector<PlanResult<State, Action, int> > &solution,
                Conflict &result) {
            int max_t = 0;
            for (const auto &sol : solution) {
                max_t = std::max<int>(max_t, sol.states.size() - 1);
            }

            for (int t = 0; t < max_t; ++t) {
                // check drive-drive vertex collisions
                for (size_t i = 0; i < solution.size(); ++i) {
                    State state1 = getState(i, solution, t);
                    for (size_t j = i + 1; j < solution.size(); ++j) {
                        State state2 = getState(j, solution, t);
                        if (isVertexConflict(i, j, state1, state2)) {
                            result.time = t;
                            result.agent1 = i;
                            result.agent2 = j;
                            result.type = Conflict::Vertex;
                            result.x1 = state1.x;
                            result.y1 = state1.y;
                            result.z1 = state1.z;
                            result.x2 = state2.x;
                            result.y2 = state2.y;
                            result.z2 = state2.z;
                            // std::cout << "VC " << t << "," << state1.x << "," << state1.y << "," << state1.z <<
                            // std::endl;
                            return true;
                        }
                    }
                }
                // drive-drive edge (swap)
                for (size_t i = 0; i < solution.size(); ++i) {
                    State state1a = getState(i, solution, t);
                    State state1b = getState(i, solution, t + 1);
                    for (size_t j = i + 1; j < solution.size(); ++j) {
                        State state2a = getState(j, solution, t);
                        State state2b = getState(j, solution, t + 1);

                        if (isEdgeConflict(i, j, state1a, state1b, state2a, state2b)) {
                            result.time = t;
                            result.agent1 = i;
                            result.agent2 = j;
                            result.type = Conflict::Edge;
                            result.x1 = state1a.x;
                            result.y1 = state1a.y;
                            result.z1 = state1a.z;
                            result.x1_2 = state1b.x;
                            result.y1_2 = state1b.y;
                            result.z1_2 = state1b.z;
                            result.x2 = state2a.x;
                            result.y2 = state2a.y;
                            result.z2 = state2a.z;
                            result.x2_2 = state2b.x;
                            result.y2_2 = state2b.y;
                            result.z2_2 = state2b.z;
                            return true;
                        }
                    }
                }
            }

            return false;
        }

        void createConstraintsFromConflict(
                const Conflict &conflict, std::map<size_t, Constraints> &constraints) {
            if (conflict.type == Conflict::Vertex) {
                Constraints c1, c2;
                c1.vertexConstraints.emplace(
                        VertexConstraint(conflict.time, conflict.x1, conflict.y1, conflict.z1));
                c2.vertexConstraints.emplace(
                        VertexConstraint(conflict.time, conflict.x2, conflict.y2, conflict.z2));
                constraints[conflict.agent1] = c1;
                constraints[conflict.agent2] = c2;
            } else if (conflict.type == Conflict::Edge) {
                Constraints c1, c2;
                c1.edgeConstraints.emplace(EdgeConstraint(
                        conflict.time, conflict.x1, conflict.y1, conflict.z1, conflict.x1_2, conflict.y1_2, conflict.z1_2));
                c2.edgeConstraints.emplace(EdgeConstraint(
                        conflict.time, conflict.x2, conflict.y2, conflict.z2, conflict.x2_2, conflict.y2_2, conflict.z2_2));
                constraints[conflict.agent1] = c1;
                constraints[conflict.agent2] = c2;
            }
        }

        void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

        void onExpandLowLevelNode(const State & /*s*/, int /*fScore*/,
                                  int /*gScore*/) {
            m_lowLevelExpanded++;
        }

        int highLevelExpanded() { return m_highLevelExpanded; }

        int lowLevelExpanded() const { return m_lowLevelExpanded; }

    private:
        State getState(size_t agentIdx,
                       const std::vector<PlanResult<State, Action, int> > &solution,
                       size_t t) {
            assert(agentIdx < solution.size());
            if (t < solution[agentIdx].states.size()) {
                return solution[agentIdx].states[t].first;
            }
            assert(!solution[agentIdx].states.empty());
            return solution[agentIdx].states.back().first;
        }

        bool stateValid(const State &s) {
            assert(m_constraints);
            const auto &con = m_constraints->vertexConstraints;
            return s.x >= 0 && s.x < m_dimx[m_agentIdx]
                   && s.y >= 0 && s.y < m_dimy[m_agentIdx]
                   && s.z >= 0 && s.z < m_dimz[m_agentIdx]
                   && m_obstacles[m_agentIdx].find(Location(s.x, s.y, s.z)) == m_obstacles[m_agentIdx].end()
                   && con.find(VertexConstraint(s.time, s.x, s.y, s.z)) == con.end();
        }

        bool transitionValid(const State &s1, const State &s2) {
            assert(m_constraints);
            const auto &con = m_constraints->edgeConstraints;
            return m_obstacle_sections[m_agentIdx].find(EdgeConstraint(0, s1.x, s1.y, s1.z, s2.x, s2.y, s2.z)) == m_obstacle_sections[m_agentIdx].end()
                   && con.find(EdgeConstraint(s1.time, s1.x, s1.y, s1.z, s2.x, s2.y, s2.z)) == con.end();
        }

//        bool isParallel(const State &state1a, const State &state1b, const State &state2a, const State &state2b) {
//            return (state1b.x - state1a.x) == (state2b.x - state2a.x) &&
//                   (state1b.y - state1a.y) == (state2b.y - state2a.y) &&
//                   (state1b.z - state1a.z) == (state2b.z - state2a.z);
//        }

        bool isVertexConflict(int i, int j, const State &state1, const State &state2) {
            double quad_r, quad_a, quad_b;
            quad_r = m_quad_collision_model[i][j].r;
            quad_a = m_quad_collision_model[i][j].a;
            quad_b = m_quad_collision_model[i][j].b;

            Vector a(stateTransform(j, state2) - stateTransform(i, state1));
            a.z = (a.z + (quad_a - quad_b) / 2) * 2 * quad_r / (quad_a + quad_b);

            double min_dist = minBoxDist_LineSegment(a, a);
            return min_dist < m_quad_collision_model[i][j].r - SP_EPSILON;
        }

        bool isEdgeConflict(int i, int j, const State &state1a, const State &state1b,
                                          const State &state2a, const State &state2b){
            double quad_r, quad_a, quad_b;
            quad_r = m_quad_collision_model[i][j].r;
            quad_a = m_quad_collision_model[i][j].a;
            quad_b = m_quad_collision_model[i][j].b;

            Vector a(stateTransform(j, state2a) - stateTransform(i, state1a));
            a.z = (a.z + (quad_b - quad_a) / 2) * 2 * quad_r / (quad_a + quad_b);
            Vector b(stateTransform(j, state2b) - stateTransform(i, state1b));
            b.z = (b.z + (quad_b - quad_a) / 2) * 2 * quad_r / (quad_a + quad_b);

            double min_dist = minBoxDist_LineSegment(a, b);
//            bool debug = min_dist < m_quad_collision_model[i][j].r - SP_EPSILON;
//            if(debug){
//                int asdf = 0;
//            }
            return min_dist < m_quad_collision_model[i][j].r - SP_EPSILON;
        }

    private:
        std::vector<int> m_dimx, m_dimy, m_dimz;
        std::vector<double> m_grid_x_min, m_grid_y_min, m_grid_z_min, m_grid_res;
        std::vector<std::unordered_set<Location>> m_obstacles;
        std::vector<std::unordered_set<EdgeConstraint>> m_obstacle_sections;
        std::vector<Location> m_goals;
        size_t m_agentIdx;
        const Constraints *m_constraints;
        int m_lastGoalConstraint;
        int m_highLevelExpanded;
        int m_lowLevelExpanded;
        std::vector<std::vector<SwarmPlanning::CollisionModel_internal>> m_quad_collision_model;

        Vector stateTransform(size_t agentIdx, const Vector& a){
            return Vector(a.x * m_grid_res[agentIdx] + m_grid_x_min[agentIdx],
                          a.y * m_grid_res[agentIdx] + m_grid_y_min[agentIdx],
                          a.z * m_grid_res[agentIdx] + m_grid_z_min[agentIdx]);
        }

        double boxDistance(const Vector& a){
            return std::max({std::abs(a.x), std::abs(a.y), std::abs(a.z)});
        }

        bool isMinBoxPointInLineSegment(const Vector& C, const Vector& D){
            double boxDist_C = boxDistance(C);
            double boxDist_D = boxDistance(D);
            double minBoxDist;
            if(boxDist_C < boxDist_D) {
                minBoxDist = boxDist_C;
                if (std::abs(C.x) == minBoxDist) {
                    return C.x * (D.x - C.x) < 0;
                } else if (std::abs(C.y) == minBoxDist) {
                    return C.y * (D.y - C.y) < 0;
                } else {
                    return C.z * (D.z - C.z) < 0;
                }
            }
            else{
                minBoxDist = boxDist_D;
                if (std::abs(D.x) == minBoxDist) {
                    return D.x * (C.x - D.x) < 0;
                } else if (std::abs(D.y) == minBoxDist) {
                    return D.y * (C.y - D.y) < 0;
                } else {
                    return D.z * (C.z - D.z) < 0;
                }
            }
        }

        double minBoxDist_LineSegment(const Vector& C, const Vector& D){
            double min_dist;
            double boxDist_C = boxDistance(C);
            double boxDist_D = boxDistance(D);

            if(boxDist_C < boxDist_D) {
                min_dist = boxDist_C;
            }
            else{
                min_dist = boxDist_D;
            }

            if(!isMinBoxPointInLineSegment(C, D)){
                return min_dist;
            }
            double d_xy = 0, d_yz = 0, d_zx = 0;
            if(D.y - C.y != 0 || D.x - C.x != 0){
                d_xy = std::abs(C.y * (D.x - C.x) - C.x * (D.y - C.y)) / (std::abs(D.y - C.y) + std::abs(D.x - C.x));
            }
            if(D.z - C.z != 0 || D.y - C.y != 0){
                d_yz = std::abs(C.z * (D.y - C.y) - C.y * (D.z - C.z)) / (std::abs(D.z - C.z) + std::abs(D.y - C.y));
            }
            if(D.x - C.x != 0 || D.z - C.z != 0){
                d_zx = std::abs(C.x * (D.z - C.z) - C.z * (D.x - C.x)) / (std::abs(D.x - C.x) + std::abs(D.z - C.z));
            }
            min_dist = std::min(min_dist, std::max(d_xy, std::max(d_yz, d_zx)));
            return min_dist;
        }
    };
}
#endif //SWARM_PLANNER_ENVIRONMENT_H
