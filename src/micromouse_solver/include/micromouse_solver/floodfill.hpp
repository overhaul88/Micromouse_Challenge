#ifndef MICROMOUSE_SOLVER__FLOODFILL_HPP_
#define MICROMOUSE_SOLVER__FLOODFILL_HPP_

#include <algorithm>
#include <deque>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace micromouse_solver
{

enum class Direction { N, E, S, W };

inline std::pair<int, int> step(int c, int r, Direction d)
{
  switch (d) {
    case Direction::N: return {c, r + 1};
    case Direction::E: return {c + 1, r};
    case Direction::S: return {c, r - 1};
    case Direction::W: return {c - 1, r};
  }
  return {c, r};
}

constexpr int kUnreachable = std::numeric_limits<int>::max();

// A floodfill distance field over an n x n grid, with a known-wall grid
// that starts optimistic (everything but the outer perimeter is assumed
// open) and only grows as walls are actually sensed. Each report_wall()
// call recomputes the distance field starting from just the affected
// cells and propagating outward only as far as values actually change -
// most of the grid is untouched by any single wall discovery, which is
// the "incremental" part: this is not a from-scratch BFS on every update.
class Floodfill
{
public:
  explicit Floodfill(int size)
  : n_(size),
    vwall_(size + 1, std::vector<bool>(size, false)),
    hwall_(size + 1, std::vector<bool>(size, false)),
    distance_(size, std::vector<int>(size, kUnreachable))
  {
    for (int r = 0; r < n_; ++r) {
      vwall_[0][r] = true;
      vwall_[n_][r] = true;
    }
    for (int c = 0; c < n_; ++c) {
      hwall_[0][c] = true;
      hwall_[n_][c] = true;
    }
  }

  void set_goal_cells(const std::vector<std::pair<int, int>> & goals)
  {
    goal_cells_ = goals;
    for (auto & row : distance_) {
      std::fill(row.begin(), row.end(), kUnreachable);
    }
    std::deque<std::pair<int, int>> queue;
    for (const auto & g : goals) {
      distance_[g.second][g.first] = 0;
      queue.push_back(g);
    }
    propagate(queue);
  }

  // Marks a wall present on side `d` of cell (c, r), and its reciprocal on
  // the neighboring cell. Returns true if this was new information (and
  // therefore triggered a replan) - false if it was already known.
  bool report_wall(int c, int r, Direction d)
  {
    if (!in_bounds(c, r) || !set_wall(c, r, d, true)) {
      return false;
    }
    std::deque<std::pair<int, int>> queue;
    queue.push_back({c, r});
    auto [nc, nr] = step(c, r, d);
    if (in_bounds(nc, nr)) {
      queue.push_back({nc, nr});
    }
    propagate(queue);
    return true;
  }

  bool is_open(int c, int r, Direction d) const
  {
    return in_bounds(c, r) && !get_wall(c, r, d);
  }

  int distance(int c, int r) const
  {
    return in_bounds(c, r) ? distance_[r][c] : kUnreachable;
  }

  bool is_goal(int c, int r) const
  {
    for (const auto & g : goal_cells_) {
      if (g.first == c && g.second == r) {
        return true;
      }
    }
    return false;
  }

  // Among open neighbors, picks the lowest-distance one. Ties are broken
  // in favor of `preferred` (typically "go straight", i.e. current
  // heading) to avoid unnecessary turning; otherwise a fixed N,E,S,W
  // order keeps the result deterministic.
  std::optional<Direction> next_direction(int c, int r, std::optional<Direction> preferred) const
  {
    std::optional<Direction> best;
    int best_distance = kUnreachable;
    static constexpr Direction kOrder[4] = {Direction::N, Direction::E, Direction::S, Direction::W};

    for (Direction d : kOrder) {
      if (!is_open(c, r, d)) {
        continue;
      }
      auto [nc, nr] = step(c, r, d);
      if (!in_bounds(nc, nr) || distance_[nr][nc] >= kUnreachable) {
        continue;
      }
      int dist = distance_[nr][nc];

      bool strictly_better = !best.has_value() || dist < best_distance;
      bool tie_preferred = best.has_value() && dist == best_distance &&
        preferred.has_value() && d == *preferred;

      if (strictly_better || tie_preferred) {
        best = d;
        best_distance = dist;
      }
    }
    return best;
  }

  int size() const { return n_; }

  // Serializes the discovered wall map to a compact text form: a header
  // line with the grid size, then one row per vwall_ line, then one per
  // hwall_ line, each a string of '0'/'1'. Distances are not stored - they
  // are a pure function of walls + goals and are recomputed on load.
  std::string serialize() const
  {
    std::ostringstream os;
    os << "micromouse_map " << n_ << "\n";
    for (const auto & col : vwall_) {
      for (bool b : col) {
        os << (b ? '1' : '0');
      }
      os << "\n";
    }
    for (const auto & row : hwall_) {
      for (bool b : row) {
        os << (b ? '1' : '0');
      }
      os << "\n";
    }
    return os.str();
  }

  // Replaces the wall map from serialize() output. Returns false (leaving
  // the current map untouched) if the data is malformed or sized for a
  // different grid. Caller must re-run set_goal_cells() afterwards to
  // rebuild the distance field.
  bool deserialize(const std::string & data)
  {
    std::istringstream is(data);
    std::string tag;
    int n = 0;
    if (!(is >> tag >> n) || tag != "micromouse_map" || n != n_) {
      return false;
    }
    auto read_grid = [&is](std::vector<std::vector<bool>> & grid) -> bool {
      for (auto & line : grid) {
        std::string s;
        if (!(is >> s) || s.size() != line.size()) {
          return false;
        }
        for (size_t i = 0; i < line.size(); ++i) {
          line[i] = (s[i] == '1');
        }
      }
      return true;
    };
    auto v = vwall_;
    auto h = hwall_;
    if (!read_grid(v) || !read_grid(h)) {
      return false;
    }
    vwall_ = std::move(v);
    hwall_ = std::move(h);
    return true;
  }

  bool save_to_file(const std::string & path) const
  {
    std::ofstream f(path);
    if (!f) {
      return false;
    }
    f << serialize();
    return f.good();
  }

  bool load_from_file(const std::string & path)
  {
    std::ifstream f(path);
    if (!f) {
      return false;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return deserialize(ss.str());
  }

private:
  bool in_bounds(int c, int r) const { return c >= 0 && c < n_ && r >= 0 && r < n_; }

  bool get_wall(int c, int r, Direction d) const
  {
    switch (d) {
      case Direction::E: return vwall_[c + 1][r];
      case Direction::W: return vwall_[c][r];
      case Direction::N: return hwall_[r + 1][c];
      case Direction::S: return hwall_[r][c];
    }
    return true;
  }

  // Sets the wall (and its reciprocal on the neighbor implicitly, since
  // both cells share the same boundary array entry). Returns true if this
  // changed the stored value.
  bool set_wall(int c, int r, Direction d, bool value)
  {
    if (get_wall(c, r, d) == value) {
      return false;
    }
    switch (d) {
      case Direction::E: vwall_[c + 1][r] = value; break;
      case Direction::W: vwall_[c][r] = value; break;
      case Direction::N: hwall_[r + 1][c] = value; break;
      case Direction::S: hwall_[r][c] = value; break;
    }
    return true;
  }

  void propagate(std::deque<std::pair<int, int>> & queue)
  {
    static constexpr Direction kOrder[4] = {Direction::N, Direction::E, Direction::S, Direction::W};
    // Generous safety bound: a real wall report from sensing an actually-
    // connected maze never needs anywhere near this many relaxations (the
    // 16x16 reference maze fully converges in well under 1000). This
    // guard only matters if something upstream ever feeds wall data that
    // contradicts itself (impossible for real ground-truth sensing, but
    // cheap insurance against the control loop ever locking up).
    constexpr int kMaxIterations = 20000;
    int iterations = 0;

    while (!queue.empty()) {
      if (++iterations > kMaxIterations) {
        break;
      }
      auto [c, r] = queue.front();
      queue.pop_front();

      bool changed;
      if (is_goal(c, r)) {
        // Goal distance is fixed at 0 and never recomputed, but its
        // neighbors' distances still depend on it, so it must still
        // propagate outward - just skip the recompute step below.
        changed = true;
      } else {
        int best = kUnreachable;
        for (Direction d : kOrder) {
          if (!is_open(c, r, d)) {
            continue;
          }
          auto [nc, nr] = step(c, r, d);
          if (in_bounds(nc, nr) && distance_[nr][nc] < kUnreachable) {
            best = std::min(best, distance_[nr][nc] + 1);
          }
        }
        changed = (best != distance_[r][c]);
        if (changed) {
          distance_[r][c] = best;
        }
      }

      if (changed) {
        // Only cells whose own optimal distance just changed can possibly
        // change their open neighbors' optimal distances, so only those
        // get re-queued - this is what keeps the update localized instead
        // of touching the whole grid. Goal cells are never re-queued:
        // their distance is fixed at 0 forever, and since goal cells can
        // be mutually adjacent (the 2x2 center block), re-queueing them
        // would otherwise let two goal cells re-trigger each other
        // indefinitely.
        for (Direction d : kOrder) {
          if (!is_open(c, r, d)) {
            continue;
          }
          auto [nc, nr] = step(c, r, d);
          if (in_bounds(nc, nr) && !is_goal(nc, nr)) {
            queue.push_back({nc, nr});
          }
        }
      }
    }
  }

  int n_;
  std::vector<std::vector<bool>> vwall_;  // vwall_[c][r]: boundary at x = c*cell_size
  std::vector<std::vector<bool>> hwall_;  // hwall_[r][c]: boundary at y = r*cell_size
  std::vector<std::vector<int>> distance_;
  std::vector<std::pair<int, int>> goal_cells_;
};

}  // namespace micromouse_solver

#endif  // MICROMOUSE_SOLVER__FLOODFILL_HPP_
