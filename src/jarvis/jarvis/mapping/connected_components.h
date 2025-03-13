#ifndef JARVIS_MAPPING_INTERNAL_CONNECTED_COMPONENTS_H_
#define JARVIS_MAPPING_INTERNAL_CONNECTED_COMPONENTS_H_

#include <map>
#include <mutex>
#include <unordered_map>
#include <vector>
namespace jarvis {
namespace mapping {

// A class that tracks the connectivity structure between trajectories.
//
// Connectivity includes both the count ("How many times have I _directly_
// connected trajectories i and j?") and the transitive connectivity.
//
// This class is thread-safe.
class ConnectedComponents {
 public:
  ConnectedComponents();

  ConnectedComponents(const ConnectedComponents&) = delete;
  ConnectedComponents& operator=(const ConnectedComponents&) = delete;

  // Add a trajectory which is initially connected to only itself.
  void Add(int trajectory_id) ;

  // Connect two trajectories. If either trajectory is untracked, it will be
  // tracked. This function is invariant to the order of its arguments. Repeated
  // calls to Connect increment the connectivity count.
  void Connect(int trajectory_id_a, int trajectory_id_b);

  // Determines if two trajectories have been (transitively) connected. If
  // either trajectory is not being tracked, returns false, except when it is
  // the same trajectory, where it returns true. This function is invariant to
  // the order of its arguments.
  bool TransitivelyConnected(int trajectory_id_a, int trajectory_id_b);

  // Return the number of _direct_ connections between 'trajectory_id_a' and
  // 'trajectory_id_b'. If either trajectory is not being tracked, returns 0.
  // This function is invariant to the order of its arguments.
  int ConnectionCount(int trajectory_id_a, int trajectory_id_b);

  // The trajectory IDs, grouped by connectivity.
  std::vector<std::vector<int>> Components();

  // The list of trajectory IDs that belong to the same connected component as
  // 'trajectory_id'.
  std::vector<int> GetComponent(int trajectory_id) ;

 private:
  // Find the representative and compresses the path to it.
  int FindSet(int trajectory_id) ;
  void Union(int trajectory_id_a, int trajectory_id_b);

  std::mutex lock_;
  // Tracks transitive connectivity using a disjoint set forest, i.e. each
  // entry points towards the representative for the given trajectory.
  std::map<int, int> forest_ ;
  // Tracks the number of direct connections between a pair of trajectories.
  std::map<std::pair<int, int>, int> connection_map_ ;
};
}  // namespace mapping
}  // namespace jarvis

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_CONNECTED_COMPONENTS_H_
