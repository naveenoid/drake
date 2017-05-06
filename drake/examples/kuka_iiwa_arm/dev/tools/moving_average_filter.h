# pragma once

#include <vector>

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace tools {

template <typename T>
class MovingAverageFilter {
 public:
  MovingAverageFilter(int window_size, int data_dimension);

  T compute(T new_data);

 private :
  const int window_size_{-1};
  const int data_dimension_{-1};

  std::vector<T> window;
};

}  // namespace tools
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
