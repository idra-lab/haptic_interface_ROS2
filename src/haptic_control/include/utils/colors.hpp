#ifndef COLORS_HPP
#define COLORS_HPP
#include <vector>
namespace utils::colors
{
// declare color as a vector of doubles
typedef std::vector<double> color;
// define some colors
const color COLOR_RED = {1.0, 0.0, 0.0, 1.0};  // red
const color COLOR_GREEN = {0.0, 1.0, 0.0, 1.0};  // green
const color COLOR_BLUE = {0.0, 0.0, 1.0, 1.0};  // blue
const color COLOR_YELLOW = {1.0, 1.0, 0.0, 1.0};  // yellow
const color COLOR_CYAN = {0.0, 1.0, 1.0, 1.0};  // cyan
const color COLOR_MAGENTA = {1.0, 0.0, 1.0, 1.0};  // magenta
} // namespace utils::colors
#endif  // COLORS_HPP
