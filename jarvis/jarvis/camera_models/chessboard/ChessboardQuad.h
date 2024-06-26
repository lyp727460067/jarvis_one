#ifndef CHESSBOARDQUAD_H
#define CHESSBOARDQUAD_H

#include <memory>

#include "camera_models/chessboard/ChessboardCorner.h"
namespace jarvis {
namespace camera_models {

class ChessboardQuad;
typedef std::shared_ptr<ChessboardQuad> ChessboardQuadPtr;

class ChessboardQuad {
 public:
  ChessboardQuad()
      : count(0), group_idx(-1), edge_len(FLT_MAX), labeled(false) {}

  int count;                       // Number of quad neighbors
  int group_idx;                   // Quad group ID
  float edge_len;                  // Smallest side length^2
  ChessboardCornerPtr corners[4];  // Coordinates of quad corners
  ChessboardQuadPtr neighbors[4];  // Pointers of quad neighbors
  bool labeled;                    // Has this corner been labeled?
};

}  // namespace camera_models
}  // namespace jarvis
#endif
