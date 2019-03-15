#ifndef _YOUR_PLANNER_H_
#define _YOUR_PLANNER_H_

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#include "RrtConConBase.h"

using namespace ::rl::plan;

/**
*	The implementation of your planner.
*	modify any of the existing methods to improve planning performance.
*/
class YourPlanner : public RrtConConBase
{
public:
  YourPlanner();

  virtual ~YourPlanner();

  virtual ::std::string getName() const;

  bool solve();

  // NEW CODE

  // Variables for Sampling near Collision Extension
  int perseverance; // number of sample around a collision point before going back to random sampling
  double randomSigma;
  double collisionSigma;
  int randomCounter[2];
  double sigma[2];
  int j;//the tree selector
  ::rl::math::Vector collisionPointA;
  ::rl::math::Vector collisionPointB;

  // Variable for Exhausted Nodes extension
  int threshold = 3;

protected:
  void choose(::rl::math::Vector& chosen);

  RrtConConBase::Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

  RrtConConBase::Vertex extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

  RrtConConBase::Neighbor nearest(const Tree& tree, const ::rl::math::Vector& chosen);

private:
};

#endif // _YOUR_PLANNER_H_
