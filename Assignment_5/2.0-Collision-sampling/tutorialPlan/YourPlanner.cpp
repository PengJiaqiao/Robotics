#include "YourPlanner.h"

#include <rl/plan/GaussianSampler.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/Verifier.h>
#include <rl/plan/Viewer.h>
#include <boost/make_shared.hpp>
#include <iostream> // for debugging purpose

YourPlanner::YourPlanner() :
  RrtConConBase(),
  perseverance(200),
  randomSigma(4.0f),
  collisionSigma(0.3f),
  randomCounter({201,201}),
  sigma({randomSigma +1,randomSigma + 1})
{

}

YourPlanner::~YourPlanner()
{
}

::std::string
YourPlanner::getName() const
{
  return "Your Planner";
}

void
YourPlanner::choose(::rl::math::Vector& chosen)
{
  //your modifications here
  //RrtConConBase::choose(chosen);

  chosen = this->sampler->generate();

  ::rl::math::Vector median;
  if(randomCounter[j] > this->perseverance){ // after some time I restart sampling randomly
    randomCounter[j] = 0;
    this->sigma[j] = this->randomSigma; // since the sigma is very high, the gaussian approximate a uniform distribution
    this->collisionPointA = this->sampler->generate(); //the collision point is set to random
  }
  else{
    randomCounter[j] ++;
  }
  //the median is the previous collision point
  median = this->collisionPointA;
  // change the width and the median of the gaussian distribution of samples.
  chosen = (chosen * this->sigma[j]) + median;

  //chosen = chosen * sigma;
}

RrtConConBase::Vertex 
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //your modifications here
  //return RrtConConBase::connect(tree, nearest, chosen);
  //Do first extend step

  ::rl::math::Real distance = nearest.second;
  ::rl::math::Real step = distance;

  bool reached = false;
  bool connectSuccesful = true;

  if (step <= this->delta)
  {
    reached = true;
  }
  else
  {
    step = this->delta;
  }

  ::rl::plan::VectorPtr last = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());

  // move "last" along the line q<->chosen by distance "step / distance"
  this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *last);

  this->model->setPosition(*last);
  this->model->updateFrames();

  if (this->model->isColliding())
  {
    return NULL;
  }

  ::rl::math::Vector next(this->model->getDof());

  while (!reached)
  {
    //Do further extend step

    distance = this->model->distance(*last, chosen);
    step = distance;

    if (step <= this->delta)
    {
      reached = true;
    }
    else
    {
      step = this->delta;
    }

    // move "next" along the line last<->chosen by distance "step / distance"
    this->model->interpolate(*last, chosen, step / distance, next);

    this->model->setPosition(next);
    this->model->updateFrames();

    if (this->model->isColliding())
    {
      connectSuccesful = false;// collision!
      break;
    }

    *last = next;
  }

  // "last" now points to the vertex where the connect step collided with the environment.
  // Add it to the tree
  Vertex connected = this->addVertex(tree, last);
  this->addEdge(nearest.first, connected, tree);
  // assign to the vertex connectSuccesful = false if the connect function ended in a collision
  tree[connected].connectSuccesful = connectSuccesful;
  return connected;
}

RrtConConBase::Vertex 
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //your modifications here
  return RrtConConBase::extend(tree, nearest, chosen);
}

bool
YourPlanner::solve()
{
  //your modifications here
  //return RrtConConBase::solve();

  RrtConConBase::choose(this->collisionPointA);
  RrtConConBase::choose(this->collisionPointB);

  this->time = ::std::chrono::steady_clock::now();
  // Define the roots of both trees
  this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector >(*this->start));
  this->begin[1] = this->addVertex(this->tree[1], ::std::make_shared< ::rl::math::Vector >(*this->goal));

  Tree* a = &this->tree[0];
  Tree* b = &this->tree[1];

  ::rl::math::Vector chosen(this->model->getDof());


  while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
  {
    //First grow tree a and then try to connect b.
    //then swap roles: first grow tree b and connect to a.
    for (::std::size_t j = 0; j < 2; ++j)
    {
      this->j = j;
      //Sample a random configuration
      this->choose(chosen);

      //Find the nearest neighbour in the tree
      Neighbor aNearest = this->nearest(*a, chosen);

      //Do a CONNECT step from the nearest neighbour to the sample
      Vertex aConnected = this->connect(*a, aNearest, chosen);

      //If a new node was inserted tree a
      if (NULL != aConnected)
      {
        // Try a CONNECT step form the other tree to the sample
        Neighbor bNearest = this->nearest(*b, *(*a)[aConnected].q);
        Vertex bConnected = this->connect(*b, bNearest, *(*a)[aConnected].q);

        if (NULL != bConnected)
        {
          //Test if we could connect both trees with each other
          if (this->areEqual(*(*a)[aConnected].q, *(*b)[bConnected].q))
          {
            this->end[0] = &this->tree[0] == a ? aConnected : bConnected;
            this->end[1] = &this->tree[1] == b ? bConnected : aConnected;
            return true;
          }
          if((*a)[aConnected].connectSuccesful == false) {

            // if I have a collision I save the collision point and I change the sigma of the gaussian to sample closer to the obstacle
            collisionPointB = *(*a)[aConnected].q;
            this->sigma[j] = collisionSigma;

          }
        }

      }

      //Swap the roles of a and b
      using ::std::swap;
      swap(a, b);

      // swap the selected collision point
      ::rl::math::Vector temp = this->collisionPointA;
      this->collisionPointA = this->collisionPointB;
      this->collisionPointB = temp;
    }

  }

  return false;

}

