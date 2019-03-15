#include "YourPlanner.h"
#include <rl/plan/Sampler.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/Verifier.h>
#include <rl/plan/Viewer.h>
#include <boost/make_shared.hpp>
#include <iostream>

YourPlanner::YourPlanner() :
  RrtConConBase()
{
}

YourPlanner::~YourPlanner()
{
}

::std::string
YourPlanner::getName() const
{
  return "Exhausted_Node_Extension_Option_2";
}

void
YourPlanner::choose(::rl::math::Vector& chosen)
{
  //your modifications here
  RrtConConBase::choose(chosen);
}

RrtConConBase::Vertex 
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{

  //your modifications here
  return RrtConConBase::connect(tree, nearest, chosen);
}

RrtConConBase::Vertex 
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{

  ::rl::math::Real distance = nearest.second;
  ::rl::math::Real step = (::std::min)(distance, this->delta);

  ::rl::plan::VectorPtr next = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());

  this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *next);

  this->model->setPosition(*next);
  this->model->updateFrames();

  if (!this->model->isColliding())
  {
    Vertex extended = this->addVertex(tree, next);
    this->addEdge(nearest.first, extended, tree);

    return extended;
  }

  return NULL;

  //your modifications here
  //return RrtConConBase::extend(tree, nearest, chosen);
}

RrtConConBase::Neighbor
YourPlanner::nearest(const Tree& tree, const ::rl::math::Vector& chosen)
{

  // // ############## Option 1 for exhausted Node extension ##############
  // // calculate penalty based on failed_extension_counter and add it to normalized distance

  // // create variables for n_max, n_min, p_max, p_min
  // int n_max = ::std::numeric_limits<int>::min();
  // int n_min = ::std::numeric_limits<int>::max();
  // ::rl::math::Real d_max = ::std::numeric_limits< ::rl::math::Real >::min();
  // ::rl::math::Real d_min = ::std::numeric_limits< ::rl::math::Real >::max();
  // ::rl::math::Real h_min = ::std::numeric_limits< ::rl::math::Real >::max();

  //  // //create an empty pair <Vertex, distance> to return
  // Neighbor p(Vertex(), (::std::numeric_limits< ::rl::math::Real >::max)());


  // //Iterate through all vertices to find out above defined values
  // for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
  // {
  //   // try optionally save current distance in tree, to reuse in 2nd loop
  //   ::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*i.first].q);
  //   // save distance in tree, so it does not need to be calculated in the second loop
  //   tree[*i.first].current_distance = d;

  //   int n = tree[*i.first].failed_extension_counter;

  //   if (d < d_min)
  //   {
  //     d_min = d;
  //   }
  //   if (d > d_max)
  //   {
  //     d_max = d;
  //   }
  //   if (n < n_min)
  //   {
  //     n_min = n;
  //   }
  //   if (n > n_max)
  //   {
  //     n_max = n;
  //   }  
  // }


  // // //Iterate through all vertices to calculate history based weighting 
  // for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
  // {
  //   ::rl::math::Real d = tree[*i.first].current_distance;
  //   std::cout << "d: " << d << std::endl;
  //   int n = tree[*i.first].failed_extension_counter;

  //   // calculate normalized distance with penalty term for nodes with high extension counters
  //   // ToDo fix division by zero exception, jetzt behelfsmäßig mit +1 gefixt
  //   ::rl::math::Real h =  (d - d_min) / (d_max - d_min + 1) + (n - n_min) / (n_max - n_min + 1);

  //   if (h < h_min) {
  //     h_min = h;
  //     p.first = *i.first;
  //     p.second = d;
  //   }

  // }

  // ############## Option 2 for exhausted Node extension ##############
  // stop considering nodes that exceeded threshold of failed_extension_counter

    //create an empty pair <Vertex, distance> to return
  Neighbor p(Vertex(), (::std::numeric_limits< ::rl::math::Real >::max)());

  //Iterate through all vertices to find the nearest neighbour
  for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
  {

    int n = tree[*i.first].failed_extension_counter;
    // std::cout << "n: " << n << std::endl;
    if (n > this->threshold) {
      continue;
    }

    ::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*i.first].q);
    if (d < p.second)
    {
      p.first = *i.first;
      p.second = d;
    }
  }


  // Compute the square root of distance
  p.second = this->model->inverseOfTransformedDistance(p.second);

  return p;
}

bool
YourPlanner::solve()
{
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
      //Sample a random configuration
      this->choose(chosen);

      //Find the nearest neighbour in the tree
      Neighbor aNearest = this->nearest(*a, chosen);

      //Do a CONNECT step from the nearest neighbour to the sample
      Vertex aConnected = this->connect(*a, aNearest, chosen);

      //If a new node was inserted tree a
      if (NULL != aConnected)
      {
        // add 1 to failed extension counter
        // at the added node it was not possible to extend further
        (*a)[aNearest.first].failed_extension_counter += 1;

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
        }
      } else {
        //  tree could not be extended --> update 'extension failed counter'
        (*a)[aNearest.first].failed_extension_counter += 1;
    }

      //Swap the roles of a and b
      using ::std::swap;
      swap(a, b);
    } 


  }

  return false;

}

