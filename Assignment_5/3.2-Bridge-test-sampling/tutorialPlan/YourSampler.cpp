#include <chrono>
#include <rl/plan/SimpleModel.h>
#include "YourSampler.h"


//this samples uses a gaussian sampler as basis to generate the samples to do the bridge-test
namespace rl
{
    namespace plan
    {
        YourSampler::YourSampler() :
            GaussianSampler(),
            ratio(static_cast< ::rl::math::Real>(5) / static_cast< ::rl::math::Real>(6))
        {
        }

        YourSampler::~YourSampler()
        {
        }

        ::rl::math::Vector
        YourSampler::generateCollision()
        {
		if(this->rand() > this->ratio)
		{
			return this->generate();
		}
		else
		{

		    ::rl::math::Vector gauss(this->model->getDof());
		    ::rl::math::Vector pos(this->model->getDof());
		    
			while(1)
			{
				//here three samples are generated pos, newpos and newpos2
			    ::rl::math::Vector newpos = this->generate();
			    this->model->setPosition(newpos);
			    this->model->updateFrames();

			    if(this->model->isColliding()) //checks if the sample lies in the observed space
			    {

				    for (::std::size_t i = 0; i < this->model->getDof(); ++i)
				    {
					gauss(i) = this->gauss();
				    }

				    ::rl::math::Vector newpos2 = this->model->generatePositionGaussian(gauss, newpos, *this->sigma);

				    if (this->model->isColliding()) //check if the samples lie in the observed space, if yes, then 										interpolates to create a point in the middle, then checks if it 									lies in the free space, if yes, it is then kept as a vertex
					{
						this->model->interpolate(newpos, newpos2, static_cast< ::rl::math::Real>(0.5), pos);

						this->model->setPosition(pos);
						this->model->updateFrames();
			
						if(!this->model->isColliding())
						{
							return pos;			
						}
					}
				}
				    
			}

		}
        }
    }
}

