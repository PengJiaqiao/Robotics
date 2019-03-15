#include <chrono>
#include <rl/plan/SimpleModel.h>
#include "YourSampler.h"

namespace rl
{
    namespace plan
    {
	//changes the used distribution to gaussian, defines the mean
        YourSampler::YourSampler() :
            UniformSampler(),
            //randDistribution(0, 1),
	    sigma(nullptr),
	    gaussianDistribution(0, 1),
            //randEngine(::std::random_device()())
	    gaussianEngine(::std::random_device()())
        {
        }

        YourSampler::~YourSampler()
        {
        }

	//::std::uniform_real_distribution< ::rl::math::Real>::result_type
	::std::normal_distribution< ::rl::math::Real>::result_type
        //YourSampler::rand()
	YourSampler::gaussian()
        {
            //return this->randDistribution(this->randEngine);
	    return this->gaussianDistribution(this->gaussianEngine);
        }

	//here the sampler is implemented, to samples are generated, pos and new pos
        ::rl::math::Vector
        YourSampler::generateCollision()
        {
            //::rl::math::Vector rand(this->model->getDof());
	    ::rl::math::Vector gaussian(this->model->getDof());
	    
		while(1)
		{
		    ::rl::math::Vector pos = this->generate();

		    for (::std::size_t i = 0; i < this->model->getDof(); ++i)
		    {
		        //rand(i) = this->rand();
			gaussian(i) = this->gaussian();
		    }

		//return this->model->generatePositionUniform(rand);
		//return this->model->generatePositionGaussian(gaussian);

			//new sample with gaussian distribution an mean=pos
		    ::rl::math::Vector newpos = this->model->generatePositionGaussian(gaussian, pos, *this->sigma);

		    this->model->setPosition(pos);
		    this->model->updateFrames();

			//comparison to determine which samples lies in the observed and free space respectively
			//the one in the free space is kept as vertex
		    if (!this->model->isColliding())
			{
				this->model->setPosition(newpos);
				this->model->updateFrames();
			
				if(this->model->isColliding())
				{
					return pos;			
				}
			}
		    else
			{
				this->model->setPosition(newpos);
				this->model->updateFrames();
			
				if(!this->model->isColliding())
				{
					return newpos;			
				}	
			}
		}

        }


        void
        YourSampler::seed(const ::std::mt19937::result_type& value)
        {
            this->gaussianEngine.seed(value);
        }
    }
}

