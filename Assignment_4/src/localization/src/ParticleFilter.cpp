#include "localization/ParticleFilter.h"
#include "localization/Util.h"

#include "tf/tf.h"

using namespace std;

ParticleFilter::ParticleFilter(int numberOfParticles) {
	this->numberOfParticles = numberOfParticles;

	// initialize particles
	for (int i = 0; i < numberOfParticles; i++) {
		this->particleSet.push_back(new Particle());
	}

	// this variable holds the estimated robot pose
	this->bestHypothesis = new Particle();

	// at each correction step of the filter only the laserSkip-th beam of a scan should be integrated
	this->laserSkip = 5;

	// distance map used for computing the likelihood field
	this->distMap = NULL;
}

ParticleFilter::~ParticleFilter() {
	// delete particles
	for (int i = 0; i < numberOfParticles; i++) {
		Particle* p = this->particleSet[i];
		delete p;
	}

	this->particleSet.clear();

	if (this->likelihoodField)
		delete[] this->likelihoodField;

	delete this->bestHypothesis;

	if (this->distMap)
		delete[] this->distMap;
}

int ParticleFilter::getNumberOfParticles() {
	return this->numberOfParticles;
}

std::vector<Particle*>* ParticleFilter::getParticleSet() {
	return &(this->particleSet);
}

void ParticleFilter::initParticlesUniform() {
    //get map properties
    int mapWidth, mapHeight;
    double mapResolution;
    this->getLikelihoodField(mapWidth, mapHeight,mapResolution);

	// TODO: here comes your code
	for (int i = 0; i < getNumberOfParticles(); ++i)
	{
		Particle *psu = this->particleSet[i];
		//Randomly adding x,y,theta values to particles which are spread across the map uniformly
		psu->x = Util::uniformRandom(0,mapWidth*mapResolution);
		psu->y = Util::uniformRandom(0,mapHeight*mapResolution);
		psu->theta = Util::uniformRandom(-M_PI, M_PI);
	}
}

void ParticleFilter::initParticlesGaussian(double mean_x, double mean_y,
		double mean_theta, double std_xx, double std_yy, double std_tt) {

	// TODO: here comes your code
	for (int i = 0; i < getNumberOfParticles(); ++i)
	{
		Particle *psg = this->particleSet[i];
		//Randomly adding x,y,theta values to particles which are spread as a gaussian at a given point on the map
		psg->x = Util::gaussianRandom(mean_x,std_xx);
		psg->y = Util::gaussianRandom(mean_y,std_yy);
		psg->theta = Util::gaussianRandom(mean_theta,std_tt);
	}
}

/**
 *  Initializes the likelihood field as our sensor model.
 */
void ParticleFilter::setMeasurementModelLikelihoodField(
		const nav_msgs::OccupancyGrid& map, double zRand, double sigmaHit) {
	ROS_INFO("Creating likelihood field for laser range finder...");

	// create the likelihood field - with the same discretization as the occupancy grid map
	this->likelihoodField = new double[map.info.height * map.info.width];
	this->likelihoodFieldWidth = map.info.width;
	this->likelihoodFieldHeight = map.info.height;
	this->likelihoodFieldResolution = map.info.resolution;

    // calculates the distance map and stores it in member variable 'distMap'
	// for every map position it contains the distance to the nearest occupied cell.
	calculateDistanceMap(map);

    // Here you have to create your likelihood field
	// HINT0: sigmaHit is given in meters. You have to take into account the resolution of the likelihood field to apply it.
	// HINT1: You will need the distance map computed 3 lines above
	// HINT2: You can visualize it in the map_view when clicking on "show likelihood field" and "publish all".
	// HINT3: Storing probabilities in each cell between 0.0 and 1.0 might lead to round-off errors, therefore it is
	// good practice to convert the probabilities into log-space, i.e. storing log(p(x,y)) in each cell. As a further
	// advantage you can simply add the log-values in your sensor model, when you weigh each particle according the
	// scan, instead of multiplying the probabilities, because: log(a*b) = log(a)+log(b).

	// TODO: here comes your code

	//Calculation of likelehodd probability P = Zhit*Phit+Zrand*Prand
	//where: Zhit = 1-Zrand, Phit inferred from map, Prand=1
	//According to HIN3, we save the probabilities in the likelihooddfield in LOG()
	double Phit = 0.0;

	for (int i = 0; i < map.info.height * map.info.width ; ++i)
	{
		Phit = Util::gaussian(this->distMap[i],(double)sigmaHit/map.info.resolution,0.0);
		this->likelihoodField[i] = log((1.0-zRand)*Phit + zRand);
	}
	
	//ROS_INFO("...DONE creating likelihood field!");
}

void ParticleFilter::calculateDistanceMap(const nav_msgs::OccupancyGrid& map) {
	// calculate distance map = distance to nearest occupied cell
	distMap = new double[likelihoodFieldWidth * likelihoodFieldHeight];
	int occupiedCellProbability = 90;
	// initialize with max distances
	for (int x = 0; x < likelihoodFieldWidth; x++) {
		for (int y = 0; y < likelihoodFieldHeight; y++) {
			distMap[x + y * likelihoodFieldWidth] = 32000.0;
		}
	}
	// set occupied cells next to unoccupied space to zero
	for (int x = 0; x < map.info.width; x++) {
		for (int y = 0; y < map.info.height; y++) {
			if (map.data[x + y * map.info.width] >= occupiedCellProbability) {
				bool border = false;
				for (int i = -1; i <= 1; i++) {
					for (int j = -1; j <= 1; j++) {
						if (!border && x + i >= 0 && y + j >= 0 && x + i
								< likelihoodFieldWidth && y + j
								< likelihoodFieldHeight && (i != 0 || j != 0)) {
							if (map.data[x + i + (y + j) * likelihoodFieldWidth]
									< occupiedCellProbability && map.data[x + i
									+ (y + j) * likelihoodFieldWidth] >= 0)
								border = true;
						}
						if (border)
							distMap[x + i + (y + j) * likelihoodFieldWidth]
									= 0.0;
					}
				}
			}
		}
	}
	// first pass -> SOUTHEAST
	for (int x = 0; x < likelihoodFieldWidth; x++)
		for (int y = 0; y < likelihoodFieldHeight; y++)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}

	// second pass -> NORTHWEST
	for (int x = likelihoodFieldWidth - 1; x >= 0; x--)
		for (int y = likelihoodFieldHeight - 1; y >= 0; y--)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}
}

double* ParticleFilter::getLikelihoodField(int& width, int& height,
		double& resolution) {
	width = this->likelihoodFieldWidth;
	height = this->likelihoodFieldHeight;
	resolution = this->likelihoodFieldResolution;

	return this->likelihoodField;
}

/**
 *  A generic measurement integration method that invokes some specific observation model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::measurementModel(
		const sensor_msgs::LaserScanConstPtr& laserScan) {
	likelihoodFieldRangeFinderModel(laserScan);
}

/**
 *  Method that implements the endpoint model for range finders.
 *  It uses a precomputed likelihood field to weigh the particles according to the scan and the map.
 */
void ParticleFilter::likelihoodFieldRangeFinderModel(
		const sensor_msgs::LaserScanConstPtr & laserScan) {
    
    

	// TODO: here comes your code
	double best_weight = 0.0; //variable to save the maximal weight which will lead to the best hypothesis
	double best_index = 0.0; // variable to save index of max weight to get best hypothesis
	double weight_summa = 0.0; //variable to add all weights for normalization

	// FOR cycle to iterate through all particles
	for (int i = 0; i < this->numberOfParticles; ++i)
	{
		//Particle *psRF = this->particleSet[i];
		//psRF->weight=0;
		double weight = 0.0; //initialization of the weight for each particle

		//FOR cycle to iterate through every 5th (=laserskip) laserscan measurements for a given particle
		for (int j = 0; j < (int)laserScan->ranges.size()/this->laserSkip; j++)
		{
            
            int idx = j*this->laserSkip;
            
			//IF the laserscan is in valid range
			if( (laserScan->range_min < laserScan->ranges[idx]) && (laserScan->ranges[idx] < laserScan->range_max) ) //if measured laser is in valid range
			{
				//Calculation of the current measurement laserbeam angle
				double calc_angle = Util::normalizeTheta(this->particleSet[i]->theta+laserScan->angle_min + idx*laserScan->angle_increment); //Util::normalizeTheta(particleSet[i]->theta + laserScan->angle_min + laserScan->angle_increment*j);
				//Calculation of the x,y coordinates for the endpoint of the laserbeam
				int x_laserbeam_end = (particleSet[i]->x + laserScan->ranges[idx]*cos(calc_angle))/this->likelihoodFieldResolution;
				int y_laserbeam_end = (particleSet[i]->y + laserScan->ranges[idx]*sin(calc_angle))/this->likelihoodFieldResolution;
				
				//Computing the index in the map depending on x and y coordinates
				int number = ParticleFilter::computeMapIndex(this->likelihoodFieldWidth, this->likelihoodFieldHeight, x_laserbeam_end, y_laserbeam_end);
				
				//Checking wheather the number is not in the likelihoodField
				//This is actually a double-check if the measured beam is outside of the field or not
				if ((number<0) || (number >= this->likelihoodFieldHeight*this->likelihoodFieldWidth))
				{
					// IF the measurement gives an invalid value, we add a low correction value to the weight
					//double correction = 0.000001;
					//Since the weights are saved in LOG, correction has to be LOG() as well
					weight =  weight/10;
				}
				else //Normal mode, if the measurement number is in likelihoddField
				{
					//we add the likelihoodField value to the weight
					weight = weight+this->likelihoodField[number];
				}		
			}
			else
			{
				//do nothing, laserscan is not in valid range
			}
		}

		//convert the LOG likelihoodField weights back with EXP
		this->particleSet[i]->weight = exp(weight);

		//SUM of all weights for normalization
		weight_summa = weight_summa + this->particleSet[i]->weight;	
	}

	//Normalization of particle weights depenging on the sum of all weights
	for (int n = 0; n < this->numberOfParticles; ++n)
	{
		if(weight_summa != 0)
		{
			this->particleSet[n]->weight = this->particleSet[n]->weight / weight_summa;
		}
		else
		{
			this->particleSet[n]->weight = 0.0;
		}

		//Check the best hypothesis according to weight
		//IF the n-th particle weight is bigger than the best_weight until now, thean set this weight to best_weight
		if (this->particleSet[n]->weight > best_weight)
		{
			best_index = n;
			best_weight = particleSet[n]->weight; // set current weight to best_weight to get MAX
		}
	}
	//set the particle with the highest weight as the bestHypothesis
	//this->bestHypothesis = particleSet[best_index];		
    

}

void ParticleFilter::setMotionModelOdometry(double alpha1, double alpha2,
		double alpha3, double alpha4) {
	this->odomAlpha1 = alpha1;
	this->odomAlpha2 = alpha2;
	this->odomAlpha3 = alpha3;
	this->odomAlpha4 = alpha4;

}

/**
 *  A generic motion integration method that invokes some specific motion model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::sampleMotionModel(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	sampleMotionModelOdometry(oldX, oldY, oldTheta, newX, newY, newTheta);
}

/**
 *  Method that implements the odometry-based motion model.
 */
void ParticleFilter::sampleMotionModelOdometry(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	// TODO: here comes your code
	double d_trans, d_rot1, d_rot2;
	double d_trans_prime, d_rot1_prime, d_rot2_prime;

	for (int i = 0; i < getNumberOfParticles(); ++i)
	{
		//Calculation odometry information for rotation1, rotation2 and translation fo motion
		d_trans = sqrt( pow(newX-oldX,2) + pow(newY-oldY,2) );
		d_rot1 = Util::diffAngle(oldTheta,atan2( newY-oldY, newX-oldX ));
		d_rot2 = Util::diffAngle(d_rot1,Util::diffAngle(oldTheta, newTheta));
		//Adding gaussion noise randomly to calculated values
		d_rot1_prime = d_rot1 + (Util::gaussianRandom(0,this->odomAlpha1*abs(d_rot1) + this->odomAlpha2*abs(d_trans)));
		d_trans_prime = d_trans + (Util::gaussianRandom(0,this->odomAlpha3*abs(d_trans) + this->odomAlpha4*(abs(Util::normalizeTheta(d_rot1+d_rot2)))));
		d_rot2_prime = d_rot2 + (Util::gaussianRandom(0,this->odomAlpha1*abs(d_rot2) + this->odomAlpha2*abs(d_trans)));
		//Updating next motion step with giving x,y location and theta orientation
		this->particleSet[i]->x = this->particleSet[i]->x + d_trans_prime * cos(Util::normalizeTheta(this->particleSet[i]->theta+d_rot1_prime));
		this->particleSet[i]->y = this->particleSet[i]->y + d_trans_prime * sin(Util::normalizeTheta(this->particleSet[i]->theta+d_rot1_prime));
		this->particleSet[i]->theta = Util::normalizeTheta(Util::normalizeTheta(this->particleSet[i]->theta + d_rot1_prime + d_rot2_prime));
	}
	
}

/**
 *  The stochastic importance resampling.
 */
void ParticleFilter::resample() {
	
    // based on the algorithm low variance sampler
    // page 110 of book
    /*
	int M = this->getNumberOfParticles();

    
    std::vector<Particle*> X;
    
    double r = Util::uniformRandom(0.0,(double)1.0/M);
    
    double c = this->particleSet[0]->weight;
    
    int i = 0;
    double U = 0;
    
    for (int m = 0; m < M; m++) {
        U = r + m * (1.0/((double)M));
        
        while ( U > c) {
            i++;
            c = c + this->particleSet[i]->weight;
            
        }
        
        this->particleSet[i]->weight = 1.0/((double)M);
        X.push_back(this->particleSet[i] );
       
    }
    
    
    this->particleSet = X;
    */
	// TODO: here comes your code

	int M = this->getNumberOfParticles(); 
	double r = Util::uniformRandom(0.0,(double)1.0/M); //random "angle" or increment for roulett wheel
	double c = this->particleSet[0]->weight; //weight of first particle will be first threshold
	int i = 0;
	std::vector<Particle*> resampleSet = this->particleSet; //Cloning original particleset to resampleSet to save values

	for (int m = 0; m < M; m++)
	{
		double U = r + ((double)m / M); // U is the "pointer" of roulette wheel with increment 0,1,2...m / numberOfparticles 
		while (U > c) //while "pointer" is above weight threshold, we should increment by adding the weights
		{
			i++;
			c += resampleSet[i]->weight;
		}
		//now we arrived in a "weight-field" above the threshold
		//we set the real particle set values according to the current particle values of the "weight-field"
		this->particleSet[m]->x = resampleSet[i]->x;
		this->particleSet[m]->y = resampleSet[i]->y;
		this->particleSet[m]->theta = resampleSet[i]->theta;
		this->particleSet[m]->weight = 1.0/M; //weight is set to 1/M because it's normalised between 0..1 depending on numberOfparticles
	}
    
}

Particle* ParticleFilter::getBestHypothesis() {
    
    int N = this->getNumberOfParticles();
    
	// search the index of the largest weight
    
    int index = 0;	
	double max_weight = this->particleSet[0]->weight;
        
	for( int k = 1; k < N; k++ ) {
		if(this->particleSet[k]->weight > max_weight){
			max_weight = this->particleSet[k]->weight;
			index = k;
		}
	}

	this->bestHypothesis = this->particleSet[index];

	return this->bestHypothesis;
}

// added for convenience
int ParticleFilter::computeMapIndex(int width, int height, int x,
		int y) {
	return x + y * width;
}

