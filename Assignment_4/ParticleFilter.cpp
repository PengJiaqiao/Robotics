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

	double Phit;

	for (int i = 0; i < map.info.height * map.info.width ; ++i)
	{
		Phit = Util::gaussian(distMap[i],sigmaHit/this->likelihoodFieldResolution,0);
		this->likelihoodField[i] = log((1-zRand)*Phit + zRand);
	}
	

	ROS_INFO("...DONE creating likelihood field!");
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
	/*
	for (int i = 0; i < this->numberOfParticles; ++i)
	{
		Particle *psRF = this->particleSet[i];
		psRF->weight=1;
		for (int j = 0; j < laserScan->ranges.size(); j+=this->laserSkip)
		{
			  
			double x_laserbeam_end= psRF->x + laserScan->ranges[j]/this->likelihoodFieldResolution * cos (psRF->theta + laserScan->angle_min + laserScan->angle_increment*j);
			double y_laserbeam_end = psRF->y + laserScan->ranges[j]/this->likelihoodFieldResolution * sin (psRF->theta + laserScan->angle_min + laserScan->angle_increment*j);
			int number= y_laserbeam_end*this->likelihoodFieldWidth+x_laserbeam_end;
			psRF->weight=psRF->weight*this->likelihoodField[number];
		}
	}
	*/
	

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
		d_trans = sqrt( pow(newX-oldX,2) + pow(newY-oldY,2) );
		d_rot1 = Util::diffAngle(atan2( newY-oldY, newX-oldX ), oldTheta);
		d_rot2 = Util::diffAngle(newTheta, Util::diffAngle(oldTheta, -d_rot1));

		d_rot1_prime = d_rot1 + (Util::gaussianRandom(0,odomAlpha1*abs(d_rot1) + Util::gaussianRandom(d_trans,odomAlpha2)));
		d_trans_prime = d_trans + (Util::gaussianRandom(0,odomAlpha2*d_trans) + Util::gaussianRandom(0,odomAlpha4*(abs(d_rot1)+abs(d_rot2))));
		d_rot2_prime = d_rot2 + (Util::gaussianRandom(0,odomAlpha1*abs(d_rot2) + Util::gaussianRandom(0,odomAlpha2*d_trans)));

		Particle *psMMO = this->particleSet[i];
		
		psMMO->x = psMMO->x + d_trans_prime * cos(psMMO->theta+d_rot1_prime);
		psMMO->y = psMMO->y + d_trans_prime * sin(psMMO->theta+d_rot1_prime);
		psMMO->theta = Util::normalizeTheta(psMMO->theta + d_rot1_prime + d_rot2_prime);
	}
	
}

/**
 *  The stochastic importance resampling.
 */
void ParticleFilter::resample() {
	// TODO: here comes your code
}

Particle* ParticleFilter::getBestHypothesis() {
	return this->bestHypothesis;
}

// added for convenience
int ParticleFilter::computeMapIndex(int width, int height, int x,
		int y) {
	return x + y * width;
}

