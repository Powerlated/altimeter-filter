#ifndef UKF_H
#define UKF_H

#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdio.h>

#define Mass 20
#define Area 0.01900948
#define Grav -9.81
#define HighAccNoise 0.05
#define LowAccNoise 0.0001
#define gyroNoise 0.00001
#define GPSNoise 1
#define stateSize 21
#define measurementSize 12
#define augmentedSize 54
#define sigmaPointSize 109
#define IDX(sig, var) ((sig) * augmentedSize + (var))

//initializes all matrixes, vectors, and processes the initial state vector
/*
We initialize the initial state estimate, X^0. For this, 
our state is Position, velocity and accelerationXYZ in ENU coordinates as well as modified rodreguez paramaters for orientation, angular 
velocityXYZ, and biases for both the gyroscope and accelerometer. 
Our measurement vector is [accelXYZ, gyroXYZ, magXYZ, GPSXYZ]
We assume position is [0, 0, distance from COM], velocity is [0, 0, 0], 
and theta uses the magnometer to make a rough estimate that is converted to ENU coords.

We then initialize a section of the covariance matrix P0 that represents our uncertainty in our initial guess.. very uncertain
It is a diagonal matrix, with the diagonals representing uncertainty in the initial state guess
we would use the following uncertainty values as an example: diagonal[1, 1, 1, .1, .1, .1, 5, 5, 5]

Now we initialize the augmented state vector X^a0, which contains the original state vector, a vector the same size of 
process noise, and a vector the same size of the measurement vector of measurement noise.
It looks like [State vector, Process Noise(Same size as state vector), Measurment Noise(same size as measurement vector)

Then we combine that P0 we created earlier, along with Q and R. These represent the process noise and measurement noise covariance,
or essentially a way to take our uncertainty in both our predict and update function, and make it a range of confidence.
P0a = [P0, 0, 0; 0, Q, 0; 0, 0, R]

Finally, we calculate an initial set of sigma points based on that distribution and then move onto the looping process.
A sigma point is an entire augmented state vector, and we use 2*augmented state size + 1 sigma points in total.
Each has a slightly different distance from the true mean. 
We take 1, and set it to the true augmented state vector. 
We then take the square root of (a constant gamma * our initial covariance matrix P0a) and save it as a temporary term.
Then we take half of the remaining points and set them to the sum of the augmented state vector and a column of that term.
Repeat this with the rest of the points, but subtract that column of the term now. 
This is essentially the hard coded initial version of the calculateSigmaPoints function below.
Finally, we initialize all other variables needed for the stepUKF function.
*/
void initializeUKF(float *gamma, float *meanWeight, float *covarianceWeight, float *otherWeight, float* X0ahat, float* P0a, float* sigmaPoints);

//calculates new sigma points before every predict function.
void calculateSigmaPoints(float gamma, float* covarianceMatrix, float* sigmaPointMatrix, float* augStateVector);

//we can decide whether to run the predict and update at the same time, everytime, or
//we can update the predict faster than the sensor poll rate, and just update it when we get new data.
//predicts the state based on the last sensor data
/*
We take the created sigma points, and pass them through the process function. This essentially takes the 
last augmented state, and predicts the next state only using the previous augmented state's information.

Then, we take a weighted average of the sigma points using the mean weight we initialized in the initializeUFf function.
This is the predicted state mean.

We then take the sum of the sigma point weights times the skew matrix of 
the difference between the predicted state mean and sigma points. 
This gives us our predicted covariance.

Next, we pass those predicted sigma points throuhg the measurement function. 
This function is also where we model the imu being a distance away from the center of mass.

Finally, we take each sigma point, multiply it by its weight, and take their mean to get the predicted measurement mean.

SO this gives us five importatnt things: The Predicted state mean, the predicted state covariance, 
the sigma points that have been propogated through the process model and also the measurement model, 
and the predicted measurement mean. We need all of these for the update portion of the ukf.
*/
void ukfPredict(float* sigmaPoints, float* processedSigmaPoints, float timestep, float* predictedStateMean, float* predictedMeasurementMean, float meanWeight, float otherWeight, float covarianceWeight, float* measuredSigmaPoints, float* predictedCovariance);

//updates the state when new data is recieved
/*
This takes the newest sensor data, whether with or without the new GPS data and 
updates the state vector, covariance, and kalman gain.

We start by taking 2 large matrixes. The first matrix uses the sigma points 
that were passed through the maeasurement function and the predicted measurement mean.
It is made of the sum of the skew of the difference between each sigma point 
passed through the measurement function and the predicted measurement mean.

The next matrix uses the sigma points passed through the process model, the predicted state 
mean, the sigma points passed through the measurement function, 
and the predicted measurement mean. It takes the sum for each sigma points of the 
following vectors multiplied together. Vector one is the difference between the 
sigma points passed through the process function and the predicted state mean. Vector 
2 is the difference between the sigma points passed through the measurment function 
and the predicted measurement mean, but tranposed. These two vectors are multiplied together to make a matrix, 
which is multiplied by the covariance weight. These matrixes are all summed.

Next the kalman gain is calcualted, which is the first matrix X the inverse of the second matrix.
The augmented state vector updated using the sum of the predicted augmented state vector + kalman gain * the difference 
between the predicted measurement mean and new measurement vector. 

Finally, the covariance is updated by taking the difference between the predicted
covariance matrix and the kalman gain * the first matrix * the tranpose of the kalman gain.
*/ 
void ukfUpdate(float* newState, float* newCovariance, float* newSensorVector, float covarianceWeight, float otherWeight, float* predictedStateMean, float* processedSigmaPoints, float* predictedMeasurementMean, float* measuredSigmaPoints, float* predictedCovariance);

//process(predict) function
/*
We model this with a euler integration. We assume velocity and position are affected by the acceleration and use a euler step.
We assume acceleration and angular velocity are constant, and the biases use a random walk.
We propogate the modified rodreguez parameters using a special kinematic 
differential equation that is in the propogateMRP function.
Finally, we add in the process noise to catch anything we didn't model.
*/
void process(float* inputSigmaPoints, float* outputSigmaPoints, float timestep);

//measurement(update) function
/*
This function takes the predicted state, and creates the correct measurement vector 
assuming the predicted state was correct. Our model is pretty simple. 
We calculate the accelerometer data using the acceleration and the acceleromter bias.
We calcualte the gyro data using the angular velocity and the gyroscope bias.
We calculate the magnometer data using the modified rodreguez parameters and angular velocity.
We calculate the GPS data using the position data.
We then add in the measurement nosie.
Keep in mind all of this is adjusted for being a vector r away from the center of mass.
Then we add in all of the measurement noise that we store to each variable.
*/
void measurement(float* inputSigmaPoints, float* outputSigmaPoints);

//creates diagonal matrix
void diag(int size, float* outMatrix, float* diagonals);

//multipies a matrix and a constant
void scaleMatrix(int size, float* inputMatrix, float scalar, float* outputMatrix);

//takes inverse of specifically a 21x21 matrix
void inverse(float* in, float* out);

//takes lower triangular, or basically square root of a matrix
void chol(int size, float* inputMatrix, float* outputMatrix);

//adds two vectors
void addVector(int size, float* inputA, float* inputB, float* output);

//scales a vector
void scaleVector(int size, float* inputVector, float scalar, float* outputVector);

//takes a 3x1 cross product
void crossProduct(float* inputA, float* inputB, float* output);

//makes gaussian noise
float gaussian_noise(float mean, float stddev);

//random walk for bias
float biasRandomWalk(float sigma, float timestep);

//Box Muller
static float gaussian_unit(void);

//seed
static void seed_rng_once(void);

//adds 2 matrixes
void addMatrix(int size,float* inputMatrix, float* addedMatrix, float* outputMatrix);

//takes the nxn skew matrix of a vector
void skew(int size, float* inputVector, float* outputMatrix);

//multiplies 2 matrixes
void multiplyMatrix(int rowASize, int colASize, int colBSize, float* inputMatrixA, float* inputMatrixB, float* outputMatrix);

//takes the dot product of 2 vectors
void dotProduct(int size, float* inputVectorA, float* intputVectorB, float *outputScalar);

//calculates the DCM from the modified rodreguez parameters
void mrp2dcm(float* inputMRP, float* outputMRP);

//calculates the tranpose of a matrix
void transpose(int sizeRow, int sizeCol, float* inputMat, float* outputMat);

#endif