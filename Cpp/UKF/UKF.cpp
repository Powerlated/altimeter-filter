#include "UKF.h"
#include "float.h"



static int rng_seeded = 0;
void initializeUKF(float *gamma, float *meanWeight, float *covarianceWeight, float *otherWeight, float* X0ahat, float* P0a, float* sigmaPoints) {
    //Weights
    float alpha = 1;
    float beta = 2.0;
    float kappa = 0.0;
    float lambda = alpha*alpha*(augmentedSize + kappa) - augmentedSize;
    *gamma = sqrt(augmentedSize + lambda);

    *meanWeight = lambda/(augmentedSize + lambda);
    *covarianceWeight = lambda/(augmentedSize + lambda) + 1 - alpha*alpha + beta;
    *otherWeight = 1/(2*(augmentedSize+lambda));

    //initializing X^0
    float X0hat[stateSize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    //Initializing that section of the covariance matrix P0
    float P0[stateSize*stateSize] = {0.0};
    float temp[stateSize] = {.1, .1, .1, .05, .05, .05, .2, .2, .2, .01, .01, .01, .01, .01, .01, .001, .001, .001, .001, .001, .001};
    diag(stateSize, P0, temp);

    //Initializing the augmented state vector X^a0
    for (int a = 0; a < stateSize; a++) {
        X0ahat[a] = X0hat[a];
    }

    //Creating the full initial covariance matrix
    float temp2[augmentedSize] = {.1,   .1,   .1,   .05,  .05,  .05,  .2,   .2,   .2,   .01,  .01,  .01,  .01,  .01,  .01,  .001, .001, .001, .001, .001, .001, 
                                  .001, .001, .001, .001, .001, .001, .001, .001, .001, .001, .001, .001, .001, .001, .001, .001, .001, .001, .001, .001, .001, 
                                   1e-6f, 1e-6f, 1e-6f, 6e-4f, 6e-4f, 6e-4f, 0.1f*0.1f, 0.1f*0.1f, 0.1f*0.1f, 25.0f, 25.0f, 25.0f};
    diag(augmentedSize, P0a, temp2);

    //Creating initial sigma points. They are stored in row major format!!!
    //multiplying gamma times the square root of the initial covariance
    float sqrtP0a[augmentedSize*augmentedSize] = {0.0};
    chol(augmentedSize, P0a, sqrtP0a);
    scaleMatrix((augmentedSize*augmentedSize), sqrtP0a, *gamma, sqrtP0a);

    //initializing the sigma points
    //setting last sigma points to initial augmented state vector
    for (int b = 0; b < augmentedSize; b++) {
        sigmaPoints[IDX(0, b)] = X0ahat[b];
    }

    //setting the rest to the initial augmented state vector + or - the column of the sqrt(gamma*initial covariance).
    for (int c = 0; c < (sigmaPointSize-1)/2; c++) {
        for (int d = 0; d < augmentedSize; d++) {
            sigmaPoints[IDX(c + 1, d)] = X0ahat[d] + sqrtP0a[((d) * augmentedSize + (c))];
            sigmaPoints[IDX((c + 1 + (sigmaPointSize-1)/2), d)] = X0ahat[d] - sqrtP0a[((d) * augmentedSize + (c))];
        }
    }
}

//function that calculates the sigma points given gamma, a previous covariance and augmented state vector, and 
//the already initialized sigmaPointMatrix and sqrtCovarianceMatrix
void calculateSigmaPoints(float gamma, float* covarianceMatrix, float* sigmaPointMatrix, float* augStateVector) {
    //calculates the term that we found earlier, sqrt(gamma * covariance)
    float sqrtCovarianceMatrix[augmentedSize*augmentedSize] = {0.0};
    chol(augmentedSize, covarianceMatrix, sqrtCovarianceMatrix);
    scaleMatrix((augmentedSize*augmentedSize), sqrtCovarianceMatrix, gamma, sqrtCovarianceMatrix);
    //assigns first sigma point to augmented state vector
    for (int b = 0; b < augmentedSize; b++) {
        sigmaPointMatrix[IDX(0, b)] = augStateVector[b];
    }

    //setting the rest to the initial augmented state vector + or - the column of the sqrt(gamma*initial covariance).
    for (int c = 0; c < (sigmaPointSize-1)/2; c++) {
        for (int d = 0; d < augmentedSize; d++) {
            sigmaPointMatrix[IDX(c + 1, d)] = augStateVector[d] + sqrtCovarianceMatrix[((d) * augmentedSize + (c))];
            sigmaPointMatrix[IDX((c + 1 + (sigmaPointSize-1)/2), d)] = augStateVector[d] - sqrtCovarianceMatrix[((d) * augmentedSize + (c))];
        }
    }
}

//integrates the sigma points through the next time step.
void process(float* inputSigmaPoints, float* outputSigmaPoints, float timestep) {
    for (int a = 0; a < sigmaPointSize; a++) {
        for (int i = 0; i < augmentedSize; i++) {
            outputSigmaPoints[IDX(a, i)] = inputSigmaPoints[IDX(a, i)];
        }
        
        // Position
        outputSigmaPoints[IDX(a, 0)] = inputSigmaPoints[IDX(a, 0)] + inputSigmaPoints[IDX(a, 3)] * timestep + 0.5f * inputSigmaPoints[IDX(a, 6)] * powf(timestep, 2);
        outputSigmaPoints[IDX(a, 1)] = inputSigmaPoints[IDX(a, 1)] + inputSigmaPoints[IDX(a, 4)] * timestep + 0.5f * inputSigmaPoints[IDX(a, 7)] * powf(timestep, 2);
        outputSigmaPoints[IDX(a, 2)] = inputSigmaPoints[IDX(a, 2)] + inputSigmaPoints[IDX(a, 5)] * timestep + 0.5f * inputSigmaPoints[IDX(a, 8)] * powf(timestep, 2);
      
        // Velocity
        outputSigmaPoints[IDX(a, 3)] = inputSigmaPoints[IDX(a, 3)] + inputSigmaPoints[IDX(a, 6)] * timestep;
        outputSigmaPoints[IDX(a, 4)] = inputSigmaPoints[IDX(a, 4)] + inputSigmaPoints[IDX(a, 7)] * timestep;
        outputSigmaPoints[IDX(a, 5)] = inputSigmaPoints[IDX(a, 5)] + inputSigmaPoints[IDX(a, 8)] * timestep;
        // Acceleration 
        outputSigmaPoints[IDX(a, 6)] = inputSigmaPoints[IDX(a, 6)];
        outputSigmaPoints[IDX(a, 7)] = inputSigmaPoints[IDX(a, 7)];
        outputSigmaPoints[IDX(a, 8)] = inputSigmaPoints[IDX(a, 8)];
        // Angular velocity 
        outputSigmaPoints[IDX(a, 12)] = inputSigmaPoints[IDX(a, 12)];
        outputSigmaPoints[IDX(a, 13)] = inputSigmaPoints[IDX(a, 13)];
        // MRP
        float sigmaSq = inputSigmaPoints[IDX(a, 9)] * inputSigmaPoints[IDX(a, 9)] + inputSigmaPoints[IDX(a, 10)] * inputSigmaPoints[IDX(a, 10)] + inputSigmaPoints[IDX(a, 11)] * inputSigmaPoints[IDX(a, 11)];
        float MRPin[3] = {inputSigmaPoints[IDX(a, 9)], inputSigmaPoints[IDX(a, 10)], inputSigmaPoints[IDX(a, 11)]};
        float omega[3] = { inputSigmaPoints[IDX(a, 12)], inputSigmaPoints[IDX(a, 13)], inputSigmaPoints[IDX(a, 14)]};
        float cross[3] = {0};
        float mrpNorm[3] = {0.0};
        crossProduct(MRPin, omega, cross);
        float dotSigmaOmega = MRPin[0] * omega[0] + MRPin[1] * omega[1] + MRPin[2] * omega[2];
        for (int b = 0; b < 3; b++) {
            float mrpDot = 0.25f * ((1.0f - sigmaSq) * omega[b] + 2.0f * cross[b] + 2.0f * MRPin[b] * dotSigmaOmega);
            mrpNorm[b] = MRPin[b] + timestep * mrpDot;
            outputSigmaPoints[IDX(a, (b+9))] = mrpNorm[b];
        }
        //if MRP are near bounds, normalize them
        float s2 = mrpNorm[0]*mrpNorm[0] + mrpNorm[1]*mrpNorm[1] + mrpNorm[2]*mrpNorm[2];
        if (s2 > 1.0) {
            float inv = 1.0/s2;
            outputSigmaPoints[IDX(a, 9)] = -1.0*mrpNorm[0]*inv;
            outputSigmaPoints[IDX(a, 10)] = -1.0*mrpNorm[1]*inv;
            outputSigmaPoints[IDX(a, 11)] = -1.0*mrpNorm[2]*inv;
        }
        // Biases
        outputSigmaPoints[IDX(a, 15)] = inputSigmaPoints[IDX(a, 15)] + biasRandomWalk(LowAccNoise, timestep);
        outputSigmaPoints[IDX(a, 16)] = inputSigmaPoints[IDX(a, 16)] + biasRandomWalk(LowAccNoise, timestep);
        outputSigmaPoints[IDX(a, 17)] = inputSigmaPoints[IDX(a, 17)] + biasRandomWalk(LowAccNoise, timestep);
        outputSigmaPoints[IDX(a, 18)] = inputSigmaPoints[IDX(a, 18)] + biasRandomWalk(gyroNoise, timestep);
        outputSigmaPoints[IDX(a, 19)] = inputSigmaPoints[IDX(a, 19)] + biasRandomWalk(gyroNoise, timestep);
        outputSigmaPoints[IDX(a, 20)] = inputSigmaPoints[IDX(a, 20)] + biasRandomWalk(gyroNoise, timestep);
        //adding augmented process noise
        for (int c = 0; c < 21; c++) {
            outputSigmaPoints[IDX(a, c)] += inputSigmaPoints[IDX(a, 21 + c)];
        }
        for(int i = 0; i < stateSize; ++i){
            float v = outputSigmaPoints[IDX(a,i)];
            if(!isfinite(v) || fabsf(v) > 1e8f){
                printf("\n💥  BAD σ‑point after process()\n");
                printf("    σ index   : %d\n", a);
                printf("    bad state : %d   value = %g\n", i, (double)v);
        
                /* echo the *input* sigma‑point that produced it */
                printf("    input σ[%d] = [", a);
                for(int j = 0; j < stateSize; ++j){
                    printf("%g%s", inputSigmaPoints[IDX(a,j)],
                           (j == stateSize-1 ? "]\n" : ", "));
                }
                /* optional: stop here so you can read the dump */
                exit(1);
            }
        }
    }
    
}


void measurement(float* inputSigmaPoints, float* outputSigmaPoints) {
    int bad; float v;
    
    float COMVector[3] = {0, 0, .825};
    float gravity[3] = {0, 0, Grav};
    //make sure to edit for midland!!
    float magfield[3] = {19.395e-6f, -2.042e-6f, 45.9656e-6f};
    

    for (int a = 0; a < sigmaPointSize; a++) {
        int offset = a * measurementSize;
        //DCM
        float mrpIn[3] = {inputSigmaPoints[IDX(a, 9)], inputSigmaPoints[IDX(a, 10)], inputSigmaPoints[IDX(a, 11)]};
        float omegaIn[3] = {inputSigmaPoints[IDX(a, 12)], inputSigmaPoints[IDX(a, 13)], inputSigmaPoints[IDX(a, 14)]};
        float accelIn[3] = {inputSigmaPoints[IDX(a, 6)], inputSigmaPoints[IDX(a, 7)], inputSigmaPoints[IDX(a, 8)]};
        float accelBiasIn[3] = {inputSigmaPoints[IDX(a, 15)], inputSigmaPoints[IDX(a, 16)], inputSigmaPoints[IDX(a, 17)]};
        float Cb2n[9] = {0.0};
        float Cb2nT[9] = {0.0};
        mrp2dcm(mrpIn, Cb2n);
        transpose(3, 3, Cb2n, Cb2nT);
        
        //acceleration
        float temp1[3] = {0.0};
        crossProduct(omegaIn, COMVector, temp1);
        float temp2[3] = {0.0};
        crossProduct(omegaIn, temp1, temp2);
        float temp3[3] = {0.0};
        multiplyMatrix(3, 3, 1, Cb2n, temp2, temp3);
        float temp4[3] = {(accelIn[0] + temp3[0]), (accelIn[1] + temp3[1]), (accelIn[2] + temp3[2])};
        float temp5[3] = {(temp4[0] - gravity[0]), (temp4[1] - gravity[1]), (temp4[2] - gravity[2])};
        float temp6[3] = {0.0};
        multiplyMatrix(3, 3, 1, Cb2nT, temp5, temp6);
        float temp7[3] = {(temp6[0] + accelBiasIn[0]), (temp6[1] + accelBiasIn[1]), (temp6[2] + accelBiasIn[2])};

        //magnometer
        float temp8[3] = {0.0};
        multiplyMatrix(3, 3, 1, Cb2nT, magfield, temp8);

        //output measurements with measurment noise
        //acceleration = force on the body + acceleration bias + measurement noise
        outputSigmaPoints[(a*measurementSize)] = temp7[0] + inputSigmaPoints[IDX(a, 42)];
        outputSigmaPoints[(a*measurementSize + 1)] = temp7[1] + inputSigmaPoints[IDX(a, 43)];
        outputSigmaPoints[(a*measurementSize + 2)] = temp7[2] + inputSigmaPoints[IDX(a, 44)];
        //gyro = angular velocity of the body + gyroBias + measurement nosie
        outputSigmaPoints[(a*measurementSize + 3)] = inputSigmaPoints[(IDX(a, 12))] + inputSigmaPoints[(IDX(a, 18))] + inputSigmaPoints[IDX(a, 45)];
        outputSigmaPoints[(a*measurementSize + 4)] = inputSigmaPoints[(IDX(a, 13))] + inputSigmaPoints[(IDX(a, 19))] + inputSigmaPoints[IDX(a, 46)];
        outputSigmaPoints[(a*measurementSize + 5)] = inputSigmaPoints[(IDX(a, 14))] + inputSigmaPoints[(IDX(a, 20))] + inputSigmaPoints[IDX(a, 47)];
        //mag = magnometer converted through DCM + measurement noise
        outputSigmaPoints[(a*measurementSize + 6)] = temp8[0] + inputSigmaPoints[IDX(a, 48)];
        outputSigmaPoints[(a*measurementSize + 7)] = temp8[1] + inputSigmaPoints[IDX(a, 49)];
        outputSigmaPoints[(a*measurementSize + 8)] = temp8[2] + inputSigmaPoints[IDX(a, 50)];
        //GPS = position + measurement noise
        outputSigmaPoints[(a*measurementSize + 9)] = inputSigmaPoints[(IDX(a, 0))] + inputSigmaPoints[IDX(a, 51)];
        outputSigmaPoints[(a*measurementSize + 10)] = inputSigmaPoints[(IDX(a, 1))] + inputSigmaPoints[IDX(a, 52)];
        outputSigmaPoints[(a*measurementSize + 11)] = inputSigmaPoints[(IDX(a, 2))] + inputSigmaPoints[IDX(a, 53)];
        float *meas_row = &outputSigmaPoints[a * measurementSize];
    }

}

//This function predicts the next state vector using several steps.
void ukfPredict(float* sigmaPoints, float* processedSigmaPoints, float timestep, float* predictedStateMean, float* predictedMeasurementMean, float meanWeight, float otherWeight, float covarianceWeight, float* measuredSigmaPoints, float* predictedCovariance) {
    //integrate sigma points
    process(sigmaPoints, processedSigmaPoints, timestep);
    float weight = 0;

    //take the weighted average of those sigma points
    for (int a = 0; a < augmentedSize; a++) {
        float temp1 = 0.0;
        for (int b = 0; b < sigmaPointSize; b++) {
            if (b == 0) {
                weight = meanWeight;
            }
            else {
                weight = otherWeight;
            }
            temp1 += weight * processedSigmaPoints[(IDX(b, a))];
        }
        predictedStateMean[a] = temp1;
    }


    float differenceVector[augmentedSize] = {0.0};
    float skewM[augmentedSize*augmentedSize] = {0.0};
    float weightedSkew[augmentedSize*augmentedSize] = {0.0};
    for (int c = 0; c < (augmentedSize*augmentedSize); c++) {
        predictedCovariance[c] = 0;
    }

    //take the weighted average of the skew of the difference between the predictedmean and each sigma point
    for (int d = 0; d < sigmaPointSize; d++) {
        if (d == 0) {
            weight = covarianceWeight;
        }
        else {
            weight = otherWeight;
        }
        for (int e = 0; e < augmentedSize; e++) {
            differenceVector[e] = predictedStateMean[e] - processedSigmaPoints[IDX(d, e)];
        }
        skew(augmentedSize, differenceVector, skewM);
        scaleMatrix((augmentedSize*augmentedSize), skewM, weight, weightedSkew);
        addMatrix((augmentedSize*augmentedSize), predictedCovariance, weightedSkew, predictedCovariance);
    }

    //pass integrated sigma points through measurement function.
    measurement(processedSigmaPoints, measuredSigmaPoints);

    //we take their weighted mean
    for (int f = 0; f < measurementSize; f++) {
        float temp2 = 0.0;
        for (int g = 0; g < sigmaPointSize; g++) {
            if (g == 0) {
                weight = meanWeight;
            }
            else {
                weight = otherWeight;
            }
            temp2 += weight * measuredSigmaPoints[((g) * measurementSize + (f))];
        }
        predictedMeasurementMean[f] = temp2;
    }
}

//updates the state vector, covariance, and kalman gain using new sensor data, 
//and the last prediction step's outputs.
void ukfUpdate(float* newState, float* newCovariance, float* newSensorVector, float covarianceWeight, float otherWeight, float* predictedStateMean, float* processedSigmaPoints, float* predictedMeasurementMean, float* measuredSigmaPoints, float* predictedCovariance) {
    //calcualting that first matrix: the weighted sum of the skew of difference vector between the predicted 
    //measurement mean and the measured sigma poiints
    float weight = 0.0;
    float matrix1[measurementSize*measurementSize] = {0.0};
    float matrix2[augmentedSize*measurementSize] = {0.0};
    float kalmanGain[augmentedSize*measurementSize] = {0.0};
    for (int a = 0; a < measurementSize*measurementSize; a++) {
        matrix1[a] = 0.0;
    }
    for (int b = 0; b < augmentedSize*measurementSize; b++) {
        matrix2[b] = 0.0;
    }
    for (int c = 0; c < sigmaPointSize; c++) {
        if (c == 0) {
            weight = covarianceWeight;
        }
        else {
            weight = otherWeight;
        }
        //first matrix
        float temp1[measurementSize*measurementSize] = {0.0};
        float temp2[measurementSize*measurementSize] = {0.0};
        float measurementDifference[measurementSize] = {0.0};
        //takes the difference between each sigma point and the predicted measurement mean
        for (int d = 0; d < measurementSize; d++) {
            measurementDifference[d] = measuredSigmaPoints[c*measurementSize + d] - predictedMeasurementMean[d];
        }
        //takes the outer product of that difference vector, multiplies that matrix times the weight, and adds it to the Pyy matrix.
        skew(measurementSize, measurementDifference, temp1);    
        scaleMatrix((measurementSize*measurementSize), temp1, weight, temp2);
        addMatrix((measurementSize*measurementSize), matrix1, temp2, matrix1);

        //calculating seocnd matrix that is the weighted sum of the product of 2 difference vectors: 
        //the difference between the processed sigma points and the predicted state mean, 
        //and the difference between the measured sigma points and the predicted measurement mean.
        float temp3[measurementSize*augmentedSize] = {0.0};
        float temp4[measurementSize*augmentedSize] = {0.0};
        float processDifference[augmentedSize] = {0.0};
        float measurementDifferenceT[measurementSize] = {0.0};
        //finds the difference between each sigma point and the predicted state mean.
        for (int e = 0; e < augmentedSize; e++) {
            processDifference[e] = processedSigmaPoints[(IDX(c, e))] - predictedStateMean[e];
        }
        //takes the tranpose of the measurement differnce vector.
        transpose(measurementSize, 1, measurementDifference, measurementDifferenceT);
        //mutliplies the tranpose of the measurement difference vector and the state difference vector
        multiplyMatrix(augmentedSize, 1, measurementSize, processDifference, measurementDifferenceT, temp3);
        //multiplies that matrix by the weight
        scaleMatrix((augmentedSize*measurementSize), temp3, weight, temp4);
        //adds that to the Pxy matrix.
        addMatrix((augmentedSize*measurementSize), temp4, matrix2, matrix2);
    }
    //adding in measurement covariance matrix
    /* --- after you finish the Pyy accumulation ------------------- */
    float R[measurementSize * measurementSize] = {0};   // build once
    static int R_init = 0;
    if(!R_init){
        float Rdiag[measurementSize] = {
            1e-6f,1e-6f,1e-6f,          /* accel  */
            6e-4f,6e-4f,6e-4f,          /* gyro   */
            0.01f*0.01f,0.01f*0.01f,0.01f*0.01f,  /* mag */
            25.0f,25.0f,25.0f           /* GPS    */
        };
        diag(measurementSize, R, Rdiag);
        R_init = 1;
    }
    addMatrix(measurementSize*measurementSize, matrix1, R, matrix1);


    float matrix1Inverse[measurementSize*measurementSize] = {0.0};
    inverse(matrix1, matrix1Inverse);
    multiplyMatrix(augmentedSize, measurementSize, measurementSize, matrix2, matrix1Inverse, kalmanGain);



    //We calcualte the new state vector using the old state vector + the kalman gain * the difference between 
    //the predicted measurment mean and real measurement vector.
    float temp5[measurementSize] = {0.0};
    for (int g = 0; g < measurementSize; g++) {
        temp5[g] = newSensorVector[g] - predictedMeasurementMean[g];
    }
    float temp6[augmentedSize] = {0.0};
    multiplyMatrix(augmentedSize, measurementSize, measurementSize, kalmanGain, temp5, temp6);
    addVector(augmentedSize, predictedStateMean, temp6, newState);

    //We calculate the new covariance matrix by taking the difference between the predicted covariance matrix 
    //and the kalman gain* the first matrix * the tranpose of the kalman gain.
    float kalmanTranspose[augmentedSize*measurementSize] = {0.0};
    transpose(augmentedSize, measurementSize, kalmanGain, kalmanTranspose);
    float temp7[augmentedSize*measurementSize] = {0.0};
    multiplyMatrix(augmentedSize, measurementSize, measurementSize, kalmanGain, matrix1, temp7);
    float temp8[augmentedSize*augmentedSize] = {0.0};
    multiplyMatrix(augmentedSize, measurementSize, augmentedSize, temp7, kalmanTranspose, temp8);
    float temp9[augmentedSize*augmentedSize] = {0.0};
    scaleMatrix((augmentedSize*augmentedSize), temp8, -1.0, temp9);
    addMatrix((augmentedSize*augmentedSize), temp9, predictedCovariance, newCovariance);

}

//takes in a size of matrix, the matrix to be modified, a 
// list of diagonal values and returns the modified matrix.
void diag(int size, float* outMatrix, float* diagonals) {
    for (int a = 0; a < size; a++) {
        outMatrix[a + a*size] = diagonals[a];
    }
}

//takes in size of matrix a, and a matrix to take the square root of, b and saves it to a new matrix, c
void chol(int size, float* inputMatrix, float* outputMatrix) {
    for (int a = 0; a < size; a++) {
        for (int b = 0; b <= a; b++) {
            float temp1 = 0.0;
            for (int f = 0; f < b; f++) {
                temp1 += outputMatrix[a*size + f]*outputMatrix[b*size + f];
            }
            if (a == b) {
                float temp2 = inputMatrix[a*size + a] - temp1;
                if (temp2 < 1e-8) {
                    temp2 = 1e-8;
                }
                outputMatrix[a*size + b] = sqrt(temp2);
            }
            else {
                outputMatrix[a*size + b] = (1.0/outputMatrix[b*size + b])*(inputMatrix[a*size + b] - temp1);
            }
        }
    }
}

void inverse(float *in, float *out){
    float a[12][24];
    int i,j,k,mx;
    float t;

    /* build [A | I] */
    for(i=0;i<12;i++){
        for(j=0;j<12;j++){
            a[i][j]     = in[i*12 + j];
            a[i][j+12]  = (i==j);
        }
    }
    /* Gauss–Jordan */
    for(i=0;i<12;i++){
        /* pivot */
        mx = i;
        for(k=i+1;k<12;k++)
            if(fabs(a[k][i]) > fabs(a[mx][i])) mx = k;
        /* swap rows i<->mx */
        for(j=0;j<24;j++){
            t = a[i][j]; a[i][j] = a[mx][j]; a[mx][j] = t;
        }
        /* normalize pivot row */
        t = a[i][i];
        for(j=0;j<24;j++) a[i][j] /= t;
        /* eliminate other rows */
        for(k=0;k<12;k++) if(k!=i){
            t = a[k][i];
            for(j=0;j<24;j++)
                a[k][j] -= t * a[i][j];
        }
    }
    /* extract inverse from [A | I] */
    for(i=0;i<12;i++)
        for(j=0;j<12;j++)
            out[i*12 + j] = a[i][j+12];
}

void addVector(int size, float* inputA, float* inputB, float* output) {
    for (int a = 0; a < size; a++) {
        output[a] = inputA[a] + inputB[a];
    }
}

void scaleVector(int size, float* inputVector, float scalar, float* outputVector) {
    for (int a = 0; a < size; a++) {
        outputVector[a] = inputVector[a]*scalar;
    }
}
//only works with 3 dimensional vectors!!!
void crossProduct(float* inputA, float* inputB, float* output) {
    output[0] = inputA[1]*inputB[2] - inputA[2]*inputB[1];
    output[1] = inputA[2]*inputB[0] - inputA[0]*inputB[2];
    output[2] = inputA[0]*inputB[1] - inputA[1]*inputB[0];
}

/* Box‑Muller: draws N(0,1) */
static float gaussian_unit(void) {
    seed_rng_once();
    float u1, u2;
    do { u1 = (rand() + 1.0f) / (RAND_MAX + 2.0f); } while (u1 <= 0.0f);
    u2 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)3.1415926 * u2);
}

/* Seed once at start‑up (call in main() or first use) */
static void seed_rng_once(void) {
    if (!rng_seeded) {
        srand((unsigned)time(NULL));
        rng_seeded = 1;
    }
}

// Basic Gaussian noise generator using Box-Muller
float gaussian_noise(float mean, float stddev) {
    return(mean+stddev*gaussian_unit());
}

float biasRandomWalk(float sigma, float timestep) {
    float stdev = sigma*sqrt(timestep);
    return(gaussian_noise(0.0, stdev));
}

void scaleMatrix(int size, float* inputMatrix, float scalar, float* outputMatrix) {
    for (int a = 0; a < size; a++) {
        outputMatrix[a] = inputMatrix[a] * scalar;
    }
}

void addMatrix(int size,float* inputMatrix, float* addedMatrix, float* outputMatrix) {
    for (int a = 0; a < size; a++) {
        outputMatrix[a] = inputMatrix[a] + addedMatrix[a];
    }
}

void skew(int size, float* inputVector, float* outputMatrix) {
    for (int a = 0; a < size; a++) {
        for (int b = 0; b < size; b++) {
            outputMatrix[a*size + b] = inputVector[a] * inputVector[b];
        }
    }
}

void multiplyMatrix(int rowASize, int colASize, int colBSize, float* inputMatrixA, float* inputMatrixB, float* outputMatrix) {
    for (int a = 0; a < rowASize; a++) {
        for (int b = 0; b < colBSize; b++) {
            float sum = 0.0;
            for (int c = 0; c < colASize; c++) {
                sum += inputMatrixA[a*colASize + c] * inputMatrixB[c*colBSize + b];
            }
            outputMatrix[(a*colBSize + b)] = sum;
        }
    }
}   

void dotProduct(int size, float* inputVectorA, float* inputVectorB, float *outputScalar) {
    *outputScalar = 0.0;
    for (int a = 0; a < size; a++) {
        *outputScalar += inputVectorA[a] * inputVectorB[a];
    }
}

void mrp2dcm(float* inputMRP, float* outputMRP) {
    float s2 = 0.0;
    dotProduct(3, inputMRP, inputMRP, &s2);
    float sCross[9] = {0.0};
    sCross[0] = 0.0;
    sCross[1] = -1.0*inputMRP[2];
    sCross[2] = inputMRP[1];
    sCross[3] = inputMRP[2];
    sCross[4] = 0.0;
    sCross[5] = -1.0*inputMRP[0];
    sCross[6] = -1.0*inputMRP[1];
    sCross[7] = inputMRP[0];
    sCross[8] = 0.0;
    float temp1[9] = {0.0};
    multiplyMatrix(3, 3, 3, sCross, sCross, temp1);
    float temp2[9] = {0.0};
    scaleMatrix(9, temp1, 8.0, temp2);
    float temp3[9] = {0.0};
    scaleMatrix(9, sCross, (-1.0*(4.0*(1.0-s2))), temp3);
    float temp4[9] = {0.0};
    addMatrix(9, temp2, temp3, temp4);
    float temp5[9] = {0.0};
    scaleMatrix(9, temp4, (1.0/(powf((1+s2), 2.0))), temp5);
    float I[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    addMatrix(9, I, temp5, outputMRP);
}

void transpose(int sizeRow, int sizeCol, float* inputMat, float* outputMat) {
    for (int a = 0; a < sizeRow; a++) {
        for (int b = 0; b < sizeCol; b++) {
            outputMat[(b*sizeRow + a)] = inputMat[(a*sizeCol + b)];
        }
    }
}