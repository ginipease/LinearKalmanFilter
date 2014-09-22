/* CODE USING WIKIPEDIA TRILAT DERIVATION AND VECTOR CORRECTION
*  REPEAT CALCULATION FOR COMBINATIONS OF 4 DATASETS
*/

#include <stdio.h>
#include <math.h>
 
#define   MAXZERO  0.0
#define min(a,b) (a<b?a:b)
#define max(a,b) (a>b?a:b)

/*---------------------------------------------------------------------------*/
/**
 * Struct for Kalman filter vars
 */
typedef struct KalmanVars KalmanVars;
struct KalmanVars {

	float		x[2];		/* state (location and velocity) */
	float		P[2][2];	/* Error covariance - uncertainty */

   /* float x[2];      // initial state (location and velocity)
    float P[2][2];   // initial uncertainty

    float u[2];      // external motion        // For Prediction
    float F[2][2];   // next state function    // For Prediction
    float H[2];      // measurement function
    float R[1];      // measurement uncertainty
    float I[2][2];   // identity matrix*/

};
/*---------------------------------------------------------------------------*/
/**
 * Struct for transpose of matrix
 */
typedef struct Transpose Transpose;
struct Transpose {

	float		T[2][2];
};
/*---------------------------------------------------------------------------*/
/**
 * \brief Take initial distance, error covariance and measurement
 * \return Kalman estimate of distance
 *
 * This function takes 
 */
struct KalmanVars kalman_predict(float x, float P);
/*---------------------------------------------------------------------------*/
/**
 * \brief Take initial distance, error covariance and measurement
 * \return Kalman estimate of distance
 *
 * This function takes 
 */
struct KalmanVars kalman_estimate(float z);
/*---------------------------------------------------------------------------*/
/**
 * \brief Take initial distance, error covariance and measurement
 * \return Kalman estimate of distance
 *
 * This function takes 
 */
struct Transpose(float matrix[10][10], uint8_t n, uint8_t m);

 
