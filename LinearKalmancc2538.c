/* CODE USING LINEAR KALMAN FILTER EQUATION
* Kalman filter, with two state Q
* 	x = A*x;
* 	P = A*P*AT + Q;
* 	K = Pp*HT*inv(H*Pp*HT + R);
* 	x = xp + K*(z - H*xp);
* 	P = Pp - K*H*Pp;
*	(T: matrix transpose)
* System model
* 	xk = A*xk+wk
*	zk = H*xk+vk
*
* Author: Sarogini Grace Pease 2014
*
* gcc -g -o linearkalman -ansi LinearKalmancc2538.c -lm
*/
#include "LinearKalmancc2538.h"

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


uint32_t alpha = 0;
RangeStats clear = {0,0,0,0,0,{0}};

int main ()
{
float estimate=0;
float x0=0;
float P0=0;

struct KalmanVars x_predict x_est;
struct KalmanVars init = {.x = x0, .P = P0 };
x_predict = kalman_predict(init);
x_est = kalman_estimate(x_predict);

return estimate;
}

/*---------------------------------------------------------------------------*/
struct KalmanVars kalman_predict(struct init)
{
	/*
	 */
	KalmanVars pred;

	float AT[2][2]=Transpose(init.A,10,10);
	float Q[2]=;   /* nxn diagonal matrix of state transition noise - with diagonal as an array to save space */
		/* external motion */
	float A[2][2];	/* State transition matrix */

	/* x = A*x */
	pred->x[0] = A[0][0]*init->x[0] + A[0][1]*init->x[1];
	pred->x[1] = A[1][0]*init->x[0] + A[1][1]*init->x[1];

	/*  P = (A*P)*AT + Q		*/
	/*  				*/
	/*  NB:				*/
	/*  	Q' = 1 0   Q = 1 1	*/
	/* 	     0 1 		*/
	/*  				*/
    pred->P[0][0] = AT[0][0] * ( A[0][0]*init->P[0][0] + A[0][1]*init->P[1][0] ) + AT[1][0] * ( A[0][0]*init->P[0][1] + A[0][1]*init->P[1][1] ) + Q[0];
    pred->P[0][1] = AT[0][1] * ( A[0][0]*init->P[0][0] + A[0][1]*init->P[1][0] ) + AT[1][1] * ( A[0][0]*init->P[0][1] + A[0][1]*init->P[1][1] ) ;
    pred->P[1][0] = AT[0][0] * ( A[1][0]*init->P[0][0] + A[1][1]*init->P[1][0] ) + AT[1][0] * ( A[1][0]*init->P[0][1] + A[1][1]*init->P[1][1] ) ;
    pred->P[1][1] = AT[0][1] * ( A[1][0]*init->P[0][0] + A[1][1]*init->P[1][0] ) + AT[1][1] * ( A[1][0]*init->P[0][1] + A[1][1]*init->P[1][1] ) + Q[1];

  return pred;
}
/*---------------------------------------------------------------------------*/
struct KalmanVars kalman_estimate(struct prediction)
{
	/*
	 */
	KalmanVars est;
	float K[2]; /* Kalman gain */

					/* System Model */
	float		H[2];		/* State to measurement matrix */
	float		R[1];		/* Covariance matrix of measurement noise */
	float		I[2][2];	/* identity matrix*/
	float		HT[2][2]=Transpose(est.H,10,10);

	K = prediction.P*HT*inv(H*prediction.P*HT + R);

	est.x = prediction.x + K*(z - H*prediction.x);

	est.Pk = prediction.P - K*H*prediction.P;

  return est;
}
/*---------------------------------------------------------------------------*/
struct Transpose(uint8_t matrix[10][10], uint8_t n, uint8_t m)
{
   for( uint8_t c = 0 ; c < m ; c++ ){
   	for( uint8_t d = 0 ; d < n ; d++ ){
		uint8_t transpose[d][c] = matrix[c][d];
	}
   }
	Transpose trans;
	trans.T = transpose[n][m];
  return trans;
}
