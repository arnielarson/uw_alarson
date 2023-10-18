#ifndef COMPLEX_H
#define COMPLEX_H

/*! @file */

/*! 
 *  A Complex number struct.
 */
typedef struct {
    double real;
    double im;
} Complex;

/*! Add two complex numbers together
 *  \param a The first complex number
 *  \param b The second complex number
 */
Complex add ( Complex a, Complex b );

/*! Negate a complex number
 *  \param a The first complex number
 *  \param b The second complex number
 */
Complex negate ( Complex a );

/*! Multiply two complex numbers together
 *  \param a The first complex number
 *  \param b The second complex number
 */
Complex multiply ( Complex a, Complex b );

/*! Return the magnitude of a complex number
 *  \param a A complex number
 */
double magnitude ( Complex a);

#endif