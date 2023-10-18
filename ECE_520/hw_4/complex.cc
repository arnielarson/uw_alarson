#include <math.h>
#include "complex.h"

double Complex::magnitude() const {
    return sqrt(re*re + im*im);
}
Complex Complex::conjugate() const {
    return Complex(real(), -imaginary());
}
double Complex::real() const {
    return re;
}
double Complex::imaginary() const {
    return im;
}

/* Operators */

bool operator<(const Complex& a, const Complex& b) {

    return a.magnitude() < b.magnitude();

}


Complex operator*(const Complex& a, const Complex& b) {

    return Complex(a.real()*b.real()-a.imaginary()*b.imaginary(), a.real()*b.imaginary() + a.imaginary()*b.real());

}
Complex operator+(const Complex& a, const Complex& b) {
    
    return Complex(a.real() + b.real(), a.imaginary() + b.imaginary()); 

}
bool operator==(const Complex& a, const Complex& b) {

    return a.real()==b.real() && a.imaginary()==b.imaginary();
}