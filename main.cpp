#include "SistemaResorte.hpp"
#include <iostream>

int main () {
    double constexpr m = 250.0; // masa de un cuarto de vehiculo
    double constexpr Wn = 10.0; // frecuencia natural no amortiguada rad/s
    double constexpr C = 0.3; // razon de amortiguamiento
    double constexpr h = 0.001; // paso del tiempo (dt)
    double t = 0.0; // tiempo
    double constexpr tiempoFinal = 5.0; // tiempo de simulacion en segundos

    QuarterCar car (1500, 25000, 250, QuarterCar::Estado{0.05,0.0});

    while (t <= tiempoFinal){
        QuarterCar::Estado estado = car.getEstado();
        std::cout << t << "," << estado.posicion << "\n";
        car.stepRK4(h,t);
        t = t + h;
    }
}
