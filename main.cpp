#include "SistemaResorte.hpp"
#include <iostream>

int main () {
    double constexpr m = 250.0; // masa de un cuarto de vehiculo
    double constexpr Wn = 10.0; // frecuencia natural no amortiguada rad/s
    double constexpr C = 0.3; // razon de amortiguamiento
    double constexpr h = 0.001; // paso del tiempo (dt)
    double t = 0.0; // tiempo
    double constexpr tiempoFinal = 5.0; // tiempo de simulacion en segundos

    double constexpr k = m * (Wn * Wn);
    double constexpr c = 2.0 * C * m * Wn;

    std::optional<QuarterCar> car = QuarterCar::createObjet(c, k, m, QuarterCar::Estado{0.05,0.0});

    if(!car.has_value()){
        std::cout << "Fallo critico, no se puede iniciar el modelo por division entre 0.\n";
        return -1;
    }

    while (t <= tiempoFinal){
        QuarterCar::Estado estado = car->getEstado();
        std::cout << t << "," << estado.posicion << "\n";
        car->stepRK4(h,t);
        t = t + h;
    }

    return 0;
}
