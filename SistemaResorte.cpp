#include "SistemaResorte.hpp"
#include <optional>

QuarterCar::QuarterCar(double c_m, double k_m, Estado estado_inicial) :
    c_m(c_m), k_m(k_m), estadoActual(estado_inicial){}

std::optional<QuarterCar> QuarterCar::createObjet(double c, double k, double m, Estado estado_inicial){
    if(m <= 0.0) return std::nullopt;
    double cm = c / m;
    double km = k / m;

    return QuarterCar(cm, km, estado_inicial);
}

QuarterCar::Estado QuarterCar::Estado::operator +(const Estado& estado_nuevo) const{
    Estado sumarEstado;
    sumarEstado.posicion = posicion + estado_nuevo.posicion;
    sumarEstado.velocidad = velocidad + estado_nuevo.velocidad;
    return sumarEstado;
}

QuarterCar::Estado QuarterCar::Estado::operator *(double escalar) const {
    Estado multiplicarEstado;
    multiplicarEstado.posicion = posicion * escalar;
    multiplicarEstado.velocidad = velocidad * escalar;
    return multiplicarEstado;
}

const QuarterCar::Estado& QuarterCar::getEstado() const{
    return estadoActual;
}

QuarterCar::Estado QuarterCar::calculateDerivatives (double t, const Estado& estado_actual) const{
    Estado estadoDerivado;
    estadoDerivado.posicion = estado_actual.velocidad;
    estadoDerivado.velocidad = (-(c_m) * estado_actual.velocidad) - ((k_m) * estado_actual.posicion);
    return estadoDerivado;
}

void QuarterCar::stepRK4(double h, double t){
        Estado k1 = calculateDerivatives(t, estadoActual);
        Estado k2 = calculateDerivatives(t + (h * 0.5), estadoActual + (k1 * (h * 0.5)));
        Estado k3 = calculateDerivatives(t + (h * 0.5), estadoActual + (k2 * (h * 0.5)));
        Estado k4 = calculateDerivatives(t + h, estadoActual + (k3 * h));
        estadoActual = estadoActual + (k1 + (k2*2) + (k3*2) + k4) * (h/6.0);
}
