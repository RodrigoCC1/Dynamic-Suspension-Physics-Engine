#ifndef SISTEMARESORTE_HPP
#define SISTEMARESORTE_HPP
#include <optional>

class QuarterCar
{
public:

    struct Estado
    {
        double posicion{};
        double velocidad{};

        Estado operator+(const Estado& estado_nuevo) const;
        Estado operator*(double escalar) const;
    };

    static std::optional<QuarterCar> createObjet(double c, double k, double m, Estado estado_inicial);

    Estado calculateDerivatives(double t, const Estado& estado_actual) const;

    void stepRK4(double h, double t);

    const Estado& getEstado() const;

private:

    double k_m;
    double c_m;

    Estado estadoActual;

        QuarterCar(
        double c_m,
        double k_m,
        Estado estado_inicial
    );
};

#endif