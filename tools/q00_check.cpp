// Scratch: exact Van Loan Q00 vs the diagonal shortcut, for the docs/3 8.3 note.
#include "truck_model/matrix_exponential.hpp"

#include <cstdio>

int main() {
    const double a = 15.0 / 7.0;              // U / L2 at phi = 0, ell_h = 0
    const double qPhi = 2.741556778080377e-3;
    const double qB = 2.0e-4;
    const double T = 0.05;

    truck_model::Matrix<2, 2> A{};
    A[0][0] = -a;
    A[0][1] = -1.0;
    truck_model::Matrix<2, 2> Qc{};
    Qc[0][0] = qPhi;
    Qc[1][1] = qB;

    const auto d = truck_model::discretizeVanLoan(A, Qc, T);
    const double exact = d.processNoise[0][0];
    const double diagonal = qPhi * T;
    const double firstOrder = qPhi * T - a * qPhi * T * T;
    const double withCubic = firstOrder +
                             (2.0 / 3.0 * a * a * qPhi + qB / 3.0) * T * T * T;

    std::printf("exact Van Loan Q00 = %.6e\n", exact);
    std::printf("diagonal  qPhi*T   = %.6e   (%.2f%% vs exact)\n", diagonal,
                100.0 * (diagonal / exact - 1.0));
    std::printf("1st-order expansion= %.6e   (%.2f%% vs exact)\n", firstOrder,
                100.0 * (firstOrder / exact - 1.0));
    std::printf("with T^3 terms     = %.6e   (%.3f%% vs exact)\n", withCubic,
                100.0 * (withCubic / exact - 1.0));
    std::printf("cross term Q01     = %.6e   (-qB*T^2/2 = %.6e)\n",
                d.processNoise[0][1], -0.5 * qB * T * T);
    return 0;
}
