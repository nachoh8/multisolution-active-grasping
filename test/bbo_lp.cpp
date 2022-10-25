#include <iostream>
#include <cmath>
#include <boost/numeric/ublas/assignment.hpp>
#include <boost/math/distributions/normal.hpp>

#include <bayesopt/bayesopt.hpp>

#define EPSILON 1e-4

namespace ublas = boost::numeric::ublas;

// Local Penalization criteria by [Javier González et al., 2015]
class LocalPenalization {
    enum Transformation{None = 0, Softplus = 1};

private:
    vecOfvec X_batch;
    int current_point = 0;

    Transformation transform;
    vectord r, s;

    void predict(vectord& mean, vectord& std_dev)
    {
        mean = vectord(current_point);
        std_dev = vectord(current_point);

        for (int i = 0; i < current_point; i++) {
            double s = 0;
            for (auto& v : X_batch[i]) {
                s += v;
            }
            mean[i] = s;
            std_dev[i] = s / 2.0;
        }
    }

    vectord hammer(const vectord& x) {
        vectord res(current_point);

        for (int i = 0; i < current_point; i++) {
            vectord aux = x - X_batch[i];
            aux = ublas::element_prod(aux, aux);
            
            double v = ublas::sum(aux);
            v = (sqrt(v) - r[i]) / s[i];
            
            res[i] = std::log(boost::math::cdf(boost::math::normal(), v));
            
            // std::cout << i << " -> " << res[i] << std::endl;
        }

        return res;
    }

    double penalized_acquisition(const vectord &x) {
        double fval = 0.0; // TODO: change to acquisition function (EI, LCB, etc)
        for (auto& v : x) {
            fval -= v/100000;
        }
        
        fval = -fval;

        switch (transform) // map to log natural space
        {
        case Transformation::Softplus:
            std::cout << "Applying Softplus transformation\n";
            fval = std::log(std::log1p(std::exp(fval))); // g(z) = ln(ln(1+e^z))
            break;
        
        default:
            std::cout << "Applying None transformation\n";
            fval = std::log(fval + 1e-50); // g(z) = ln(z)
            break;
        }

        fval = -fval;
        
        if (current_point > 0) {
            vectord exlcusion = hammer(x);
            fval += ublas::sum(exlcusion * -1.0);
        }

        return fval;
    }

public:

    LocalPenalization(const int batch_size, Transformation transform = Transformation::None) {
        X_batch = vecOfvec(batch_size - 1);
        
        r = vectord(X_batch.size());
        s = vectord(X_batch.size());

        this->transform = transform;
    }

    void update_batch(const vectord& x, const double L, const double min) {
        X_batch[current_point++] = x;

        vectord mean, std_dev;
        predict(mean, std_dev); // TODO: model.predict(X), size: num_pts x 1

        for (int i = 0; i < current_point; i++) {
            if (std_dev[i] < 1e-16) {
                std_dev[i] = 1e-16;
            }
            s[i] = sqrt(std_dev[i]) / L;
            r[i] = (mean[i] - min) / L;

            // std::cout << i << " -> " << mean[i] << ", " << std_dev[i] << ", " << s[i] << ", " << r[i] << std::endl;
        }
    }
    
    void reset() {
        current_point = 0;
    }

    double operator() (const vectord &x) {
        return penalized_acquisition(x);
    }
};

double gradient_check(const vectord& x) {
    const int d = x.size();
    vectord g(d); // gradient
    for (int i = 0; i < d; i++) { // for each dimension
        zvectord e(d);
        e[i] = EPSILON;

        double vx_plus = model.predict(x + e); // i-th is incremented
        double vx_minus = model.predict(x - e); // i-th is decreased

        g[i] = (vx_plus - vx_minus) / (2.0 * EPSILON);
    }

    double L = ublas::norm_2(g);
    return -L; // to minimize
}

double estimateL() {
    vecOfvec samples = sample_points_uniform(N);
    vectord pred_samples = [gradient_check(x) for x in samples];
    vectord min_x = pred_samples.argmin(); // get point at min

    double res = optimize(gradient_check, min_x, MAX_ITER); // search new min
    
    double L = -res;
    if (L < 1e-7) { // avoid problems in cases in which the model is flat.
        return 10;
    }

    return L;
}

int main(int argc, char *argv[]) {
    /*const double L = 1.0;
    const double Min = 0.0;
    LocalPenalization f(4);
    vecOfvec X(3);
    X[0] = vectord(2);
    X[0][0] = 0.0;
    X[0][1] = 1.0;
    f.update_batch(X[0], L, Min);
    X[1] = vectord(2);
    X[1][0] = 2.0;
    X[1][1] = 3.0;
    f.update_batch(X[1], L, Min);
    X[2] = vectord(2);
    X[2][0] = 4.0;
    X[2][1] = 5.0;
    f.update_batch(X[2], L, Min);
    
    vectord x(2);
    x[0] = 1.0;
    x[1] = 0.5;

    double fval = f(x);
    std::cout << "fval = " << fval << std::endl;*/

    vectord x = zvectord(1);
    std::cout << x << std::endl;
    x = svectord(1,1.0);
    std::cout << x << std::endl;
    return 0;
}
