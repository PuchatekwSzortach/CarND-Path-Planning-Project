#ifndef POLYFIT_H_
#define POLYFIT_H_

// Adapted from https://github.com/natedomin/polyfit

int polyfit(const double* const dependentValues,
            const double* const independentValues,
            unsigned int        countOfElements,
            unsigned int        order,
            double*             coefficients);


#endif
