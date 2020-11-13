#include "../matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main()
{
    plt::plot({1, 3, 2, 4});
    plt::xlabel("Time[s]");
    plt::ylabel("Magnitude");
    plt::show();
}
