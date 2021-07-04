//
// Created by Athos Innocenti on 04/07/21.
//

#include <numeric>
#include "Performance.h"

Performance::Performance() = default;

Performance::~Performance() = default;

double Performance::calculateAvg(const std::list<double> &list) {
    assert(!list.empty());
    double avg = std::accumulate(std::begin(list), std::end(list), 0.0);
    avg /= list.size();
    return avg;
}

void Performance::storeData(const std::list<double> &list, const std::string &name_file) {
    data_file.open(name_file);
    for (double value : list)
        data_file << value << "\n";
    data_file.close();
}
