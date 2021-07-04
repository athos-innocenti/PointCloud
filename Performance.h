//
// Created by Athos Innocenti on 04/07/21.
//

#ifndef ICP_PERFORMANCE_H
#define ICP_PERFORMANCE_H

#include <fstream>
#include <list>

class Performance {
public:
    Performance();

    virtual ~Performance();

    static double calculateAvg(const std::list<double> &list);

    void storeData(const std::list<double> &list, const std::string &name_file);

private:
    std::ofstream data_file;
};


#endif //ICP_PERFORMANCE_H
