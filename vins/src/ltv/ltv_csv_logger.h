#pragma once

#include "ltv_types.h"

#include <fstream>
#include <string>

namespace ltv
{

class LtvCsvLogger
{
  public:
    void configure(bool enabled, const std::string &path);
    void close();
    void write(const LtvSnapshot &snapshot);
    bool isOpen() const;

  private:
    std::ofstream stream_;
};

} // namespace ltv
