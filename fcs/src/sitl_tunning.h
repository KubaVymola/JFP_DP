#ifndef SITLTUNNING_H
#define SITLTUNNING_H

#ifdef SITL

#include <map>
#include <string>

extern "C" void init_override(std::map<std::string, float> &config);
void tunning_config();

#endif // SITL

#endif
