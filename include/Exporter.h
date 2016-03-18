#ifndef EXPORTER_H
#define EXPORTER_H

#include "Map.h"

namespace ORB_SLAM
{

class Map;

class Exporter
{
    Map *map_view;
public:
    Exporter(Map *map);
    void WriteKeyFrames(const char *path = NULL) const;
    void WriteMapPointsTXT(const char *path = NULL) const;
};

}// namespace ORB_SLAM

#endif // EXPORTER_H
