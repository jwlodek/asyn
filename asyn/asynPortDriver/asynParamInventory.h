#ifndef asynParamInventory_H
#define asynParamInventory_H

#include <map>
#include <string>
#include <vector>
#include <stdio.h>

#include <asynAPI.h>

class asynPortDriver;

namespace asynParamInventory {

struct RecordRef {
    std::string name;
    std::string recordType;
    std::string dtyp;
    std::string link;
};

struct ParamInfo {
    std::string name;
    std::string asynType;
    int addr;
    int index;
    std::vector<RecordRef> setpoints;
    std::vector<RecordRef> readbacks;
};

struct PortInfo {
    std::string portName;
    std::string driverClass; /* May be empty if the runtime class name is not cheaply available. */
    std::vector<ParamInfo> params;
    std::vector<RecordRef> unmatched;
};

typedef std::map<std::string, PortInfo> Inventory;

/* Returns a point-in-time copy of the current process-wide inventory. */
ASYN_API void registerPort(asynPortDriver *port);
ASYN_API void unregisterPort(const char *portName);
ASYN_API Inventory getInventory();
/* Writes the current inventory to fp.  If portName is NULL or empty, report all ports. */
ASYN_API void report(FILE *fp, const char *portName);

} /* namespace asynParamInventory */

#endif /* asynParamInventory_H */
