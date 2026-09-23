/*
 * asynParamInventory.cpp
 *
 * Optional process-wide inventory of registered asynPortDriver ports,
 * their parameters, and any EPICS records linked to those parameters.
 */

#include <ctype.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <map>
#include <set>
#include <string>
#include <vector>

#include <epicsExport.h>
#include <epicsGuard.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <errlog.h>
#include <initHooks.h>
#include <iocsh.h>

#include "asynPortDriver.h"
#include "asynParamInventory.h"

#ifndef EPICS_LIBCOM_ONLY
#include <dbStaticLib.h>
#endif

#if defined(WITH_PVXS)
#include <pvxs/data.h>
#include <pvxs/iochooks.h>
#include <pvxs/server.h>
#include <pvxs/sharedpv.h>
#elif defined(WITH_PVA)
#include <tr1/memory>
#include <pv/channelProviderLocal.h>
#include <pv/pvData.h>
#include <pv/pvDatabase.h>
#endif

namespace {

using asynParamInventory::Inventory;
using asynParamInventory::ParamInfo;
using asynParamInventory::PortInfo;
using asynParamInventory::RecordRef;

struct RegisteredPort {
    std::string portName;
    asynPortDriver *driver;
};

struct ParsedLink {
    ParsedLink() : addr(0), valid(false) {}
    std::string portName;
    int addr;
    std::string userParam;
    bool valid;
};

epicsMutex& inventoryMutex()
{
    static epicsMutex mutex;
    return mutex;
}

std::map<std::string, asynPortDriver*>& registry()
{
    static std::map<std::string, asynPortDriver*> ports;
    return ports;
}

std::string& publishedPvName()
{
    static std::string name;
    return name;
}

bool& initHookRegistered()
{
    static bool registered = false;
    return registered;
}

#if defined(WITH_PVXS)
pvxs::server::SharedPV& sharedPv()
{
    static pvxs::server::SharedPV pv(pvxs::server::SharedPV::buildReadonly());
    return pv;
}

bool& sharedPvAdded()
{
    static bool added = false;
    return added;
}
#endif

const char* paramTypeName(asynParamType type)
{
    switch (type) {
    case asynParamNotDefined:    return "asynParamNotDefined";
    case asynParamInt32:         return "asynParamInt32";
    case asynParamInt64:         return "asynParamInt64";
    case asynParamUInt32Digital: return "asynParamUInt32Digital";
    case asynParamFloat64:       return "asynParamFloat64";
    case asynParamOctet:         return "asynParamOctet";
    case asynParamInt8Array:     return "asynParamInt8Array";
    case asynParamInt16Array:    return "asynParamInt16Array";
    case asynParamInt32Array:    return "asynParamInt32Array";
    case asynParamInt64Array:    return "asynParamInt64Array";
    case asynParamFloat32Array:  return "asynParamFloat32Array";
    case asynParamFloat64Array:  return "asynParamFloat64Array";
    case asynParamGenericPointer:return "asynParamGenericPointer";
    default:                     return "asynParamUnknown";
    }
}

std::string toFieldName(const std::string& value)
{
    std::string name(value);
    size_t i;
    for (i=0; i<name.size(); i++) {
        char c = name[i];
        int ok = ((c >= 'A') && (c <= 'Z')) ||
                 ((c >= 'a') && (c <= 'z')) ||
                 ((c >= '0') && (c <= '9')) ||
                 (c == '_');
        if (!ok) name[i] = '_';
    }
    if (name.empty() || ((name[0] >= '0') && (name[0] <= '9'))) {
        name = "_" + name;
    }
    return name;
}

std::string makeUniqueFieldName(const std::string& name, std::set<std::string>& usedNames)
{
    std::string base = toFieldName(name);
    std::string candidate = base;
    size_t suffix = 1;
    while (usedNames.find(candidate) != usedNames.end()) {
        char buffer[32];
        epicsSnprintf(buffer, sizeof(buffer), "_%lu", (unsigned long)suffix++);
        candidate = base + buffer;
    }
    usedNames.insert(candidate);
    return candidate;
}

const char *skipSpace(const char *text, int commaOk)
{
    const char *next = text;
    while (*next && (isspace((int)*next) || (commaOk && (*next == ',')))) {
        next++;
    }
    return next;
}

int parseIntField(const char *text, const char **endText, long *value)
{
    char *endp = 0;
    errno = 0;
    *value = strtol(text, &endp, 0);
    if (errno || (endp == text)) return 0;
    *endText = endp;
    return 1;
}

ParsedLink parseAsynLink(const std::string& link)
{
    ParsedLink parsed;
    const char *text = link.c_str();
    const char *start = strstr(text, "asyn(");
    int hasMask = 0;
    if (!start) {
        start = strstr(text, "asynMask(");
        hasMask = 1;
    }
    if (!start) return parsed;

    start += hasMask ? 9 : 5;
    start = skipSpace(start, 0);
    const char *portStart = start;
    while (*start && !isspace((int)*start) && (*start != ',') && (*start != ')')) {
        start++;
    }
    if ((start == portStart) || (*start == '\0')) return parsed;

    parsed.portName.assign(portStart, start - portStart);

    start = skipSpace(start, 1);
    if (*start == ')') {
        parsed.valid = true;
        start++;
    } else {
        long addr = 0;
        if (!parseIntField(start, &start, &addr)) return parsed;
        parsed.addr = (int)addr;
        start = skipSpace(start, 1);
        if (hasMask) {
            long ignored = 0;
            if (!parseIntField(start, &start, &ignored)) return parsed;
            start = skipSpace(start, 1);
        }
        if (*start != ')') {
            char *endp = 0;
            errno = 0;
            (void)strtod(start, &endp);
            if (errno || (endp == start)) return parsed;
            start = endp;
            start = skipSpace(start, 0);
            if (*start != ')') return parsed;
        }
        parsed.valid = true;
        start++;
    }

    start = skipSpace(start, 0);
    if (*start) {
        parsed.userParam = start;
    }
    return parsed;
}

int parseReason(const std::string& userParam, int *reason)
{
    const char *text = userParam.c_str();
    const char *start = strstr(text, "reason(");
    char *endp = 0;
    long value = 0;

    if (!start) return 0;
    start += 7;
    errno = 0;
    value = strtol(start, &endp, 0);
    if (errno || (endp == start)) return 0;
    endp = (char*)skipSpace(endp, 0);
    if (*endp != ')') return 0;
    *reason = (int)value;
    return 1;
}

void buildPortInfo(asynPortDriver *driver, PortInfo *portInfo)
{
    int addr;

    portInfo->portName = driver->portName ? driver->portName : "";
    portInfo->driverClass = "";

    driver->lock();
    for (addr=0; addr<driver->maxAddr; addr++) {
        int numParams = 0;
        if (driver->getNumParams(addr, &numParams) != asynSuccess) continue;
        for (int index=0; index<numParams; index++) {
            const char *paramName = 0;
            asynParamType type = asynParamNotDefined;
            if (driver->getParamName(addr, index, &paramName) != asynSuccess) continue;
            if (driver->getParamType(addr, index, &type) != asynSuccess) continue;

            ParamInfo info;
            info.name = paramName ? paramName : "";
            info.asynType = paramTypeName(type);
            info.addr = addr;
            info.index = index;
            portInfo->params.push_back(info);
        }
    }
    driver->unlock();
}

ParamInfo *findParamInfo(PortInfo *portInfo, const ParsedLink& parsed)
{
    std::vector<ParamInfo>::iterator it;
    int reason = -1;

    if (!parsed.userParam.empty()) {
        for (it = portInfo->params.begin(); it != portInfo->params.end(); ++it) {
            if ((it->addr == parsed.addr) && (it->name == parsed.userParam)) {
                return &(*it);
            }
        }
    }

    if (parseReason(parsed.userParam, &reason)) {
        for (it = portInfo->params.begin(); it != portInfo->params.end(); ++it) {
            if ((it->addr == parsed.addr) && (it->index == reason)) {
                return &(*it);
            }
        }
    }

    return 0;
}

void addRecordRef(PortInfo *portInfo, const ParsedLink& parsed, const RecordRef& ref, int readback)
{
    ParamInfo *paramInfo = findParamInfo(portInfo, parsed);
    if (!paramInfo) {
        portInfo->unmatched.push_back(ref);
        return;
    }

    if (readback) {
        paramInfo->readbacks.push_back(ref);
    } else {
        paramInfo->setpoints.push_back(ref);
    }
}

#ifndef EPICS_LIBCOM_ONLY
std::string getFieldString(const DBENTRY& entry, const char *fieldName)
{
    std::string value;
    DBENTRY *field = dbCopyEntry(&entry);
    if (!field) return value;
    if (!dbFindField(field, fieldName)) {
        char *text = dbGetString(field);
        if (text) value = text;
    }
    dbFreeEntry(field);
    return value;
}

std::string getInfoString(const DBENTRY& entry, const char *infoName)
{
    std::string value;
    DBENTRY *copy = dbCopyEntry(&entry);
    if (!copy) return value;
    {
        const char *text = dbGetInfo(copy, infoName);
        if (text) value = text;
    }
    dbFreeEntry(copy);
    return value;
}

void scanLinkField(PortInfo *portInfo,
                   const RecordRef& ref,
                   const std::string& linkText,
                   int readback)
{
    ParsedLink parsed = parseAsynLink(linkText);
    if (!parsed.valid) return;
    if (parsed.portName != portInfo->portName) return;
    addRecordRef(portInfo, parsed, ref, readback);
}

void attachRecordBindings(Inventory *inventory)
{
    DBENTRY entry;

    if (!pdbbase) return;

    dbInitEntry(pdbbase, &entry);
    for (long status = dbFirstRecord(&entry); !status; status = dbNextRecord(&entry)) {
        RecordRef ref;
        Inventory::iterator portIt;
        std::string dtyp = getFieldString(entry, "DTYP");
        std::string inp = getFieldString(entry, "INP");
        std::string out = getFieldString(entry, "OUT");
        std::string readbackInfo = getInfoString(entry, "asyn:READBACK");

        ref.name = dbGetRecordName(&entry) ? dbGetRecordName(&entry) : "";
        ref.recordType = dbGetRecordTypeName(&entry) ? dbGetRecordTypeName(&entry) : "";
        ref.dtyp = dtyp;

        if (!inp.empty()) {
            ref.link = inp;
            for (portIt = inventory->begin(); portIt != inventory->end(); ++portIt) {
                scanLinkField(&portIt->second, ref, inp, 1);
            }
        }

        if (!out.empty()) {
            int readback = (!readbackInfo.empty() && (atoi(readbackInfo.c_str()) != 0));
            ref.link = out;
            for (portIt = inventory->begin(); portIt != inventory->end(); ++portIt) {
                scanLinkField(&portIt->second, ref, out, readback);
            }
        }
    }
    dbFinishEntry(&entry);
}
#endif

void printRecordRefs(FILE *fp, const char *label, const std::vector<RecordRef>& refs)
{
    size_t i;
    fprintf(fp, "      %s (%lu)\n", label, (unsigned long)refs.size());
    for (i=0; i<refs.size(); i++) {
        fprintf(fp, "        %s [%s]", refs[i].name.c_str(), refs[i].recordType.c_str());
        if (!refs[i].dtyp.empty()) fprintf(fp, " DTYP=%s", refs[i].dtyp.c_str());
        if (!refs[i].link.empty()) fprintf(fp, " LINK=%s", refs[i].link.c_str());
        fprintf(fp, "\n");
    }
}

#if defined(WITH_PVXS) || defined(WITH_PVA)
struct PortFieldMapEntry {
    std::string portName;
    std::string fieldName;
};

std::vector<PortFieldMapEntry> buildPortFieldMap(const Inventory& inventory)
{
    std::vector<PortFieldMapEntry> mapping;
    std::set<std::string> usedNames;
    Inventory::const_iterator it;
    for (it = inventory.begin(); it != inventory.end(); ++it) {
        PortFieldMapEntry entry;
        entry.portName = it->first;
        entry.fieldName = makeUniqueFieldName(it->first, usedNames);
        mapping.push_back(entry);
    }
    return mapping;
}

#if defined(WITH_PVXS)
std::vector<pvxs::Member> recordRefMembers()
{
    std::vector<pvxs::Member> members;
    members.push_back(pvxs::members::String("name"));
    members.push_back(pvxs::members::String("recordType"));
    members.push_back(pvxs::members::String("dtyp"));
    members.push_back(pvxs::members::String("link"));
    return members;
}

void assignRecordRefs(pvxs::Value arrayField, const std::vector<RecordRef>& refs)
{
    pvxs::shared_array<pvxs::Value> rows(refs.size());
    size_t i;
    for (i=0; i<refs.size(); i++) {
        pvxs::Value row = arrayField.allocMember();
        row["name"] = refs[i].name;
        row["recordType"] = refs[i].recordType;
        row["dtyp"] = refs[i].dtyp;
        row["link"] = refs[i].link;
        rows[i] = row;
    }
    arrayField = rows.freeze();
}

void publishPvxs(const Inventory& inventory)
{
    std::vector<PortFieldMapEntry> portFields = buildPortFieldMap(inventory);
    std::vector<pvxs::Member> refMembers = recordRefMembers();
    std::vector<pvxs::Member> valueMembers;
    size_t i;

    for (i=0; i<portFields.size(); i++) {
        std::vector<pvxs::Member> portMembers;
        portMembers.push_back(pvxs::members::String("portName"));
        portMembers.push_back(pvxs::members::String("driverClass"));
        portMembers.push_back(
            pvxs::members::StructA("params", {
                pvxs::members::String("name"),
                pvxs::members::String("asynType"),
                pvxs::members::Int32("addr"),
                pvxs::members::Int32("index"),
                pvxs::members::StructA("setpoints", refMembers),
                pvxs::members::StructA("readbacks", refMembers)
            }));
        portMembers.push_back(pvxs::members::StructA("unmatched", refMembers));
        valueMembers.push_back(pvxs::members::Struct(portFields[i].fieldName, portMembers));
    }

    pvxs::Value root = pvxs::TypeDef(
        pvxs::TypeCode::Struct,
        "asyn:paramInventory:1.0",
        { pvxs::members::Struct("value", valueMembers) }).create();

    for (i=0; i<portFields.size(); i++) {
        const PortInfo& portInfo = inventory.find(portFields[i].portName)->second;
        pvxs::Value portField = root["value"][portFields[i].fieldName];
        pvxs::shared_array<pvxs::Value> params(portInfo.params.size());
        size_t j;

        portField["portName"] = portInfo.portName;
        portField["driverClass"] = portInfo.driverClass;

        for (j=0; j<portInfo.params.size(); j++) {
            pvxs::Value param = portField["params"].allocMember();
            param["name"] = portInfo.params[j].name;
            param["asynType"] = portInfo.params[j].asynType;
            param["addr"] = portInfo.params[j].addr;
            param["index"] = portInfo.params[j].index;
            assignRecordRefs(param["setpoints"], portInfo.params[j].setpoints);
            assignRecordRefs(param["readbacks"], portInfo.params[j].readbacks);
            params[j] = param;
        }
        portField["params"] = params.freeze();
        assignRecordRefs(portField["unmatched"], portInfo.unmatched);
    }

    if (!sharedPvAdded()) {
        pvxs::ioc::server().addPV(publishedPvName(), sharedPv());
        sharedPvAdded() = true;
    }
    if (sharedPv().isOpen()) {
        sharedPv().close();
    }
    sharedPv().open(root);
}
#elif defined(WITH_PVA)
class InventoryRecord;
typedef std::tr1::shared_ptr<InventoryRecord> InventoryRecordPtr;

class InventoryRecord : public epics::pvDatabase::PVRecord {
public:
    static InventoryRecordPtr create(const std::string& name,
                                     epics::pvData::PVStructurePtr const& pvStructure)
    {
        InventoryRecordPtr record(new InventoryRecord(name, pvStructure));
        if (!record->init()) record.reset();
        return record;
    }
    virtual bool init()
    {
        initPVRecord();
        return true;
    }
    virtual void process() {}
private:
    InventoryRecord(const std::string& name,
                    epics::pvData::PVStructurePtr const& pvStructure)
        : epics::pvDatabase::PVRecord(name, pvStructure)
    {}
};

void assignRecordRefs(epics::pvData::PVStructureArrayPtr array,
                      const std::vector<RecordRef>& refs)
{
    using namespace epics::pvData;
    StructureConstPtr refType = array->getStructureArray()->getStructure();
    PVStructureArray::svector rows(refs.size());
    size_t i;

    for (i=0; i<refs.size(); i++) {
        PVStructurePtr row = getPVDataCreate()->createPVStructure(refType);
        row->getSubField<PVString>("name")->put(refs[i].name);
        row->getSubField<PVString>("recordType")->put(refs[i].recordType);
        row->getSubField<PVString>("dtyp")->put(refs[i].dtyp);
        row->getSubField<PVString>("link")->put(refs[i].link);
        rows[i] = row;
    }
    array->replace(freeze(rows));
}

void publishPva(const Inventory& inventory)
{
    using namespace epics::pvData;
    std::vector<PortFieldMapEntry> portFields = buildPortFieldMap(inventory);
    FieldBuilderPtr builder = getFieldCreate()->createFieldBuilder();
    size_t i;

    builder->addNestedStructure("value");
    for (i=0; i<portFields.size(); i++) {
        builder->addNestedStructure(portFields[i].fieldName)
               ->add("portName", pvString)
               ->add("driverClass", pvString)
               ->addNestedStructureArray("params")
                    ->add("name", pvString)
                    ->add("asynType", pvString)
                    ->add("addr", pvInt)
                    ->add("index", pvInt)
                    ->addNestedStructureArray("setpoints")
                        ->add("name", pvString)
                        ->add("recordType", pvString)
                        ->add("dtyp", pvString)
                        ->add("link", pvString)
                    ->endNested()
                    ->addNestedStructureArray("readbacks")
                        ->add("name", pvString)
                        ->add("recordType", pvString)
                        ->add("dtyp", pvString)
                        ->add("link", pvString)
                    ->endNested()
               ->endNested()
               ->addNestedStructureArray("unmatched")
                    ->add("name", pvString)
                    ->add("recordType", pvString)
                    ->add("dtyp", pvString)
                    ->add("link", pvString)
               ->endNested()
               ->endNested();
    }
    builder->endNested();

    StructureConstPtr structure = builder->createStructure();
    PVStructurePtr root = getPVDataCreate()->createPVStructure(structure);

    for (i=0; i<portFields.size(); i++) {
        const PortInfo& portInfo = inventory.find(portFields[i].portName)->second;
        std::string portPath = "value." + portFields[i].fieldName;
        PVStructurePtr portField = root->getSubField<PVStructure>(portPath);
        PVStructureArrayPtr params = portField->getSubField<PVStructureArray>("params");
        PVStructureArray::svector paramRows(portInfo.params.size());
        StructureConstPtr paramType = params->getStructureArray()->getStructure();
        size_t j;

        portField->getSubField<PVString>("portName")->put(portInfo.portName);
        portField->getSubField<PVString>("driverClass")->put(portInfo.driverClass);

        for (j=0; j<portInfo.params.size(); j++) {
            PVStructurePtr param = getPVDataCreate()->createPVStructure(paramType);
            param->getSubField<PVString>("name")->put(portInfo.params[j].name);
            param->getSubField<PVString>("asynType")->put(portInfo.params[j].asynType);
            param->getSubField<PVInt>("addr")->put(portInfo.params[j].addr);
            param->getSubField<PVInt>("index")->put(portInfo.params[j].index);
            assignRecordRefs(param->getSubField<PVStructureArray>("setpoints"),
                             portInfo.params[j].setpoints);
            assignRecordRefs(param->getSubField<PVStructureArray>("readbacks"),
                             portInfo.params[j].readbacks);
            paramRows[j] = param;
        }
        params->replace(freeze(paramRows));
        assignRecordRefs(portField->getSubField<PVStructureArray>("unmatched"),
                         portInfo.unmatched);
    }

    epics::pvDatabase::PVDatabasePtr master = epics::pvDatabase::PVDatabase::getMaster();
    epics::pvDatabase::getChannelProviderLocal();
    if (master->findRecord(publishedPvName())) {
        master->removeRecord(master->findRecord(publishedPvName()));
    }
    InventoryRecordPtr record = InventoryRecord::create(publishedPvName(), root);
    if (record) {
        master->addRecord(record);
    }
}
#endif

void publishInventory()
{
    if (publishedPvName().empty()) return;
#if defined(WITH_PVXS)
    publishPvxs(asynParamInventory::getInventory());
#elif defined(WITH_PVA)
    publishPva(asynParamInventory::getInventory());
#endif
}

void inventoryInitHook(initHookState state)
{
    if (state == initHookAfterIocRunning) {
        publishInventory();
    }
}
#endif

} /* namespace */

namespace asynParamInventory {

void registerPort(asynPortDriver *port)
{
    epicsGuard<epicsMutex> guard(inventoryMutex());
    if (!port || !port->portName) return;
    registry()[port->portName] = port;
}

void unregisterPort(const char *portName)
{
    epicsGuard<epicsMutex> guard(inventoryMutex());
    if (!portName) return;
    registry().erase(portName);
}

Inventory getInventory()
{
    Inventory inventory;
    std::vector<RegisteredPort> ports;
    std::map<std::string, asynPortDriver*>::iterator it;

    {
        epicsGuard<epicsMutex> guard(inventoryMutex());
        for (it = registry().begin(); it != registry().end(); ++it) {
            RegisteredPort port;
            port.portName = it->first;
            port.driver = it->second;
            ports.push_back(port);
        }
    }

    for (size_t i=0; i<ports.size(); i++) {
        PortInfo portInfo;
        buildPortInfo(ports[i].driver, &portInfo);
        inventory[ports[i].portName] = portInfo;
    }

#ifndef EPICS_LIBCOM_ONLY
    attachRecordBindings(&inventory);
#endif

    return inventory;
}

void report(FILE *fp, const char *portName)
{
    Inventory inventory = getInventory();
    Inventory::const_iterator it;

    for (it = inventory.begin(); it != inventory.end(); ++it) {
        size_t i;
        if (portName && *portName && (it->first != portName)) continue;

        fprintf(fp, "asynParamInventory port=%s\n", it->second.portName.c_str());
        if (!it->second.driverClass.empty()) {
            fprintf(fp, "  driverClass=%s\n", it->second.driverClass.c_str());
        }
        fprintf(fp, "  parameters (%lu)\n", (unsigned long)it->second.params.size());
        for (i=0; i<it->second.params.size(); i++) {
            const ParamInfo& param = it->second.params[i];
            fprintf(fp, "    [%d,%d] %s type=%s\n",
                    param.addr,
                    param.index,
                    param.name.c_str(),
                    param.asynType.c_str());
            printRecordRefs(fp, "setpoints", param.setpoints);
            printRecordRefs(fp, "readbacks", param.readbacks);
        }
        printRecordRefs(fp, "unmatched", it->second.unmatched);
    }
}

} /* namespace asynParamInventory */

extern "C" int asynParamInventoryReport(const char *portName)
{
    asynParamInventory::report(stdout, portName);
    return 0;
}

extern "C" int asynParamInventoryConfigure(const char *pvName)
{
    publishedPvName() = pvName ? pvName : "";

#if defined(WITH_PVA) && !defined(WITH_PVXS)
    epics::pvDatabase::getChannelProviderLocal();
#endif

#if defined(WITH_PVXS) || defined(WITH_PVA)
    if (!initHookRegistered()) {
        initHookRegister(&inventoryInitHook);
        initHookRegistered() = true;
    }
#else
    if (!publishedPvName().empty()) {
        errlogPrintf("asynParamInventoryConfigure: built without PVAccess publishing support, inventory will not be published\n");
    }
#endif

    return 0;
}

static const iocshArg reportArg0 = { "portName", iocshArgString };
static const iocshArg * const reportArgs[] = { &reportArg0 };
static const iocshFuncDef reportDef = { "asynParamInventoryReport", 1, reportArgs };
static void reportCall(const iocshArgBuf *args)
{
    asynParamInventoryReport(args[0].sval);
}

static const iocshArg configArg0 = { "pvName", iocshArgString };
static const iocshArg * const configArgs[] = { &configArg0 };
static const iocshFuncDef configDef = { "asynParamInventoryConfigure", 1, configArgs };
static void configCall(const iocshArgBuf *args)
{
    asynParamInventoryConfigure(args[0].sval);
}

extern "C" void asynParamInventoryRegister(void)
{
    iocshRegister(&reportDef, reportCall);
    iocshRegister(&configDef, configCall);
}

extern "C" {
epicsExportRegistrar(asynParamInventoryRegister);
}
